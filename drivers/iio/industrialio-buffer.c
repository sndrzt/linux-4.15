/* The industrial I/O core
 *
 * Copyright (c) 2008 Jonathan Cameron
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Handling of buffer allocation / resizing.
 *
 *
 * Things to look at here.
 * - Better memory allocation techniques?
 * - Alternative access techniques?
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched/signal.h>

#include <linux/iio/iio.h>
#include "iio_core.h"
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>

static const char * const iio_endian_prefix[] = {
	[IIO_BE] = "be",
	[IIO_LE] = "le",
};

static bool iio_buffer_is_active(struct iio_buffer *buf)
{
	return !list_empty(&buf->buffer_list);
}

static size_t iio_buffer_data_available(struct iio_buffer *buf)
{
	return buf->access->data_available(buf);
}

static int iio_buffer_flush_hwfifo(struct iio_dev *indio_dev,
				   struct iio_buffer *buf, size_t required)
{
	if (!indio_dev->info->hwfifo_flush_to_buffer)
		return -ENODEV;

	return indio_dev->info->hwfifo_flush_to_buffer(indio_dev, required);
}

static bool iio_buffer_ready(struct iio_dev *indio_dev, struct iio_buffer *buf,
			     size_t to_wait, int to_flush)
{
	size_t avail;
	int flushed = 0;

	/* wakeup if the device was unregistered */
	if (!indio_dev->info)
		return true;

	/* drain the buffer if it was disabled */
	if (!iio_buffer_is_active(buf)) {
		to_wait = min_t(size_t, to_wait, 1);
		to_flush = 0;
	}

	avail = iio_buffer_data_available(buf);

	if (avail >= to_wait) {
		/* force a flush for non-blocking reads */
		if (!to_wait && avail < to_flush)
			iio_buffer_flush_hwfifo(indio_dev, buf,
						to_flush - avail);
		return true;
	}

	if (to_flush)
		flushed = iio_buffer_flush_hwfifo(indio_dev, buf,
						  to_wait - avail);
	if (flushed <= 0)
		return false;

	if (avail + flushed >= to_wait)
		return true;

	return false;
}

/**
 * iio_buffer_read_first_n_outer() - chrdev read for buffer access
 * @filp:	File structure pointer for the char device
 * @buf:	Destination buffer for iio buffer read
 * @n:		First n bytes to read
 * @f_ps:	Long offset provided by the user as a seek position
 *
 * This function relies on all buffer implementations having an
 * iio_buffer as their first element.
 *
 * Return: negative values corresponding to error codes or ret != 0
 *	   for ending the reading activity
 **/
ssize_t iio_buffer_read_first_n_outer(struct file *filp, char __user *buf,
				      size_t n, loff_t *f_ps)
{
	struct iio_dev *indio_dev = filp->private_data;
	struct iio_buffer *rb = indio_dev->buffer;
	DEFINE_WAIT_FUNC(wait, woken_wake_function);
	size_t datum_size;
	size_t to_wait;
	int ret = 0;

	if (!indio_dev->info)
		return -ENODEV;

	if (!rb || !rb->access->read_first_n)
		return -EINVAL;

	datum_size = rb->bytes_per_datum;

	/*
	 * If datum_size is 0 there will never be anything to read from the
	 * buffer, so signal end of file now.
	 */
	if (!datum_size)
		return 0;

	if (filp->f_flags & O_NONBLOCK)
		to_wait = 0;
	else
		to_wait = min_t(size_t, n / datum_size, rb->watermark);

	add_wait_queue(&rb->pollq, &wait);
	do {
		if (!indio_dev->info) {
			ret = -ENODEV;
			break;
		}

		if (!iio_buffer_ready(indio_dev, rb, to_wait, n / datum_size)) {
			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
				break;
			}

			wait_woken(&wait, TASK_INTERRUPTIBLE,
				   MAX_SCHEDULE_TIMEOUT);
			continue;
		}

		ret = rb->access->read_first_n(rb, n, buf);
		if (ret == 0 && (filp->f_flags & O_NONBLOCK))
			ret = -EAGAIN;
	} while (ret == 0);
	remove_wait_queue(&rb->pollq, &wait);

	return ret;
}

/**
 * iio_buffer_poll() - poll the buffer to find out if it has data
 * @filp:	File structure pointer for device access
 * @wait:	Poll table structure pointer for which the driver adds
 *		a wait queue
 *
 * Return: (POLLIN | POLLRDNORM) if data is available for reading
 *	   or 0 for other cases
 */
unsigned int iio_buffer_poll(struct file *filp,
			     struct poll_table_struct *wait)
{
	struct iio_dev *indio_dev = filp->private_data;
	struct iio_buffer *rb = indio_dev->buffer;

	if (!indio_dev->info || rb == NULL)
		return 0;

	poll_wait(filp, &rb->pollq, wait);
	if (iio_buffer_ready(indio_dev, rb, rb->watermark, 0))
		return POLLIN | POLLRDNORM;
	return 0;
}

/**
 * iio_buffer_wakeup_poll - Wakes up the buffer waitqueue
 * @indio_dev: The IIO device
 *
 * Wakes up the event waitqueue used for poll(). Should usually
 * be called when the device is unregistered.
 */
void iio_buffer_wakeup_poll(struct iio_dev *indio_dev)
{
	if (!indio_dev->buffer)
		return;

	wake_up(&indio_dev->buffer->pollq);
}

void iio_buffer_init(struct iio_buffer *buffer)
{
	INIT_LIST_HEAD(&buffer->demux_list);
	INIT_LIST_HEAD(&buffer->buffer_list);
	init_waitqueue_head(&buffer->pollq);
	kref_init(&buffer->ref);
	if (!buffer->watermark)
		buffer->watermark = 1;
}
EXPORT_SYMBOL(iio_buffer_init);

/**
 * iio_buffer_set_attrs - Set buffer specific attributes
 * @buffer: The buffer for which we are setting attributes
 * @attrs: Pointer to a null terminated list of pointers to attributes
 */
void iio_buffer_set_attrs(struct iio_buffer *buffer,
			 const struct attribute **attrs)
{
	buffer->attrs = attrs;
}
EXPORT_SYMBOL_GPL(iio_buffer_set_attrs);

static ssize_t iio_show_scan_index(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	return sprintf(buf, "%u\n", to_iio_dev_attr(attr)->c->scan_index);
}

static ssize_t iio_show_fixed_type(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	u8 type = this_attr->c->scan_type.endianness;

	if (type == IIO_CPU) {
#ifdef __LITTLE_ENDIAN
		type = IIO_LE;
#else
		type = IIO_BE;
#endif
	}
	if (this_attr->c->scan_type.repeat > 1)
		return sprintf(buf, "%s:%c%d/%dX%d>>%u\n",
		       iio_endian_prefix[type],
		       this_attr->c->scan_type.sign,
		       this_attr->c->scan_type.realbits,
		       this_attr->c->scan_type.storagebits,
		       this_attr->c->scan_type.repeat,
		       this_attr->c->scan_type.shift);
		else
			return sprintf(buf, "%s:%c%d/%d>>%u\n",
		       iio_endian_prefix[type],
		       this_attr->c->scan_type.sign,
		       this_attr->c->scan_type.realbits,
		       this_attr->c->scan_type.storagebits,
		       this_attr->c->scan_type.shift);
}

static ssize_t iio_scan_el_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	/* Ensure ret is 0 or 1. */
	ret = !!test_bit(to_iio_dev_attr(attr)->address,
		       indio_dev->buffer->scan_mask);

	return sprintf(buf, "%d\n", ret);
}

/* Note NULL used as error indicator as it doesn't make sense. */
static const unsigned long *iio_scan_mask_match(const unsigned long *av_masks,
					  unsigned int masklength,
					  const unsigned long *mask,
					  bool strict)
{
	if (bitmap_empty(mask, masklength))
		return NULL;
	while (*av_masks) {
		if (strict) {
			if (bitmap_equal(mask, av_masks, masklength))
				return av_masks;
		} else {
			if (bitmap_subset(mask, av_masks, masklength))
				return av_masks;
		}
		av_masks += BITS_TO_LONGS(masklength);
	}
	return NULL;
}

static bool iio_validate_scan_mask(struct iio_dev *indio_dev,
	const unsigned long *mask)
{
	if (!indio_dev->setup_ops->validate_scan_mask)
		return true;

	return indio_dev->setup_ops->validate_scan_mask(indio_dev, mask);
}

/**
 * iio_scan_mask_set() - set particular bit in the scan mask
 * @indio_dev: the iio device
 * @buffer: the buffer whose scan mask we are interested in
 * @bit: the bit to be set.
 *
 * Note that at this point we have no way of knowing what other
 * buffers might request, hence this code only verifies that the
 * individual buffers request is plausible.
 */
static int iio_scan_mask_set(struct iio_dev *indio_dev,
		      struct iio_buffer *buffer, int bit)
{
	const unsigned long *mask;
	unsigned long *trialmask;

	trialmask = kcalloc(BITS_TO_LONGS(indio_dev->masklength),
			    sizeof(*trialmask), GFP_KERNEL);
	if (trialmask == NULL)
		return -ENOMEM;
	if (!indio_dev->masklength) {
		WARN(1, "Trying to set scanmask prior to registering buffer\n");
		goto err_invalid_mask;
	}
	bitmap_copy(trialmask, buffer->scan_mask, indio_dev->masklength);
	set_bit(bit, trialmask);

	if (!iio_validate_scan_mask(indio_dev, trialmask))
		goto err_invalid_mask;

	if (indio_dev->available_scan_masks) {
		mask = iio_scan_mask_match(indio_dev->available_scan_masks,
					   indio_dev->masklength,
					   trialmask, false);
		if (!mask)
			goto err_invalid_mask;
	}
	bitmap_copy(buffer->scan_mask, trialmask, indio_dev->masklength);

	kfree(trialmask);

	return 0;

err_invalid_mask:
	kfree(trialmask);
	return -EINVAL;
}

static int iio_scan_mask_clear(struct iio_buffer *buffer, int bit)
{
	clear_bit(bit, buffer->scan_mask);
	return 0;
}

static int iio_scan_mask_query(struct iio_dev *indio_dev,
			       struct iio_buffer *buffer, int bit)
{
	if (bit > indio_dev->masklength)
		return -EINVAL;

	if (!buffer->scan_mask)
		return 0;

	/* Ensure return value is 0 or 1. */
	return !!test_bit(bit, buffer->scan_mask);
};

static ssize_t iio_scan_el_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t len)
{
	int ret;
	bool state;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_buffer *buffer = indio_dev->buffer;
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	ret = strtobool(buf, &state);
	if (ret < 0)
		return ret;
	mutex_lock(&indio_dev->mlock);
	if (iio_buffer_is_active(indio_dev->buffer)) {
		ret = -EBUSY;
		goto error_ret;
	}
	ret = iio_scan_mask_query(indio_dev, buffer, this_attr->address);
	if (ret < 0)
		goto error_ret;
	if (!state && ret) {
		ret = iio_scan_mask_clear(buffer, this_attr->address);
		if (ret)
			goto error_ret;
	} else if (state && !ret) {
		ret = iio_scan_mask_set(indio_dev, buffer, this_attr->address);
		if (ret)
			goto error_ret;
	}

error_ret:
	mutex_unlock(&indio_dev->mlock);

	return ret < 0 ? ret : len;

}

static ssize_t iio_scan_el_ts_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	return sprintf(buf, "%d\n", indio_dev->buffer->scan_timestamp);
}

static ssize_t iio_scan_el_ts_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t len)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	bool state;

	ret = strtobool(buf, &state);
	if (ret < 0)
		return ret;

	mutex_lock(&indio_dev->mlock);
	if (iio_buffer_is_active(indio_dev->buffer)) {
		ret = -EBUSY;
		goto error_ret;
	}
	indio_dev->buffer->scan_timestamp = state;
error_ret:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static int iio_buffer_add_channel_sysfs(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	int ret, attrcount = 0;
	struct iio_buffer *buffer = indio_dev->buffer;

	ret = __iio_add_chan_devattr("index",
				     chan,
				     &iio_show_scan_index,
				     NULL,
				     0,
				     IIO_SEPARATE,
				     &indio_dev->dev,
				     &buffer->scan_el_dev_attr_list);
	if (ret)
		return ret;
	attrcount++;
	ret = __iio_add_chan_devattr("type",
				     chan,
				     &iio_show_fixed_type,
				     NULL,
				     0,
				     0,
				     &indio_dev->dev,
				     &buffer->scan_el_dev_attr_list);
	if (ret)
		return ret;
	attrcount++;
	if (chan->type != IIO_TIMESTAMP)
		ret = __iio_add_chan_devattr("en",
					     chan,
					     &iio_scan_el_show,
					     &iio_scan_el_store,
					     chan->scan_index,
					     0,
					     &indio_dev->dev,
					     &buffer->scan_el_dev_attr_list);
	else
		ret = __iio_add_chan_devattr("en",
					     chan,
					     &iio_scan_el_ts_show,
					     &iio_scan_el_ts_store,
					     chan->scan_index,
					     0,
					     &indio_dev->dev,
					     &buffer->scan_el_dev_attr_list);
	if (ret)
		return ret;
	attrcount++;
	ret = attrcount;
	return ret;
}

static ssize_t iio_buffer_read_length(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_buffer *buffer = indio_dev->buffer;

	return sprintf(buf, "%d\n", buffer->length);
}

static ssize_t iio_buffer_write_length(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_buffer *buffer = indio_dev->buffer;
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val == buffer->length)
		return len;

	mutex_lock(&indio_dev->mlock);
	if (iio_buffer_is_active(indio_dev->buffer)) {
		ret = -EBUSY;
	} else {
		buffer->access->set_length(buffer, val);
		ret = 0;
	}
	if (ret)
		goto out;
	if (buffer->length && buffer->length < buffer->watermark)
		buffer->watermark = buffer->length;
out:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t iio_buffer_show_enable(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	return sprintf(buf, "%d\n", iio_buffer_is_active(indio_dev->buffer));
}

static unsigned int iio_storage_bytes_for_si(struct iio_dev *indio_dev,
					     unsigned int scan_index)
{
	const struct iio_chan_spec *ch;
	unsigned int bytes;

	ch = iio_find_channel_from_si(indio_dev, scan_index);
	bytes = ch->scan_type.storagebits / 8;
	if (ch->scan_type.repeat > 1)
		bytes *= ch->scan_type.repeat;
	return bytes;
}

static unsigned int iio_storage_bytes_for_timestamp(struct iio_dev *indio_dev)
{
	return iio_storage_bytes_for_si(indio_dev,
					indio_dev->scan_index_timestamp);
}

static int iio_compute_scan_bytes(struct iio_dev *indio_dev,
				const unsigned long *mask, bool timestamp)
{
	unsigned bytes = 0;
	int length, i, largest = 0;

	/* How much space will the demuxed element take? */
	for_each_set_bit(i, mask,
			 indio_dev->masklength) {
		length = iio_storage_bytes_for_si(indio_dev, i);
		bytes = ALIGN(bytes, length);
		bytes += length;
		largest = max(largest, length);
	}

	if (timestamp) {
		length = iio_storage_bytes_for_timestamp(indio_dev);
		bytes = ALIGN(bytes, length);
		bytes += length;
		largest = max(largest, length);
	}

	bytes = ALIGN(bytes, largest);
	return bytes;
}

static void iio_buffer_activate(struct iio_dev *indio_dev,
	struct iio_buffer *buffer)
{
	iio_buffer_get(buffer);
	list_add(&buffer->buffer_list, &indio_dev->buffer_list);
}

static void iio_buffer_deactivate(struct iio_buffer *buffer)
{
	list_del_init(&buffer->buffer_list);
	wake_up_interruptible(&buffer->pollq);
	iio_buffer_put(buffer);
}

static void iio_buffer_deactivate_all(struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer, *_buffer;

	list_for_each_entry_safe(buffer, _buffer,
			&indio_dev->buffer_list, buffer_list)
		iio_buffer_deactivate(buffer);
}

static int iio_buffer_enable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev)
{
	if (!buffer->access->enable)
		return 0;
	return buffer->access->enable(buffer, indio_dev);
}

static int iio_buffer_disable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev)
{
	if (!buffer->access->disable)
		return 0;
	return buffer->access->disable(buffer, indio_dev);
}

static void iio_buffer_update_bytes_per_datum(struct iio_dev *indio_dev,
	struct iio_buffer *buffer)
{
	unsigned int bytes;

	if (!buffer->access->set_bytes_per_datum)
		return;

	bytes = iio_compute_scan_bytes(indio_dev, buffer->scan_mask,
		buffer->scan_timestamp);

	buffer->access->set_bytes_per_datum(buffer, bytes);
}

static int iio_buffer_request_update(struct iio_dev *indio_dev,
	struct iio_buffer *buffer)
{
	int ret;

	iio_buffer_update_bytes_per_datum(indio_dev, buffer);
	if (buffer->access->request_update) {
		ret = buffer->access->request_update(buffer);
		if (ret) {
			dev_dbg(&indio_dev->dev,
			       "Buffer not started: buffer parameter update failed (%d)\n",
				ret);
			return ret;
		}
	}

	return 0;
}

static void iio_free_scan_mask(struct iio_dev *indio_dev,
	const unsigned long *mask)
{
	/* If the mask is dynamically allocated free it, otherwise do nothing */
	if (!indio_dev->available_scan_masks)
		kfree(mask);
}

struct iio_device_config {
	unsigned int mode;
	unsigned int watermark;
	const unsigned long *scan_mask;
	unsigned int scan_bytes;
	bool scan_timestamp;
};

static int iio_verify_update(struct iio_dev *indio_dev,
	struct iio_buffer *insert_buffer, struct iio_buffer *remove_buffer,
	struct iio_device_config *config)
{
	unsigned long *compound_mask;
	const unsigned long *scan_mask;
	bool strict_scanmask = false;
	struct iio_buffer *buffer;
	bool scan_timestamp;
	unsigned int modes;

	memset(config, 0, sizeof(*config));
	config->watermark = ~0;

	/*
	 * If there is just one buffer and we are removing it there is nothing
	 * to verify.
	 */
	if (remove_buffer && !insert_buffer &&
		list_is_singular(&indio_dev->buffer_list))
			return 0;

	modes = indio_dev->modes;

	list_for_each_entry(buffer, &indio_dev->buffer_list, buffer_list) {
		if (buffer == remove_buffer)
			continue;
		modes &= buffer->access->modes;
		config->watermark = min(config->watermark, buffer->watermark);
	}

	if (insert_buffer) {
		modes &= insert_buffer->access->modes;
		config->watermark = min(config->watermark,
			insert_buffer->watermark);
	}

	/* Definitely possible for devices to support both of these. */
	if ((modes & INDIO_BUFFER_TRIGGERED) && indio_dev->trig) {
		config->mode = INDIO_BUFFER_TRIGGERED;
	} else if (modes & INDIO_BUFFER_HARDWARE) {
		/*
		 * Keep things simple for now and only allow a single buffer to
		 * be connected in hardware mode.
		 */
		if (insert_buffer && !list_empty(&indio_dev->buffer_list))
			return -EINVAL;
		config->mode = INDIO_BUFFER_HARDWARE;
		strict_scanmask = true;
	} else if (modes & INDIO_BUFFER_SOFTWARE) {
		config->mode = INDIO_BUFFER_SOFTWARE;
	} else {
		/* Can only occur on first buffer */
		if (indio_dev->modes & INDIO_BUFFER_TRIGGERED)
			dev_dbg(&indio_dev->dev, "Buffer not started: no trigger\n");
		return -EINVAL;
	}

	/* What scan mask do we actually have? */
	compound_mask = kcalloc(BITS_TO_LONGS(indio_dev->masklength),
				sizeof(long), GFP_KERNEL);
	if (compound_mask == NULL)
		return -ENOMEM;

	scan_timestamp = false;

	list_for_each_entry(buffer, &indio_dev->buffer_list, buffer_list) {
		if (buffer == remove_buffer)
			continue;
		bitmap_or(compound_mask, compound_mask, buffer->scan_mask,
			  indio_dev->masklength);
		scan_timestamp |= buffer->scan_timestamp;
	}

	if (insert_buffer) {
		bitmap_or(compound_mask, compound_mask,
			  insert_buffer->scan_mask, indio_dev->masklength);
		scan_timestamp |= insert_buffer->scan_timestamp;
	}

	if (indio_dev->available_scan_masks) {
		scan_mask = iio_scan_mask_match(indio_dev->available_scan_masks,
				    indio_dev->masklength,
				    compound_mask,
				    strict_scanmask);
		kfree(compound_mask);
		if (scan_mask == NULL)
			return -EINVAL;
	} else {
	    scan_mask = compound_mask;
	}

	config->scan_bytes = iio_compute_scan_bytes(indio_dev,
				    scan_mask, scan_timestamp);
	config->scan_mask = scan_mask;
	config->scan_timestamp = scan_timestamp;

	return 0;
}

/**
 * struct iio_demux_table - table describing demux memcpy ops
 * @from:	index to copy from
 * @to:		index to copy to
 * @length:	how many bytes to copy
 * @l:		list head used for management
 */
struct iio_demux_table {
	unsigned from;
	unsigned to;
	unsigned length;
	struct list_head l;
};

static void iio_buffer_demux_free(struct iio_buffer *buffer)
{
	struct iio_demux_table *p, *q;
	list_for_each_entry_safe(p, q, &buffer->demux_list, l) {
		list_del(&p->l);
		kfree(p);
	}
}

static int iio_buffer_add_demux(struct iio_buffer *buffer,
	struct iio_demux_table **p, unsigned int in_loc, unsigned int out_loc,
	unsigned int length)
{

	if (*p && (*p)->from + (*p)->length == in_loc &&
		(*p)->to + (*p)->length == out_loc) {
		(*p)->length += length;
	} else {
		*p = kmalloc(sizeof(**p), GFP_KERNEL);
		if (*p == NULL)
			return -ENOMEM;
		(*p)->from = in_loc;
		(*p)->to = out_loc;
		(*p)->length = length;
		list_add_tail(&(*p)->l, &buffer->demux_list);
	}

	return 0;
}

static int iio_buffer_update_demux(struct iio_dev *indio_dev,
				   struct iio_buffer *buffer)
{
	int ret, in_ind = -1, out_ind, length;
	unsigned in_loc = 0, out_loc = 0;
	struct iio_demux_table *p = NULL;

	/* Clear out any old demux */
	iio_buffer_demux_free(buffer);
	kfree(buffer->demux_bounce);
	buffer->demux_bounce = NULL;

	/* First work out which scan mode we will actually have */
	if (bitmap_equal(indio_dev->active_scan_mask,
			 buffer->scan_mask,
			 indio_dev->masklength))
		return 0;

	/* Now we have the two masks, work from least sig and build up sizes */
	for_each_set_bit(out_ind,
			 buffer->scan_mask,
			 indio_dev->masklength) {
		in_ind = find_next_bit(indio_dev->active_scan_mask,
				       indio_dev->masklength,
				       in_ind + 1);
		while (in_ind != out_ind) {
			length = iio_storage_bytes_for_si(indio_dev, in_ind);
			/* Make sure we are aligned */
			in_loc = roundup(in_loc, length) + length;
			in_ind = find_next_bit(indio_dev->active_scan_mask,
					       indio_dev->masklength,
					       in_ind + 1);
		}
		length = iio_storage_bytes_for_si(indio_dev, in_ind);
		out_loc = roundup(out_loc, length);
		in_loc = roundup(in_loc, length);
		ret = iio_buffer_add_demux(buffer, &p, in_loc, out_loc, length);
		if (ret)
			goto error_clear_mux_table;
		out_loc += length;
		in_loc += length;
	}
	/* Relies on scan_timestamp being last */
	if (buffer->scan_timestamp) {
		length = iio_storage_bytes_for_timestamp(indio_dev);
		out_loc = roundup(out_loc, length);
		in_loc = roundup(in_loc, length);
		ret = iio_buffer_add_demux(buffer, &p, in_loc, out_loc, length);
		if (ret)
			goto error_clear_mux_table;
		out_loc += length;
		in_loc += length;
	}
	buffer->demux_bounce = kzalloc(out_loc, GFP_KERNEL);
	if (buffer->demux_bounce == NULL) {
		ret = -ENOMEM;
		goto error_clear_mux_table;
	}
	return 0;

error_clear_mux_table:
	iio_buffer_demux_free(buffer);

	return ret;
}

static int iio_update_demux(struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;
	int ret;

	list_for_each_entry(buffer, &indio_dev->buffer_list, buffer_list) {
		ret = iio_buffer_update_demux(indio_dev, buffer);
		if (ret < 0)
			goto error_clear_mux_table;
	}
	return 0;

error_clear_mux_table:
	list_for_each_entry(buffer, &indio_dev->buffer_list, buffer_list)
		iio_buffer_demux_free(buffer);

	return ret;
}

static int iio_enable_buffers(struct iio_dev *indio_dev,
	struct iio_device_config *config)
{
	struct iio_buffer *buffer;
	int ret;

	indio_dev->active_scan_mask = config->scan_mask;
	indio_dev->scan_timestamp = config->scan_timestamp;
	indio_dev->scan_bytes = config->scan_bytes;

	iio_update_demux(indio_dev);

	/* Wind up again */
	if (indio_dev->setup_ops->preenable) {
		ret = indio_dev->setup_ops->preenable(indio_dev);
		if (ret) {
			dev_dbg(&indio_dev->dev,
			       "Buffer not started: buffer preenable failed (%d)\n", ret);
			goto err_undo_config;
		}
	}

	if (indio_dev->info->update_scan_mode) {
		ret = indio_dev->info
			->update_scan_mode(indio_dev,
					   indio_dev->active_scan_mask);
		if (ret < 0) {
			dev_dbg(&indio_dev->dev,
				"Buffer not started: update scan mode failed (%d)\n",
				ret);
			goto err_run_postdisable;
		}
	}

	if (indio_dev->info->hwfifo_set_watermark)
		indio_dev->info->hwfifo_set_watermark(indio_dev,
			config->watermark);

	list_for_each_entry(buffer, &indio_dev->buffer_list, buffer_list) {
		ret = iio_buffer_enable(buffer, indio_dev);
		if (ret)
			goto err_disable_buffers;
	}

	indio_dev->currentmode = config->mode;

	if (indio_dev->setup_ops->postenable) {
		ret = indio_dev->setup_ops->postenable(indio_dev);
		if (ret) {
			dev_dbg(&indio_dev->dev,
			       "Buffer not started: postenable failed (%d)\n", ret);
			goto err_disable_buffers;
		}
	}

	return 0;

err_disable_buffers:
	list_for_each_entry_continue_reverse(buffer, &indio_dev->buffer_list,
					     buffer_list)
		iio_buffer_disable(buffer, indio_dev);
err_run_postdisable:
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	if (indio_dev->setup_ops->postdisable)
		indio_dev->setup_ops->postdisable(indio_dev);
err_undo_config:
	indio_dev->active_scan_mask = NULL;

	return ret;
}

static int iio_disable_buffers(struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;
	int ret = 0;
	int ret2;

	/* Wind down existing buffers - iff there are any */
	if (list_empty(&indio_dev->buffer_list))
		return 0;

	/*
	 * If things go wrong at some step in disable we still need to continue
	 * to perform the other steps, otherwise we leave the device in a
	 * inconsistent state. We return the error code for the first error we
	 * encountered.
	 */

	if (indio_dev->setup_ops->predisable) {
		ret2 = indio_dev->setup_ops->predisable(indio_dev);
		if (ret2 && !ret)
			ret = ret2;
	}

	list_for_each_entry(buffer, &indio_dev->buffer_list, buffer_list) {
		ret2 = iio_buffer_disable(buffer, indio_dev);
		if (ret2 && !ret)
			ret = ret2;
	}

	indio_dev->currentmode = INDIO_DIRECT_MODE;

	if (indio_dev->setup_ops->postdisable) {
		ret2 = indio_dev->setup_ops->postdisable(indio_dev);
		if (ret2 && !ret)
			ret = ret2;
	}

	iio_free_scan_mask(indio_dev, indio_dev->active_scan_mask);
	indio_dev->active_scan_mask = NULL;

	return ret;
}

static int __iio_update_buffers(struct iio_dev *indio_dev,
		       struct iio_buffer *insert_buffer,
		       struct iio_buffer *remove_buffer)
{
	struct iio_device_config new_config;
	int ret;

	ret = iio_verify_update(indio_dev, insert_buffer, remove_buffer,
		&new_config);
	if (ret)
		return ret;

	if (insert_buffer) {
		ret = iio_buffer_request_update(indio_dev, insert_buffer);
		if (ret)
			goto err_free_config;
	}

	ret = iio_disable_buffers(indio_dev);
	if (ret)
		goto err_deactivate_all;

	if (remove_buffer)
		iio_buffer_deactivate(remove_buffer);
	if (insert_buffer)
		iio_buffer_activate(indio_dev, insert_buffer);

	/* If no buffers in list, we are done */
	if (list_empty(&indio_dev->buffer_list))
		return 0;

	ret = iio_enable_buffers(indio_dev, &new_config);
	if (ret)
		goto err_deactivate_all;

	return 0;

err_deactivate_all:
	/*
	 * We've already verified that the config is valid earlier. If things go
	 * wrong in either enable or disable the most likely reason is an IO
	 * error from the device. In this case there is no good recovery
	 * strategy. Just make sure to disable everything and leave the device
	 * in a sane state.  With a bit of luck the device might come back to
	 * life again later and userspace can try again.
	 */
	iio_buffer_deactivate_all(indio_dev);

err_free_config:
	iio_free_scan_mask(indio_dev, new_config.scan_mask);
	return ret;
}

void iio_disable_all_buffers(struct iio_dev *indio_dev)
{
	iio_disable_buffers(indio_dev);
	iio_buffer_deactivate_all(indio_dev);
}

static ssize_t iio_buffer_store_enable(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t len)
{
	int ret;
	bool requested_state;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	bool inlist;

	ret = strtobool(buf, &requested_state);
	if (ret < 0)
		return ret;

	mutex_lock(&indio_dev->mlock);

	/* Find out if it is in the list */
	inlist = iio_buffer_is_active(indio_dev->buffer);
	/* Already in desired state */
	if (inlist == requested_state)
		goto done;

	if (requested_state)
		ret = __iio_update_buffers(indio_dev,
					 indio_dev->buffer, NULL);
	else
		ret = __iio_update_buffers(indio_dev,
					 NULL, indio_dev->buffer);

done:
	mutex_unlock(&indio_dev->mlock);
	return (ret < 0) ? ret : len;
}

static const char * const iio_scan_elements_group_name = "scan_elements";

static ssize_t iio_buffer_show_watermark(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_buffer *buffer = indio_dev->buffer;

	return sprintf(buf, "%u\n", buffer->watermark);
}

static ssize_t iio_buffer_store_watermark(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_buffer *buffer = indio_dev->buffer;
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;
	if (!val)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	if (val > buffer->length) {
		ret = -EINVAL;
		goto out;
	}

	if (iio_buffer_is_active(indio_dev->buffer)) {
		ret = -EBUSY;
		goto out;
	}

	buffer->watermark = val;
out:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static DEVICE_ATTR(length, S_IRUGO | S_IWUSR, iio_buffer_read_length,
		   iio_buffer_write_length);
static struct device_attribute dev_attr_length_ro = __ATTR(length,
	S_IRUGO, iio_buffer_read_length, NULL);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		   iio_buffer_show_enable, iio_buffer_store_enable);
static DEVICE_ATTR(watermark, S_IRUGO | S_IWUSR,
		   iio_buffer_show_watermark, iio_buffer_store_watermark);
static struct device_attribute dev_attr_watermark_ro = __ATTR(watermark,
	S_IRUGO, iio_buffer_show_watermark, NULL);

static struct attribute *iio_buffer_attrs[] = {
	&dev_attr_length.attr,
	&dev_attr_enable.attr,
	&dev_attr_watermark.attr,
};

int iio_buffer_alloc_sysfs_and_mask(struct iio_dev *indio_dev)
{
	struct iio_dev_attr *p;
	struct attribute **attr;
	struct iio_buffer *buffer = indio_dev->buffer;
	int ret, i, attrn, attrcount, attrcount_orig = 0;
	const struct iio_chan_spec *channels;

	channels = indio_dev->channels;
	if (channels) {
		int ml = indio_dev->masklength;

		for (i = 0; i < indio_dev->num_channels; i++)
			ml = max(ml, channels[i].scan_index + 1);
		indio_dev->masklength = ml;
	}

	if (!buffer)
		return 0;

	attrcount = 0;
	if (buffer->attrs) {
		while (buffer->attrs[attrcount] != NULL)
			attrcount++;
	}

	attr = kcalloc(attrcount + ARRAY_SIZE(iio_buffer_attrs) + 1,
		       sizeof(struct attribute *), GFP_KERNEL);
	if (!attr)
		return -ENOMEM;

	memcpy(attr, iio_buffer_attrs, sizeof(iio_buffer_attrs));
	if (!buffer->access->set_length)
		attr[0] = &dev_attr_length_ro.attr;

	if (buffer->access->flags & INDIO_BUFFER_FLAG_FIXED_WATERMARK)
		attr[2] = &dev_attr_watermark_ro.attr;

	if (buffer->attrs)
		memcpy(&attr[ARRAY_SIZE(iio_buffer_attrs)], buffer->attrs,
		       sizeof(struct attribute *) * attrcount);

	attr[attrcount + ARRAY_SIZE(iio_buffer_attrs)] = NULL;

	buffer->buffer_group.name = "buffer";
	buffer->buffer_group.attrs = attr;

	indio_dev->groups[indio_dev->groupcounter++] = &buffer->buffer_group;

	if (buffer->scan_el_attrs != NULL) {
		attr = buffer->scan_el_attrs->attrs;
		while (*attr++ != NULL)
			attrcount_orig++;
	}
	attrcount = attrcount_orig;
	INIT_LIST_HEAD(&buffer->scan_el_dev_attr_list);
	channels = indio_dev->channels;
	if (channels) {
		/* new magic */
		for (i = 0; i < indio_dev->num_channels; i++) {
			if (channels[i].scan_index < 0)
				continue;

			ret = iio_buffer_add_channel_sysfs(indio_dev,
							 &channels[i]);
			if (ret < 0)
				goto error_cleanup_dynamic;
			attrcount += ret;
			if (channels[i].type == IIO_TIMESTAMP)
				indio_dev->scan_index_timestamp =
					channels[i].scan_index;
		}
		if (indio_dev->masklength && buffer->scan_mask == NULL) {
			buffer->scan_mask = kcalloc(BITS_TO_LONGS(indio_dev->masklength),
						    sizeof(*buffer->scan_mask),
						    GFP_KERNEL);
			if (buffer->scan_mask == NULL) {
				ret = -ENOMEM;
				goto error_cleanup_dynamic;
			}
		}
	}

	buffer->scan_el_group.name = iio_scan_elements_group_name;

	buffer->scan_el_group.attrs = kcalloc(attrcount + 1,
					      sizeof(buffer->scan_el_group.attrs[0]),
					      GFP_KERNEL);
	if (buffer->scan_el_group.attrs == NULL) {
		ret = -ENOMEM;
		goto error_free_scan_mask;
	}
	if (buffer->scan_el_attrs)
		memcpy(buffer->scan_el_group.attrs, buffer->scan_el_attrs,
		       sizeof(buffer->scan_el_group.attrs[0])*attrcount_orig);
	attrn = attrcount_orig;

	list_for_each_entry(p, &buffer->scan_el_dev_attr_list, l)
		buffer->scan_el_group.attrs[attrn++] = &p->dev_attr.attr;
	indio_dev->groups[indio_dev->groupcounter++] = &buffer->scan_el_group;

	return 0;

error_free_scan_mask:
	kfree(buffer->scan_mask);
error_cleanup_dynamic:
	iio_free_chan_devattr_list(&buffer->scan_el_dev_attr_list);
	kfree(indio_dev->buffer->buffer_group.attrs);

	return ret;
}

void iio_buffer_free_sysfs_and_mask(struct iio_dev *indio_dev)
{
	if (!indio_dev->buffer)
		return;

	kfree(indio_dev->buffer->scan_mask);
	kfree(indio_dev->buffer->buffer_group.attrs);
	kfree(indio_dev->buffer->scan_el_group.attrs);
	iio_free_chan_devattr_list(&indio_dev->buffer->scan_el_dev_attr_list);
}

static const void *iio_demux(struct iio_buffer *buffer,
				 const void *datain)
{
	struct iio_demux_table *t;

	if (list_empty(&buffer->demux_list))
		return datain;
	list_for_each_entry(t, &buffer->demux_list, l)
		memcpy(buffer->demux_bounce + t->to,
		       datain + t->from, t->length);

	return buffer->demux_bounce;
}

static int iio_push_to_buffer(struct iio_buffer *buffer, const void *data)
{
	const void *dataout = iio_demux(buffer, data);
	int ret;

	ret = buffer->access->store_to(buffer, dataout);
	if (ret)
		return ret;

	/*
	 * We can't just test for watermark to decide if we wake the poll queue
	 * because read may request less samples than the watermark.
	 */
	wake_up_interruptible_poll(&buffer->pollq, POLLIN | POLLRDNORM);
	return 0;
}

/**
 * iio_push_to_buffers() - push to a registered buffer.
 * @indio_dev:		iio_dev structure for device.
 * @data:		Full scan.
 */
int iio_push_to_buffers(struct iio_dev *indio_dev, const void *data)
{
	int ret;
	struct iio_buffer *buf;

	list_for_each_entry(buf, &indio_dev->buffer_list, buffer_list) {
		ret = iio_push_to_buffer(buf, data);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(iio_push_to_buffers);

/**
 * iio_buffer_release() - Free a buffer's resources
 * @ref: Pointer to the kref embedded in the iio_buffer struct
 *
 * This function is called when the last reference to the buffer has been
 * dropped. It will typically free all resources allocated by the buffer. Do not
 * call this function manually, always use iio_buffer_put() when done using a
 * buffer.
 */
static void iio_buffer_release(struct kref *ref)
{
	struct iio_buffer *buffer = container_of(ref, struct iio_buffer, ref);

	buffer->access->release(buffer);
}

/**
 * iio_buffer_get() - Grab a reference to the buffer
 * @buffer: The buffer to grab a reference for, may be NULL
 *
 * Returns the pointer to the buffer that was passed into the function.
 */
struct iio_buffer *iio_buffer_get(struct iio_buffer *buffer)
{
	if (buffer)
		kref_get(&buffer->ref);

	return buffer;
}
EXPORT_SYMBOL_GPL(iio_buffer_get);

/**
 * iio_buffer_put() - Release the reference to the buffer
 * @buffer: The buffer to release the reference for, may be NULL
 */
void iio_buffer_put(struct iio_buffer *buffer)
{
	if (buffer)
		kref_put(&buffer->ref, iio_buffer_release);
}
EXPORT_SYMBOL_GPL(iio_buffer_put);

/**
 * iio_device_attach_buffer - Attach a buffer to a IIO device
 * @indio_dev: The device the buffer should be attached to
 * @buffer: The buffer to attach to the device
 *
 * This function attaches a buffer to a IIO device. The buffer stays attached to
 * the device until the device is freed. The function should only be called at
 * most once per device.
 */
void iio_device_attach_buffer(struct iio_dev *indio_dev,
			      struct iio_buffer *buffer)
{
	indio_dev->buffer = iio_buffer_get(buffer);
}
EXPORT_SYMBOL_GPL(iio_device_attach_buffer);

/* The industrial I/O core, trigger handling functions
 *
 * Copyright (c) 2008 Jonathan Cameron
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/idr.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/slab.h>

#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include "iio_core.h"
#include "iio_core_trigger.h"
#include <linux/iio/trigger_consumer.h>

/* RFC - Question of approach
 * Make the common case (single sensor single trigger)
 * simple by starting trigger capture from when first sensors
 * is added.
 *
 * Complex simultaneous start requires use of 'hold' functionality
 * of the trigger. (not implemented)
 *
 * Any other suggestions?
 */

static DEFINE_IDA(iio_trigger_ida);

/* Single list of all available triggers */
static LIST_HEAD(iio_trigger_list);
static DEFINE_MUTEX(iio_trigger_list_lock);

/**
 * iio_trigger_read_name() - retrieve useful identifying name
 * @dev:	device associated with the iio_trigger
 * @attr:	pointer to the device_attribute structure that is
 *		being processed
 * @buf:	buffer to print the name into
 *
 * Return: a negative number on failure or the number of written
 *	   characters on success.
 */
static ssize_t iio_trigger_read_name(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	return sprintf(buf, "%s\n", trig->name);
}

static DEVICE_ATTR(name, S_IRUGO, iio_trigger_read_name, NULL);

static struct attribute *iio_trig_dev_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};
ATTRIBUTE_GROUPS(iio_trig_dev);

static struct iio_trigger *__iio_trigger_find_by_name(const char *name);

int __iio_trigger_register(struct iio_trigger *trig_info,
               struct module *this_mod)
{
    int ret;

    trig_info->owner = this_mod;

    trig_info->id = ida_simple_get(&iio_trigger_ida, 0, 0, GFP_KERNEL);
    if (trig_info->id < 0)
        return trig_info->id;

    /* Set the name used for the sysfs directory etc */
    dev_set_name(&trig_info->dev, "trigger%ld",
             (unsigned long) trig_info->id);

    ret = device_add(&trig_info->dev);
    if (ret)
        goto error_unregister_id;

    /* Add to list of available triggers held by the IIO core */
    mutex_lock(&iio_trigger_list_lock);
    if (__iio_trigger_find_by_name(trig_info->name)) {
        pr_err("Duplicate trigger name '%s'\n", trig_info->name);
        ret = -EEXIST;
        goto error_device_del;
    }
    list_add_tail(&trig_info->list, &iio_trigger_list);
    mutex_unlock(&iio_trigger_list_lock);

    return 0;

error_device_del:
    mutex_unlock(&iio_trigger_list_lock);
    device_del(&trig_info->dev);
error_unregister_id:
    ida_simple_remove(&iio_trigger_ida, trig_info->id);
    return ret;
}
EXPORT_SYMBOL(__iio_trigger_register);

void iio_trigger_unregister(struct iio_trigger *trig_info)
{
	mutex_lock(&iio_trigger_list_lock);
	list_del(&trig_info->list);
	mutex_unlock(&iio_trigger_list_lock);

	ida_simple_remove(&iio_trigger_ida, trig_info->id);
	/* Possible issue in here */
	device_del(&trig_info->dev);
}
EXPORT_SYMBOL(iio_trigger_unregister);

/* Search for trigger by name, assuming iio_trigger_list_lock held */
static struct iio_trigger *__iio_trigger_find_by_name(const char *name)
{
	struct iio_trigger *iter;

	list_for_each_entry(iter, &iio_trigger_list, list)
		if (!strcmp(iter->name, name))
			return iter;

	return NULL;
}

static struct iio_trigger *iio_trigger_acquire_by_name(const char *name)
{
	struct iio_trigger *trig = NULL, *iter;

	mutex_lock(&iio_trigger_list_lock);
	list_for_each_entry(iter, &iio_trigger_list, list)
		if (sysfs_streq(iter->name, name)) {
			trig = iter;
			iio_trigger_get(trig);
			break;
		}
	mutex_unlock(&iio_trigger_list_lock);

	return trig;
}

void iio_trigger_poll(struct iio_trigger *trig)
{
	int i;

	if (!atomic_read(&trig->use_count)) {
		atomic_set(&trig->use_count, CONFIG_IIO_CONSUMERS_PER_TRIGGER);

		for (i = 0; i < CONFIG_IIO_CONSUMERS_PER_TRIGGER; i++) {
			if (trig->subirqs[i].enabled)
				generic_handle_irq(trig->subirq_base + i);
			else
				iio_trigger_notify_done(trig);
		}
	}
}
EXPORT_SYMBOL(iio_trigger_poll);

void iio_trigger_notify_done(struct iio_trigger *trig)
{
	if (atomic_dec_and_test(&trig->use_count) && trig->ops &&
	    trig->ops->try_reenable)
		if (trig->ops->try_reenable(trig))
			/* Missed an interrupt so launch new poll now */
			iio_trigger_poll(trig);
}
EXPORT_SYMBOL(iio_trigger_notify_done);

/* Trigger Consumer related functions */
static int iio_trigger_get_irq(struct iio_trigger *trig)
{
	int ret;
	mutex_lock(&trig->pool_lock);
	ret = bitmap_find_free_region(trig->pool,
				      CONFIG_IIO_CONSUMERS_PER_TRIGGER,
				      ilog2(1));
	mutex_unlock(&trig->pool_lock);
	if (ret >= 0)
		ret += trig->subirq_base;

	return ret;
}

static void iio_trigger_put_irq(struct iio_trigger *trig, int irq)
{
	mutex_lock(&trig->pool_lock);
	clear_bit(irq - trig->subirq_base, trig->pool);
	mutex_unlock(&trig->pool_lock);
}

/* Complexity in here.  With certain triggers (datardy) an acknowledgement
 * may be needed if the pollfuncs do not include the data read for the
 * triggering device.
 * This is not currently handled.  Alternative of not enabling trigger unless
 * the relevant function is in there may be the best option.
 */
/* Worth protecting against double additions? */
static int iio_trigger_attach_poll_func(struct iio_trigger *trig,
					struct iio_poll_func *pf)
{
	int ret = 0;
	bool notinuse
		= bitmap_empty(trig->pool, CONFIG_IIO_CONSUMERS_PER_TRIGGER);

	/* Prevent the module from being removed whilst attached to a trigger */
	__module_get(pf->indio_dev->driver_module);

	/* Get irq number */
	pf->irq = iio_trigger_get_irq(trig);
	if (pf->irq < 0)
		goto out_put_module;

	/* Request irq */
	ret = request_threaded_irq(pf->irq, pf->h, pf->thread,
				   pf->type, pf->name,
				   pf);
	if (ret < 0)
		goto out_put_irq;

	/* Enable trigger in driver */
	if (trig->ops && trig->ops->set_trigger_state && notinuse) {
		ret = trig->ops->set_trigger_state(trig, true);
		if (ret < 0)
			goto out_free_irq;
	}

	/*
	 * Check if we just registered to our own trigger: we determine that
	 * this is the case if the IIO device and the trigger device share the
	 * same parent device.
	 */
	if (pf->indio_dev->dev.parent == trig->dev.parent)
		trig->attached_own_device = true;

	return ret;

out_free_irq:
	free_irq(pf->irq, pf);
out_put_irq:
	iio_trigger_put_irq(trig, pf->irq);
out_put_module:
	module_put(pf->indio_dev->driver_module);
	return ret;
}

static int iio_trigger_detach_poll_func(struct iio_trigger *trig,
					 struct iio_poll_func *pf)
{
	int ret = 0;
	bool no_other_users
		= (bitmap_weight(trig->pool,
				 CONFIG_IIO_CONSUMERS_PER_TRIGGER)
		   == 1);
	if (trig->ops && trig->ops->set_trigger_state && no_other_users) {
		ret = trig->ops->set_trigger_state(trig, false);
		if (ret)
			return ret;
	}
	if (pf->indio_dev->dev.parent == trig->dev.parent)
		trig->attached_own_device = false;
	iio_trigger_put_irq(trig, pf->irq);
	free_irq(pf->irq, pf);
	module_put(pf->indio_dev->driver_module);

	return ret;
}

irqreturn_t iio_pollfunc_store_time(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	pf->timestamp = iio_get_time_ns(pf->indio_dev);
	return IRQ_WAKE_THREAD;
}
EXPORT_SYMBOL(iio_pollfunc_store_time);

struct iio_poll_func
*iio_alloc_pollfunc(irqreturn_t (*h)(int irq, void *p),
		    irqreturn_t (*thread)(int irq, void *p),
		    int type,
		    struct iio_dev *indio_dev,
		    const char *fmt,
		    ...)
{
	va_list vargs;
	struct iio_poll_func *pf;

	pf = kmalloc(sizeof *pf, GFP_KERNEL);
	if (pf == NULL)
		return NULL;
	va_start(vargs, fmt);
	pf->name = kvasprintf(GFP_KERNEL, fmt, vargs);
	va_end(vargs);
	if (pf->name == NULL) {
		kfree(pf);
		return NULL;
	}
	pf->h = h;
	pf->thread = thread;
	pf->type = type;
	pf->indio_dev = indio_dev;

	return pf;
}
EXPORT_SYMBOL_GPL(iio_alloc_pollfunc);

void iio_dealloc_pollfunc(struct iio_poll_func *pf)
{
	kfree(pf->name);
	kfree(pf);
}
EXPORT_SYMBOL_GPL(iio_dealloc_pollfunc);

/**
 * iio_trigger_read_current() - trigger consumer sysfs query current trigger
 * @dev:	device associated with an industrial I/O device
 * @attr:	pointer to the device_attribute structure that
 *		is being processed
 * @buf:	buffer where the current trigger name will be printed into
 *
 * For trigger consumers the current_trigger interface allows the trigger
 * used by the device to be queried.
 *
 * Return: a negative number on failure, the number of characters written
 *	   on success or 0 if no trigger is available
 */
static ssize_t iio_trigger_read_current(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	if (indio_dev->trig)
		return sprintf(buf, "%s\n", indio_dev->trig->name);
	return 0;
}

/**
 * iio_trigger_write_current() - trigger consumer sysfs set current trigger
 * @dev:	device associated with an industrial I/O device
 * @attr:	device attribute that is being processed
 * @buf:	string buffer that holds the name of the trigger
 * @len:	length of the trigger name held by buf
 *
 * For trigger consumers the current_trigger interface allows the trigger
 * used for this device to be specified at run time based on the trigger's
 * name.
 *
 * Return: negative error code on failure or length of the buffer
 *	   on success
 */
static ssize_t iio_trigger_write_current(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_trigger *oldtrig = indio_dev->trig;
	struct iio_trigger *trig;
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
		mutex_unlock(&indio_dev->mlock);
		return -EBUSY;
	}
	if (indio_dev->trig_readonly) {
		mutex_unlock(&indio_dev->mlock);
		return -EPERM;
	}
	mutex_unlock(&indio_dev->mlock);

	trig = iio_trigger_acquire_by_name(buf);
	if (oldtrig == trig) {
		ret = len;
		goto out_trigger_put;
	}

	if (trig && indio_dev->info->validate_trigger) {
		ret = indio_dev->info->validate_trigger(indio_dev, trig);
		if (ret)
			goto out_trigger_put;
	}

	if (trig && trig->ops && trig->ops->validate_device) {
		ret = trig->ops->validate_device(trig, indio_dev);
		if (ret)
			goto out_trigger_put;
	}

	indio_dev->trig = trig;

	if (oldtrig) {
		if (indio_dev->modes & INDIO_EVENT_TRIGGERED)
			iio_trigger_detach_poll_func(oldtrig,
						     indio_dev->pollfunc_event);
		iio_trigger_put(oldtrig);
	}
	if (indio_dev->trig) {
		if (indio_dev->modes & INDIO_EVENT_TRIGGERED)
			iio_trigger_attach_poll_func(indio_dev->trig,
						     indio_dev->pollfunc_event);
	}

	return len;

out_trigger_put:
	if (trig)
		iio_trigger_put(trig);
	return ret;
}

static DEVICE_ATTR(current_trigger, S_IRUGO | S_IWUSR,
		   iio_trigger_read_current,
		   iio_trigger_write_current);

static struct attribute *iio_trigger_consumer_attrs[] = {
	&dev_attr_current_trigger.attr,
	NULL,
};

static const struct attribute_group iio_trigger_consumer_attr_group = {
	.name = "trigger",
	.attrs = iio_trigger_consumer_attrs,
};

static void iio_trig_release(struct device *device)
{
	struct iio_trigger *trig = to_iio_trigger(device);
	int i;

	if (trig->subirq_base) {
		for (i = 0; i < CONFIG_IIO_CONSUMERS_PER_TRIGGER; i++) {
			irq_modify_status(trig->subirq_base + i,
					  IRQ_NOAUTOEN,
					  IRQ_NOREQUEST | IRQ_NOPROBE);
			irq_set_chip(trig->subirq_base + i,
				     NULL);
			irq_set_handler(trig->subirq_base + i,
					NULL);
		}

		irq_free_descs(trig->subirq_base,
			       CONFIG_IIO_CONSUMERS_PER_TRIGGER);
	}
	kfree(trig->name);
	kfree(trig);
}

static const struct device_type iio_trig_type = {
	.release = iio_trig_release,
	.groups = iio_trig_dev_groups,
};

static void iio_trig_subirqmask(struct irq_data *d)
{
	struct irq_chip *chip = irq_data_get_irq_chip(d);
	struct iio_trigger *trig
		= container_of(chip,
			       struct iio_trigger, subirq_chip);
	trig->subirqs[d->irq - trig->subirq_base].enabled = false;
}

static void iio_trig_subirqunmask(struct irq_data *d)
{
	struct irq_chip *chip = irq_data_get_irq_chip(d);
	struct iio_trigger *trig
		= container_of(chip,
			       struct iio_trigger, subirq_chip);
	trig->subirqs[d->irq - trig->subirq_base].enabled = true;
}

static struct iio_trigger *viio_trigger_alloc(const char *fmt, va_list vargs)
{
	struct iio_trigger *trig;
	int i;

	trig = kzalloc(sizeof *trig, GFP_KERNEL);
	if (!trig)
		return NULL;

	trig->dev.type = &iio_trig_type;
	trig->dev.bus = &iio_bus_type;
	device_initialize(&trig->dev);

	mutex_init(&trig->pool_lock);
	trig->subirq_base = irq_alloc_descs(-1, 0,
					    CONFIG_IIO_CONSUMERS_PER_TRIGGER,
					    0);
	if (trig->subirq_base < 0)
		goto free_trig;

	trig->name = kvasprintf(GFP_KERNEL, fmt, vargs);
	if (trig->name == NULL)
		goto free_descs;

	trig->subirq_chip.name = trig->name;
	trig->subirq_chip.irq_mask = &iio_trig_subirqmask;
	trig->subirq_chip.irq_unmask = &iio_trig_subirqunmask;
	for (i = 0; i < CONFIG_IIO_CONSUMERS_PER_TRIGGER; i++) {
		irq_set_chip(trig->subirq_base + i, &trig->subirq_chip);
		irq_set_handler(trig->subirq_base + i, &handle_simple_irq);
		irq_modify_status(trig->subirq_base + i,
				  IRQ_NOREQUEST | IRQ_NOAUTOEN, IRQ_NOPROBE);
	}

	return trig;

free_descs:
	irq_free_descs(trig->subirq_base, CONFIG_IIO_CONSUMERS_PER_TRIGGER);
free_trig:
	kfree(trig);
	return NULL;
}

void iio_trigger_free(struct iio_trigger *trig)
{
	if (trig)
		put_device(&trig->dev);
}
EXPORT_SYMBOL(iio_trigger_free);

static void devm_iio_trigger_release(struct device *dev, void *res)
{
	iio_trigger_free(*(struct iio_trigger **)res);
}

static int devm_iio_trigger_match(struct device *dev, void *res, void *data)
{
	struct iio_trigger **r = res;

	if (!r || !*r) {
		WARN_ON(!r || !*r);
		return 0;
	}

	return *r == data;
}

/**
 * devm_iio_trigger_alloc - Resource-managed iio_trigger_alloc()
 * @dev:		Device to allocate iio_trigger for
 * @fmt:		trigger name format. If it includes format
 *			specifiers, the additional arguments following
 *			format are formatted and inserted in the resulting
 *			string replacing their respective specifiers.
 *
 * Managed iio_trigger_alloc.  iio_trigger allocated with this function is
 * automatically freed on driver detach.
 *
 * If an iio_trigger allocated with this function needs to be freed separately,
 * devm_iio_trigger_free() must be used.
 *
 * RETURNS:
 * Pointer to allocated iio_trigger on success, NULL on failure.
 */
struct iio_trigger *devm_iio_trigger_alloc(struct device *dev,
						const char *fmt, ...)
{
	struct iio_trigger **ptr, *trig;
	va_list vargs;

	ptr = devres_alloc(devm_iio_trigger_release, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr)
		return NULL;

	/* use raw alloc_dr for kmalloc caller tracing */
	va_start(vargs, fmt);
	trig = viio_trigger_alloc(fmt, vargs);
	va_end(vargs);
	if (trig) {
		*ptr = trig;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return trig;
}
EXPORT_SYMBOL_GPL(devm_iio_trigger_alloc);

static void devm_iio_trigger_unreg(struct device *dev, void *res)
{
	iio_trigger_unregister(*(struct iio_trigger **)res);
}

void iio_device_register_trigger_consumer(struct iio_dev *indio_dev)
{
	indio_dev->groups[indio_dev->groupcounter++] =
		&iio_trigger_consumer_attr_group;
}

void iio_device_unregister_trigger_consumer(struct iio_dev *indio_dev)
{
	/* Clean up an associated but not attached trigger reference */
	if (indio_dev->trig)
		iio_trigger_put(indio_dev->trig);
}

int iio_triggered_buffer_postenable(struct iio_dev *indio_dev)
{
	return iio_trigger_attach_poll_func(indio_dev->trig,
					    indio_dev->pollfunc);
}
EXPORT_SYMBOL(iio_triggered_buffer_postenable);

int iio_triggered_buffer_predisable(struct iio_dev *indio_dev)
{
	return iio_trigger_detach_poll_func(indio_dev->trig,
					     indio_dev->pollfunc);
}
EXPORT_SYMBOL(iio_triggered_buffer_predisable);
