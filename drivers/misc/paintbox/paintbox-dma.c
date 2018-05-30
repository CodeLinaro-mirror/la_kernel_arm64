/*
 * Core driver for the Paintbox programmable IPU
 *
 * Copyright (C) 2015 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#include <uapi/paintbox.h>

#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-dma-debug.h"
#include "paintbox-dma-dram.h"
#include "paintbox-dma-lbp.h"
#include "paintbox-dma-mipi.h"
#include "paintbox-dma-stp.h"
#include "paintbox-io.h"
#include "paintbox-irq.h"
#include "paintbox-lbp.h"
#include "paintbox-mipi.h"
#include "paintbox-power.h"
#include "paintbox-regs.h"
#include "paintbox-sim-regs.h"

#define MAX_DMA_STOP_ATTEMPTS 2

#define DMA_RESET_HOLD_PERIOD 10 /* us */

static inline bool paintbox_dma_src_is_mipi(uint32_t mode)
{
	return ((mode & DMA_CHAN_MODE_SRC_MASK) >> DMA_CHAN_MODE_SRC_SHIFT) ==
			DMA_CHAN_MODE_SRC_MIPI_IN;
}

static inline bool paintbox_dma_src_is_lbp(uint32_t mode)
{
	return ((mode & DMA_CHAN_MODE_SRC_MASK) >> DMA_CHAN_MODE_SRC_SHIFT) ==
			DMA_CHAN_MODE_SRC_LBP;
}

static inline bool paintbox_dma_dst_is_mipi(uint32_t mode)
{
	return ((mode & DMA_CHAN_MODE_DST_MASK) >> DMA_CHAN_MODE_DST_SHIFT) ==
			DMA_CHAN_MODE_DST_MIPI_OUT;
}

static inline bool paintbox_dma_dst_is_lbp(uint32_t mode)
{
	return ((mode & DMA_CHAN_MODE_DST_MASK) >> DMA_CHAN_MODE_DST_SHIFT) ==
			DMA_CHAN_MODE_DST_LBP;
}

static inline bool paintbox_dma_is_gather(uint32_t mode)
{
	return mode & DMA_CHAN_MODE_GATHER_MASK;
}

static inline bool paintbox_dma_is_gather_channel(
		struct paintbox_dma_channel *channel)
{
	return channel->session->dma_gather_channel_mask &
			(1 << channel->channel_id);
}
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_enable_channel_interrupts(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	if (channel->interrupts_enabled)
		return;

	channel->interrupts_enabled = true;

	pb->dma.regs.irq_imr |= 1 << channel->channel_id;
	writel(pb->dma.regs.irq_imr, pb->dma.dma_base + DMA_IRQ_IMR);

	/* Enable both VA_ERR and MS_ERR (MIPI Overflow) error interrupts for
	 * the channel.
	 */
	pb->dma.regs.irq_err_imr |= 1 << channel->channel_id;
	pb->dma.regs.irq_err_imr |= 1ULL << (channel->channel_id +
			DMA_IRQ_ERR_IMR_MS_ERR_SHIFT);
	writeq(pb->dma.regs.irq_err_imr, pb->dma.dma_base + DMA_IRQ_ERR_IMR);

	/* Enable the DMA channel interrupt in the top-level IPU_IMR */
	paintbox_enable_dma_channel_interrupt(pb, channel->channel_id);

	/* Enable the DMA channel error interrupt in the top-level IPU_IMR */
	paintbox_enable_dma_channel_error_interrupt(pb, channel->channel_id);

	dev_dbg(&pb->pdev->dev, "dma channel%u: interrupts enabled\n",
			channel->channel_id);
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_disable_channel_interrupts(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	if (!channel->interrupts_enabled)
		return;

	channel->interrupts_enabled = false;

	pb->dma.regs.irq_imr &= ~(1 << channel->channel_id);
	writel(pb->dma.regs.irq_imr, pb->dma.dma_base + DMA_IRQ_IMR);

	/* Disable both VA_ERR and MS_ERR (MIPI Overflow) error interrupts for
	 * the channel.
	 */
	pb->dma.regs.irq_err_imr &= ~(1 << channel->channel_id);
	pb->dma.regs.irq_err_imr &= ~(1ULL << (channel->channel_id +
			DMA_IRQ_ERR_IMR_MS_ERR_SHIFT));
	writeq(pb->dma.regs.irq_err_imr, pb->dma.dma_base + DMA_IRQ_ERR_IMR);

	paintbox_disable_dma_channel_interrupt(pb, channel->channel_id);
	paintbox_disable_dma_channel_error_interrupt(pb, channel->channel_id);

	dev_dbg(&pb->pdev->dev, "dma channel%u: interrupts disabled\n",
			channel->channel_id);
}
#else
/* The caller to this function must hold pb->dma.dma_lock and DMA_CHAN_SEL must
 * be set.
 */
static void paintbox_dma_enable_channel_interrupts(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	if (channel->interrupts_enabled)
		return;

	channel->interrupts_enabled = true;

	writel(DMA_CHAN_IMR_EOF_MASK | DMA_CHAN_IMR_VA_ERR_MASK,
			pb->dma.dma_base + DMA_CHAN_IMR);

	/* Enable the DMA channel interrupt in the top-level IPU_IMR */
	paintbox_enable_dma_channel_interrupt(pb, channel->channel_id);

	dev_dbg(&pb->pdev->dev, "dma channel%u: interrupts enabled\n",
			channel->channel_id);
}

/* The caller to this function must hold pb->dma.dma_lock and DMA_CHAN_SEL must
 * be set.
 */
static void paintbox_dma_disable_channel_interrupts(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	if (!channel->interrupts_enabled)
		return;

	channel->interrupts_enabled = false;

	writel(0, pb->dma.dma_base + DMA_CHAN_IMR);
	paintbox_disable_dma_channel_interrupt(pb, channel->channel_id);

	dev_dbg(&pb->pdev->dev, "dma channel%u: interrupts disabled\n",
			channel->channel_id);
}
#endif

static inline void paintbox_dma_chan_ctrl_set(struct paintbox_data *pb,
		uint64_t mask, uint64_t value)
{
	pb->dma.regs.chan_ctrl &= ~mask;
	pb->dma.regs.chan_ctrl |= value;
	writeq(pb->dma.regs.chan_ctrl, pb->dma.dma_base + DMA_CHAN_CTRL);
}

/* The caller to this function must hold pb->lock. */
static struct paintbox_dma_transfer *paintbox_allocate_transfer(
		struct paintbox_data *pb)
{
	struct paintbox_dma_transfer *transfer;

	if (pb->dma.free_count == 0) {
		transfer = kzalloc(sizeof(struct paintbox_dma_transfer),
				GFP_KERNEL);
	} else {
		transfer = list_entry(pb->dma.free_list.next,
				struct paintbox_dma_transfer, entry);
		list_del(&transfer->entry);
		pb->dma.free_count--;
		memset(transfer, 0, sizeof(struct paintbox_dma_transfer));
	}

	return transfer;
}

/* The caller to this function must hold pb->lock. */
static void paintbox_free_transfer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &pb->dma.free_list);
	pb->dma.free_count++;
}

/* The caller to this function must hold pb->lock */
int validate_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id)
{
	if (channel_id >= pb->dma.num_channels) {
		dev_err(&pb->pdev->dev, "%s: invalid dma channel id %d\n",
				__func__, channel_id);
		return -EINVAL;
	}

	if (pb->dma.channels[channel_id].session != session) {
		dev_err(&pb->pdev->dev, "%s: access error, dma channel id %d\n",
				__func__, channel_id);
		return -EACCES;
	}

	return 0;
}

/* The caller to this function must hold pb->lock */
struct paintbox_dma_channel *get_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id,
		int *err)
{
	int ret;

	ret = validate_dma_channel(pb, session, channel_id);
	if (ret < 0) {
		*err = ret;
		return NULL;
	}

	*err = 0;
	return &pb->dma.channels[channel_id];
}

/* The caller to this function must hold pb->lock. */
static void drain_queue(struct paintbox_data *pb,
		struct list_head *transfer_list, unsigned int *count)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	while (!list_empty(transfer_list)) {
		struct paintbox_dma_transfer *transfer;

		transfer = list_entry(transfer_list->next,
				struct paintbox_dma_transfer, entry);
		list_del(&transfer->entry);
		(*count)--;
		WARN_ON(*count < 0);

		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

		ipu_dma_release_buffer(pb, transfer);

		paintbox_free_transfer(pb, transfer);

		spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);
	}

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

static void paintbox_dma_discard_queue_work(struct work_struct *work)
{
	struct paintbox_dma *dma = container_of(work, struct paintbox_dma,
			discard_queue_work);
	struct paintbox_data *pb = container_of(dma, struct paintbox_data, dma);

	mutex_lock(&pb->lock);
	drain_queue(pb, &dma->discard_list, &dma->discard_count);
	mutex_unlock(&pb->lock);
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_discard_transfer(struct paintbox_data *pb,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &pb->dma.discard_list);
	pb->dma.discard_count++;
	queue_work(system_wq, &pb->dma.discard_queue_work);
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_enqueue_pending_read(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &channel->completed_list);
	channel->completed_count++;
	channel->stats.reported_completions++;
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void paintbox_dma_enqueue_pending_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	list_add_tail(&transfer->entry, &channel->pending_list);
	channel->pending_count++;
}

static struct paintbox_dma_transfer *paintbox_dequeue_active_transfer(
		struct paintbox_data *pb, struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;

	if (list_empty(&channel->active_list) || channel->active_count == 0) {
		dev_warn(&pb->pdev->dev,
				"%s: channel%u no active transfer\n", __func__,
				channel->channel_id);
		return NULL;
	}

	transfer = list_entry(channel->active_list.next,
				struct paintbox_dma_transfer, entry);
	list_del(&transfer->entry);
	channel->active_count--;

	return transfer;
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void dma_report_completion(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer, ktime_t timestamp,
		int err)
{
	dev_dbg(&pb->pdev->dev, "dma channel%u: transfer %p completed err %d\n",
			channel->channel_id, transfer, err);

	if (!transfer->notify_on_completion) {
		paintbox_dma_discard_transfer(pb, transfer);
		channel->stats.reported_discards++;
		return;
	}

	/* Buffers that need to be copied back to user space go on a separate
	 * read queue that is emptied by read_dma_transfer_ioctl().  All other
	 * transfers go on a discard queue and are cleaned up by a work queue.
	 *
	 * TODO: Remove the read queue support when b/62371806 is
	 * fixed.
	 */
	if (transfer->buffer_type == DMA_DRAM_BUFFER_USER &&
			transfer->dir == DMA_FROM_DEVICE) {
		paintbox_dma_enqueue_pending_read(pb, channel, transfer);
	} else {
		paintbox_dma_discard_transfer(pb, transfer);
		channel->stats.reported_discards++;
	}

	paintbox_irq_waiter_signal(pb, channel->irq, timestamp, 0 /* data */,
			err);
}

int paintbox_dma_channel_active_count(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;
	int active_count;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	active_count = channel ? channel->active_count : 0;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	return active_count;
}

int bind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_interrupt_config __user *user_req;
	struct dma_interrupt_config req;
	struct paintbox_dma_channel *channel;
	int ret;

	user_req = (struct dma_interrupt_config __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "dma channel%u bind irq%u\n", req.channel_id,
			req.interrupt_id);

	ret = bind_dma_interrupt(pb, session, channel, req.interrupt_id);

	mutex_unlock(&pb->lock);

	return ret;
}

int unbind_dma_interrupt_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	ret = unbind_dma_interrupt(pb, session, channel);
	if (ret < 0) {
		dev_err(&pb->pdev->dev,
				"dma channel%u unbind irq failed, err %d\n",
				channel_id, ret);
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "dma channel%u unbind irq\n", channel_id);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock and pb->dma.dma_lock */
static void dma_reset_channel_locked(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, bool force)
{
	uint64_t reset_mask = 1ULL << channel->channel_id;
	struct paintbox_dma_transfer *transfer;

	/* If there are no active transfers and a reset is not being forced then
	 * there is no need to reset the channel.
	 */
	if (!force && channel->active_count == 0) {
		paintbox_dma_disable_channel_interrupts(pb, channel);
		paintbox_pm_disable_dma_channel(pb, channel);
		return;
	}

	paintbox_dma_select_channel(pb, channel->channel_id);
	paintbox_dma_chan_ctrl_set(pb, reset_mask, reset_mask);

	/* Reset bits are not self-clearing so clear it here. */
	paintbox_dma_chan_ctrl_set(pb, reset_mask, 0);

	if (channel->active_count > 0) {
		channel->active_count--;

		transfer = list_entry(channel->active_list.next,
				struct paintbox_dma_transfer, entry);
		if (!WARN_ON(transfer == NULL)) {
			list_del(&transfer->entry);
			dev_warn(&pb->pdev->dev, "dma channel%u reset, transfer canceled\n",
					channel->channel_id);
			dma_report_completion(pb, channel, transfer,
					ktime_get_boottime(), -ECANCELED);
		}
	}
}

/* The caller to this function must hold pb->lock */
void dma_reset_channel(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	dma_reset_channel_locked(pb, channel, false /* force */);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
void dma_stop_transfer(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	struct paintbox_dma *dma = &pb->dma;
	unsigned long irq_flags, wait_ret;
	uint32_t mode;
	uint64_t stop_mask = 1ULL << (channel->channel_id +
			DMA_CHAN_CTRL_STOP_SHIFT);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* If the DMA channel is powered down then there is nothing to stop. */
	if (!channel->pm_enabled) {
		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		return;
	}

	paintbox_dma_select_channel(pb, channel->channel_id);

	/* Disable the channel to prevent any pending transfers from starting.
	 */
	mode = readl(dma->dma_base + DMA_CHAN_MODE);
	if (mode & DMA_CHAN_MODE_CHAN_ENA_MASK) {
		mode &= ~DMA_CHAN_MODE_CHAN_ENA_MASK;
		writel(mode, dma->dma_base + DMA_CHAN_MODE);
	}

	/* If there is no active transfer then disable the interrupt and
	 * power down the channel.
	 */
	if (channel->active_count == 0) {
		paintbox_dma_disable_channel_interrupts(pb, channel);
		paintbox_pm_disable_dma_channel(pb, channel);

		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		return;
	}

	mode = readl(dma->dma_base + DMA_CHAN_MODE_RO);

	/* If MIPI is the source or destination of the transfer then cleanup the
	 * MIPI stream first before initiating the DMA stop request.
	 */
	if (paintbox_dma_src_is_mipi(mode) || paintbox_dma_dst_is_mipi(mode)) {
		/* It should not be possible to program a MIPI transfer without
		 * having a MIPI stream associated with the channel.
		 */
		if (!WARN_ON(!channel->mipi_stream))
			mipi_request_cleanup(pb, channel->mipi_stream);
	}

	/* If there is an active transfer then issue a stop request. */
	paintbox_dma_chan_ctrl_set(pb, stop_mask, stop_mask);

	reinit_completion(&channel->stop_completion);

	channel->stop_request = true;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "dma channel%u stopping\n",
			channel->channel_id);

	/* Wait for the EOF frame interrupt to occur.  If we timeout waiting
	 * for the interrupt then the DMA channel will be reset below.  This can
	 * lead to loss of SSP pointers.
	 */
	wait_ret = wait_for_completion_timeout(&channel->stop_completion,
			usecs_to_jiffies(max(DMA_STOP_TIMEOUT_US,
			MIPI_CLEANUP_TIMEOUT_US)));
	if (wait_ret == 0)
		dev_err(&pb->pdev->dev,
				"dma channel%u failed to stop in %uus\n",
				channel->channel_id, max(DMA_STOP_TIMEOUT_US,
				MIPI_CLEANUP_TIMEOUT_US));

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* Re-select the channel, the DMA_SEL field might have changed while the
	 * pb->dma.dma_lock spinlock was not held.
	 */
	paintbox_dma_select_channel(pb, channel->channel_id);

	/* Stop bits are not self-clearing so clear it here. */
	paintbox_dma_chan_ctrl_set(pb, stop_mask, 0);
	paintbox_dma_disable_channel_interrupts(pb, channel);

	/* If the transfer was to or from a line buffer then reset the
	 * line buffer too.
	 */
	if (paintbox_dma_src_is_lbp(mode) || paintbox_dma_dst_is_lbp(mode)) {
		uint32_t chan_node = readl(dma->dma_base + DMA_CHAN_NODE_RO);

		reset_lb(pb, chan_node & DMA_CHAN_NODE_RO_CORE_ID_MASK,
				(chan_node & DMA_CHAN_NODE_RO_LB_ID_MASK) >>
				DMA_CHAN_NODE_RO_LB_ID_SHIFT);
	}

	/* If this was a MIPI related transfer then verify that the cleanup
	 * operation completed.
	 */
	if (paintbox_dma_src_is_mipi(mode) || paintbox_dma_dst_is_mipi(mode)) {
		/* It should not be possible to program a MIPI transfer without
		 * having a MIPI stream associated with the channel.
		 */
		if (!WARN_ON(!channel->mipi_stream))
			verify_cleanup_completion(pb, channel->mipi_stream);
	}

	/* The DMA channel should be quiescent at this point.  In the case of
	 * DMA double buffering the second DMA transfer (in the kernel active
	 * queue but pending in hardware) will be reported as canceled.  Note,
	 * if the DMA stop of the active transfer had failed then it will also
	 * be reported as canceled here.
	 */
	while (channel->active_count > 0) {
		struct paintbox_dma_transfer *transfer;

		channel->active_count--;

		transfer = list_entry(channel->active_list.next,
				struct paintbox_dma_transfer, entry);
		if (!WARN_ON(transfer == NULL)) {
			list_del(&transfer->entry);
			dma_report_completion(pb, channel, transfer,
					ktime_get_boottime(), -ECANCELED);
		}
	}

	WARN_ON(!list_empty(&channel->active_list));

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
	/* On the first version of the hardware we always do a reset after
	 * stopping the channel.
	 */
	channel->stop_request = false;
	dma_reset_channel_locked(pb, channel, true /* force */);
#else
	/* If the channel didn't stop within the timeout then reset it.  This
	 * situation can occur if there are no outstanding requests between the
	 * src and dst or if the transfer has not started.
	 */
	if (channel->stop_request) {
		channel->stop_request = false;

		dma_reset_channel_locked(pb, channel, false /* force */);
	}
#endif

	/* If there are no active transfers then power down the DMA channel. */
	if (channel->active_count == 0)
		paintbox_pm_disable_dma_channel(pb, channel);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	dev_dbg(&pb->pdev->dev, "dma channel%u stopped\n", channel->channel_id);
}

/* The caller to this function must hold pb->lock */
void release_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;

	dev_dbg(&pb->pdev->dev, "dma channel%u release\n", channel->channel_id);

	dma_stop_transfer(pb, channel);

	drain_queue(pb, &channel->pending_list, &channel->pending_count);
	drain_queue(pb, &channel->active_list, &channel->active_count);
	drain_queue(pb, &channel->completed_list, &channel->completed_count);
	drain_queue(pb, &pb->dma.discard_list, &pb->dma.discard_count);

	/* If there is a MIPI stream associated with this DMA channel then
	 * notify it that the DMA channel has been released.
	 */
	if (channel->mipi_stream) {
		mipi_handle_dma_channel_released(pb, channel->mipi_stream);

		spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

		channel->mipi_stream = NULL;

		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
	}

	/* Remove the DMA channel from the session. */
	list_del(&channel->session_entry);

	channel->session = NULL;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* Make sure the DMA channel is powered down. */
	if (!WARN_ON(channel->active_count != 0))
		paintbox_pm_disable_dma_channel(pb, channel);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
void dma_handle_mipi_stream_released(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	channel->mipi_stream = NULL;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
}

/* The caller to this function must hold pb->lock */
struct paintbox_dma_channel *dma_handle_mipi_stream_allocated(
		struct paintbox_data *pb, struct paintbox_session *session,
		struct paintbox_mipi_stream *stream, unsigned int channel_id,
		int *ret)
{
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;

	if (channel_id >= pb->dma.num_channels) {
		dev_err(&pb->pdev->dev, "%s: invalid dma channel id %d\n",
				__func__, channel_id);
		*ret = -EINVAL;
		return NULL;
	}

	/* If the channel has not been allocated to a session then return with
	 * no error.  This is not an error if the MIPI stream is allocated
	 * before the DMA channel.  Cross-linking the DMA channel with the MIPI
	 * stream will be handled in mipi_dma_channel_allocated() once the DMA
	 * channel is allocated.
	 */
	if (pb->dma.channels[channel_id].session == NULL) {
		*ret = 0;
		return NULL;
	}

	/* If the DMA channel is owned by a different session then it can not be
	 * associated with this MIPI stream.
	 */
	if (pb->dma.channels[channel_id].session != session) {
		dev_err(&pb->pdev->dev,
				"%s: dma channel%u is not part of this session, mipi %s stream%u can not be used.\n",
				__func__, channel_id,
				stream->is_input ? "input" : "output",
				stream->stream_id);
		*ret = -EACCES;
		return NULL;
	}

	channel = &pb->dma.channels[channel_id];

	/* If the DMA channel is already associated with a MIPI stream, then it
	 * cannot be associated with this stream. This can happen on Canvas,
	 * which shares DMA channels across MIPI output interafces.
	 */
	if (channel->mipi_stream) {
		dev_warn(&pb->pdev->dev,
				"%s: dma channel%u is already used for mipi %s stream%u, and cannot be used for mipi %s stream%u.\n",
				__func__, channel_id,
				channel->mipi_stream->is_input ? "input" : "output",
				channel->mipi_stream->stream_id,
				stream->is_input ? "input" : "output",
				stream->stream_id);
		*ret = -EBUSY;
		return NULL;
	}

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	channel->mipi_stream = stream;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	return channel;
}

int release_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	/* Unbind any associated interrupt */
	unbind_dma_interrupt(pb, session, channel);

	release_dma_channel(pb, session, channel);

	mutex_unlock(&pb->lock);

	return ret;
}

int flush_dma_transfers_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg) {
	struct dma_transfer_flush __user *user_req;
	struct dma_transfer_flush req;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	user_req = (struct dma_transfer_flush __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "dma channel%u flush transfers\n",
			req.channel_id);

	if (req.flush_pending)
		drain_queue(pb, &channel->pending_list,
				&channel->pending_count);

	if (req.flush_active) {
		/* Stop any running transfers before flushing the active queue.
		 */
		dma_stop_transfer(pb, channel);
		drain_queue(pb, &channel->active_list, &channel->active_count);
	}

	if (req.flush_completed)
		drain_queue(pb, &channel->completed_list,
				&channel->completed_count);

	mutex_unlock(&pb->lock);

	return 0;
}

/* The caller to this function must hold pb->lock */
int allocate_dma_channel(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned int channel_id)
{
	struct paintbox_dma_channel *channel;

	channel = &pb->dma.channels[channel_id];
	if (channel->session) {
		dev_warn(&pb->pdev->dev,
				"%s: access error, dma channel id %d\n",
				__func__, channel_id);
		return -EACCES;
	}

	dev_dbg(&pb->pdev->dev, "dma channel%u allocated\n", channel_id);

	channel->session = session;
	list_add_tail(&channel->session_entry, &session->dma_list);

	/* MIPI streams and DMA channels have a fixed mapping, i.e. a specific
	 * DMA channel must be used for transfers from a particular MIPI stream.
	 * Notify the MIPI code that this DMA channel has been allocated to the
	 * session.
	 */
	channel->mipi_stream = mipi_handle_dma_channel_allocated(pb, session,
			channel);

	return 0;
}

int allocate_dma_channel_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	int ret;
	unsigned int channel_id = (unsigned int)arg;

	if (channel_id >= pb->dma.num_channels) {
		dev_err(&pb->pdev->dev, "%s: invalid dma channel id %d\n",
				__func__, channel_id);
		return -EINVAL;
	}

	mutex_lock(&pb->lock);
	ret = allocate_dma_channel(pb, session, channel_id);
	if (ret < 0)
		dev_err(&pb->pdev->dev,
				"%s: allocate dma channel id %d error %d\n",
				__func__, channel_id, ret);
	mutex_unlock(&pb->lock);

	return ret;
}

/* The caller to this function must hold pb->dma.dma_lock and DMA_CHAN_SEL must
 * be set.
 */
static inline void paintbox_dma_load_channel_regs(void __iomem *dma_base,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	if (transfer->chan_img_format != channel->regs.chan_img_format) {
		channel->regs.chan_img_format = transfer->chan_img_format;
		writel(transfer->chan_img_format, dma_base +
				DMA_CHAN_IMG_FORMAT);
	}

	if (transfer->chan_img_size != channel->regs.chan_img_size) {
		channel->regs.chan_img_size = transfer->chan_img_size;
		writel(transfer->chan_img_size, dma_base + DMA_CHAN_IMG_SIZE);
	}

	if (transfer->chan_img_pos != channel->regs.chan_img_pos) {
		channel->regs.chan_img_pos = transfer->chan_img_pos;
		writeq(transfer->chan_img_pos, dma_base + DMA_CHAN_IMG_POS);
	}

	if (transfer->chan_img_layout != channel->regs.chan_img_layout) {
		channel->regs.chan_img_layout = transfer->chan_img_layout;
		writeq(transfer->chan_img_layout, dma_base +
				DMA_CHAN_IMG_LAYOUT);
	}

	if (transfer->chan_bif_xfer != channel->regs.chan_bif_xfer) {
		channel->regs.chan_bif_xfer = transfer->chan_bif_xfer;
		writel(transfer->chan_bif_xfer, dma_base + DMA_CHAN_BIF_XFER);
	}

	/* TODO:  The trace_combiner used in register trace generation
	 * relies on the DMA_CHAN_VA and DMA_CHAN_VA_BDRY registers being
	 * present in register traces for DMA transfers.  When running on the
	 * Simulator the driver needs to always write these registers
	 * until the trace combiner is fixed.  b/37688363
	 */
#ifdef CONFIG_PAINTBOX_SIMULATOR_SUPPORT
	channel->regs.chan_va = transfer->chan_va;
	writeq(transfer->chan_va, dma_base + DMA_CHAN_VA);

	channel->regs.chan_va_bdry = transfer->chan_va_bdry;
	writeq(transfer->chan_va_bdry, dma_base + DMA_CHAN_VA_BDRY);
#else
	if (transfer->chan_va != channel->regs.chan_va) {
		channel->regs.chan_va = transfer->chan_va;
		writeq(transfer->chan_va, dma_base + DMA_CHAN_VA);
	}

	if (transfer->chan_va_bdry != channel->regs.chan_va_bdry) {
		channel->regs.chan_va_bdry = transfer->chan_va_bdry;
		writeq(transfer->chan_va_bdry, dma_base + DMA_CHAN_VA_BDRY);
	}
#endif

	if (transfer->chan_noc_xfer != channel->regs.chan_noc_xfer) {
		channel->regs.chan_noc_xfer = transfer->chan_noc_xfer;
		writeq(transfer->chan_noc_xfer, dma_base + DMA_CHAN_NOC_XFER);
	}

	if (transfer->chan_node != channel->regs.chan_node) {
		channel->regs.chan_node = transfer->chan_node;
		writel(transfer->chan_node, dma_base + DMA_CHAN_NODE);
	}
}

/* The caller to this function must hold pb->dma.dma_lock. */
static void commit_transfer_to_hardware(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	struct paintbox_dma *dma = &pb->dma;

	list_add_tail(&transfer->entry, &channel->active_list);
	channel->active_count++;

	paintbox_dma_select_channel(pb, channel->channel_id);
	paintbox_dma_load_channel_regs(pb->dma.dma_base, channel, transfer);
	paintbox_dma_enable_channel_interrupts(pb, channel);

	if (channel->stats.time_stats_enabled)
		transfer->start_time = ktime_get_boottime();

	/* Write the channel mode register last as this will enqueue the
	 * transfer into the hardware.
	 */
	writel(transfer->chan_mode, dma->dma_base + DMA_CHAN_MODE);

	LOG_DMA_REGISTERS(pb, channel);
}

int setup_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_config __user *user_config;
	struct dma_transfer_config config;
	struct paintbox_dma_channel *channel;
	struct paintbox_dma_transfer *transfer;
	unsigned long irq_flags;
#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t enq_start;
	ktime_t setup_start;
#endif
	int ret = 0;

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		setup_start = ktime_get_boottime();
#endif

	user_config = (struct dma_transfer_config __user *)arg;
	if (copy_from_user(&config, user_config, sizeof(config)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, config.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (channel->stats.time_stats_enabled)
		channel->stats.setup_start_time = ktime_get_boottime();

	transfer = paintbox_allocate_transfer(pb);
	if (!transfer) {
		mutex_unlock(&pb->lock);
		dev_err(&pb->pdev->dev, "%s: allocation failure\n", __func__);
		return -ENOMEM;
	}

	switch (config.transfer_type) {
	case DMA_DRAM_TO_LBP:
		ret = dma_setup_dram_to_lbp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_DRAM_TO_STP:
		ret = dma_setup_dram_to_stp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_LBP_TO_DRAM:
		ret = dma_setup_lbp_to_dram_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_MIPI_TO_LBP:
		ret = dma_setup_mipi_to_lbp_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_LBP_TO_MIPI:
		ret = dma_setup_lbp_to_mipi_transfer(pb, session, channel,
				transfer, &config);
		break;
	case DMA_MIPI_TO_DRAM:
		ret = dma_setup_mipi_to_dram_transfer(pb, session, channel,
				transfer, &config);
		break;
	default:
		dev_err(&pb->pdev->dev, "dma: invalid transfer type %u\n",
			config.transfer_type);
		ret = -EINVAL;
	}

	if (ret < 0) {
		paintbox_free_transfer(pb, transfer);
		mutex_unlock(&pb->lock);
		return ret;
	}

	transfer->notify_on_completion = config.notify_on_completion;
	transfer->auto_start_transfer = config.auto_start_transfer;

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled) {
		enq_start = ktime_get_boottime();
		paintbox_debug_log_non_ioctl_stats(pb, PB_STATS_DMA_SETUP,
				setup_start, enq_start, 0);
	}
#endif

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	/* If the transfer is marked for auto start then check to see if there
	 * are any transfers in the pending queue and that there is space in the
	 * active queue before.  If these are conditions are met then the
	 * transfer can be enqueued in the active queue, otherwise it goes to
	 * the pending queue.
	 */
	if (transfer->auto_start_transfer && channel->pending_count == 0 &&
			channel->active_count < MAX_ACTIVE_TRANSFERS) {
		paintbox_pm_enable_dma_channel(pb, channel);
		commit_transfer_to_hardware(pb, channel, transfer);
	} else {
		paintbox_dma_enqueue_pending_transfer(pb, channel, transfer);
	}

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	if (transfer->auto_start_transfer &&
			paintbox_dma_src_is_mipi(transfer->chan_mode))
		ret = paintbox_mipi_enable_input_stream(pb,
				channel->mipi_stream);

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb,  PB_STATS_DMA_ENQ,
				enq_start, ktime_get_boottime(), 0);
#endif

	if (channel->stats.time_stats_enabled)
		channel->stats.setup_finish_time = ktime_get_boottime();

	mutex_unlock(&pb->lock);

	return ret;
}

int read_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	struct dma_transfer_read __user *user_req;
	struct dma_transfer_read req;
	struct paintbox_dma_channel *channel;
	struct paintbox_dma_transfer *transfer;
	unsigned long irq_flags;
	int ret;

	user_req = (struct dma_transfer_read __user *)arg;
	if (copy_from_user(&req, user_req, sizeof(req)))
		return -EFAULT;

	mutex_lock(&pb->lock);

	channel = get_dma_channel(pb, session, req.channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	if (!access_ok(VERIFY_READ, req.host_vaddr, req.len_bytes)) {
		mutex_unlock(&pb->lock);
		return -EFAULT;
	}

	dev_dbg(&pb->pdev->dev, "dma channel%u read transfer\n",
			channel->channel_id);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	if (list_empty(&channel->completed_list)) {
		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		mutex_unlock(&pb->lock);
		return -ENOENT;
	}

	channel->completed_count--;
	WARN_ON(channel->completed_count < 0);

	transfer = list_entry(channel->completed_list.next,
			struct paintbox_dma_transfer, entry);
	list_del(&transfer->entry);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	ret = ipu_dma_release_and_copy_buffer(pb, transfer, req.host_vaddr,
			req.len_bytes);

	paintbox_free_transfer(pb, transfer);

	mutex_unlock(&pb->lock);

	return ret;
}

int stop_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dma_stop_transfer(pb, channel);

	mutex_unlock(&pb->lock);

	return 0;
}

int start_dma_transfer_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_transfer *transfer;
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	dev_dbg(&pb->pdev->dev, "dma channel%u start\n", channel_id);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	if (list_empty(&channel->pending_list)) {
		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		dev_err(&pb->pdev->dev, "dma channel%u no pending transfers\n",
				channel->channel_id);
		ret = -ENOENT;
		goto err_exit;
	}

	if (channel->active_count >= MAX_ACTIVE_TRANSFERS) {
		spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);
		dev_err(&pb->pdev->dev, "dma channel%u too many transfers\n",
				channel->channel_id);
		ret = -EAGAIN;
		goto err_exit;
	}

	transfer = list_entry(channel->pending_list.next,
			struct paintbox_dma_transfer, entry);
	list_del(&transfer->entry);
	channel->pending_count--;
	WARN_ON(channel->pending_count < 0);

	paintbox_pm_enable_dma_channel(pb, channel);

	commit_transfer_to_hardware(pb, channel, transfer);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	/* Verify that the MIPI stream is enabled.  If the stream is not enabled
	 * then this function will enable it.  If it is enabled then it will
	 * return quickly.
	 */
	if (paintbox_dma_src_is_mipi(transfer->chan_mode))
		ret = paintbox_mipi_enable_input_stream(pb,
				channel->mipi_stream);

err_exit:
	mutex_unlock(&pb->lock);

	return ret;
}

/* Returns the number of DMA transfers that have been completed and are ready
 * to be read out by the HAL.
 */
int get_completed_transfer_count_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	unsigned long irq_flags;
	int ret;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret < 0) {
		mutex_unlock(&pb->lock);
		return ret;
	}

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	ret = channel->completed_count;

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	dev_dbg(&pb->pdev->dev,
			"dma channel%u get completed transfer count %d\n",
			channel_id, ret);

	return ret;
}

/* The caller to this function must hold pb->dma.dma_lock */
static void dma_report_channel_error_locked(struct paintbox_data *pb,
		unsigned int channel_id, int err)
{
	struct paintbox_dma_channel *channel;
	struct paintbox_dma_transfer *transfer;
	uint32_t mode;

	channel = &pb->dma.channels[channel_id];

	paintbox_dma_select_channel(pb, channel->channel_id);

	/* Disable the channel to prevent any pending transfers from starting.
	 */
	mode = readl(pb->dma.dma_base + DMA_CHAN_MODE);
	if (mode & DMA_CHAN_MODE_CHAN_ENA_MASK) {
		mode &= ~DMA_CHAN_MODE_CHAN_ENA_MASK;
		writel(mode, pb->dma.dma_base + DMA_CHAN_MODE);
	}

	/* If there is an active stop request on this channel then cancel it. */
	if (channel->stop_request) {
		channel->stop_request = false;
		complete(&channel->stop_completion);
	}

	/* If there is no active transfer then there isn't much more we can do.
	 */
	if (!channel->active_count)
		return;

	/* If there is an active request then then complete it and report the
	 * error to the client.
	 */
	channel->active_count--;
	WARN_ON(channel->active_count < 0);

	transfer = list_entry(channel->active_list.next,
			struct paintbox_dma_transfer, entry);
	if (!WARN_ON(transfer == NULL)) {
		list_del(&transfer->entry);
		dma_report_completion(pb, channel, transfer,
				ktime_get_boottime(), err);
	}
}

/* This function must be called in an interrupt context */
void dma_report_channel_error(struct paintbox_data *pb,
		unsigned int channel_id, int err)
{
	spin_lock(&pb->dma.dma_lock);

	dma_report_channel_error_locked(pb, channel_id, err);

	spin_unlock(&pb->dma.dma_lock);
}

/* This function must be called in an interrupt context */
void dma_report_error_all_channels(struct paintbox_data *pb, int err)
{
	unsigned int channel_id;

	spin_lock(&pb->dma.dma_lock);

	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++)
		dma_report_channel_error_locked(pb, channel_id, err);

	spin_unlock(&pb->dma.dma_lock);
}

/* The caller to this function must hold pb->dma.dma_lock */
static void paintbox_dma_reset_locked(struct paintbox_data *pb)
{
	unsigned int channel_id;

	writel(DMA_CTRL_DMA_RESET_MASK, pb->dma.dma_base + DMA_CTRL);

	/* TODO:  There should be no need to hold the DMA reset
	 * register high for a minimum period but the FPGA will lockup if the
	 * reset register is cleared immediately following a VA Error Interrupt.
	 * This needs to be evaluated on the real hardware.  b/35779292
	 *
	 */
	udelay(DMA_RESET_HOLD_PERIOD);
	writel(0, pb->dma.dma_base + DMA_CTRL);

	/* Notify all channels that there has been a DMA block reset */
	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++)
		dma_report_channel_error_locked(pb, channel_id, -EIO);
}

#ifdef CONFIG_PAINTBOX_TEST_SUPPORT
int dma_test_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned long irq_flags;

	mutex_lock(&pb->lock);

	spin_lock_irqsave(&pb->dma.dma_lock, irq_flags);

	paintbox_dma_reset_locked(pb);

	spin_unlock_irqrestore(&pb->dma.dma_lock, irq_flags);

	mutex_unlock(&pb->lock);

	return 0;
}

int dma_test_channel_reset_ioctl(struct paintbox_data *pb,
		struct paintbox_session *session, unsigned long arg)
{
	unsigned int channel_id = (unsigned int)arg;
	struct paintbox_dma_channel *channel;
	int ret = 0;

	mutex_lock(&pb->lock);
	channel = get_dma_channel(pb, session, channel_id, &ret);
	if (ret >= 0)
		dma_reset_channel(pb, channel);

	mutex_unlock(&pb->lock);

	return ret;
}
#endif

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
/* This function must be called in an interrupt context */
int dma_set_mipi_error(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, int err)
{
	struct paintbox_dma_transfer *transfer;

	spin_lock(&pb->dma.dma_lock);

	/* Verify that there is an active transfer. */
	if (!channel->active_count) {
		spin_unlock(&pb->dma.dma_lock);
		return -ENOENT;
	}

	transfer = list_entry(channel->active_list.next,
			struct paintbox_dma_transfer, entry);
	if (!WARN_ON(transfer == NULL)) {
		/* Verify that the active transfer is a MIPI transfer */
		if (paintbox_dma_src_is_mipi(transfer->chan_mode) ||
				paintbox_dma_dst_is_mipi(transfer->chan_mode))
			transfer->error = err;
	}

	spin_unlock(&pb->dma.dma_lock);

	return 0;
}
#endif

/* This function must be called in an interrupt context */
void dma_report_mipi_output_completed(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel)
{
	spin_lock(&pb->dma.dma_lock);

	if (channel->active_count == 0)
		paintbox_pm_disable_dma_channel(pb, channel);

	spin_unlock(&pb->dma.dma_lock);
}

/* This function must be called in an interrupt context and the caller must
 * hold pb->dma.dma_lock.
 */
void paintbox_dma_disable_gather_channels(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_dma_channel *channel, *channel_next;

	/* Gather DMA channels can only be powered down when both DMA channels
	 * in the gather have completed.  The driver does not have information
	 * about the relationship between DMA channels so the driver tracks
	 * all gather channels within the session.  Once all gather transfers
	 * for the session have completed the gather DMA channels can be
	 * powered down.
	 */
	list_for_each_entry_safe(channel, channel_next, &session->dma_list,
			session_entry) {
		if (!(session->dma_gather_channel_mask &
				(1 << channel->channel_id)))
			continue;

		/* If this channel has other active transfers then skip it. */
		if (channel->active_count > 0)
			continue;

		paintbox_pm_disable_dma_channel(pb, channel);
	}

	session->dma_gather_channel_mask = 0;
}

/* This function must be called in an interrupt context and the caller must
 * hold pb->dma.dma_lock.
 */
void paintbox_dma_eof_interrupt(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer, ktime_t timestamp)
{
	struct paintbox_dma_transfer *next_transfer;
	int err = transfer->error;

	channel->stats.eof_interrupts++;

	if (channel->stats.time_stats_enabled)
		channel->stats.last_transfer_time_us = ktime_to_us(ktime_sub(
				timestamp, transfer->start_time));

	/* If there was a stop request then clear it. */
	if (channel->stop_request) {
		channel->stop_request = false;
		complete(&channel->stop_completion);
		/* In the case of a MIPI error and a cancel error then the
		 * cancel will take precedence.
		 */
		err = -ECANCELED;
	}

	dev_dbg(&pb->pdev->dev, "dma channel%u EOF err %d\n",
			channel->channel_id, err);

	dma_report_completion(pb, channel, transfer, timestamp, err);

	/* If there are no pending transfers or the channel has been stopped
	 * then return now.
	 */
	if (list_empty(&channel->pending_list) || err == -ECANCELED)
		return;

	/* There should be space in the active queue if not then there is a
	 * bug.
	 */
	if (WARN_ON(channel->active_count >= MAX_ACTIVE_TRANSFERS))
		return;

	next_transfer = list_entry(channel->pending_list.next,
			struct paintbox_dma_transfer, entry);

	if (next_transfer->auto_start_transfer) {
		list_del(&next_transfer->entry);
		channel->pending_count--;
		WARN_ON(channel->pending_count < 0);

		commit_transfer_to_hardware(pb, channel, next_transfer);
	}
}

/* This function must be called in an interrupt context and the caller must
 * hold pb->dma.dma_lock.
 */
void paintbox_dma_va_interrupt(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer, ktime_t timestamp)
{
	channel->stats.va_interrupts++;

	/* If there was a stop request then clear it.  The VA error takes
	 * precedence over the stop request.
	 */
	if (channel->stop_request) {
		channel->stop_request = false;
		complete(&channel->stop_completion);
	}

	dev_dbg(&pb->pdev->dev, "dma channel%u VA ERR %d\n",
			channel->channel_id, -EIO);

	if (!transfer) {
		dev_warn(&pb->pdev->dev,
				"%s: channel%u no active transfer\n", __func__,
				channel->channel_id);
		return;
	}

	/* Reporting non-recoverable to all channel since DMA VA error cannot
	 * be recovered. This will trigger a full IPU reset.
	 */
	dma_report_error_all_channels(pb, -ENOTRECOVERABLE);
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	paintbox_dma_reset_locked(pb);
#endif
}

static struct paintbox_dma_transfer *paintbox_dma_interrupt_common(
		struct paintbox_data *pb, struct paintbox_dma_channel *channel)
{
	struct paintbox_dma_transfer *transfer;

	transfer = paintbox_dequeue_active_transfer(pb, channel);
	if (!transfer)
		return NULL;

	/* If this is a MIPI_IN transfer then notify the MIPI code that the DMA
	 * transfer has completed.
	 */
	if (paintbox_dma_src_is_mipi(transfer->chan_mode))
		mipi_input_handle_dma_completed(pb, channel->mipi_stream);

	if (paintbox_dma_is_gather(transfer->chan_mode))
		channel->session->dma_gather_transfer_count--;

	return transfer;
}

static void paintbox_dma_pm_disable_if_idle(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct paintbox_dma_transfer *transfer)
{
	/* If there is an active transfer then the channel is not idle and can
	 * not be powered down.
	 */
	if (channel->active_count > 0)
		return;

	/* If the last transfer was a MIPI out transfer then the channel will
	 * not be idle until the MIPI EOF interrupt.  The MIPI code will notify
	 * the DMA code when the MIPI EOF has occurred.
	 */
	if (paintbox_dma_dst_is_mipi(transfer->chan_mode))
		return;

	/* If this is a gather channel then verify that all gather transfers for
	 * the session have completed.  Gather DMA transfers are performed using
	 * DMA channel pairs.  The DMA channels involved can not be powered down
	 * until both DMA transfers have completed.
	 *
	 * The driver does not know the relationship between the DMA channels so
	 * the driver tracks all gather transfers and DMA channels on a per
	 * session basis.  Gather channels will be powered down when all gather
	 * transfers for the session have completed.
	 */
	if (paintbox_dma_is_gather_channel(channel)) {
		if (channel->session->dma_gather_transfer_count == 0)
			paintbox_dma_disable_gather_channels(pb,
					channel->session);
		return;
	}

	/* The DMA channel is idle, power it down. */
	paintbox_pm_disable_dma_channel(pb, channel);
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
/* This function must be called in an interrupt context and pb->dma.dma_lock
 * must be held.
 */
static void paintbox_dma_process_ms_err_and_eof_interrupts(
		struct paintbox_data *pb, uint32_t status, ktime_t timestamp,
		bool process_ms_err)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels && status;
			channel_id++, status >>= 1) {
		struct paintbox_dma_transfer *transfer;
		struct paintbox_dma_channel *channel;

		if (!(status & 0x01))
			continue;

		channel = &pb->dma.channels[channel_id];

		transfer = paintbox_dma_interrupt_common(pb, channel);
		if (!transfer) {
			/* If there is no active transfer associated with this
			 * interrupt then mark the channel as idle and move on
			 * to the next one.
			 */
			paintbox_pm_disable_dma_channel(pb, channel);
			continue;
		}

		if (process_ms_err) {
			/* In the case of a MIPI overflow error we set the
			 * transfer error and then use the EOF handler to
			 * finalize the transfer.
			 */
			transfer->error = -EIO;
		}

		paintbox_dma_eof_interrupt(pb, channel, transfer, timestamp);

		/* Determine if the channel is idle and power it down if
		 * possible.
		 */
		paintbox_dma_pm_disable_if_idle(pb, channel, transfer);
	}
}

/* This function must be called in an interrupt context and pb->dma.dma_lock
 * must be held.
 */
static void paintbox_dma_process_va_err_interrupts(struct paintbox_data *pb,
		uint32_t status, ktime_t timestamp)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels && status;
			channel_id++, status >>= 1) {
		struct paintbox_dma_transfer *transfer;
		struct paintbox_dma_channel *channel;

		if (!(status & 0x01))
			continue;

		channel = &pb->dma.channels[channel_id];

		transfer = paintbox_dma_interrupt_common(pb, channel);
		if (!transfer) {
			/* If there is no active transfer associated with this
			 * interrupt then mark the channel as idle and move on
			 * to the next one.
			 */
			paintbox_pm_disable_dma_channel(pb, channel);
			continue;
		}

		paintbox_dma_va_interrupt(pb, channel, transfer, timestamp);

		/* Determine if the channel is idle and power it down if
		 * possible.
		 */
		paintbox_dma_pm_disable_if_idle(pb, channel, transfer);
	}
}

irqreturn_t paintbox_dma_channel_interrupt(struct paintbox_data *pb,
		ktime_t timestamp)
{
	uint32_t status;

	status = readl(pb->dma.dma_base + DMA_IRQ_ISR);
	if (status) {
		writel(status, pb->dma.dma_base + DMA_IRQ_ISR);
		paintbox_dma_process_ms_err_and_eof_interrupts(pb, status,
				timestamp, false /* process_ms_err */);
	}

	status = readl(pb->dma.dma_base + DMA_IRQ_ISR_OVF);
	if (status) {
		writel(status, pb->dma.dma_base + DMA_IRQ_ISR_OVF);
		paintbox_dma_process_ms_err_and_eof_interrupts(pb, status,
				timestamp, false /* process_ms_err */);
	}

	return IRQ_HANDLED;
}

irqreturn_t paintbox_dma_channel_error_interrupt(struct paintbox_data *pb,
		ktime_t timestamp)
{
	uint64_t status;

	status = readq(pb->dma.dma_base + DMA_IRQ_ERR_ISR);
	if (status) {
		writeq(status, pb->dma.dma_base + DMA_IRQ_ERR_ISR);

		if (status & DMA_IRQ_ERR_ISR_MS_ERR_MASK)
			paintbox_dma_process_ms_err_and_eof_interrupts(pb,
					(status &
					DMA_IRQ_ERR_ISR_MS_ERR_MASK) >>
					DMA_IRQ_ERR_ISR_MS_ERR_SHIFT,
					timestamp, true /* process_ms_err */);


		if (status & DMA_IRQ_ERR_ISR_VA_ERR_MASK)
			paintbox_dma_process_va_err_interrupts(pb, status &
					DMA_IRQ_ERR_ISR_VA_ERR_MASK, timestamp);
	}

	status = readq(pb->dma.dma_base + DMA_IRQ_ERR_ISR_OVF);
	if (status) {
		writeq(status, pb->dma.dma_base + DMA_IRQ_ERR_ISR_OVF);

		if (status & DMA_IRQ_ERR_ISR_MS_ERR_MASK)
			paintbox_dma_process_ms_err_and_eof_interrupts(pb,
					(status &
					DMA_IRQ_ERR_ISR_MS_ERR_MASK) >>
					DMA_IRQ_ERR_ISR_MS_ERR_SHIFT,
					timestamp, true /* process_ms_err */);


		if (status & DMA_IRQ_ERR_ISR_VA_ERR_MASK)
			paintbox_dma_process_va_err_interrupts(pb, status &
					DMA_IRQ_ERR_ISR_VA_ERR_MASK, timestamp);
	}

	return IRQ_HANDLED;
}
#else
/* This function must be called in an interrupt context and pb->dma.dma_lock
 * must be held.  This function will return true if a power down can occur.
 */
static void paintbox_dma_process_transfer_v0(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel, uint32_t status,
		ktime_t timestamp)
{
	struct paintbox_dma_transfer *transfer;

	transfer = paintbox_dma_interrupt_common(pb, channel);
	if (!transfer) {
		paintbox_pm_disable_dma_channel(pb, channel);
		return;
	}

	if (status & DMA_CHAN_ISR_VA_ERR_MASK)
		paintbox_dma_va_interrupt(pb, channel, transfer, timestamp);
	else if (status & DMA_CHAN_ISR_EOF_MASK)
		paintbox_dma_eof_interrupt(pb, channel, transfer, timestamp);

	paintbox_dma_pm_disable_if_idle(pb, channel, transfer);
}

/* This function must be called in an interrupt context */
irqreturn_t paintbox_dma_interrupt(struct paintbox_data *pb,
		uint32_t channel_mask, ktime_t timestamp)
{
	unsigned int channel_id;

	for (channel_id = 0; channel_id < pb->dma.num_channels && channel_mask;
			channel_id++, channel_mask >>= 1) {
		struct paintbox_dma_channel *channel;
		uint32_t status;

		if (!(channel_mask & 0x01))
			continue;

		spin_lock(&pb->dma.dma_lock);

		channel = &pb->dma.channels[channel_id];
		channel->stats.irq_activations++;

		paintbox_dma_select_channel(pb, channel_id);

		/* Each DMA channel has an ISR register and an ISR_OVF register.
		 * If if an interrupt occurs on the channel then the appropriate
		 * bit will be set in the ISR register.  If a second interupt
		 * occurs before the first ISR bit is cleared then the
		 * appropriate bit will be set in the ISR_OVF register.
		 */
		status = readl(pb->dma.dma_base + DMA_CHAN_ISR);
		writel(status, pb->dma.dma_base + DMA_CHAN_ISR);

		paintbox_dma_process_transfer_v0(pb, channel, status,
				timestamp);

		/* Check the ISR_OVF status register in case a second
		 * interrupt occured.  If there has been a second interrupt then
		 * pop the other active transfer off the active transfer queue
		 * and process it.
		 */
		if (channel->active_count > 0) {
			status = readl(pb->dma.dma_base + DMA_CHAN_ISR_OVF);
			if (status) {
				writel(status, pb->dma.dma_base +
						DMA_CHAN_ISR_OVF);

				paintbox_dma_process_transfer_v0(pb, channel,
						status, timestamp);
			}
		}

		spin_unlock(&pb->dma.dma_lock);
	}

	return IRQ_HANDLED;
}
#endif

/* Outside of init the caller to this function must hold pb->dma.dma_lock */
static void paintbox_dma_channel_register_init(
		struct paintbox_dma_channel *channel)
{
	channel->regs.chan_img_format = DMA_CHAN_IMG_FORMAT_DEF;
	channel->regs.chan_img_size = DMA_CHAN_IMG_SIZE_DEF;
	channel->regs.chan_img_pos = DMA_CHAN_IMG_POS_DEF;
	channel->regs.chan_img_layout = DMA_CHAN_IMG_LAYOUT_DEF;
	channel->regs.chan_bif_xfer = DMA_CHAN_BIF_XFER_DEF;
	channel->regs.chan_va = DMA_CHAN_VA_DEF;
	channel->regs.chan_va_bdry = DMA_CHAN_VA_BDRY_DEF;
	channel->regs.chan_noc_xfer = DMA_CHAN_NOC_XFER_DEF;
	channel->regs.chan_node = DMA_CHAN_NODE_DEF;
}

/* The caller to this function must hold pb->lock */
void paintbox_dma_release(struct paintbox_data *pb,
		struct paintbox_session *session)
{
	struct paintbox_dma_channel *channel, *channel_next;

	/* Disable any dma channels associated with the channel */
	list_for_each_entry_safe(channel, channel_next, &session->dma_list,
			session_entry) {
		unbind_dma_interrupt(pb, session, channel);
		release_dma_channel(pb, session, channel);
	}
}

/* All sessions must be released before remove can be called. */
void paintbox_dma_remove(struct paintbox_data *pb)
{
	unsigned int channel_id;

	flush_work(&pb->dma.discard_queue_work);

	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		struct paintbox_dma_channel *channel =
				&pb->dma.channels[channel_id];
		paintbox_dma_channel_debug_remove(pb, channel);
	}

	kfree(pb->dma.channels);

	paintbox_dma_debug_remove(pb);
}

static void cancel_queue(struct paintbox_data *pb,
		struct paintbox_dma_channel *channel,
		struct list_head *transfer_list, unsigned int *count,
		ktime_t timestamp)
{
	while (!list_empty(transfer_list)) {
		struct paintbox_dma_transfer *transfer;

		transfer = list_entry(transfer_list->next,
				struct paintbox_dma_transfer, entry);
		list_del(&transfer->entry);
		(*count)--;
		WARN_ON(*count < 0);
		dma_report_completion(pb, channel, transfer, timestamp,
				-ECANCELED);
	}
}

/* Resets shadows and state and cancels active and pending DMAs.
 * pb->lock is held, so no stops are pending.
 */
void paintbox_dma_post_ipu_reset(struct paintbox_data *pb)
{
	ktime_t timestamp = ktime_get_boottime();
	unsigned int channel_id;

	dev_info(&pb->pdev->dev, "%s\n", __func__);

	pb->dma.bif_outstanding = ((DMA_CHAN_BIF_XFER_DEF &
			DMA_CHAN_BIF_XFER_OUTSTANDING_MASK) >>
			DMA_CHAN_BIF_XFER_OUTSTANDING_SHIFT) + 1;

	pb->dma.selected_dma_channel_id = (DMA_CTRL_DEF &
			DMA_CTRL_DMA_CHAN_SEL_MASK) >>
			DMA_CTRL_DMA_CHAN_SEL_SHIFT;

	pb->dma.regs.chan_ctrl = DMA_CHAN_CTRL_DEF;
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	pb->dma.regs.irq_imr = DMA_IRQ_IMR_DEF;
	pb->dma.regs.irq_err_imr = DMA_IRQ_ERR_IMR_DEF;
#endif

	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		struct paintbox_dma_channel *channel =
				&pb->dma.channels[channel_id];

		channel->pm_enabled = false;
		channel->interrupts_enabled = false;
		paintbox_dma_channel_register_init(channel);
		cancel_queue(pb, channel, &channel->active_list,
				&channel->active_count, timestamp);
		cancel_queue(pb, channel, &channel->pending_list,
				&channel->pending_count, timestamp);
	}
}

int paintbox_dma_init(struct paintbox_data *pb)
{
	unsigned int channel_id;

	pb->dma.bif_outstanding = ((DMA_CHAN_BIF_XFER_DEF &
			DMA_CHAN_BIF_XFER_OUTSTANDING_MASK) >>
			DMA_CHAN_BIF_XFER_OUTSTANDING_SHIFT) + 1;

	pb->dma.selected_dma_channel_id = (DMA_CTRL_DEF &
			DMA_CTRL_DMA_CHAN_SEL_MASK) >>
			DMA_CTRL_DMA_CHAN_SEL_SHIFT;

	pb->dma.regs.chan_ctrl = DMA_CHAN_CTRL_DEF;
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	pb->dma.regs.irq_imr = DMA_IRQ_IMR_DEF;
	pb->dma.regs.irq_err_imr = DMA_IRQ_ERR_IMR_DEF;
#endif

	spin_lock_init(&pb->dma.dma_lock);
	INIT_LIST_HEAD(&pb->dma.discard_list);
	INIT_LIST_HEAD(&pb->dma.free_list);

	paintbox_dma_debug_init(pb);

	pb->dma.channels = kzalloc(sizeof(struct paintbox_dma_channel) *
			pb->dma.num_channels, GFP_KERNEL);
	if (!pb->dma.channels)
		return -ENOMEM;

	/* Store channel id with object as a convenience to avoid doing a
	 * lookup later on.
	 */
	for (channel_id = 0; channel_id < pb->dma.num_channels; channel_id++) {
		struct paintbox_dma_channel *channel =
				&pb->dma.channels[channel_id];
		channel->channel_id = channel_id;

		paintbox_dma_channel_register_init(channel);

		INIT_LIST_HEAD(&channel->pending_list);
		INIT_LIST_HEAD(&channel->active_list);
		INIT_LIST_HEAD(&channel->completed_list);
		init_completion(&channel->stop_completion);
		paintbox_dma_channel_debug_init(pb, channel);
	}

	INIT_WORK(&pb->dma.discard_queue_work, paintbox_dma_discard_queue_work);

	dev_dbg(&pb->pdev->dev, "dma: base %p len %lu dma channels %u\n",
			pb->dma.dma_base, DMA_BLOCK_LEN,
			pb->dma.num_channels);

	return 0;
}
