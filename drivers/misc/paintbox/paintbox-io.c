/*
 * IO support for the Paintbox programmable IPU
 *
 * Copyright (C) 2016 Google, Inc.
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

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "paintbox-bif.h"
#include "paintbox-common.h"
#include "paintbox-debug.h"
#include "paintbox-dma.h"
#include "paintbox-io.h"
#include "paintbox-mipi.h"
#include "paintbox-mmu.h"
#include "paintbox-regs.h"
#include "paintbox-stp.h"

#ifdef CONFIG_PAINTBOX_DEBUG
static uint64_t paintbox_io_apb_reg_entry_read(
		struct paintbox_debug_reg_entry *reg_entry)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_io *io = container_of(debug, struct paintbox_io,
			apb_debug);
	return readq(io->apb_base + reg_entry->reg_offset);
}

static void paintbox_io_apb_reg_entry_write(
		struct paintbox_debug_reg_entry *reg_entry, uint64_t val)
{
	struct paintbox_debug *debug = reg_entry->debug;
	struct paintbox_io *io = container_of(debug, struct paintbox_io,
			apb_debug);
	writeq(val, io->apb_base + reg_entry->reg_offset);
}

static const char *io_apb_reg_names[IO_APB_NUM_REGS] = {
	REG_NAME_ENTRY(IPU_ISR),
	REG_NAME_ENTRY(IPU_IMR),
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	REG_NAME_ENTRY(IPU_IER),
	REG_NAME_ENTRY(IPU_ITR),
	REG_NAME_ENTRY(DMA_ERR_ISR),
	REG_NAME_ENTRY(DMA_ERR_IMR),
	REG_NAME_ENTRY(DMA_ERR_IER),
	REG_NAME_ENTRY(DMA_ERR_ITR),
	REG_NAME_ENTRY(STP_ERR_ISR),
	REG_NAME_ENTRY(STP_ERR_IMR),
	REG_NAME_ENTRY(STP_ERR_IER),
	REG_NAME_ENTRY(STP_ERR_ITR),
	REG_NAME_ENTRY(MIF_ERR_ISR),
	REG_NAME_ENTRY(MIF_ERR_IMR),
	REG_NAME_ENTRY(MIF_ERR_IER),
	REG_NAME_ENTRY(MIF_ERR_ITR),
	REG_NAME_ENTRY(IPU_STP_GRP_SEL),
#endif
};

static inline int paintbox_dump_io_apb_reg(struct paintbox_data *pb,
		uint32_t reg_offset, char *buf, int *written, size_t len)
{
	const char *reg_name = io_apb_reg_names[REG_INDEX(reg_offset)];
	return dump_ipu_register64(pb, pb->io.apb_base, reg_offset, reg_name,
			buf, written, len);
}

int paintbox_dump_io_apb_registers(struct paintbox_debug *debug, char *buf,
		size_t len)
{
	struct paintbox_data *pb = debug->pb;
	unsigned int i;
	int ret, written = 0;

	for (i = 0; i < IO_APB_NUM_REGS; i++) {
		if (io_apb_reg_names[i] != NULL) {
			ret = paintbox_dump_io_apb_reg(pb, i * IPU_REG_WIDTH,
					buf, &written, len);
			if (ret < 0)
				goto err_exit;
		}
	}

	return written;

err_exit:
	dev_err(&pb->pdev->dev, "%s: register dump error, err = %d", __func__,
			ret);
	return ret;
}
#endif

static irqreturn_t paintbox_io_interrupt(int irq, void *arg)
{
	struct paintbox_data *pb = (struct paintbox_data *)arg;
	uint64_t status;
	ktime_t timestamp;

#ifdef CONFIG_PAINTBOX_DEBUG
	ktime_t start_time;

	if (pb->stats.ioctl_time_enabled)
		start_time = ktime_get_boottime();
#endif

	pb->io.irq_activations++;

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	status = readq(pb->io.apb_base + IPU_ISR);
#else
	status = readl(pb->io.apb_base + IPU_ISR);
#endif
	if (status == 0)
		return IRQ_NONE;

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	writeq(status, pb->io.apb_base + IPU_ISR);
#else
	writel(status, pb->io.apb_base + IPU_ISR);
#endif

	timestamp = ktime_get_boottime();

	pb->io.ipu_interrupts++;

	if (status & IPU_ISR_BIF_INTR_MASK)
		paintbox_bif_interrupt(pb);

	if (status & IPU_ISR_MMU_INTR_MASK)
		paintbox_mmu_interrupt(pb);

	/* MIPI interrupts need to be processed before DMA interrupts so error
	 * conditions like MIPI input overflow can be reported properly.  A
	 * MIPI OVF interrupt will cause an early EOF on the associated DMA
	 * channel.  This DMA EOF interrupt needs to be reported as an error and
	 * not as a normal completion.
	 */
	if (status & IPU_ISR_MPI_INTR_MASK)
		paintbox_mipi_input_interrupt(pb, (status &
				IPU_ISR_MPI_INTR_MASK) >>
				IPU_ISR_MPI_INTR_SHIFT, timestamp);

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	if (status & IPU_ISR_MIF_ERR_INTR_MASK) {
		uint32_t error_status;

		error_status = readl(pb->io.apb_base + MIF_ERR_ISR);
		writel(error_status, pb->io.apb_base + MIF_ERR_ISR);
		paintbox_mipi_input_error_interrupt(pb, timestamp);
	}
#endif

	if (status & IPU_ISR_MPO_INTR_MASK)
		paintbox_mipi_output_interrupt(pb, (status &
				IPU_ISR_MPO_INTR_MASK) >>
				IPU_ISR_MPO_INTR_SHIFT, timestamp);



#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	if (status & IPU_ISR_DMA_CHAN_INTR_MASK)
		paintbox_dma_channel_interrupt(pb, timestamp);

	if (status & IPU_ISR_DMA_ERR_INTR_MASK) {
		uint32_t error_status;

		error_status = readl(pb->io.apb_base + DMA_ERR_ISR);
		writel(error_status, pb->io.apb_base + DMA_ERR_ISR);
		paintbox_dma_channel_error_interrupt(pb, timestamp);
	}
#else
	if (status & IPU_ISR_DMA_CHAN_INTR_MASK)
		paintbox_dma_interrupt(pb, status & IPU_ISR_DMA_CHAN_INTR_MASK,
				timestamp);
#endif

	if (status & IPU_ISR_STP_INTR_MASK)
		paintbox_stp_interrupt(pb, (status &
				IPU_ISR_STP_INTR_MASK) >>
				IPU_ISR_STP_INTR_SHIFT, timestamp);

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	if (status & IPU_ISR_STP_ERR_INTR_MASK) {
		uint32_t error_status;

		error_status = readl(pb->io.apb_base + STP_ERR_ISR);
		writel(error_status, pb->io.apb_base + STP_ERR_ISR);
		paintbox_stp_error_interrupt(pb, error_status, timestamp);
	}
#endif

#ifdef CONFIG_PAINTBOX_DEBUG
	if (pb->stats.ioctl_time_enabled)
		paintbox_debug_log_non_ioctl_stats(pb,  PB_STATS_IO_INTERRUPT_HANDLE,
				start_time, ktime_get_boottime(), 0);
#endif

	return IRQ_HANDLED;
}

void paintbox_io_enable_interrupt(struct paintbox_data *pb,
		uint64_t enable_mask)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.ipu_imr |= enable_mask;
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	writeq(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
#else
	writel(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
#endif
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_io_disable_interrupt(struct paintbox_data *pb,
		uint64_t disable_mask)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);
	pb->io.regs.ipu_imr &= ~disable_mask;
#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
	writeq(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
#else
	writel(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
#endif
	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

#if CONFIG_PAINTBOX_VERSION_MAJOR >= 1
void paintbox_enable_dma_channel_error_interrupt(struct paintbox_data *pb,
		unsigned int channel_id)
{
	uint32_t dma_err_imr;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	dma_err_imr = readl(pb->io.apb_base + DMA_ERR_IMR);
	dma_err_imr |= 1 << (channel_id + DMA_ERR_IMR_DMA_CHAN_ERR_SHIFT);
	writel(dma_err_imr, pb->io.apb_base + DMA_ERR_IMR);

	if (!(pb->io.regs.ipu_imr & IPU_IMR_DMA_ERR_INTR_MASK)) {
		pb->io.regs.ipu_imr |= IPU_IMR_DMA_ERR_INTR_MASK;
		writeq(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
	}

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_disable_dma_channel_error_interrupt(struct paintbox_data *pb,
		unsigned int channel_id)
{
	uint32_t dma_err_imr;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	dma_err_imr = readl(pb->io.apb_base + DMA_ERR_IMR);
	dma_err_imr &= ~(1 << (channel_id + DMA_ERR_IMR_DMA_CHAN_ERR_SHIFT));
	writel(dma_err_imr, pb->io.apb_base + DMA_ERR_IMR);

	if (!dma_err_imr) {
		pb->io.regs.ipu_imr &= ~IPU_IMR_DMA_ERR_INTR_MASK;
		writeq(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
	}

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_enable_mipi_input_interface_error_interrupt(
		struct paintbox_data *pb, unsigned int interface_id)
{
	uint32_t mif_err_imr;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	mif_err_imr = readl(pb->io.apb_base + MIF_ERR_IMR);
	mif_err_imr |= 1 << (interface_id + MIF_ERR_IER_MIF_ERR_SHIFT);
	writel(mif_err_imr, pb->io.apb_base + MIF_ERR_IMR);

	if (!(pb->io.regs.ipu_imr & IPU_IMR_MIF_ERR_INTR_MASK)) {
		pb->io.regs.ipu_imr |= IPU_IMR_MIF_ERR_INTR_MASK;
		writeq(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
	}

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_disable_mipi_input_interface_error_interrupt(
		struct paintbox_data *pb, unsigned int interface_id)
{
	uint32_t mif_err_imr;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	mif_err_imr = readl(pb->io.apb_base + MIF_ERR_IMR);
	mif_err_imr &= ~(1 << (interface_id + MIF_ERR_IER_MIF_ERR_SHIFT));
	writel(mif_err_imr, pb->io.apb_base + MIF_ERR_IMR);

	if (!mif_err_imr) {
		pb->io.regs.ipu_imr &= ~IPU_IMR_MIF_ERR_INTR_MASK;
		writeq(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
	}

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_enable_stp_error_interrupt(struct paintbox_data *pb,
		unsigned int stp_id)
{
	uint32_t stp_index = stp_id_to_index(stp_id);
	uint32_t stp_err_imr;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	stp_err_imr = readl(pb->io.apb_base + STP_ERR_IMR);
	stp_err_imr |= 1 << (stp_index + STP_ERR_IMR_STP_ERR_SHIFT);
	writel(stp_err_imr, pb->io.apb_base + STP_ERR_IMR);

	if (!(pb->io.regs.ipu_imr & IPU_IMR_STP_ERR_INTR_MASK)) {
		pb->io.regs.ipu_imr |= IPU_IMR_STP_ERR_INTR_MASK;
		writeq(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
	}

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}

void paintbox_disable_stp_error_interrupt(struct paintbox_data *pb,
		unsigned int stp_id)
{
	uint32_t stp_index = stp_id_to_index(stp_id);
	uint32_t stp_err_imr;
	unsigned long irq_flags;

	spin_lock_irqsave(&pb->io.io_lock, irq_flags);

	stp_err_imr = readl(pb->io.apb_base + STP_ERR_IMR);
	stp_err_imr &= ~(1 << (stp_index + STP_ERR_ISR_STP_ERR_SHIFT));
	writel(stp_err_imr, pb->io.apb_base + STP_ERR_IMR);

	if (!stp_err_imr) {
		pb->io.regs.ipu_imr &= ~IPU_IMR_STP_ERR_INTR_MASK;
		writeq(pb->io.regs.ipu_imr, pb->io.apb_base + IPU_IMR);
	}

	spin_unlock_irqrestore(&pb->io.io_lock, irq_flags);
}
#endif

#if CONFIG_PAINTBOX_VERSION_MAJOR == 0
bool get_mipi_input_interface_interrupt_state(struct paintbox_data *pb,
		unsigned int interface_id)
{
	return !!(readl(pb->io.apb_base + IPU_ISR) &
			(1ULL << (interface_id + IPU_IMR_MPI_INTR_SHIFT)));
}

bool get_mipi_output_interface_interrupt_state(struct paintbox_data *pb,
		unsigned int interface_id)
{
	return !!(readl(pb->io.apb_base + IPU_ISR) &
			(1ULL << (interface_id + IPU_IMR_MPO_INTR_SHIFT)));
}
#endif

/* All sessions must be released before remove can be called. */
void paintbox_io_apb_remove(struct paintbox_data *pb)
{
	devm_free_irq(&pb->pdev->dev, pb->io.irq, pb);

#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_debug_free_reg_entries(&pb->io.apb_debug);
	paintbox_debug_free_entry(&pb->io.apb_debug);
#endif
}

/* Resets shadows. */
void paintbox_io_apb_post_ipu_reset(struct paintbox_data *pb)
{
	pb->io.regs.ipu_imr = IPU_IMR_DEF;
	pb->io.regs.dma_chan_en = IPU_DMA_CHAN_EN_DEF;
}

int paintbox_io_apb_init(struct paintbox_data *pb)
{
	int ret;

	spin_lock_init(&pb->io.io_lock);

	pb->io.regs.ipu_imr = IPU_IMR_DEF;
	pb->io.regs.dma_chan_en = IPU_DMA_CHAN_EN_DEF;

#ifdef CONFIG_PAINTBOX_DEBUG
	paintbox_debug_create_entry(pb, &pb->io.apb_debug, pb->debug_root,
			"apb", -1, paintbox_dump_io_apb_registers, NULL,
			&pb->io);

	paintbox_debug_create_reg_entries(pb, &pb->io.apb_debug,
			io_apb_reg_names, IO_APB_NUM_REGS,
			paintbox_io_apb_reg_entry_write,
			paintbox_io_apb_reg_entry_read);
#endif

	/* Update the number of available interrupts reported to the user space.
	 * This value is also used to allocate the number of IRQ waiter objects.
	 */
	pb->io.num_interrupts = pb->dma.num_channels + pb->stp.num_stps
			+ NUM_BIF_INTERRUPTS + NUM_MMU_INTERRUPTS;

	if (pb->io_ipu.num_mipi_input_interfaces > 0) {
		/* For the number of interrupts available that is reported to
		 * the user space we want to have an interrupt per MIPI stream
		 * rather than per interface.
		 */
		pb->io.num_interrupts += pb->io_ipu.num_mipi_input_streams;
	}

	if (pb->io_ipu.num_mipi_output_interfaces > 0) {
		/* For the number of interrupts available that is reported to
		 * the user space we want to have an interrupt per MIPI stream
		 * rather than per interface.
		 */
		pb->io.num_interrupts += pb->io_ipu.num_mipi_output_streams;
	}

	pb->io.available_irq_mask = (1ULL << pb->io.num_interrupts) - 1;

	ret = devm_request_irq(&pb->pdev->dev, pb->io.irq,
			paintbox_io_interrupt, IRQF_SHARED, pb->pdev->name, pb);
	if (ret < 0)
		return ret;

	dev_dbg(&pb->pdev->dev, "io_apb: base %p len %lu\n",
			pb->io.apb_base, IO_APB_BLOCK_LEN);

	return 0;
}
