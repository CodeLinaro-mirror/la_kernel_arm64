/*
 * pcie_ep.c - Monhette Hill PCIe EndPoint driver
 *
 * Copyright (C) 2016 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program;
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Marko Bartscherer <marko.bartscherer@intel.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/mnh_pcie_ep.h>
#include <linux/mnh_pcie_reg.h>
#include <linux/mnh_pcie_str.h>
/* #include <asm-generic/page.h> */
#include <linux/mm.h>
#include <linux/rwsem.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/pci.h>
#include <linux/intel-hwio.h>
#include <soc/mnh/mnh-hwio-pcie-ep.h>
#include <soc/mnh/mnh-hwio-pcie-ss.h>
#include <soc/mnh/mnh-hwio-scu.h>
#include <soc/mnh/mnh-trace.h>
#include <linux/pagemap.h>
#include <linux/dma-buf.h>
#include <linux/jiffies.h>
#include <../thermal/mnh-clk.h>


#define VENDOR_ID				0x8086
#define DEVICE_ID				0x3140

/* timeout for msi request to grant, in microseconds */
#define MSI_REQUEST_TIMEOUT 50

#define COMBINE_SG	1

int (*irq_callback)(struct mnh_pcie_irq *irq);
int (*dma_callback)(struct mnh_dma_irq *irq);
struct mnh_pcie_ep_device *pcie_ep_dev;
struct delayed_work msi_work;
struct delayed_work power_state_work;
struct work_struct msi_rx_work, vm0_work, vm1_work, wake_work;
struct mnh_pcie_ep_power_state target_state;
#define DEVICE_NAME "mnh_pcie_ep"
#define CLASS_NAME "pcie_ep"
#define MSI_DELAY (HZ/20) /* TODO: Need to understand what this should be */
#define POWER_DELAY (100*1000) /* delay between L-power state checks, usecs */
#define MAX_STR_COPY	32
uint32_t rw_address_sysfs, rw_size_sysfs;
struct mnh_pcie_irq sysfs_irq;

static int pcie_ll_destroy(struct mnh_dma_ll *ll);
static int pcie_set_l_one(uint32_t enable, uint32_t clkpm);
static void pcie_set_power_mode_state(
	struct mnh_pcie_ep_power_state *power_state);
static void pcie_get_power_mode_state(
	struct mnh_pcie_ep_power_state *power_state);

/* callback when pm state changes */
static pm_callback_t mnh_ep_pm_callback;

#if MNH_PCIE_DEBUG_ENABLE

/* read from pcie cluster register */
static uint32_t pcie_cluster_read(uint64_t address)
{
	uint32_t data;
	uint64_t addr = (uint64_t)pcie_ep_dev->clust_mem + address;

	data = ioread32((uint32_t *) addr);
	return data;
}

/* write to pcie cluster register */

static int pcie_cluster_write(uint64_t address, uint32_t data)
{
	uint64_t addr = (uint64_t)pcie_ep_dev->clust_mem + address;

	iowrite32(data, (uint32_t *) addr);
	return 0;
}

/* read from pcie config register */

static uint32_t pcie_config_read(uint64_t address)
{
	uint32_t data;
	uint64_t addr = (uint64_t)pcie_ep_dev->conf_mem + address;

	data = ioread32((uint32_t *) addr);
	return data;
}

/* write to pcie config register */

static int pcie_config_write(uint64_t address, uint32_t data)
{
	uint64_t addr = (uint64_t)pcie_ep_dev->conf_mem + address;

	iowrite32(data, (uint32_t *) addr);
	return 0;
}

#endif

#ifdef CONFIG_MNH_PCIE_BOOT_TRACE
DEFINE_SPINLOCK(trace_lock);
static void *mnh_scu_base;

/* Read back trace log from SCU scratch register */
static uint32_t mnh_trace_get(void)
{
	uint32_t *reg;
	uint32_t value = -1;	/* All F */

	spin_lock(&trace_lock);

	if (!mnh_scu_base)
		mnh_scu_base = ioremap_nocache(MNH_SCU_PHYS_BASE, MNH_SCU_SIZE);

	if (!mnh_scu_base)
		goto bail;

	reg = (mnh_scu_base + MNH_SCU_TRACE_OFFSET);
	value = readl(reg);

bail:
	spin_unlock(&trace_lock);
	return value;
}

/* Log to SCU scratch register */
void mnh_trace(uint32_t value)
{
	uint32_t *reg;

	spin_lock(&trace_lock);

	if (!mnh_scu_base)
		mnh_scu_base = ioremap_nocache(MNH_SCU_PHYS_BASE, MNH_SCU_SIZE);

	if (!mnh_scu_base)
		goto bail;

	reg = (mnh_scu_base + MNH_SCU_TRACE_OFFSET);
	writel(value, reg);

bail:
	spin_unlock(&trace_lock);
}
EXPORT_SYMBOL(mnh_trace);

#else  /* CONFIG_MNH_PCIE_BOOT_TRACE */

static uint32_t mnh_trace_get(void)
{
	return 0;  /* No implementation */
}

void mnh_trace(uint32_t value)
{
	/* No implementation */
}
EXPORT_SYMBOL(mnh_trace);
#endif  /* CONFIG_MNH_PCIE_BOOT_TRACE */

static int check_sram_init_done(void)
{

	return CSR_INf(PCIE_APP_STS, PCIE_PHY_SRAM_INIT_DONE);
}

static int pcie_link_up(void)
{
	uint32_t data, mask;

	mask = TYPE0_MASK(TYPE0_HDR_STATUS_COMMAND, PCI_TYPE0_IO_EN) |
		TYPE0_MASK(TYPE0_HDR_STATUS_COMMAND, PCI_TYPE0_MEM_SPACE_EN) |
		TYPE0_MASK(TYPE0_HDR_STATUS_COMMAND, PCI_TYPE0_BUS_MASTER_EN);
	data = TYPE0_IN(TYPE0_HDR_STATUS_COMMAND) & mask;
	if (data == 0)
		return 0;
	return 1;
}

static int pcie_link_init(void)
{

	if (!pcie_link_up()) {
		/* Magic value Only used during boot */
		CSR_OUTx(PCIE_GP, 0, 0x0);
		/* program PCIe controller registers
		* not needed at this point in time
		*/

		/* TODO: read eFuses to update device address */
		while (!check_sram_init_done())
			udelay(1);

		/* program PCIe PHY PCS PATCH registers
		*not needed at this point in time
		*/
		CSR_OUTf(PCIE_APP_CTRL, PCIE_PHY_SRAM_LD_DONE, 1);

		/* program PCIe PHY registers not needed
		* at this point in time
		*/
		/* Set BAR sizes */

		TYPE0S_OUT(TYPE0_HDR_BAR2, B8M_BAR);
		TYPE0S_OUT(TYPE0_HDR_BAR4, B4M_BAR);

		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_LTSSM_EN, 1);
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_XFER_PEND, 1);

		/* waiting for magic value */
		//while (CSR_INx(PCIE_GP, 0) == 0)
		//	udelay(1);

		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_XFER_PEND, 0);

	};

	/*enable L1 entry */
	PCIECAP_OUTf(PCIE_CAP_LINK_CONTROL_LINK_STATUS,
		     PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL, 0x2);
	PCIECAP_OUTf(PCIE_CAP_LINK_CAPABILITIES,
		     PCIE_CAP_CLOCK_POWER_MAN, 0x1);
	PCIECAP_OUTf(PCIE_CAP_LINK_CAPABILITIES,
		     PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT, 0x1);
	PCIECAP_OUTf(PCIE_CAP_LINK_CONTROL_LINK_STATUS,
		     PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL, 0x2);
	PCIECAP_OUTf(PCIE_CAP_LINK_CONTROL_LINK_STATUS,
		     PCIE_CAP_EN_CLK_POWER_MAN, 0x1);
	PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CAPABILITY,
			   L1_PMSUB_SUPPORT, 0x1);
	PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CAPABILITY,
			   L1_1_ASPM_SUPPORT, 0x1);
	PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CAPABILITY,
			   L1_2_ASPM_SUPPORT, 0x1);
	PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CAPABILITY,
			   L1_1_PCIPM_SUPPORT, 0x1);
	PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CAPABILITY,
			   L1_2_PCIPM_SUPPORT, 0x1);
	pcie_get_power_mode_state(&pcie_ep_dev->power_state);
	pcie_set_l_one(pcie_ep_dev->power_state.l1state,
		       pcie_ep_dev->power_state.clkpm);

	/* Enable interupts */
	CSR_OUT(PCIE_SS_INTR_EN, PCIE_SS_IRQ_MASK);
	/* Clear all interrupts */
	CSR_OUT(PCIE_SW_INTR_TRIGG, MNH_PCIE_SW_IRQ_CLEAR);

	return  CSR_INx(PCIE_GP, 0);
}

static void pcie_set_low_power(unsigned int enabled)
{
	dev_dbg(pcie_ep_dev->dev, "%s enabled=%d\n", __func__, enabled);
	mnh_axi_clock_gating(enabled);
}

static void force_link_up(void)
{
	pcie_set_low_power(0);

	if (CSR_INf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1))
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 0);
	CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 1);
	CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_XFER_PEND, 1);
}

static void release_link(void)
{
	CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_XFER_PEND, 0);
	CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 0);
}

static int pcie_send_msi_p(uint32_t msi)
{
	uint32_t tc, data, msg;
	int timeout;

	data = CSR_IN(PCIE_MSI_TRIG) &
		CSR_MASK(PCIE_MSI_TRIG, PCIE_VEN_MSI_REQ);
	tc = MNH_TC0 << 13;
	if (pcie_ep_dev->msimode == 2)
		msg = msi << 8;
	else {
		msg = 0x0 << 8;
		CSR_OUTx(PCIE_GP, 3, msi);
	}
	data |= tc | msg;
	CSR_OUT(PCIE_MSI_TRIG, data);
	CSR_OUTf(PCIE_MSI_TRIG, PCIE_VEN_MSI_REQ, 1);

	/* make sure the request was granted, takes 20-40 usecs */
	usleep_range(15, 20);
	for (timeout = 0; timeout < MSI_REQUEST_TIMEOUT; timeout++) {
		if (!CSR_INf(PCIE_MSI_TRIG, PCIE_VEN_MSI_REQ))
			break;

		udelay(1);
	}

	if (CSR_INf(PCIE_MSI_TRIG, PCIE_VEN_MSI_REQ)) {
		dev_err(pcie_ep_dev->dev, "%s: send msi timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}


static int pcie_get_msi_mask(void)
{
	uint32_t data, mask;

	data = MSICAP_INf(MSI_CAP_PCI_MSI_CAP_ID_NEXT_CTRL,
			PCI_MSI_64_BIT_ADDR_CAP);
	if (data) {
	/* 64 bit ADDR used */
		mask = MSICAP_IN(MSI_CAP_MSI_CAP_OFF_10H);
	} else {
	/* 32 bit ADDR used */
		mask = MSICAP_IN(MSI_CAP_MSI_CAP_OFF_0CH);
	}

	return mask;
}

static int pcie_check_msi_mask(uint32_t msi)
{
	uint32_t vmask, mask;

	if (pcie_ep_dev->msimode == 2)
		vmask = 0x1 << (msi);
	else
		vmask = 0x1;
	mask = pcie_get_msi_mask();
	if (mask & vmask) {
		/* need to set pending bit, than start polling */
		if (CSR_IN(PCIE_MSI_PEND) & vmask)
			dev_err(pcie_ep_dev->dev, "PEND MSI overflow\n");
		CSR_OUT(PCIE_MSI_PEND, CSR_IN(PCIE_MSI_PEND) | vmask);
		if (pcie_ep_dev->msimode == 1)
			pcie_ep_dev->pendingmsi = msi;
		cancel_delayed_work(&msi_work);
		schedule_delayed_work(&msi_work, MSI_DELAY);
		return 1;
	}
	return 0;
}

static uint32_t b2h(uint32_t arg)
{
	/* Converts a hex number to a bit 3 -> 0100 */
	if (arg > 0 && (arg < 32))
		return (0x1 << (arg - 1));
	return 0;
}

static void process_pend_msi(uint32_t pend, uint32_t vmask, uint32_t arg)
{
	if ((pend & b2h(arg)) &&
		!(vmask & b2h(arg))) {
		if (pcie_ep_dev->msimode == 2)
			pcie_send_msi_p(arg);
		else {
			pcie_send_msi_p(pcie_ep_dev->pendingmsi);
			pcie_ep_dev->pendingmsi = 0;
		}
		CSR_OUT(PCIE_MSI_PEND, (pend & ~b2h(arg)));
	}
}

static void send_pending_msi(void)
{
	uint32_t vmask, pend;

	vmask = pcie_get_msi_mask();
	pend = CSR_IN(PCIE_MSI_PEND);
	process_pend_msi(pend, vmask, MSG_SEND_M);
	if (pcie_ep_dev->msimode == 2) {
		process_pend_msi(pend, vmask, PET_WATCHDOG);
		process_pend_msi(pend, vmask, CRASH_DUMP);
		process_pend_msi(pend, vmask, BOOTSTRAP_SET);
		process_pend_msi(pend, vmask, APPDEFINED_1_M);
	}
}

static void pcie_msi_worker(struct work_struct *work)
{
	uint32_t pend;

	force_link_up();
	send_pending_msi();
	pend = CSR_IN(PCIE_MSI_PEND);
	if (pend) {
		cancel_delayed_work(&msi_work);
		schedule_delayed_work(&msi_work, MSI_DELAY);
	}

}

static void pcie_power_state_worker(struct work_struct *work)
{
	uint32_t l1_state, l1_sub_state;

	/* Check the l1 state and turn PCIe power savings on when we enter L1 */
	l1_state = CSR_INf(PCIE_APP_STS, PCIE_PM_LINKST_IN_L1);
	l1_sub_state = CSR_INf(PCIE_APP_STS, PCIE_PM_LINKST_IN_L1sub);
	dev_dbg(pcie_ep_dev->dev, "%s l1=%d, l1sub=%d\n", __func__,
		l1_state, l1_sub_state);

	if (!l1_state) {
		/* repeat the loop */
		cancel_delayed_work(&power_state_work);
		schedule_delayed_work(&power_state_work,
			usecs_to_jiffies(POWER_DELAY));
	} else {
		if (pcie_ep_dev->power_state.l1state != target_state.l1state ||
		    pcie_ep_dev->power_state.clkpm != target_state.clkpm) {
			pcie_ep_dev->power_state.l1state = target_state.l1state;
			pcie_ep_dev->power_state.clkpm = target_state.clkpm;
			pcie_set_l_one(target_state.l1state,
				       target_state.clkpm);

		}
		/* Enable PCIe power savings */
		if (target_state.axi_gating) {
			pcie_set_low_power(target_state.axi_gating);
			pcie_ep_dev->power_state.axi_gating =
				target_state.axi_gating;
		}

	}


}

static void pcie_set_msi_mode(void)
{
/*
 * Force single MSI for interop with MSM8998 kernel, which doesn't
 * implement multiple MSIs for an EP properly.	Google bug b/31716267.
 */
#ifdef CONFIG_MNH_PCIE_MULTIPLE_MSI
	if (MSICAP_INf(MSI_CAP_PCI_MSI_CAP_ID_NEXT_CTRL,
			PCI_MSI_MULTIPLE_MSG_EN) == 0)
		pcie_ep_dev->msimode = 1;
	else
		pcie_ep_dev->msimode = 2;
#else
		pcie_ep_dev->msimode = 1;
#endif
}


static int pcie_send_msi(uint32_t msi)
{
	uint32_t pend;

	if (pcie_ep_dev->msimode == 0)
		pcie_set_msi_mode();
	if ((msi < MSI_START) || (msi > MSI_END))
		return -EINVAL;
	force_link_up();
	if (pcie_check_msi_mask(msi))
		return -EINVAL; /* MSI is masked */
	pend = CSR_IN(PCIE_MSI_PEND);
	if (pend)
		send_pending_msi();
	pcie_send_msi_p(msi);
	return 0;
}

static int pcie_send_vm(uint32_t vm)
{
	uint32_t data;

	force_link_up();
	data = (VENDOR_ID << 16) | VM_BYTE_8_9;
	CSR_OUT(PCIE_TX_MSG_PYLD_REG1, data);
	HW_OUT(pcie_ep_dev->clust_mem,  PCIE_SS, PCIE_TX_MSG_PYLD_REG1, data);
	CSR_OUT(PCIE_TX_MSG_PYLD_REG0, vm);
	HW_OUT(pcie_ep_dev->clust_mem,  PCIE_SS, PCIE_TX_MSG_PYLD_REG0, vm);
	CSR_OUTf(PCIE_TX_MSG_REQ, PCIE_TX_MSG_TAG, MNH_PCIE_VM_TAG);
	CSR_OUTf(PCIE_TX_MSG_REQ, PCIE_TX_MSG_CODE, MNH_PCIE_VM_MSG_CODE_VD);
	CSR_OUTf(PCIE_TX_MSG_REQ, PCIE_TX_MSG_REQ, 1);
	/*
	*data = (MNH_PCIE_VM_TAG << 24) | (MNH_PCIE_VM_MSG_CODE_VD << 16)
	*	| MNH_PCIE_TX_MSG_REQ;
	*pcie_cluster_write(MNH_PCIE_TX_MSG_REQ_REG, data);
	*/
	return 0;
}

static int pcie_send_ltr(uint32_t ltr)
{

	force_link_up();
	CSR_OUT(PCIE_LTR_MSG_LATENCY, ltr);
	CSR_OUTf(PCIE_LTR_MSG_CTRL, PCIE_LTR_MSG_REQ, 1);
	return 0;
}

/* Interrupt Service routines */

static int handle_vm(int vm)
{
	struct mnh_pcie_irq inc;

	inc.msi_irq = 0;
	inc.pcie_irq = 0;
	inc.vm = 0;

	if (vm == 0) {
		if (CSR_INf(PCIE_RX_VMSG0_ID, PCIE_RADM_MSG0_REQ_ID)
				& MNH_VALID_VM) {
			vm = CSR_IN(PCIE_RX_MSG0_PYLD_REG0);
			inc.vm = vm;
			dev_dbg(pcie_ep_dev->dev, "Vendor msg %x rcvd\n", vm);
			irq_callback(&inc);
			CSR_OUTf(PCIE_RX_VMSG0_ID, PCIE_RADM_MSG0_REQ_ID,
				MNH_CLEAR_VM);
		}
	} else {
		if (CSR_INf(PCIE_RX_VMSG1_ID, PCIE_RADM_MSG1_REQ_ID)
				& MNH_VALID_VM) {
			vm = CSR_IN(PCIE_RX_MSG0_PYLD_REG1);
			inc.vm = vm;
			dev_dbg(pcie_ep_dev->dev, "Vendor msg %x rcvd\n", vm);
			irq_callback(&inc);
			CSR_OUTf(PCIE_RX_VMSG1_ID, PCIE_RADM_MSG1_REQ_ID,
				MNH_CLEAR_VM);
		}
	}
	return 0;
}

static void dma_rx_handler(void)
{
	uint32_t data, data1, i;
	struct mnh_dma_irq dma_event;

	if (dma_callback != NULL) {
		data1 = CSR_INx(PCIE_GP, 2);
		data = data1;
		dev_dbg(pcie_ep_dev->dev, "DMA msg %x rcvd\n", data);
		if (data & DMA_READ_DONE_MASK) {
			dma_event.type = MNH_DMA_READ;
			dma_event.status = MNH_DMA_DONE;
			data = data & DMA_READ_DONE_MASK;
			i = 0;
			while ((data != 1) && (i < 8)) {
				data = data >> 1;
				i++;
				};
			dma_event.channel = i;
			dma_callback(&dma_event);
			CSR_OUTx(PCIE_GP, 2, (data & ~DMA_READ_DONE_MASK));
		}
		data = data1;
		if (data & DMA_READ_ABORT_MASK) {
			dma_event.type = MNH_DMA_READ;
			dma_event.status = MNH_DMA_ABORT;
			data = (data & DMA_READ_ABORT_MASK) >> 8;
			i = 0;
			while ((data != 1) && (i < 8)) {
				data = data >> 1;
				i++;
				};
			dma_event.channel = i;
			dma_callback(&dma_event);
			CSR_OUTx(PCIE_GP, 2, (data & ~DMA_READ_ABORT_MASK));
		}
		data = data1;
		if (data & DMA_WRITE_DONE_MASK) {
			dma_event.type = MNH_DMA_WRITE;
			dma_event.status = MNH_DMA_DONE;
			data = (data & DMA_WRITE_DONE_MASK) >> 16;
			i = 0;
			while ((data != 1) && (i < 8)) {
				data = data >> 1;
				i++;
				};
			dma_event.channel = i;
			dma_callback(&dma_event);
			CSR_OUTx(PCIE_GP, 2, (data & ~DMA_WRITE_DONE_MASK));
		}
		data = data1;
		if (data & DMA_WRITE_ABORT_MASK) {
			dma_event.type = MNH_DMA_WRITE;
			dma_event.status = MNH_DMA_ABORT;
			data = (data & DMA_WRITE_ABORT_MASK) >> 24;
			i = 0;
			while ((data != 1) && (i < 8)) {
				data = data >> 1;
				i++;
				};
			dma_event.channel = i;
			dma_callback(&dma_event);
			CSR_OUTx(PCIE_GP, 2, (data & ~DMA_WRITE_ABORT_MASK));
		}
	}
}
static void msi_rx_worker(struct work_struct *work)
{
	uint32_t apirq;
	struct mnh_pcie_irq inc;

	inc.msi_irq = 0;
	inc.pcie_irq = 0;
	inc.vm = 0;

	apirq = CSR_IN(PCIE_SW_INTR_TRIGG);
	while (apirq != 0) {
		dev_dbg(pcie_ep_dev->dev, "AP IRQ %x received\n", apirq);

		if (apirq & (0x1 << MSG_SEND_I)) {
			if (irq_callback != NULL) {
				inc.msi_irq =  MSG_SEND_I;
				irq_callback(&inc);
			}
			CSR_OUT(PCIE_SW_INTR_TRIGG, (MNH_PCIE_SW_IRQ_CLEAR
				& (0x1 << MSG_SEND_I)));
		}
		if (apirq & (0x1 << DMA_STATUS)) {
			dma_rx_handler();
			CSR_OUT(PCIE_SW_INTR_TRIGG, (MNH_PCIE_SW_IRQ_CLEAR
				& (0x1 << DMA_STATUS)));
		}
		if (apirq & (0x1 << APPDEFINED_1_I)) {
			if (irq_callback != NULL) {
				inc.msi_irq =  APPDEFINED_1_I;
				irq_callback(&inc);
			}
			CSR_OUT(PCIE_SW_INTR_TRIGG, (MNH_PCIE_SW_IRQ_CLEAR
				& (0x1 << APPDEFINED_1_I)));
		}

		/* Clear unused IRQ bits just in case, don't block new IRQs */
		CSR_OUT(PCIE_SW_INTR_TRIGG, MNH_PCIE_SW_IRQ_CLEAR &
                        ~((0x1 << MSG_SEND_I) |
                          (0x1 << DMA_STATUS) |
                          (0x1 << APPDEFINED_1_I)));
		apirq = CSR_IN(PCIE_SW_INTR_TRIGG);
	}
	CSR_OUT(PCIE_SW_INTR_EN, MNH_PCIE_SW_IRQ_CLEAR);
}

static void vm1_worker(struct work_struct *work)
{
	handle_vm(1);
}

static void vm0_worker(struct work_struct *work)
{
	handle_vm(0);
}


static irqreturn_t pcie_handle_cluster_irq(int irq, void *dev_id)
{
	uint32_t pcieirq;

		pcieirq = CSR_IN(PCIE_SS_INTR_STS);
		if (pcieirq != 0) {
			if (pcieirq & MNH_PCIE_MSI_SENT) {
				dev_dbg(pcie_ep_dev->dev, "MSI Sent\n");
				release_link();
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_MSI_SENT));
			}
			if (pcieirq & MNH_PCIE_VMSG_SENT) {
				dev_dbg(pcie_ep_dev->dev, "VM Sent\n");
				release_link();
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_VMSG_SENT));
			}
			if ((pcieirq & MNH_PCIE_VMSG1_RXD)
				& (irq_callback != NULL)) {
				dev_dbg(pcie_ep_dev->dev, "VM1 received\n");
				schedule_work(&vm1_work);
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_VMSG1_RXD));
			}
			if ((pcieirq & MNH_PCIE_VMSG0_RXD)
				& (irq_callback != NULL)) {
				dev_dbg(pcie_ep_dev->dev, "VM2 received\n");
				schedule_work(&vm0_work);
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_VMSG0_RXD));
			}
			if (pcieirq & MNH_PCIE_LINK_EQ_REQ_INT) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_LINK_EQ_REQ_INT received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_LINK_EQ_REQ_INT));
			}
			if (pcieirq & MNH_PCIE_LINK_REQ_RST_NOT) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_LINK_REQ_RST_NOT received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_LINK_REQ_RST_NOT));
			}
			if (pcieirq & MNH_PCIE_LTR_SENT) {
				release_link();
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_LTR_SENT));
				}
			if (pcieirq & MNH_PCIE_COR_ERR) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_COR_ERR received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_COR_ERR));
			}
			if (pcieirq & MNH_PCIE_NONFATAL_ERR) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_NONFATAL_ERR received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_NONFATAL_ERR));
			}
			if (pcieirq & MNH_PCIE_FATAL_ERR) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_FATAL_ERR received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_FATAL_ERR));
				}
			if (pcieirq & MNH_PCIE_RADM_MSG_UNLOCK) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_RADM_MSG_UNLOCK received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_RADM_MSG_UNLOCK));
			}
			if (pcieirq & MNH_PCIE_PM_TURNOFF) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_PM_TURNOFF received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_PM_TURNOFF));
			}
			if (pcieirq & MNH_PCIE_RADM_CPL_TIMEOUT) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_RADM_CPL_TIMEOUT received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_RADM_CPL_TIMEOUT));
			}
			if (pcieirq & MNH_PCIE_TRGT_CPL_TIMEOUT) {
				dev_dbg(pcie_ep_dev->dev,
				"MNH_PCIE_TRGT_CPL_TIMEOUT received\n");
				CSR_OUT(PCIE_SS_INTR_STS,
					(MNH_PCIE_TRGT_CPL_TIMEOUT));
				}
		}
	/* return interrupt handled */
	return IRQ_HANDLED;
}

static irqreturn_t pcie_handle_sw_irq(int irq, void *dev_id)
{
	CSR_OUT(PCIE_SW_INTR_EN, (~MNH_PCIE_SW_IRQ_CLEAR));
	schedule_work(&msi_rx_work);

	/* return interrupt handled */
	return IRQ_HANDLED;
}

static int pcie_set_rb_base(uint32_t base)
{
	uint32_t msi = BOOTSTRAP_SET;

	CSR_OUTx(PCIE_GP, 1, base);
	pcie_send_msi(msi);
	return 0;
}

static int pcie_read_data(uint8_t *buff, uint32_t size, uint64_t adr)
{

	uint64_t addr = (uint64_t)pcie_ep_dev->outb_mem + adr;

	memcpy(buff, (char *) addr, size);
	dev_dbg(pcie_ep_dev->dev, "finished copy\n");
	return 0;
}

static int pcie_write_data(uint8_t *buff, uint32_t size, uint64_t adr)
{

	uint64_t addr = (uint64_t)pcie_ep_dev->outb_mem + adr;

	memcpy((char *) addr, buff, size);
	dev_dbg(pcie_ep_dev->dev, "finished copy\n");
	return 0;
}

static void pcie_set_power_mode_state(
	struct mnh_pcie_ep_power_state *power_state)
{
	dev_dbg(pcie_ep_dev->dev, "%s axi_gating=%d, l1state=%d, clkpm=%d\n",
		 __func__,
		power_state->axi_gating,
		power_state->l1state,
		power_state->clkpm);
	pcie_set_low_power(0);
	SCUS_OUTf(GP_POWER_MODE, PCIE_AXI_CG_EN,
		power_state->axi_gating);
	SCUS_OUTf(GP_POWER_MODE, PCIE_L1_2_EN,
		power_state->l1state);
	SCUS_OUTf(GP_POWER_MODE, PCIE_CLKPM_EN,
		power_state->clkpm);
}

static void pcie_get_power_mode_state(
	struct mnh_pcie_ep_power_state *power_state)
{
	pcie_set_low_power(0);
	power_state->axi_gating = SCUS_INf(GP_POWER_MODE, PCIE_AXI_CG_EN);
	power_state->l1state = SCUS_INf(GP_POWER_MODE, PCIE_L1_2_EN);
	power_state->clkpm = SCUS_INf(GP_POWER_MODE, PCIE_CLKPM_EN);
	dev_dbg(pcie_ep_dev->dev, "%s axi_gating=%d, l1state=%d, clkpm=%d\n",
		__func__,
		power_state->axi_gating,
		power_state->l1state,
		power_state->clkpm);
}

static int pcie_set_l_one(uint32_t enable, uint32_t clkpm)
{
	dev_dbg(pcie_ep_dev->dev, "%s:%d enable=%d clkpm=%d\n",
		__func__, __LINE__, enable, clkpm);

	pcie_set_low_power(0);

	if (clkpm == 1) {
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_CLK_PM_EN, 1);
	} else {
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_CLK_PM_EN, 0);
	}
	if (enable == 1) {
		PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CONTROL1,
				L1_1_ASPM_EN, 0x1);
		PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CONTROL1,
				L1_2_ASPM_EN, 0x1);
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_ENTRY_L1, 1);
	} else {
		PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CONTROL1,
					L1_1_ASPM_EN, 0x0);
		PCIECAP_L1SUB_OUTf(L1SUB_CAP_L1SUB_CONTROL1,
					L1_2_ASPM_EN, 0x0);
		CSR_OUTf(PCIE_APP_CTRL, PCIE_APP_REQ_EXIT_L1, 1);
	}
	/* send an msi just to wake up AP and notice the L1.2 state change */
	mnh_send_msi(PET_WATCHDOG);
	return 0;
}


static void pcie_wake_worker(struct work_struct *work) {
	pcie_set_low_power(0);

	/* read pcie target state from the gp reg */
	pcie_get_power_mode_state(&target_state);
	cancel_delayed_work(&power_state_work);
	schedule_delayed_work(&power_state_work,
		usecs_to_jiffies(POWER_DELAY));
	dev_dbg(pcie_ep_dev->dev, "%s:%d curstate=%d target state=%d\n",
		__func__, __LINE__,
		pcie_ep_dev->power_state.l1state,
		target_state.l1state);
}

static irqreturn_t pcie_handle_wake_irq(int irq, void *dev_id)
{
	CSR_OUTf(PCIE_APP_STS, PCIE_WAKE_EVENT, 0x1);
	schedule_work(&wake_work);

	return IRQ_HANDLED;
}

static enum dma_data_direction mnh_to_dma_dir(enum mnh_dma_chan_dir_t mnh_dir)
{
	/*
	 * This is the EP version, so the DMA direction is opposite to
	 * AP version.
	 */
	return (mnh_dir == DMA_AP2EP) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
}

/**
 * Convert Linux scatterlist to array of entries used by PCIe EP DMA engine
 * @param[in] sc_list   Scatter gather list for DMA buffer
 * @param[in] count  Number of entries in scatterlist
 * @param[in] off  Offset, in bytes, from which to start transfer.
 * @param[in] size  Size, in bytes, of transfer. This represents actual data.
 * @param[in] width Width, in bytes. Represents a line of data.
 * @param[in] stride Stride, in bytes. Must be equal or greater to width.
 * @param[out] sg  Array generated dma addresses and length.
 * @param[in] maxsg  Allocated max array number of the sg
 * @return a count of sg entries used on success
 *         -EINVAL if exceeding maxsg
 */
static int scatterlist_to_mnh_sg(struct scatterlist *sc_list, int count,
	uint32_t off, uint32_t size, uint32_t width, uint32_t stride,
	struct mnh_sg_entry *sg, size_t maxsg)
{
	struct scatterlist *in_sg;
	int i, u;
	uint32_t sc_base = 0; /* Base of an sc segment */
	uint32_t sc_end = 0; /* End of the sc segment */
	uint32_t sc_off = off; /* Offset where data is written */
	uint32_t size_rem = size; /* Overall remaining size */
	uint32_t width_rem = width; /* Current line remaining width */
	uint32_t in_sc_off = 0; /* Used to calculate paddr */
	uint32_t in_sc_rem = 0; /* Leftover size in an sc segment */

	if (stride < width) {
		dev_err(pcie_ep_dev->dev,
			"stride (%0d )must be greater or equal to width (%0d)\n",
			stride, width);
		return -EINVAL;
	}

	if (width > size) {
		dev_err(pcie_ep_dev->dev,
			"width(%0d) must be less or equal to size(%0d)\n",
			width, size);
		return -EINVAL;
	}

	i = 0;	/* iterator of *sc_list */
	u = 0;	/* iterator of *sg */

	for_each_sg(sc_list, in_sg, count, i) {
		/* Last entry is reserved for the NULL terminator */
		if (u >= (maxsg - 1)) {
			dev_err(pcie_ep_dev->dev, "maxsg exceeded\n");
			return -EINVAL;
		}

		sc_end = sc_base + sg_dma_len(in_sg);
		while ((sc_off < sc_end) && size_rem) {
			in_sc_off = sc_off - sc_base;
			in_sc_rem = sc_end - sc_off;
			sg[u].paddr = sg_dma_address(in_sg) + in_sc_off;
			sg[u].size = width_rem < in_sc_rem ? width_rem :
								in_sc_rem;
			width_rem -= sg[u].size;
			size_rem -= sg[u].size;
			sc_off += sg[u].size;

			if (width_rem == 0) {
			    width_rem = size_rem > width ? width : size_rem;
			    sc_off += stride - width;
			}
			dev_dbg(pcie_ep_dev->dev, "sg[%d] : Address %pa , length %zu\n",
	                        u, &sg[u].paddr, sg[u].size);

#ifdef COMBINE_SG
			if ((u > 0) && (sg[u-1].paddr + sg[u-1].size ==
				sg[u].paddr)) {
				sg[u-1].size = sg[u-1].size + sg[u].size;
				sg[u].size = 0;
			} else {
				u++;
			}
#else
			u++;
#endif
		}

		sc_base = sc_end;

		if (!size_rem)
			break;
	}


	if (size_rem) {
		dev_err(pcie_ep_dev->dev,
			"%s: transfer oob: off %u, size %u, sc list size %u\n",
			__func__, off, size, sc_end);
		return -EINVAL;
	}

	memset(&sg[u], 0, sizeof(sg[0]));
	sg[u].paddr = 0x0;	/* list terminator value */
	u++;

	dev_dbg(pcie_ep_dev->dev, "SGL with %d/%d entries\n", u, i);

	return u;
}

/**
 * Import dma_buf (from ION buffer)
 * @param[in] fd   Handle of dma_buf passed from user
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries
 * @return 0        on SUCCESS
 *         negative on failure
 */
static int mnh_sg_import_dma_buf(int fd, struct mnh_sg_list *sgl)
{
	int ret;

	sgl->dma_buf = dma_buf_get(fd);
	if (IS_ERR(sgl->dma_buf)) {
		ret = PTR_ERR(sgl->dma_buf);
		dev_err(pcie_ep_dev->dev,
				"%s: failed to get dma_buf, err %d\n",
				__func__, ret);
		return ret;
	}

	sgl->attach = dma_buf_attach(sgl->dma_buf, pcie_ep_dev->dev);
	if (IS_ERR(sgl->attach)) {
		ret = PTR_ERR(sgl->attach);
		dev_err(pcie_ep_dev->dev,
				"%s: failed to attach dma_buf, err %d\n",
				__func__, ret);
		goto err_put;
	}

	sgl->sg_table = dma_buf_map_attachment(sgl->attach,
						mnh_to_dma_dir(sgl->dir));
	if (IS_ERR(sgl->sg_table)) {
		ret = PTR_ERR(sgl->sg_table);
		dev_err(pcie_ep_dev->dev,
				"%s: failed to map dma_buf, err %d\n",
				__func__, ret);
		goto err_detach;
	}

	return 0;

err_detach:
	dma_buf_detach(sgl->dma_buf, sgl->attach);
err_put:
	dma_buf_put(sgl->dma_buf);
	return ret;
}

/**
 * API to build a scatter-gather list for multi-block DMA transfer for a
 * dma_buf
 * @param[in] fd   Handle of dma_buf passed from user
 * @param[in] off  Offset within DMA buffer from which transfer should start.
 * @param[in] size Size, in bytes, of transfer.
 * @param[out] sg  Array of maxsg pointers to struct mnh_sg_entry, allocated
 *			and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries.
 * @return 0        on SUCCESS
 *         negative on failure
 */
int mnh_sg_retrieve_from_dma_buf(int fd, uint32_t off, uint32_t size,
		uint32_t width, uint32_t stride, struct mnh_sg_entry **sg,
		struct mnh_sg_list *sgl)
{
	int ret;
	size_t maxsg;

	/* Retrieve sg_table from dma_buf framework */
	ret = mnh_sg_import_dma_buf(fd, sgl);
	if (ret)
		return ret;

	/* Use sg_table->sgl as our sc_list */
	sgl->sc_list = sgl->sg_table->sgl;
	sgl->n_num = sgl->sg_table->nents;

	/*
	 * The driver assumes either ION userspace code (e.g. Camera HAL)
	 * or dma_buf provider has handled cache correctly.
	 */
	/* dma_sync_sg_for_device(pcie_ep_dev->dev, sgl->sc_list,
				sgl->n_num, mnh_to_dma_dir(sgl->dir)); */

	/*
	 * Allocate enough for one entry per sc_list entry, plus end of list.
	 * Normally the maximum number of DMA entries would be simply,
	 * equal or less than "sgl->num+1" (including null entry). However we
	 * add the term "(size-1)/width" to account that striding may introduce
	 * more DMA entries per sgl segment. Consider the following example:
	 * sgl->n_num=1 sgl->len=1024 size=16 width=4 stride=8 offset=0
	 * Theoretically would need max_sg = 5 (4 dma entries + 1 null)
	 * With the (size-1)/width the formula now gives us:
	 * max_sg=1+(15/4)+1=5 which is the correct number of entries.
	 * On extreme cases the formula will allocate almost twice the elements,
	 * needed, but this is not a common case.
	 */
	maxsg = sgl->n_num + ((size-1)/width) + 1;
	*sg = kcalloc(maxsg, sizeof(struct mnh_sg_entry), GFP_KERNEL);
	if (!(*sg)) {
		mnh_sg_release_from_dma_buf(sgl);
		return -ENOMEM;
	}

	dev_dbg(pcie_ep_dev->dev,
		"Enter %s: n_num:%d maxsg:%zu off:%d size:%d width:%d stride:%d\n",
		 __func__, sgl->n_num, maxsg, off, size, width, stride);

	/* Convert sc_list to a Synopsys compatible linked-list */
	sgl->length = scatterlist_to_mnh_sg(sgl->sc_list, sgl->n_num, off, size,
					width, stride, *sg, maxsg);
	if (IS_ERR(&sgl->length)) {
		kfree((*sg));
		*sg = NULL;
		ret = PTR_ERR(&sgl->length);
		mnh_sg_release_from_dma_buf(sgl);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(mnh_sg_retrieve_from_dma_buf);

/**
 * API to release a scatter-gather list for a dma_buf
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		mnh_sg_retrieve_from_dma_buf
 * @return 0 for SUCCESS
 */
int mnh_sg_release_from_dma_buf(struct mnh_sg_list *sgl)
{
	/* dma_sync_sg_for_cpu(pcie_ep_dev->dev, sgl->sc_list,
				sgl->n_num, mnh_to_dma_dir(sgl->dir)); */
	dma_buf_unmap_attachment(sgl->attach, sgl->sg_table,
						mnh_to_dma_dir(sgl->dir));
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	sgl->length = 0;
	dma_buf_detach(sgl->dma_buf, sgl->attach);
	dma_buf_put(sgl->dma_buf);
	return 0;
}
EXPORT_SYMBOL(mnh_sg_release_from_dma_buf);

int pcie_sg_build(void *dmadest, size_t size, struct mnh_sg_entry *sg,
			struct mnh_sg_list *sgl, uint32_t maxsg)
{

	struct scatterlist *in_sg;
	int i, u, z, fp_offset, count;
	int n_num, p_num;
	dma_addr_t test_addr;
	int test_len;
	int first_page, last_page;

	/* page num calculation */
	first_page = ((unsigned long) dmadest & PAGE_MASK) >> PAGE_SHIFT;
	last_page = (((unsigned long) dmadest + size - 1) & PAGE_MASK)
			>> PAGE_SHIFT;
	fp_offset = (unsigned long) dmadest & ~PAGE_MASK;
	p_num = last_page - first_page + 1;

	sgl->mypage = kcalloc(p_num, sizeof(struct page *), GFP_KERNEL);
	if (!sgl->mypage) {
		sgl->n_num = 0;
		sgl->length = 0;
		dev_err(pcie_ep_dev->dev, "failed to assign pages\n");
		return -EINVAL;
	}
	sgl->sc_list = kcalloc(p_num, sizeof(struct scatterlist), GFP_KERNEL);
	if (!sgl->sc_list) {
		sgl->n_num = 0;
		sgl->length = 0;
		kfree(sgl->mypage);
		dev_err(pcie_ep_dev->dev, "failed to assign sc_list\n");
		return -EINVAL;
	}
	down_read(&current->mm->mmap_sem);
	n_num = get_user_pages_remote(current, current->mm,
			(unsigned long) dmadest, p_num, 1, 1, sgl->mypage,
				NULL);
	up_read(&current->mm->mmap_sem);
	if (n_num < 0) {
		sgl->n_num = 0;
		sgl->length = 0;
		kfree(sgl->sc_list);
		kfree(sgl->mypage);
		return -EINVAL;
	}
	if (n_num < maxsg) {
		sg_init_table(sgl->sc_list, n_num);
		if (n_num == 1)
			sg_set_page(sgl->sc_list, *(sgl->mypage),
					size, fp_offset);
		else {
			sg_set_page(sgl->sc_list, *(sgl->mypage),
					PAGE_SIZE - fp_offset, fp_offset);
			for (i = 1; i < n_num-1; i++)
				sg_set_page(sgl->sc_list + i,
				*(sgl->mypage + i), PAGE_SIZE, 0);
			sg_set_page(sgl->sc_list + (n_num - 1),
				*(sgl->mypage + (n_num - 1)),
				size - (PAGE_SIZE - fp_offset)
				- ((n_num-2)*PAGE_SIZE), 0);
		}
		count = dma_map_sg(pcie_ep_dev->dev, sgl->sc_list,
				n_num, mnh_to_dma_dir(sgl->dir));
		i = 0;
		u = 0;
		for_each_sg(sgl->sc_list, in_sg, count, i) {
			if (u < maxsg) {
				sg[u].paddr = sg_dma_address(in_sg);
				sg[u].size = sg_dma_len(in_sg);
				test_addr = sg_dma_address(in_sg);
				test_len = sg_dma_len(in_sg);
#ifdef COMBINE_SG
				if ((u > 0) && (sg[u-1].paddr + sg[u-1].size ==
					sg[u].paddr)) {
					sg[u-1].size = sg[u-1].size
						+ sg[u].size;
				} else {
					u++;
				}
#else
				u++;
#endif
			} else {
				dev_err(pcie_ep_dev->dev, "maxsg exceeded\n");
				dma_unmap_sg(pcie_ep_dev->dev,
					sgl->sc_list, n_num, mnh_to_dma_dir(sgl->dir));
				sgl->n_num = 0;
				sgl->length = 0;
				for (z = 0; z < n_num; z++)
					put_page(*(sgl->mypage + z));
				kfree(sgl->mypage);
				kfree(sgl->sc_list);
				return -EINVAL;
			}

		}
		sg[u].paddr = 0x0;
	} else {
		dev_err(pcie_ep_dev->dev, "maxsg exceeded\n");
		dma_unmap_sg(pcie_ep_dev->dev, sgl->sc_list,
			sgl->n_num, mnh_to_dma_dir(sgl->dir));
		sgl->n_num = 0;
		sgl->length = 0;
		for (z = 0; z < n_num; z++)
			put_page(*(sgl->mypage + z));
		kfree(sgl->mypage);
		kfree(sgl->sc_list);
		return -EINVAL;
	}
	sgl->n_num = n_num;
	sgl->length = u;
	return 0;
}

static int pcie_sg_sync(struct mnh_sg_list *sgl)
{
	dma_sync_sg_for_cpu(pcie_ep_dev->dev, sgl->sc_list,
			sgl->n_num, mnh_to_dma_dir(sgl->dir));
	return 0;
}


static int pcie_sg_destroy(struct mnh_sg_list *sgl)
{
	int i;
	struct page *page;

	//dma_sync_sg_for_cpu(pcie_ep_dev->dev, sgl->sc_list,
	//		sgl->n_num, DMA_BIDIRECTIONAL);
	dma_unmap_sg(pcie_ep_dev->dev, sgl->sc_list,
			sgl->n_num, mnh_to_dma_dir(sgl->dir));
	for (i = 0; i < sgl->n_num; i++) {
		page = *(sgl->mypage + i);
		/* Mark page as dirty before releasing the pages. */
		if (!PageReserved(page))
			SetPageDirty(page);
		put_page(page);
	}
	kfree(sgl->mypage);
	sgl->mypage = NULL;
	kfree(sgl->sc_list);
	sgl->sc_list = NULL;
	sgl->n_num = 0;
	return 0;
}

static int pcie_ll_build(struct mnh_sg_entry *src_sg,
			struct mnh_sg_entry *dst_sg, struct mnh_dma_ll *ll)
{
	struct mnh_dma_ll_element *ll_element, *tmp_element;
	struct mnh_sg_entry sg_dst, sg_src;
	dma_addr_t dma;
	int i, s, u;

	ll_element = dma_alloc_coherent(pcie_ep_dev->dev,
		DMA_LL_LENGTH *sizeof(struct mnh_dma_ll_element), &dma, GFP_KERNEL);
	ll->size = 0;
	ll->ll_element[0] = ll_element;
	ll->dma[0] = dma;
	if (!ll_element) {
		dev_err(pcie_ep_dev->dev, "LL alloc failed \n");
		return -EINVAL;
		}
	i = 0;
	s = 0;
	u = 0;
	sg_src = src_sg[i];
	sg_dst = dst_sg[s];
	dev_dbg(pcie_ep_dev->dev, "LL checkpoint 2\n");
	if ((sg_src.paddr == 0x0) || (sg_dst.paddr == 0x0)) {
		dev_err(pcie_ep_dev->dev, "Input lists invalid\n");
		return -EINVAL;
	}
	while ((sg_src.paddr != 0x0) && (sg_dst.paddr != 0x0)) {
		if (sg_src.size == sg_dst.size) {
			ll_element[u].header = LL_DATA_ELEMENT;
			ll_element[u].size = sg_src.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			i++;
			s++;
			sg_src = src_sg[i];
			sg_dst = dst_sg[s];
		} else if (sg_src.size > sg_dst.size) {
			ll_element[u].header = LL_DATA_ELEMENT;
			ll_element[u].size = sg_dst.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			sg_src.paddr = sg_src.paddr + sg_dst.size;
			sg_src.size = sg_src.size - sg_dst.size;
			s++;
			sg_dst = dst_sg[s];
		} else {
			ll_element[u].header = LL_DATA_ELEMENT;
			ll_element[u].size = sg_src.size;
			ll_element[u].sar_low = LOWER(sg_src.paddr);
			ll_element[u].sar_high = UPPER(sg_src.paddr);
			ll_element[u].dar_low = LOWER(sg_dst.paddr);
			ll_element[u].dar_high = UPPER(sg_dst.paddr);
			sg_dst.paddr = sg_dst.paddr + sg_src.size;
			sg_dst.size = sg_dst.size - sg_src.size;
			i++;
			sg_src = src_sg[i];
		}
		u++;
		if (u == DMA_LL_LENGTH -1) {
			ll_element[u].header = LL_LINK_ELEMENT;
			if ((sg_src.paddr == 0x0) || (sg_dst.paddr == 0x0)) {
				ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
				ll_element[u].header = LL_LAST_LINK_ELEMENT;
				ll_element[u].sar_low =
					LOWER((uint64_t) ll->dma[0]);
				ll_element[u].sar_high =
					UPPER((uint64_t) ll->dma[0]);
				return 0;
			}
			if (ll->size >= (MNH_MAX_LL_ELEMENT-1)) {
				dev_err(pcie_ep_dev->dev, "Out of dma elements\n");
				ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
				ll_element[u].header = LL_LAST_LINK_ELEMENT;
				ll_element[u].sar_low =
					LOWER((uint64_t) ll->dma[0]);
				ll_element[u].sar_high =
					UPPER((uint64_t) ll->dma[0]);
				pcie_ll_destroy(ll);
				return -EINVAL;
			}
			tmp_element = dma_alloc_coherent(pcie_ep_dev->dev,
			DMA_LL_LENGTH * sizeof(struct mnh_dma_ll_element),
				&dma, GFP_KERNEL);
			if (!tmp_element) {
				dev_err(pcie_ep_dev->dev, "Element allcation failed\n");
				ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
				ll_element[u].header = LL_LAST_LINK_ELEMENT;
				ll_element[u].sar_low =
					LOWER((uint64_t) ll->dma[0]);
				ll_element[u].sar_high =
					UPPER((uint64_t) ll->dma[0]);
				pcie_ll_destroy(ll);
				return -EINVAL;
			}
			ll_element[u].sar_low =
				LOWER((uint64_t) dma);
			ll_element[u].sar_high =
				UPPER((uint64_t) dma);
			ll_element = tmp_element;
			u = 0;
			ll->size++;
			ll->ll_element[ll->size] = ll_element;
			ll->dma[ll->size] = dma;
		}
	}
	ll_element[u-1].header = LL_IRQ_DATA_ELEMENT;
	ll_element[u].header = LL_LAST_LINK_ELEMENT;
	ll_element[u].sar_low = LOWER((uint64_t) ll->dma[0]);
	ll_element[u].sar_high = UPPER((uint64_t) ll->dma[0]);
	return 0;
}

static int pcie_ll_destroy(struct mnh_dma_ll *ll)
{
	int i;

	i = 0;
	while (i <= ll->size) {
		mnh_free_coherent(DMA_LL_LENGTH
			* sizeof(struct mnh_dma_ll_element),
			ll->ll_element[i], ll->dma[i]);
		i++;
	}
	return 0;
}

/* APIs exposed to message passing protocol */

/* API to generate MSI to AP */
int mnh_send_msi(enum mnh_msi_msg_t msi)
{

	return pcie_send_msi(msi);
}
EXPORT_SYMBOL(mnh_send_msi);

/* API to generate LTR to AP */
/* Latency Tolerance Reporting */
int mnh_send_ltr(uint32_t ltr)
{

	return pcie_send_ltr(ltr);
}
EXPORT_SYMBOL(mnh_send_ltr);

/* API to set PCIE endpoint into L1 */
int mnh_set_l_one(uint32_t enable, uint32_t clkpm)
{
	pcie_ep_dev->power_state.clkpm = clkpm;
	pcie_ep_dev->power_state.l1state = enable;
	pcie_set_power_mode_state(&pcie_ep_dev->power_state);
	return pcie_set_l_one(enable, clkpm);
}
EXPORT_SYMBOL(mnh_set_l_one);

/* API to get PCIE power saving settings  */
int mnh_get_power_mode(struct mnh_pcie_ep_power_state *power_state)
{
	pcie_get_power_mode_state(power_state);
	return 0;
}
EXPORT_SYMBOL(mnh_get_power_mode);

/* API to set PCIE power saving settings  */
int mnh_set_power_mode(struct mnh_pcie_ep_power_state *power_state)
{
	pcie_set_power_mode_state(power_state);
	pcie_get_power_mode_state(&target_state);
	cancel_delayed_work(&power_state_work);
	schedule_delayed_work(&power_state_work,
		usecs_to_jiffies(POWER_DELAY));
	return 0;
}
EXPORT_SYMBOL(mnh_set_power_mode);

/* API to send Vendor message to AP */
int mnh_send_vm(struct mnh_pcie_vm *vm)
{
	return pcie_send_vm(vm->vm);
}
EXPORT_SYMBOL(mnh_send_vm);

/* API to register IRQ callbacks */
int mnh_reg_irq_callback(int (*callback)(struct mnh_pcie_irq *irq),
					int (*dmacallback)(struct mnh_dma_irq *irq))
{
	irq_callback = *callback;
	dma_callback = *dmacallback;

	return 0;
}
EXPORT_SYMBOL(mnh_reg_irq_callback);

/* API to program ringbuffer base address to PCIE BOOTSTRAP REGISTER */
int mnh_set_rb_base(uint64_t rb_base)
{
	return pcie_set_rb_base(LOWER(rb_base));
}
EXPORT_SYMBOL(mnh_set_rb_base);

/* API to read data from AP */
int mnh_pcie_read(uint8_t *buff, uint32_t size, uint64_t adr)
{
	int ret;

	ret = pcie_read_data(buff, size, adr);
	return ret;
}
EXPORT_SYMBOL(mnh_pcie_read);

/* API to write data from AP */
int mnh_pcie_write(uint8_t *buff, uint32_t size, uint64_t adr)
{
	int ret;

	ret = pcie_write_data(buff, size, adr);
	return ret;
}
EXPORT_SYMBOL(mnh_pcie_write);

int mnh_sg_build(void *dmadest, size_t size, struct mnh_sg_entry **sg,
		struct mnh_sg_list *sgl)
{
	int ret;

	size_t maxsg = (size / PAGE_SIZE) + 3;
	*sg = kcalloc(maxsg, sizeof(struct mnh_sg_entry), GFP_KERNEL);
	if (!(*sg))
		return -ENOMEM;

	ret = pcie_sg_build(dmadest, size, *sg, sgl, maxsg);

	if (ret) {
		kfree(*sg);
		*sg = NULL;
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(mnh_sg_build);

int mnh_sg_sync(struct mnh_sg_list *sgl)
{
	return pcie_sg_sync(sgl);
}
EXPORT_SYMBOL(mnh_sg_sync);


int mnh_sg_destroy(struct mnh_sg_list *sgl)
{
	return pcie_sg_destroy(sgl);
}
EXPORT_SYMBOL(mnh_sg_destroy);

int mnh_ll_build(struct mnh_sg_entry *src_sg, struct mnh_sg_entry *dst_sg,
				struct mnh_dma_ll *ll)
{
	return pcie_ll_build(src_sg, dst_sg, ll);
}
EXPORT_SYMBOL(mnh_ll_build);

int mnh_ll_destroy(struct mnh_dma_ll *ll)
{
	return pcie_ll_destroy(ll);
}
EXPORT_SYMBOL(mnh_ll_destroy);

void *mnh_alloc_coherent(size_t size, dma_addr_t *dma_adr)
{
	return dma_alloc_coherent(pcie_ep_dev->dev, size, dma_adr, GFP_KERNEL);
}
EXPORT_SYMBOL(mnh_alloc_coherent);

void mnh_free_coherent(size_t size, void *cpu_addr, dma_addr_t dma_addr)
{
	dma_free_coherent(pcie_ep_dev->dev, size, cpu_addr, dma_addr);
}
EXPORT_SYMBOL(mnh_free_coherent);

/** API to register pm callback to receive MNH suspend/resume notifications
 * @param[in] pm_callback  handler for pm events
 * @return 0
 */
int mnh_ep_reg_pm_callback(pm_callback_t pm_callback)
{
	dev_info(pcie_ep_dev->dev, "registering ep pm callback\n");
	mnh_ep_pm_callback = pm_callback;
	return 0;
}
EXPORT_SYMBOL(mnh_ep_reg_pm_callback);

static void mnh_ep_do_pm_callback(enum mnh_ep_pm_event_t event)
{
	if (mnh_ep_pm_callback)
		mnh_ep_pm_callback(event, NULL);
}

static int config_mem(struct platform_device *pdev)
{
	pcie_ep_dev->config_mem =
		platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pcie_ep_dev->config_mem)
		return -ENOMEM;
	if (!request_mem_region(pcie_ep_dev->config_mem->start,
		resource_size(pcie_ep_dev->config_mem),
		pcie_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		return -ENOMEM;
	}
	pcie_ep_dev->conf_mem =
		ioremap_nocache(pcie_ep_dev->config_mem->start,
			resource_size(pcie_ep_dev->config_mem));
	if (!pcie_ep_dev->conf_mem) {
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->cluster_mem =
		platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pcie_ep_dev->cluster_mem) {
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		return -ENOMEM;
	}
	if (!request_mem_region(pcie_ep_dev->cluster_mem->start,
		resource_size(pcie_ep_dev->cluster_mem), pcie_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->clust_mem =
		ioremap_nocache(pcie_ep_dev->cluster_mem->start,
			resource_size(pcie_ep_dev->cluster_mem));
	if (!pcie_ep_dev->clust_mem) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->outbound_mem = platform_get_resource(pdev,
					IORESOURCE_MEM, 2);
	if (!pcie_ep_dev->outbound_mem) {
		iounmap(&pcie_ep_dev->clust_mem);
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
			resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		return -ENOMEM;
	}
	if (!request_mem_region(pcie_ep_dev->outbound_mem->start,
		resource_size(pcie_ep_dev->outbound_mem), pcie_ep_dev->name)) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->clust_mem);
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->outb_mem = ioremap(pcie_ep_dev->outbound_mem->start,
				resource_size(pcie_ep_dev->outbound_mem));
	if (!pcie_ep_dev->outb_mem) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->clust_mem);
		iounmap(&pcie_ep_dev->conf_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		release_mem_region(pcie_ep_dev->outbound_mem->start,
				resource_size(pcie_ep_dev->outbound_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->scu_mem = platform_get_resource(pdev,
					IORESOURCE_MEM, 3);
	if (!pcie_ep_dev->scu_mem) {
		iounmap(&pcie_ep_dev->clust_mem);
		iounmap(&pcie_ep_dev->conf_mem);
		iounmap(&pcie_ep_dev->outb_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
			resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		return -ENOMEM;
	}
	pcie_ep_dev->scu = ioremap_nocache(pcie_ep_dev->scu_mem->start,
				resource_size(pcie_ep_dev->scu_mem));
	if (!pcie_ep_dev->scu) {
		dev_err(&pdev->dev, "unable to request mem region\n");
		iounmap(&pcie_ep_dev->clust_mem);
		iounmap(&pcie_ep_dev->conf_mem);
		iounmap(&pcie_ep_dev->outb_mem);
		release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
		release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
		release_mem_region(pcie_ep_dev->outbound_mem->start,
				resource_size(pcie_ep_dev->outbound_mem));
		return -ENOMEM;
	}
	return 0;
}

static int clear_mem(void)
{
	iounmap(&pcie_ep_dev->conf_mem);
	iounmap(&pcie_ep_dev->clust_mem);
	iounmap(&pcie_ep_dev->outb_mem);
	iounmap(&pcie_ep_dev->scu);
	release_mem_region(pcie_ep_dev->config_mem->start,
				resource_size(pcie_ep_dev->config_mem));
	release_mem_region(pcie_ep_dev->cluster_mem->start,
				resource_size(pcie_ep_dev->cluster_mem));
	release_mem_region(pcie_ep_dev->outbound_mem->start,
				resource_size(pcie_ep_dev->outbound_mem));
	release_mem_region(pcie_ep_dev->scu_mem->start,
				resource_size(pcie_ep_dev->scu_mem));
	return 0;
}

/* IRQ Callback function for testing purposes */

#if MNH_PCIE_DEBUG_ENABLE

int test_callback(struct mnh_pcie_irq *irq)
{
	sysfs_irq = *irq;
	dev_dbg(pcie_ep_dev->dev, "PCIE MSI IRQ %x PCIE IRQ %x VM IRQ %x",
		sysfs_irq.msi_irq, sysfs_irq.pcie_irq, sysfs_irq.vm);
	return 0;
}

int test_dma_callback(struct mnh_dma_irq *irq)
{
	/*TODO do something */
	dev_dbg(pcie_ep_dev->dev, "DMA Event channel %x type %x Event %x",
		irq->channel, irq->type, irq->status);
	return 0;
}

/* SYS_FS for debugging and testing */

static ssize_t show_sysfs_send_msi(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_send_msi(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if ((val < MSI_START) | (val > MSI_END))
		return -EINVAL;
	mnh_send_msi((enum mnh_msi_msg_t) val);
	return count;
}

static DEVICE_ATTR(send_msi, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_send_msi, sysfs_send_msi);

static ssize_t show_sysfs_send_vm(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_send_vm(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;
	struct mnh_pcie_vm vm;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	vm.vm = (uint32_t)val;
	mnh_send_vm(&vm);
	return count;
}

static DEVICE_ATTR(send_vm, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_send_vm, sysfs_send_vm);


static ssize_t show_sysfs_rb_base(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint64_t rb_base;

	rb_base = pcie_cluster_read(MNH_PCIE_GP_1);
	return snprintf(buf, MAX_STR_COPY, "%llx\n", rb_base);
}

static ssize_t sysfs_rb_base(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	mnh_set_rb_base(val);
	return count;
}

static DEVICE_ATTR(rb_base, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_rb_base, sysfs_rb_base);

static ssize_t show_sysfs_rw_address(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "%lx\n",
				(unsigned long) rw_address_sysfs);
}

static ssize_t sysfs_rw_address(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	rw_address_sysfs = (uint32_t)val;
	return count;
}

static DEVICE_ATTR(rw_address, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_rw_address, sysfs_rw_address);

static ssize_t show_sysfs_rw_size(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "%lx\n",
				(unsigned long) rw_size_sysfs);
}

static ssize_t sysfs_rw_size(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	rw_size_sysfs = (uint32_t)val;
	return count;
}

static DEVICE_ATTR(rw_size, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_rw_size, sysfs_rw_size);

static ssize_t show_sysfs_read_cluster(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{	unsigned long val;

	val = (unsigned long) pcie_cluster_read(rw_address_sysfs);
	return snprintf(buf, MAX_STR_COPY, "%lx\n", val);
}

static ssize_t sysfs_write_cluster(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	pcie_cluster_write(rw_address_sysfs, (uint32_t)val);
	return count;
}

static DEVICE_ATTR(rw_cluster, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_read_cluster, sysfs_write_cluster);

static ssize_t show_sysfs_read_config(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{	unsigned long val;

	val = (unsigned long)pcie_config_read(rw_address_sysfs);
	return snprintf(buf, MAX_STR_COPY, "%lx\n", val);
}

static ssize_t sysfs_write_config(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	pcie_config_write(rw_address_sysfs, (uint32_t)val);
	return count;
}

static DEVICE_ATTR(rw_config, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_read_config, sysfs_write_config);

static ssize_t show_sysfs_read_data(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long val;
	uint8_t *buffer;

	buffer = kmalloc(MAX_STR_COPY, GFP_KERNEL);
	if (!buffer)
		return -EINVAL;
	if (mnh_pcie_read(buffer, rw_size_sysfs, rw_address_sysfs)) {
		kfree(buffer);
		return -EINVAL;
	}
	val = copy_to_user(buf, buffer, rw_size_sysfs);
	if (val) {
		kfree(buffer);
		return -EINVAL;
	}
	kfree(buffer);
	return rw_size_sysfs;
}

static ssize_t sysfs_write_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	if (mnh_pcie_write((uint8_t *)buf, count, rw_address_sysfs)) {
		dev_err(pcie_ep_dev->dev, "Write buffer failed\n");
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(rw_data, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_read_data, sysfs_write_data);

static ssize_t show_sysfs_test_callback(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY,
		"PCIE MSI IRQ %d PCIE IRQ %d VM IRQ %d",
		sysfs_irq.msi_irq, sysfs_irq.pcie_irq, sysfs_irq.vm);
}

static ssize_t sysfs_test_callback(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val == 1)
		mnh_reg_irq_callback(&test_callback, &test_dma_callback);
	else
		mnh_reg_irq_callback(NULL, NULL);

	return count;
}

static DEVICE_ATTR(test_callback, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_test_callback, sysfs_test_callback);

static ssize_t show_sysfs_send_ltr(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_send_ltr(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	mnh_send_ltr((uint32_t) val);
	return count;
}

static DEVICE_ATTR(send_ltr, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_send_ltr, sysfs_send_ltr);

static ssize_t show_sysfs_set_lone(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "No support\n");
}

static ssize_t sysfs_set_lone(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;
	uint32_t enable, pmclock;
	uint8_t *token;
	const char *delim = ";";

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 1))
			return -EINVAL;
		enable = val;
		dev_dbg(pcie_ep_dev->dev, "L1 state is %lx\n", val);
	} else {
		return -EINVAL;
	}

	token = strsep((char **) &buf, delim);

	if (token) {
		if (kstrtoul(token, 0, &val))
			return -EINVAL;
		if ((val < 0) || (val > 1))
			return -EINVAL;
		pmclock = val;
		dev_dbg(pcie_ep_dev->dev, "PM Clock is %lx\n", val);
	} else {
		return -EINVAL;
	}
	mnh_set_l_one(enable, pmclock);
	return count;
}

static DEVICE_ATTR(set_lone, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_set_lone, sysfs_set_lone);

static ssize_t show_sysfs_set_power_mode(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	uint32_t mode = 0;
	struct mnh_pcie_ep_power_state power_state;

	mnh_get_power_mode(&power_state);
	mode = power_state.clkpm <<
		HWIO_SCU_GP_POWER_MODE_PCIE_CLKPM_EN_FLDSHFT;
	mode |= power_state.l1state <<
		HWIO_SCU_GP_POWER_MODE_PCIE_L1_2_EN_FLDSHFT;
	mode |= power_state.axi_gating <<
		HWIO_SCU_GP_POWER_MODE_PCIE_AXI_CG_EN_FLDSHFT;

	return snprintf(buf, MAX_STR_COPY, "0x%x\n", mode);
}

static ssize_t sysfs_set_power_mode(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val = 0;
	struct mnh_pcie_ep_power_state power_state;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	dev_dbg(pcie_ep_dev->dev, "Setting power_mode 0x%lx, 0x%lx\n", val,
		val & HWIO_SCU_GP_POWER_MODE_PCIE_L1_2_EN_FLDMASK);

	power_state.clkpm =
		(val & HWIO_SCU_GP_POWER_MODE_PCIE_CLKPM_EN_FLDMASK) >>
		HWIO_SCU_GP_POWER_MODE_PCIE_CLKPM_EN_FLDSHFT;
	power_state.l1state =
		(val & HWIO_SCU_GP_POWER_MODE_PCIE_L1_2_EN_FLDMASK) >>
		HWIO_SCU_GP_POWER_MODE_PCIE_L1_2_EN_FLDSHFT;
	power_state.axi_gating =
		(val &	HWIO_SCU_GP_POWER_MODE_PCIE_AXI_CG_EN_FLDMASK) >>
		HWIO_SCU_GP_POWER_MODE_PCIE_AXI_CG_EN_FLDSHFT;

	mnh_set_power_mode(&power_state);
	return count;
}

static DEVICE_ATTR(power_mode, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_set_power_mode, sysfs_set_power_mode);

static ssize_t show_sysfs_mnh_trace(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "0x%x", mnh_trace_get());
}

static ssize_t sysfs_mnh_trace(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val = 0;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	dev_dbg(pcie_ep_dev->dev, "User setting trace value 0x%lx\n", val);

	mnh_trace(val);

	return count;
}

static DEVICE_ATTR(mnh_trace, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_mnh_trace, sysfs_mnh_trace);

static void clean_sysfs(void)
{
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_msi);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_vm);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rb_base);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_address);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_size);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_cluster);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_config);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_rw_data);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_test_callback);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_send_ltr);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_set_lone);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_power_mode);
	device_remove_file(pcie_ep_dev->dev,
			&dev_attr_mnh_trace);
}

static int init_sysfs(void)
{
	int ret;

	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_send_msi);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: send_msi\n");
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_send_vm);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: send_vm\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rb_base);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rb_base\n");
		clean_sysfs();
		return -EINVAL;
	}
	rw_address_sysfs = 0x0;
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_address);
	if (ret) {
		dev_err(pcie_ep_dev->dev,
			"Failed to create sysfs: rw_address\n");
		clean_sysfs();
		return -EINVAL;
	}
	rw_size_sysfs = 0x1;
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_size);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rw_size\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_cluster);
	if (ret) {
		dev_err(pcie_ep_dev->dev,
			"Failed to create sysfs: rw_cluster\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_config);
	if (ret) {
		dev_err(pcie_ep_dev->dev,
			"Failed to create sysfs: rw_config\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_rw_data);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: rw_data\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_test_callback);
	if (ret) {
		dev_err(pcie_ep_dev->dev,
			"Failed to create sysfs: test_callback\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_send_ltr);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: send_ltr\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_set_lone);
	if (ret) {
		dev_err(pcie_ep_dev->dev, "Failed to create sysfs: set_lone\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_power_mode);
	if (ret) {
		dev_err(pcie_ep_dev->dev,
			"Failed to create sysfs: power_mode\n");
		clean_sysfs();
		return -EINVAL;
	}
	ret = device_create_file(pcie_ep_dev->dev,
			&dev_attr_mnh_trace);
	if (ret) {
		dev_err(pcie_ep_dev->dev,
			"Failed to create sysfs: mnh_trace\n");
		clean_sysfs();
		return -EINVAL;
	}

	return 0;
}


#endif
static int mnh_pcie_ep_probe(struct platform_device *pdev)
{
	int err;

	mnh_trace(MNH_TRACE_PCIE_PROBE);
	dev_info(&pdev->dev, "PCIE endpoint probe start\n");
	pcie_ep_dev = kzalloc(sizeof(*pcie_ep_dev), GFP_KERNEL);
	if (!pcie_ep_dev) {
		dev_err(&pdev->dev, "Could not allocated pcie_ep_dev\n");
		return -ENOMEM;
	}
	pcie_ep_dev->dev = &pdev->dev;
	pcie_ep_dev->pendingmsi = 0;
	pcie_ep_dev->msimode = 0;
	pcie_ep_dev->rb_base = 0;
	strcpy(pcie_ep_dev->name, DEVICE_NAME);
	irq_callback = NULL;
	dma_callback = NULL;
	err = config_mem(pdev);
	if (err)
		return err;

	/* declare MSI worker */
	INIT_DELAYED_WORK(&msi_work, pcie_msi_worker);
	INIT_DELAYED_WORK(&power_state_work, pcie_power_state_worker);
	INIT_WORK(&wake_work, pcie_wake_worker);
	INIT_WORK(&msi_rx_work, msi_rx_worker);
	INIT_WORK(&vm0_work, vm0_worker);
	INIT_WORK(&vm1_work, vm1_worker);
#if MNH_PCIE_DEBUG_ENABLE
	init_sysfs();
#endif

	pcie_link_init();
	err = 0;

	if (dma_set_mask(pcie_ep_dev->dev, DMA_BIT_MASK(64)) ||
		dma_set_coherent_mask(pcie_ep_dev->dev, DMA_BIT_MASK(64))) {

		err = dma_set_mask(pcie_ep_dev->dev, DMA_BIT_MASK(32));
		if (err)
			err = dma_set_coherent_mask(pcie_ep_dev->dev,
							DMA_BIT_MASK(32));
	}
	if (err) {
		dev_err(pcie_ep_dev->dev, "No usable DMA configuration, aborting\n");
		goto fail_dma_set_mask;
	}


	/* Register IRQs */

	CSR_OUT(PCIE_SW_INTR_TRIGG, MNH_PCIE_SW_IRQ_CLEAR);
	pcie_ep_dev->sw_irq = platform_get_irq(pdev, 0);
	err = request_irq(pcie_ep_dev->sw_irq, pcie_handle_sw_irq,
			IRQF_SHARED, DEVICE_NAME, pcie_ep_dev);
	if (err) {
		err = -EINVAL;
		goto fail_sw_irq;
	}
	pcie_ep_dev->cluster_irq = platform_get_irq(pdev, 1);
	err = request_irq(pcie_ep_dev->cluster_irq, pcie_handle_cluster_irq,
			IRQF_SHARED, DEVICE_NAME, pcie_ep_dev->dev);
	if (err) {
		err = -EINVAL;
		goto fail_cluster_irq;
	}
	pcie_ep_dev->wake_irq = platform_get_irq(pdev, 2);
	err = request_irq(pcie_ep_dev->wake_irq, pcie_handle_wake_irq,
			IRQF_SHARED, DEVICE_NAME, pcie_ep_dev->dev);
	if (err) {
		err = -EINVAL;
		goto fail_wake_irq;
	}

	mnh_trace(MNH_TRACE_PCIE_PROBE_DONE);

	return 0;

fail_wake_irq:
	free_irq(pcie_ep_dev->cluster_irq, pcie_ep_dev->dev);
fail_cluster_irq:
	free_irq(pcie_ep_dev->sw_irq, pcie_ep_dev->dev);
fail_sw_irq:
fail_dma_set_mask:
#if MNH_PCIE_DEBUG_ENABLE
	clean_sysfs();
#endif
	clear_mem();
	return err;
}

static int mnh_pcie_ep_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&msi_work);
	cancel_delayed_work_sync(&power_state_work);
#if MNH_PCIE_DEBUG_ENABLE
	clean_sysfs();
#endif
	clear_mem();
	free_irq(pcie_ep_dev->sw_irq, pcie_ep_dev->dev);
	free_irq(pcie_ep_dev->cluster_irq, pcie_ep_dev->dev);
	return 0;
}

static int mnh_pcie_ep_suspend(struct platform_device *pdev, pm_message_t state)
{
	dev_dbg(pcie_ep_dev->dev, "%s\n", __func__);
	mnh_ep_do_pm_callback(MNH_EP_WILL_SUSPEND);

	cancel_delayed_work_sync(&msi_work);
	cancel_delayed_work_sync(&power_state_work);
	pcie_ep_dev->power_state.l1state = 0;
	pcie_ep_dev->power_state.clkpm = 0;
	pcie_ep_dev->rb_base = pcie_cluster_read(MNH_PCIE_GP_1);

	mnh_ep_do_pm_callback(MNH_EP_DID_SUSPEND);
	return 0;
}

static int mnh_pcie_ep_resume(struct platform_device *pdev)
{
	dev_dbg(pcie_ep_dev->dev, "%s\n", __func__);
	mnh_ep_do_pm_callback(MNH_EP_WILL_RESUME);

	/*enable L1 entry */
	PCIECAP_OUTf(PCIE_CAP_LINK_CONTROL_LINK_STATUS,
			PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL, 0x2);
	pcie_get_power_mode_state(&target_state);
	cancel_delayed_work(&power_state_work);
	schedule_delayed_work(&power_state_work,
		usecs_to_jiffies(POWER_DELAY));

	/* Enable interupts */
	CSR_OUT(PCIE_SS_INTR_EN, PCIE_SS_IRQ_MASK);
	/* Clear all interrupts */
	CSR_OUT(PCIE_SW_INTR_TRIGG, MNH_PCIE_SW_IRQ_CLEAR);
	/* set ringbuffer base address and send bootstrap msi */
	pcie_set_rb_base(pcie_ep_dev->rb_base);

	mnh_ep_do_pm_callback(MNH_EP_DID_RESUME);
	return 0;
}

/*
 * of_device_id structure
 */
static const struct of_device_id mnh_pcie_ep[] = {
	{ .compatible = "snps, dw_pcie_ep" },
	{ }
};


MODULE_DEVICE_TABLE(of, mnh_pcie_ep);
/*
 * Platform driver structure
 */
static struct platform_driver __refdata mnh_pcie_ep_pdrv = {
	.remove = mnh_pcie_ep_remove,
	.probe  = mnh_pcie_ep_probe,
	.resume = mnh_pcie_ep_resume,
	.suspend = mnh_pcie_ep_suspend,
	.driver   = {
		.name   = "snps, dw_pcie_ep",
		.owner = THIS_MODULE,
		.of_match_table = mnh_pcie_ep,
	},
};


module_platform_driver(mnh_pcie_ep_pdrv);

MODULE_AUTHOR("Marko Bartscherer <marko.bartscherer@intel.com>");
MODULE_DESCRIPTION("Monette Hill PCIE EndPoint Driver");
MODULE_LICENSE("GPL");
