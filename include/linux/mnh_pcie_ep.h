
/** @mainpage  MNH PCIE EP(EndPoint) driver
*
* @section intro Introduction
* The PCIE endpoint driver is implemented as a character driver
* allowing the applications to access the entire address range
* exposed thru PCIE.
* Specific functions are exposed through the API's.
*/


/**
 * @file    mnh_pcie_ep.h
 * @brief   PCIe EP driver header includes function API and data structure
 * @author  Intel
 * @date    27 Apr 2016
 * @version 1.0
 */

#include <linux/types.h>
#ifndef __LINUX_MNH_PCIE_EP_H
#define __LINUX_MNH_PCIE_EP_H
#define MNH_PCIE_DEBUG_ENABLE 1
#define MNH_MAX_LL 256
#define MNH_MAX_LL_ELEMENT 64
/* TODO implement to mask sysfs and other code */

/*****************************************************************************
 *
 *  Data structure defined for APIs
 *
 ****************************************************************************/

/** enum value used for msi msg type for mnh_send_msi() API */
enum  mnh_msi_msg_t {
	INVAL_MSI = 0,
	MSI_START = 1,
	MSG_SEND_M = MSI_START,
	PET_WATCHDOG,
	CRASH_DUMP,
	BOOTSTRAP_SET,
	APPDEFINED_1_M,
	MSI_END = APPDEFINED_1_M
};

/** enum value used for irq structure passed IRQ callback function */
enum mnh_irq_msg_t {
	INVAL_IRQ = 0,
	MSG_SEND_I,
	DMA_STATUS,
	APPDEFINED_1_I,
};

/** enum value used for pcie interupts passed to IRQ callback function */
enum mnh_irq_pcie_t {
	INVAL_PCIE = 0,
	MSI_SEND,
	VM_SEND,
	LTR_SEND
};

/**
 * DMA channel direction type
 */
enum mnh_dma_chan_dir_t {
	DMA_EP2AP = 0, /**< EP2AP(WRITE_CH):To write data to AP from EP */
	DMA_AP2EP      /**< AP2EP(READ_CH):To read data from AP and send to EP*/
};

/** enum value used for pcie interupts passed to IRQ callback function */
enum  mnh_dma_type_t {
	MNH_DMA_READ = 0,
	MNH_DMA_WRITE
};

/** enum value used for pcie interupts passed to IRQ callback function */
enum mnh_dma_status_t {
	MNH_DMA_DONE = 0,
	MNH_DMA_ABORT
};


/** structure used for mnh_send_vm() API */
struct mnh_pcie_vm {
	uint32_t vm;       /**< vendor defined message */
};

/** structure used for IRQ callback function */
struct mnh_pcie_irq {
	enum mnh_irq_msg_t	msi_irq;
				/**< Will be 0 if no IRQ received
				 *from the AP otherwise it holds IRQ type
				 */
	enum mnh_irq_pcie_t	pcie_irq;
				/**< Will be 0 if no IRQ in the pcie controller
				  *otherwise it will hold the IRQ type
				  */
	uint32_t		vm; /**< Will be 0 if no vm was received
				    *otherwise it holds the interrupt
				    */
};

struct mnh_dma_irq {
	uint32_t		channel;
	enum mnh_dma_type_t	type;
	enum mnh_dma_status_t	status;
};

/**
* Structure of scatter gather list entry
*/
struct mnh_sg_entry {
	phys_addr_t paddr; /**< Physical address */
	size_t size;       /**< size of entry */
};

/**
 * MNH (EP side) pm event
 */
enum mnh_ep_pm_event_t {
	MNH_EP_WILL_SUSPEND = 0,  /* PCIe EP will suspend */
	MNH_EP_DID_SUSPEND,       /* PCIe EP did suspend  */
	MNH_EP_WILL_RESUME,       /* PCIe EP will resume  */
	MNH_EP_DID_RESUME,        /* PCIe EP did resume   */
};

struct mnh_sg_list {
	struct page **mypage;
	struct scatterlist *sc_list;
	int n_num;
	int length;
	enum mnh_dma_chan_dir_t dir;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sg_table;
};


struct mnh_dma_ll_element {
	uint32_t header;
	uint32_t size;
	uint32_t sar_low;
	uint32_t sar_high;
	uint32_t dar_low;
	uint32_t dar_high;
};

struct mnh_dma_ll {
	uint32_t size;
	struct mnh_dma_ll_element *ll_element[MNH_MAX_LL_ELEMENT];
	dma_addr_t dma[MNH_MAX_LL_ELEMENT];
};

typedef int (*pm_callback_t)(enum mnh_ep_pm_event_t event, void *param);


/*******************************************************************************
 *
 *  APIs exposed
 *
 ******************************************************************************/

/** API to generate MSI to AP */
int mnh_send_msi(enum mnh_msi_msg_t msi);

/** API to generate LTR to AP */
/* add more info */
int mnh_send_ltr(uint32_t ltr);

/** API to send Vendor message to AP */
int mnh_send_vm(struct mnh_pcie_vm *vm);

/** API to set PCIE endpoint into L1 */
int mnh_set_l_one(uint32_t enable, uint32_t clkpm);

/** API to register IRQ callback */
int mnh_reg_irq_callback(int (*callback)(struct mnh_pcie_irq *irq),
		int (*dmacallback)(struct mnh_dma_irq *irq));

/** API to program ringbuffer base address to PCIE BOOTSTRAP REGISTER */
int mnh_set_rb_base(uint64_t rb_base);

/** API to read data from AP */
int mnh_pcie_read(uint8_t *buff, uint32_t size, uint64_t adr);

/** API to write data to AP */
int mnh_pcie_write(uint8_t *buff, uint32_t size, uint64_t adr);

/**
 * API to build Scatter Gather list to do Multi-block DMA transfer for a user
 * buffer
 * @param[in] dmadest  Starting virtual addr of the DMA destination
 * @param[in] size Totalsize of the transfer in bytes
 * @param[out] sg  Array of maxsg pointers to struct mnh_sg_entry, allocated
 *			and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			page list, scatter gather list and num of its entries.
 * @return The number of sg[] entries filled out by the routine, negative if
 *		   overflow or sg[] not allocated.
 */
int mnh_sg_build(void *dmadest, size_t size, struct mnh_sg_entry **sg,
		struct mnh_sg_list *sgl);

int mnh_sg_sync(struct mnh_sg_list *sgl);

/**
 * API to release scatter gather list for a user buffer
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		mnh_sg_build
 * @return 0 for SUCCESS
 */
int mnh_sg_destroy(struct mnh_sg_list *sgl);;

/**
 * API to build a scatter-gather list for multi-block DMA transfer for a
 * dma_buf
 * @param[in] fd   Handle of dma_buf passed from user
 * @param[out] sg  Array of maxsg pointers to struct mnh_sg_entry, allocated
 *			and filled out by this routine.
 * @param[out] sgl pointer of Scatter gather list which has information of
 *			scatter gather list and num of its entries.
 * @return 0        on SUCCESS
 *         negative on failure
 */
int mnh_sg_retrieve_from_dma_buf(int fd, struct mnh_sg_entry **sg,
		struct mnh_sg_list *sgl);

/**
 * API to release a scatter-gather list for a dma_buf
 * @param[in] *sgl pointer to the scatter gather list that was built during
 *		mnh_sg_retrieve_from_dma_buf
 * @return 0 for SUCCESS
 */
int mnh_sg_release_from_dma_buf(struct mnh_sg_list *sgl);

int mnh_ll_build(struct mnh_sg_entry *src_sg, struct mnh_sg_entry *dst_sg,
					struct mnh_dma_ll *ll);

int mnh_ll_destroy(struct mnh_dma_ll *ll);

void *mnh_alloc_coherent(size_t size, dma_addr_t *dma_adr);

void mnh_free_coherent(size_t size, void *cpu_addr, dma_addr_t dma_addr);

/** API to register pm callback to receive MNH suspend/resume notifications
 * @param[in] pm_callback  handler for pm events
 * @return 0
 */
int mnh_ep_reg_pm_callback(pm_callback_t pm_callback);

#endif
