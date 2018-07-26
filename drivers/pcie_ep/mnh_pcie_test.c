/*
 * pcie_ep_tst.c - Monhette Hill PCIe EndPoint driver
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

#define VENDOR_ID				0x8086
#define DEVICE_ID				0x3140

struct cdev pci_ep_tst_dev;
dev_t mht_pcie_ep_tst_dev;
static struct class *pcie_ep_tst_class;
static struct device *pcie_ep_tst_device;
#define DEVICE_NAME "mth_pcie_ep_test"
#define CLASS_NAME "pcie_ep_test"
#define MSI_DELAY (HZ/20) /* TODO: Need to understand what this should be */
#define MAX_STR_COPY	32

struct mnh_sg_entry *sg1, *sg2;
static dma_addr_t dma2;

struct mnh_sg_list *sgl;

uint32_t ll_index, status, dma_dir;
struct mnh_dma_ll *ll_adr;

static int buildll(void)
{
	dev_err(pcie_ep_tst_device, "Start LL build \n");
	if (dma_dir == 1) {
		if (mnh_ll_build(sg2, sg1, ll_adr) == 0)
			dev_err(pcie_ep_tst_device, "LL built successfully\n");
	} else if (mnh_ll_build(sg1, sg2, ll_adr) == 0)
			dev_err(pcie_ep_tst_device, "LL built successfully\n");
	mnh_set_rb_base(mnh_ll_base_addr(ll_adr));
	status = 2;
	//status = 3;
	return 0;
}

int test2_callback(struct mnh_pcie_irq *irq)
{

	dev_err(pcie_ep_tst_device, "IRQ received %d \n",irq->pcie_irq);
	if (irq->msi_irq == MSG_SEND_I) {
		/*if (status == 0) {
			status =1;
			if (ll_index == 1)
				buildll();
		} */
	}
	return 0;
}

int test2_dma_callback(struct mnh_dma_irq *irq)
{
	/*TODO do something */
	dev_err(pcie_ep_tst_device, "DMA callback %x \n",irq->status);
	if ((irq->status == MNH_DMA_DONE) && (status == 0)) {
			status =1;
			if (ll_index == 1)
				buildll();
	} else if ((irq->status == MNH_DMA_DONE) && (status ==2))
		status =3;
	return 0;
}

static int mth_fs_pcie_open(struct inode *inode, struct file *file)
{
	dev_err(pcie_ep_tst_device, "File Open\n");
	ll_index = 0;
	status = 0;

	sg2 = mnh_alloc_coherent(SGL_SIZE * sizeof(struct mnh_sg_entry), &dma2);
	if (!sg2) {
		dev_err(pcie_ep_tst_device, "failed to assign sgl 2\n");
		return -EINVAL;
	}
	sgl = kcalloc(SGL_SIZE, sizeof(struct mnh_sg_list), GFP_KERNEL);
	if (!sgl) {
		dev_err(pcie_ep_tst_device, "failed to assign sgl\n");
		mnh_free_coherent(SGL_SIZE * sizeof(struct mnh_sg_entry),
				  sg2, dma2);
		return -EINVAL;
	}
	ll_adr = kmalloc(sizeof(struct mnh_dma_ll), GFP_KERNEL);
	if (!ll_adr) {
		dev_err(pcie_ep_tst_device, "failed to assign ll_adr \n");
		return -EINVAL;
	}
	mnh_reg_irq_callback(&test2_callback, &test2_dma_callback);
	mnh_set_rb_base(dma2);
	return 0;
}

static int mth_fs_pcie_close(struct inode *inode, struct file *file)
{
	dev_err(pcie_ep_tst_device, "File Close\n");
	mnh_reg_irq_callback(NULL, NULL);
	if (status)
		mnh_ll_destroy(ll_adr);
	mnh_sg_destroy(sgl);
	kfree(ll_adr);
	kfree(sg1);
	sg1 = NULL;
	mnh_free_coherent(SGL_SIZE * sizeof(struct mnh_sg_entry), sg2, dma2);
	kfree(sgl);
	return 0;
}

static ssize_t  mth_fs_pcie_write(struct file *file, const char __user *buf, size_t count, loff_t *off)
{

	dev_err(pcie_ep_tst_device, "File write \n");
	mnh_sg_build((void *)buf, count, &sg1, sgl);
	dev_err(pcie_ep_tst_device, "SG list build  %d %d\n",ll_index, status);
	ll_index =1;
	if (status == 1)
		buildll();
	dev_err(pcie_ep_tst_device, "SG list build  %d %d\n",ll_index, status);
	//status = 3;
	return 0;
}


static ssize_t mth_fs_pcie_read (struct file *filp,char *buf, size_t length, loff_t * offset)
{
	static char msg[5];
	unsigned long ret = 0;

	if (status == 3) {
		//mnh_sg_destroy(sgl);
		mnh_sg_sync(sgl);
		sprintf(msg, "DONE\n");
		//sprintf(msg, "WAIT\n");
		if (length < 6)
			return 0;
		ret = copy_to_user(buf,&msg,5);
		return 6;
	} else {
		sprintf(msg, "WAIT\n");
		if (length < 6)
			return 0;
		ret = copy_to_user(buf,&msg,5);
		return 6;
	}
}


static struct file_operations pcie_ep_tst_fops = {
	.owner = THIS_MODULE,
	.open = mth_fs_pcie_open,
	.release = mth_fs_pcie_close,
	.read = mth_fs_pcie_read,
	.write = mth_fs_pcie_write
};



/* SYS_FS for debugging and testing */

static ssize_t show_sysfs_dma(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, MAX_STR_COPY, "dma direction %d\n",dma_dir);
}

static ssize_t sysfs_dma(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (val == 1)
		dma_dir = 1;
	else
		dma_dir =0;

	return count;
}

static DEVICE_ATTR(dma_direct, S_IRUGO | S_IWUSR | S_IWGRP,
			show_sysfs_dma, sysfs_dma);



static int init_sysfs(void)
{
	int ret;

	ret = device_create_file(pcie_ep_tst_device,
			&dev_attr_dma_direct);
	if (ret) {
		dev_err(pcie_ep_tst_device, "Failed to create sysfs: send_msi\n");
		return -EINVAL;
	}

	return 0;
}

static void clean_sysfs(void)
{
	device_remove_file(pcie_ep_tst_device,
			&dev_attr_dma_direct);
}

static int __init mth_pcie_tst_drv_init(void)
{
	int err, major;

	pr_info("mth_pcie_ep_test: Init\n");
	err = alloc_chrdev_region(&mht_pcie_ep_tst_dev, 0, 1, DEVICE_NAME);
	cdev_init(&pci_ep_tst_dev, &pcie_ep_tst_fops);
	pci_ep_tst_dev.owner = THIS_MODULE;
	pci_ep_tst_dev.ops = &pcie_ep_tst_fops;
	err = cdev_add(&pci_ep_tst_dev, mht_pcie_ep_tst_dev, 1);
	if (err) {
		unregister_chrdev_region(mht_pcie_ep_tst_dev, 1);
		return err;
	}
	major = MAJOR(mht_pcie_ep_tst_dev);
	pcie_ep_tst_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(pcie_ep_tst_class)) {
		cdev_del(&pci_ep_tst_dev);
		unregister_chrdev_region(mht_pcie_ep_tst_dev, 1);
		err = PTR_ERR(pcie_ep_tst_class);
		return err;
	}
	pcie_ep_tst_device = device_create(pcie_ep_tst_class, NULL, MKDEV(major, 0),
			NULL, CLASS_NAME "_" DEVICE_NAME);
	if (IS_ERR(pcie_ep_tst_device)) {
		cdev_del(&pci_ep_tst_dev);
		unregister_chrdev_region(mht_pcie_ep_tst_dev, 1);
		err = PTR_ERR(pcie_ep_tst_device);
		return err;
	}
	dma_dir = 0;
	init_sysfs();
	return 0;
}

static void __exit mth_pcie_tst_drv_exit(void)
{
	clean_sysfs();
	cdev_del(&pci_ep_tst_dev);
	unregister_chrdev_region(mht_pcie_ep_tst_dev, 1);
}

module_init(mth_pcie_tst_drv_init);
module_exit(mth_pcie_tst_drv_exit);

MODULE_AUTHOR("Marko Bartscherer <marko.bartscherer@intel.com>");
MODULE_DESCRIPTION("Monhette Hill PCIE EndPoint Test Driver");
MODULE_LICENSE("GPL");
