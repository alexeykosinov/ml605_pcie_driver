// https://habr.com/ru/post/348042/
// https://static.lwn.net/images/pdf/LDD3/ch12.pdf
// https://betontalpfa.medium.com/oh-no-i-need-to-write-a-pci-driver-2b389720a9d0

#ifndef __XIL_PCI_DRIVER_H_
#define __XIL_PCI_DRIVER_H_

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>


// #include "xil_pci_driver_char.h"


#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

#define PCI_DRIVER_NAME		    "ml605_pcie"
#define PCI_VID				    0x10EE 	    /* Xilinx 			                */
#define PCI_PID				    0x0505 	    /* Device ID 		                */

#define DMA_BUFFER_SIZE         16*1024

/* AXI DMA Scatter/Gather Mode Register Address Mapping (C_INCLUDE_SG = 1)      */
#define AXI_DMA_OFFSET(chnum)   (sizeof(uint32_t) * chnum)
#define AXI_DMA_MM2S_DMACR      0x00        /* MM2S DMA Control Register        */
#define AXI_DMA_MM2S_DMASR      0x04        /* MM2S DMA Status Register         */
#define AXI_DMA_MM2S_CURDESC    0x08        /* MM2S Current Descriptor Pointer  */
#define AXI_DMA_MM2S_TAILDESC   0x10        /* MM2S Tail Descriptor Pointer     */
#define AXI_DMA_SG_CTL          0x2C        /* Scatter/Gather User and Cache    */
#define AXI_DMA_S2MM_DMACR      0x30        /* S2MM DMA Control Register        */
#define AXI_DMA_S2MM_DMASR      0x34        /* S2MM DMA Status Register         */
#define AXI_DMA_S2MM_CURDESC    0x38        /* S2MM Current Descriptor Pointer  */
#define AXI_DMA_S2MM_TAILDESC   0x40        /* S2MM Tail Descriptor Pointer     */

#define AXI_DMA_DMASR_MASK_IDLE 0x00000002  /* DMASR Status Idle Bit Mask       */


/* AXI DMA Control Bits */

/* 
* Interrupt on Error Interrupt Enable. When set to 1, allows
* DMASR.Err_Irq to generate an interrupt out.
*/
#define AXI_DMA_DMACR_ERR_IrqEn (1 << 14)   // Enable interrupt on error

/* 
* Interrupt on Delay Timer Interrupt Enable. When set to 1, allows
* DMASR.Dly_Irq to generate an interrupt out.
*/
#define AXI_DMA_DMACR_DLY_IrqEn (1 << 13)   // Enable completion interrupt

/* 
* Interrupt on Complete (IOC) Interrupt Enable. When set to 1,
* allows DMASR.IOC_Irq to generate an interrupt out for descriptors
* with the IOC bit set.
*/
#define AXI_DMA_DMACR_IOC_IrqEn (1 << 12)   // Enable completion interrupt

/* 
* Soft reset for resetting the AXI DMA core. Setting this bit to a 1
* causes the AXI DMA to be reset. Reset is accomplished gracefully.
* Pending commands/transfers are flushed or completed.
* AXI4-Stream outs are potentially terminated early. Setting either
* MM2S_DMACR.Reset = 1 or S2MM_DMACR.Reset = 1 resets the
* entire AXI DMA engine. After completion of a soft reset, all
* registers and bits are in the Reset State.
*/
#define AXI_DMA_DMACR_Reset     (1 << 2)

/* AXI DMA Status Bits */
#define AXI_DMA_DMASR_Err_Irq   (1 << 14)   // DMA error event seen
#define AXI_DMA_DMASR_Dly_Irq   (1 << 13)   // 
#define AXI_DMA_DMASR_IOC_Irq   (1 << 12)   // DMA completion event seen
#define AXI_DMA_DMASR_SGDecErr  (1 << 10)   // 
#define AXI_DMA_DMASR_SGSlvErr  (1 << 9)    // 
#define AXI_DMA_DMASR_SGIntErr  (1 << 8)    // 
#define AXI_DMA_DMASR_DMADecErr (1 << 6)    // Address decode error seen
#define AXI_DMA_DMASR_DMASlvErr (1 << 5)    // Slave response error seen
#define AXI_DMA_DMASR_DMAIntErr (1 << 4)    // DMA internal error seen
#define AXI_DMA_DMASR_SGIncld   (1 << 3)    // 1 indicates the Scatter Gather engine is included
#define AXI_DMA_DMASR_Idle      (1 << 1)    // Last command completed

/* AXI Base Address Translation Configuration Registers */
#define AXIBAR2PCIEBAR_0U       0X208
#define AXIBAR2PCIEBAR_0L       0X20C

#define DESC_COUNT              4
#define BUFFER_SIZE             64

static int 		pci_probe(struct pci_dev *pdev, const struct pci_device_id *id);
static void 	pci_remove(struct pci_dev *pdev);

static struct pci_device_id pci_id_table[] = { 
    { PCI_DEVICE(PCI_VID, PCI_PID), }, 
    { 0, } 
};

/*
* name 		- Уникальное имя драйвера, которое будет использовано ядром в /sys/bus/pci/drivers
* id_table 	- Таблица пар Vendor ID и Product ID, с которым может работать драйвер.
* probe 	- Функция вызываемая ядром после загрузки драйвера, служит для инициализации оборудования
* remove 	- Функция вызываемая ядром при выгрузке драйвера, служит для освобождения каких-либо ранее занятых ресурсов
*/
static struct pci_driver pci_drv = {
    .name 		= PCI_DRIVER_NAME,
    .id_table 	= pci_id_table,
    .probe 		= pci_probe,
    .remove 	= pci_remove
};

/* Указатель на память ввода/вывода устройства */
struct pci_driver_priv {
    int             irq;
    void __iomem    *base_addr;
    /* resource_size_t - CPU physical addresses */
    resource_size_t start;
    resource_size_t end;
    resource_size_t length;
    resource_size_t flags;
};

static int 		ml605_open(struct inode *inod, struct file *fil);
static int 		ml605_close(struct inode *inod, struct file *fil);
static ssize_t 	ml605_read(struct file *fil, char *buf, size_t len, loff_t *off);
static ssize_t 	ml605_write(struct file *fil, const char *buf, size_t len, loff_t *off);
static int      ml605_mmap(struct file *filp, struct vm_area_struct *vma);
#endif
