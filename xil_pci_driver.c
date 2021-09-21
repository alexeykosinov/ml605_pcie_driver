// Урок https://habr.com/ru/post/348042/
// Linux Device Drivers, PCI chapter https://static.lwn.net/images/pdf/LDD3/ch12.pdf
// https://betontalpfa.medium.com/oh-no-i-need-to-write-a-pci-driver-2b389720a9d0


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

#define PCI_DRIVER_NAME		    "ML605_PCIe"
#define PCI_VID				    0x10EE 	    /* Xilinx 			                */
#define PCI_PID				    0x0505 	    /* Device ID 		                */


#define DMA_BUFFER_SIZE         8192

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

#define AXIBAR2PCIEBAR_0U       0X208
#define AXIBAR2PCIEBAR_0L       0X20C

static void *dmabuf = NULL;

/*
* This type can hold any valid DMA address for the platform and should be used
* everywhere you hold a DMA address returned from the DMA mapping functions.
*/
static dma_addr_t dma_handle = 0;

/* Символьное устройство */
static dev_t first;
static struct class *driver_class = NULL;
static struct cdev ml605_cdev;
static struct device *ml605_device;

static struct pci_device_id pci_id_table[] = { 
    { PCI_DEVICE(PCI_VID, PCI_PID), }, 
    { 0, } 
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Kosinov");
MODULE_DESCRIPTION("ML605 PCIe x4");
MODULE_VERSION("0.1");
MODULE_DEVICE_TABLE(pci, pci_id_table);

static int 		pci_probe(struct pci_dev *pdev, const struct pci_device_id *id);
static void 	pci_remove(struct pci_dev *pdev);

static int 		ml605_open(struct inode *inod, struct file *fil);
static int 		ml605_close(struct inode *inod, struct file *fil);
static ssize_t 	ml605_read(struct file *fil, char *buf, size_t len, loff_t *off);
static ssize_t 	ml605_write(struct file *fil, const char *buf, size_t len, loff_t *off);

static struct file_operations fops = {
    .read	 = ml605_read,
    .write	 = ml605_write,
    .open	 = ml605_open,
    .release = ml605_close,
};


/*
* name 		— Уникальное имя драйвера, которое будет использовано ядром в /sys/bus/pci/drivers
* id_table 	— Таблица пар Vendor ID и Product ID, с которым может работать драйвер.
* probe 	— Функция вызываемая ядром после загрузки драйвера, служит для инициализации оборудования
* remove 	— Функция вызываемая ядром при выгрузке драйвера, служит для освобождения каких-либо ранее занятых ресурсов
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
    resource_size_t start;
    resource_size_t end;
    resource_size_t length;
    resource_size_t flags;
};

struct pci_driver_priv *axi_dma = NULL;


/* Регистрации драйвера */
static int __init pci_init(void) { return pci_register_driver(&pci_drv); }

/* Выгрузка драйвера */
static void __exit pci_exit(void) { pci_unregister_driver(&pci_drv); }

void release_device(struct pci_dev *pdev) {
    pci_free_irq_vectors(pdev);
    free_irq(pdev->irq, pdev);
    iounmap(axi_dma->base_addr);
    dma_free_coherent(&pdev->dev, DMA_BUFFER_SIZE, dmabuf, dma_handle);
    pci_release_region(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
    pci_disable_device(pdev);
}

static irqreturn_t ml605_pci_irq(int irq, void *lp){
    printk("( ML605 PCIe ) [I] Interrupt Occured\n");
    return IRQ_HANDLED;
}

static int pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
    int i;
    int err;
    u16 vendor, device, status;
    // unsigned short reg16;
    u8 irq, cashline, lattimer;

    axi_dma = kzalloc(sizeof(struct pci_driver_priv), GFP_KERNEL);
    if (!axi_dma) {
        release_device(pdev);
        return -ENOMEM;
    }

    /* Инициализируем память устройства
    * Initialize device before it's used by a driver
    * Ask low-level code to enable Memory resources
    * Wake up the device if it was suspended
    * Beware, this function can fail
    */
    err = pci_enable_device(pdev);
    if (err) { 
        printk(KERN_INFO "( ML605 PCIe ) [E] Enabling the PCI Device: %d\n", err);
        return err;
    }

    // pci_read_config_word(pdev, PCI_COMMAND, &reg16);
	// reg16 |= PCI_COMMAND_MASTER | PCI_COMMAND_IO | PCI_COMMAND_MEMORY;
	// pci_write_config_word(pdev, PCI_COMMAND, reg16);


    pci_set_drvdata(pdev, axi_dma);

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
    if (err) {
        printk(KERN_INFO "pci_set_dma_mask error.\n");
        return -EIO;
    }
    
	err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
    if (err) {
        printk(KERN_INFO "pci_set_consistent_dma_mask error.\n");
        return -ENODEV;
    }

    pci_read_config_word(pdev, PCI_VENDOR_ID, &vendor); 		// Регистр фирмы изготовителя устройства
    pci_read_config_word(pdev, PCI_DEVICE_ID, &device); 		// Регистр ID устройства
    pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &irq);		// Регистр прерываний
    pci_read_config_word(pdev, PCI_STATUS, &status);			// Регистр статуса
    pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cashline);	// 
    pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &lattimer);	// 
    /* Второй аргумент это один из шести [0-5] базовых адресных регистров (BARs) */
    axi_dma->start  = pci_resource_start(pdev,  0);
    axi_dma->end    = pci_resource_end(pdev,    0);
    axi_dma->length = pci_resource_len(pdev,    0);
    axi_dma->flags  = pci_resource_flags(pdev,  0);

    printk(KERN_INFO "( ML605 PCIe ) [I] Initialization process...");
    printk(KERN_INFO "( ML605 PCIe ) [I] VID/PID            : 0x%04X/0x%04X\n", vendor, device);
    printk(KERN_INFO "( ML605 PCIe ) [I] IRQ Line           : 0x%02X\n", irq);
    printk(KERN_INFO "( ML605 PCIe ) [I] Status	            : 0x%04X\n", status);
    printk(KERN_INFO "( ML605 PCIe ) [I] Cash Line          : 0x%02X\n", cashline);
    printk(KERN_INFO "( ML605 PCIe ) [I] Latency Timer      : 0x%02X\n", lattimer);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 Flags         : 0x%llX\n", axi_dma->flags);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 Address Range : 0x%llX-0x%llX\n", axi_dma->start, axi_dma->end);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 Length        : %lld\n", axi_dma->length);


    /*
    * The following sequence checks if the resource is in the
    * IO/Memory/Interrupt/DMA address space
    */
    if (!(axi_dma->flags & IORESOURCE_MEM)) {
        printk("( ML605 PCIe ) [E] CDMA MEM not found\n");
        return -ENODEV;
    }
  
    if ((axi_dma->base_addr = ioremap(axi_dma->start, axi_dma->length)) == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Memmory error BAR1\n");
        return -ENOMEM;
    }
  
    /* Включение MSI прерываний */
    if (!pci_enable_msi(pdev)) {
        if (!pdev->msi_enabled) { 
            printk(KERN_INFO "( ML605 PCIe ) [E] MSI interrupt disabled\n");
        }
    }

    /* 
    * Запрос и установка коллбека функции прерываний 
    * Проверить корректность можно через cat /proc/interrupts
    * Там должен быть устройство с названием PCI_DRIVER_NAME
    * Там же можно увидеть номер прерывания
    */
    err = request_irq(pdev->irq, ml605_pci_irq, IRQF_SHARED, PCI_DRIVER_NAME, pdev);
    if (err < 0) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Could not request MSI interrupt #%d, error %d\n", pdev->irq, err);
    }

    if ((dmabuf = dma_alloc_coherent(&pdev->dev, DMA_BUFFER_SIZE, &dma_handle, GFP_USER)) == NULL) {
        printk(KERN_INFO "dma_alloc_coherent error.\n");
        return -ENODEV;
    }


    /* Запрашиваем необходимый регион памяти
    * Mark the PCI region associated with PCI device pdev BAR bar as being reserved by owner res_name.
    * Do not access any address inside the PCI regions unless this call returns successfully.
    * Returns 0 on success, or EBUSY on error.
    * A warning message is also printed on failure.
    */
    err = pci_request_region(pdev, 0, PCI_DRIVER_NAME);
    if (err) {
        printk(KERN_INFO "( ML605 PCIe ) [E] PCIe request regions failed, %d\n", err);
        pci_disable_device(pdev);
        return -ENODEV;
    }

    pci_set_master(pdev);

    printk(KERN_INFO "( ML605 PCIe ) [I] MM2S DMA Control Register       : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_DMACR));
    printk(KERN_INFO "( ML605 PCIe ) [I] MM2S DMA Status Register        : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_DMASR));
    printk(KERN_INFO "( ML605 PCIe ) [I] MM2S Current Descriptor Pointer : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_CURDESC));
    printk(KERN_INFO "( ML605 PCIe ) [I] MM2S Tail Descriptor Pointer    : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_TAILDESC));
    printk(KERN_INFO "( ML605 PCIe ) [I] Scatter/Gather User and Cache   : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_SG_CTL));
    printk(KERN_INFO "( ML605 PCIe ) [I] S2MM DMA Control Register       : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_S2MM_DMACR));
    printk(KERN_INFO "( ML605 PCIe ) [I] S2MM DMA Status Register        : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_S2MM_DMASR));
    printk(KERN_INFO "( ML605 PCIe ) [I] S2MM Current Descriptor Pointer : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_S2MM_CURDESC));
    printk(KERN_INFO "( ML605 PCIe ) [I] S2MM Tail Descriptor Pointer    : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_S2MM_TAILDESC));

    dma_sync_single_for_cpu(&pdev->dev, dma_handle, DMA_BUFFER_SIZE, DMA_TO_DEVICE);

    // for(i = 0; i < 256; i++) {
    //     *(u32*)(dmabuf + i)= 0xDEADBEEF;
    // }

    dma_sync_single_for_device(&pdev->dev, dma_handle, DMA_BUFFER_SIZE, DMA_TO_DEVICE);

    // iowrite32(dma_handle, axi_dma->base_addr + CDMA_SA);   // Уставновка адреса источника 
    // iowrite32(0xC0000000, axi_dma->base_addr + CDMA_DA); 	// Устанвка адреса назначеня, адрес ddr в пространстве CDMA в ПЛИС
    // iowrite32(0x100, axi_dma->base_addr + CDMA_BTT);		// Передать 100 байт

    // mdelay(100);

    // i = 0;
    // while(!(ioread32(axi_dma->base_addr + AXI_DMA_MM2S_DMASR) & AXI_DMA_DMASR_MASK_IDLE)) {
    // 	if (i == 10000) {
    // 		printk(KERN_INFO "( ML605 PCIe ) [I] CDMASR IDLE : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_DMASR));
    // 		break;
    // 	}
    // 	else {
    // 		i++;
    // 	}
    // }



    /* Доступ к устройству со стороны пользовательского пространства */

    /* 
    * Allocates a range of char device numbers
    * The major number will be chosen dynamically, and returned (along with the first minor number) in dev
    * Returns zero or a negative error code
    */
    if (alloc_chrdev_region(&first, 0, 1, PCI_DRIVER_NAME) < 0) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Allocates a range of char device numbers is failed\n");
        return -1;
    }

    /*
    * This is used to create a struct class pointer that can then be used in calls to device_create.
    * Returns struct class pointer on success, or ERR_PTR on error.
    * Note, the pointer created here is to be destroyed when finished by making a call to class_destroy.
    */
    driver_class = class_create(THIS_MODULE, "ML605_DRIVER_CLASS");
    if (driver_class == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Create class failed\n");
        unregister_chrdev_region(first, 1);
        return -1;
    }

    /* 
    * Simple interfaces attached to a subsystem. 
    * Multiple interfaces can attach to a subsystem and its devices. 
    * Unlike drivers, they do not exclusively claim or control devices. 
    * Interfaces usually represent a specific functionality of a subsystem/class of devices.
    * Y'll see the name in /dev/
    */
    ml605_device = device_create(driver_class, NULL, first, NULL, "ml605");
    if (ml605_device == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Create device failed\n");
        class_destroy(driver_class);
        unregister_chrdev_region(first, 1);
        return -1;
    }

    /*
    * Initializes cdev, remembering fops, making it ready to add to the system with cdev_add
    */
    cdev_init(&ml605_cdev, &fops);
    if (cdev_add(&ml605_cdev, first, 1) == -1){
        printk(KERN_INFO "( ML605 PCIe ) [E] Create character device failed\n");
        device_destroy(driver_class, first);
        class_destroy(driver_class);
        unregister_chrdev_region(first, 1);
        return -1;
    }

    return 0;

}



/* Освобождение занятых ресурсов */
static void pci_remove(struct pci_dev *pdev) {
    release_device(pdev);
}

static int ml605_open(struct inode *inod, struct file *fil){
    printk(KERN_INFO "( ML605 PCIe ) [I] Char device opened\n");
    return 0;
}

static int ml605_close(struct inode *inod, struct file *fil){
    printk(KERN_INFO "( ML605 PCIe ) [I] Char device closed\n");
    return 0;
}

static ssize_t ml605_read(struct file *fil, char *buf, size_t len, loff_t *off){
    // int n;
    // Return the result only once (otherwise a simple cat will loop)
    // Copy from kernel space to user space
    // printk(KERN_INFO "( ML605 PCIe ) [I] Reading device rx: %d\n", (int) len);

    // n = sprintf(ker_buf, "%d\n", *regA);
    // Copy back to user the result (to,from,size)
    // copy_to_user(buf, ker_buf, n);
    // printk(KERN_INFO "( ML605 PCIe ) [I] Returning %s rx: %d\n", ker_buf, n);
    printk(KERN_INFO "( ML605 PCIe ) [I] Performing read operation\n");
    // return n;
    return 0;
}

static ssize_t ml605_write(struct file *fil, const char *buf, size_t len, loff_t *off){
    // Get data from user space to kernel space
    // copy_from_user(ker_buf, buf, len);
    // sscanf(ker_buf,"%d%c", &operand_1);
    // ker_buf[len] = 0;
    // Change the IP registers to the parsed operands (on rega and regb)
    // *regA = (unsigned int) operand_1;
    printk(KERN_INFO "( ML605 PCIe ) [I] Performing write operation\n");
    // return len;
    return 0;
}



module_init(pci_init);
module_exit(pci_exit);
