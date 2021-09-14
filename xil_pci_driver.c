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

#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#define PCI_DRIVER_NAME		"ML605_PCIe"
#define PCI_VID				0x10EE 	/* Xilinx 			*/
#define PCI_PID				0x0505 	/* Device ID 		*/

#define CDMACR 		  	  	0x00 /* CDMA control register offset */
#define CDMASR 		  	  	0x04 /* CDMA status register offset */
#define SA 		          	0x18 // адрес источника
#define DA 		          	0x20 // адрес назначения
#define BTT 		      	0x28 /* TX Byte Size */




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
MODULE_AUTHOR("Alexey");
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
    int irq;
    void __iomem *base_addr; 
};

/* Регистрации драйвера */
static int __init pci_init(void) { return pci_register_driver(&pci_drv); }

/* Выгрузка драйвера */
static void __exit pci_exit(void) { pci_unregister_driver(&pci_drv); }

void release_device(struct pci_dev *pdev) {
    free_irq(pdev->irq, pdev);
    pci_release_region(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
    pci_disable_device(pdev);
}

static irqreturn_t ml605_pci_irq(int irq, void *lp){
    printk("( ML605 PCIe ) [I] Interrupt Occured\n");
    return IRQ_HANDLED;
}


/* 
* Чтение 8/16/32 битных конфигурационных регистров: 
* int pci_read_config_byte(struct pci_dev *pdev, int where, u8 *ptr);
* int pci_read_config_word(struct pci_dev *pdev, int where, u16 *ptr);
* int pci_read_config_dword(struct pci_dev *pdev, int where, u32 *ptr);
* Запись 8/16/32 битных конфигурационных регистров: 
* int pci_write_config_byte(struct pci_dev *pdev, int where, u8 val);
* int pci_write_config_word(struct pci_dev *pdev, int where, u16 val);
* int pci_write_config_dword(struct pci_dev *pdev, int where, u32 val);
*/

static int pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
    int i = 0;
    int err;
    u16 vendor, device, status;
    u8 irq, cashline, lattimer;
    
    resource_size_t mmio_start;
    resource_size_t mmio_end;
    resource_size_t mmio_len;
    resource_size_t mmio_flags;

    struct pci_driver_priv *bar0_priv = NULL;

    /*
    * This type can hold any valid DMA address for the platform and should be used
    * everywhere you hold a DMA address returned from the DMA mapping functions.
    */
    dma_addr_t dma_handle;

    // Дескриптор буфера для DMA
    struct dma_desc_priv {
        void *addr;
        size_t size;
    } desc;





    pci_read_config_word(pdev, PCI_VENDOR_ID, &vendor); 		// Регистр фирмы изготовителя устройства
    pci_read_config_word(pdev, PCI_DEVICE_ID, &device); 		// Регистр ID устройства
    pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &irq);		// Регистр прерываний
    pci_read_config_word(pdev, PCI_STATUS, &status);			// Регистр статуса
    pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cashline);	// 
    pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &lattimer);	// 

    printk(KERN_INFO "( ML605 PCIe ) [I] Initialization process...");
    printk(KERN_INFO "( ML605 PCIe ) [I] VID/PID  : 0x%04X/0x%04X\n", vendor, device);
    printk(KERN_INFO "( ML605 PCIe ) [I] IRQ Line : 0x%02X\n", irq);
    printk(KERN_INFO "( ML605 PCIe ) [I] STATUS	  : 0x%04X\n", status);

    /* Второй аргумент это один из шести [0-5] базовых адресных регистров (BARs) */
    mmio_start = pci_resource_start(pdev, 0);
    mmio_end = pci_resource_end(pdev, 0);
    mmio_len = pci_resource_len(pdev, 0);
    mmio_flags = pci_resource_flags(pdev, 0);

    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 address range: 0x%llX-0x%llX\n", mmio_start, mmio_end);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 length %lld\n", mmio_len);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 flags 0x%llX\n", mmio_flags);

    /*
    * The following sequence checks if the resource is in the
    * IO/Memory/Interrupt/DMA address space
    */
    if (!(mmio_flags & IORESOURCE_MEM)) {
        printk("( ML605 PCIe ) [E] MEM not found\n");
        return -ENODEV;
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


    /* Запрашиваем необходимый регион памяти
    * Mark the PCI region associated with PCI device pdev BAR bar as being reserved by owner res_name.
    * Do not access any address inside the PCI regions unless this call returns successfully.
    * Returns 0 on success, or EBUSY on error.
    * A warning message is also printed on failure.
    */
    err = pci_request_region(pdev, 0, PCI_DRIVER_NAME);
    if (err) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Request Region: %d\n", err);
        pci_disable_device(pdev);
        return -ENODEV;
    }

    /* Выделяем память */
    bar0_priv = kzalloc(sizeof(struct pci_driver_priv), GFP_KERNEL);
    if (!bar0_priv) {
        release_device(pdev);
        return -ENOMEM;
    }

    /* отображаем выделенную память к аппаратуре */
    bar0_priv->base_addr = pci_iomap(pdev, 0, mmio_len);
    if(!bar0_priv->base_addr) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Memmory error BAR0\n");
        return -ENOMEM;
    }

    pci_set_master(pdev);

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



    /* Set driver private data */
    /* Now we can access mapped "base_addr" from the any driver's function */
    pci_set_drvdata(pdev, bar0_priv);





    /*
    * dma_map_single() could fail and return error.  
    * Doing so will ensure that the mapping code will work correctly on all
    * DMA implementations without any dependency on the specifics of the underlying implementation. 
    * Using the returned address without checking for errors could result in failures ranging from panics to silent data corruption.  
    * The same applies to dma_map_page() as well.
    */
    err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
    if (err) {
    	printk(KERN_INFO "( ML605 PCIe ) [E] DMA Mask error, %d\n", err);
    	return -EIO;
    }

    desc.addr = kzalloc(256, GFP_DMA);
    desc.size = 256;

    dma_handle = dma_map_single(&pdev->dev, desc.addr, desc.size, DMA_BIDIRECTIONAL);

    err = dma_mapping_error(&pdev->dev, dma_handle);
    if (err) {
    	printk(KERN_INFO "( ML605 PCIe ) [E] DMA mapping error, %d\n", err);
        return -EIO;
    }

    printk(KERN_INFO "( ML605 PCIe ) [I] Valid DMA address (dma_addr_t) 0x%llX\n", dma_handle);
    printk(KERN_INFO "( ML605 PCIe ) [I] Valid BAR0 address (bar0_priv->base_addr) 0x%p\n", bar0_priv->base_addr);

    dma_sync_single_for_cpu(&pdev->dev, dma_handle, desc.size, DMA_BIDIRECTIONAL);
    dma_sync_single_for_device(&pdev->dev, dma_handle, desc.size, DMA_BIDIRECTIONAL);

    iowrite32(dma_handle, bar0_priv->base_addr + SA);   // Уставновка адреса источника 
    iowrite32(0xC0000000, bar0_priv->base_addr + DA); 	// Устанвка адреса назначеня, адрес ddr в пространстве CDMA в ПЛИС
    iowrite32(0x100, bar0_priv->base_addr + BTT);		// Передать 100 байт

    mdelay(100);

    printk(KERN_INFO "( ML605 PCIe ) [I] CDMA Control             0x%X\n", ioread32(bar0_priv->base_addr + CDMACR));
    printk(KERN_INFO "( ML605 PCIe ) [I] CDMA Status              0x%X\n", ioread32(bar0_priv->base_addr + CDMASR));
    printk(KERN_INFO "( ML605 PCIe ) [I] CDMA Source Address      0x%X\n", ioread32(bar0_priv->base_addr + SA));
    printk(KERN_INFO "( ML605 PCIe ) [I] CDMA Destination Address 0x%X\n", ioread32(bar0_priv->base_addr + DA));
    printk(KERN_INFO "( ML605 PCIe ) [I] CDMA Payload Size        0x%X\n", ioread32(bar0_priv->base_addr + BTT));



    // i = 0;
    // ожидание завершения передачи от cdma
    // while(!(ioread32(bar0_priv->base_addr + CDMASR) & 0x2)) {
    // 	// если передача так и не завершилось, выдти из ожидания
    // 	if (i == 10000) {
    // 		printk(KERN_INFO "( ML605 PCIe ) [I] ST ERR %X\n",ioread32(bar0_priv->base_addr + CDMASR));
    // 		break;
    // 	}
    // 	else {
    // 		i++;
    // 	}
    // }

    // чтение статусного регистра
    // printk(KERN_INFO "( ML605 PCIe ) [I] ST %X\n", ioread32(bar0_priv->base_addr + CDMASR));


    dma_unmap_single(&pdev->dev, dma_handle, desc.size, DMA_BIDIRECTIONAL);
    kfree(desc.addr);




    /* Проверка записи */
    //iowrite32(0xDEADBEEF, bar0_priv->base_addr);

    /* Проверка чтения */
    //printk(KERN_INFO "( ML605 PCIe ) [I] CDMA Status: 0x%X\n", ioread32(bar0_priv->base_addr + 0xC004));
    //printk(KERN_INFO "( ML605 PCIe ) [I] AXIBAR2PCIEBAR_0U: 0x%X\n", ioread32(bar0_priv->base_addr + 0x8208));
    //printk(KERN_INFO "( ML605 PCIe ) [I] AXIBAR2PCIEBAR_0l: 0x%X\n", ioread32(bar0_priv->base_addr + 0x820C));
    //printk(KERN_INFO "( ML605 PCIe ) [I] AXIBAR2PCIEBAR_1U: 0x%X\n", ioread32(bar0_priv->base_addr + 0x8210));
    //printk(KERN_INFO "( ML605 PCIe ) [I] AXIBAR2PCIEBAR_1L: 0x%X\n", ioread32(bar0_priv->base_addr + 0x8214));












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
    struct pci_driver_priv *bar0_priv = pci_get_drvdata(pdev);
    if (bar0_priv) { 
        if (bar0_priv->base_addr) {
            iounmap(bar0_priv->base_addr);
        }
        pci_free_irq_vectors(pdev);
        kfree(bar0_priv); 
    }
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
