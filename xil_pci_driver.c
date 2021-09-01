// Урок https://habr.com/ru/post/348042/
// Linux Device Drivers, PCI chapter https://static.lwn.net/images/pdf/LDD3/ch12.pdf
// https://betontalpfa.medium.com/oh-no-i-need-to-write-a-pci-driver-2b389720a9d0

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>


#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#define PCI_DRIVER_NAME		"ML605_PCIe"
#define PCI_VID				0x10EE 	/* Xilinx 			*/
#define PCI_PID				0x0505 	/* Device ID 		*/

/* Character device */
static dev_t first;
static struct class *driver_class = NULL;
static struct cdev ml605_cdev; // Global variable for the character device
static struct device *ml605_device;


// static char ker_buf[100];
// static int operand_1 = 0;

// volatile unsigned int *regA;


static struct pci_device_id pci_id_table[] = { 
	{ PCI_DEVICE(PCI_VID, PCI_PID), }, 
	{ 0, } 
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Kosinov");
MODULE_DESCRIPTION("ML605 PCIe x4");
MODULE_VERSION("0.1");
MODULE_DEVICE_TABLE(pci, pci_id_table);


static int pci_probe(struct pci_dev *pdev, const struct pci_device_id *id);
static void pci_remove(struct pci_dev *pdev);


/* Character callbacks prototype */

static int 		ml_open(struct inode *inod, struct file *fil);
static int 		ml_close(struct inode *inod, struct file *fil);
static ssize_t 	ml_rd(struct file *fil, char *buf, size_t len, loff_t *off);
static ssize_t 	ml_wr(struct file *fil, const char *buf, size_t len, loff_t *off);

static struct file_operations fops = {
	.read	 = ml_rd,
	.write	 = ml_wr,
	.open	 = ml_open,
	.release = ml_close,
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
static int __init pci_init(void) {
	return pci_register_driver(&pci_drv); 
}

/* Выгрузка драйвера */
static void __exit pci_exit(void) { pci_unregister_driver(&pci_drv); }

void release_device(struct pci_dev *pdev) {
	pci_release_region(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	pci_disable_device(pdev);
}

// static irqreturn_t ml605_pci_irq(int irq, void *lp){
//     printk("( ML605 PCIe ) [I] Interrupt Occured\n");
//     return IRQ_HANDLED;
// }


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

	int err;
	u16 vendor, device, status;
	u8 irq, cashline, lattimer;
	unsigned long mmio_start;
	unsigned long mmio_end;
	unsigned long mmio_len;
	unsigned long mmio_flags;

	struct pci_driver_priv *test_priv = NULL;

	/* В качестве теста пробуем считать VID и PID устройства */
	pci_read_config_word(pdev, PCI_VENDOR_ID, &vendor); // Чтение фирмы изготовителя устройства
	pci_read_config_word(pdev, PCI_DEVICE_ID, &device); // Чтения ID устройства
	pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &irq);
	pci_read_config_word(pdev, PCI_STATUS, &status);
	pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cashline);
	pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &lattimer);

	printk(KERN_INFO "( ML605 PCIe ) [I] Initialization process...");
	printk(KERN_INFO "( ML605 PCIe ) [I] VID/PID  : 0x%04X/0x%04X\n", vendor, device);
	printk(KERN_INFO "( ML605 PCIe ) [I] IRQ Line : 0x%02X\n", irq);
	printk(KERN_INFO "( ML605 PCIe ) [I] STATUS	  : 0x%04X\n", status);


	/* Инициализируем память устройства
	* Initialize device before it's used by a driver.s
	* Ask low-level code to enable Memory resources.
	* Wake up the device if it was suspended.
	* Beware, this function can fail.
	*/
	err = pci_enable_device_mem(pdev);
	//err = pci_enable_device(pdev);
	//err = pcim_enable_device(pdev);
	if (err) { 
		printk(KERN_INFO "( ML605 PCIe ) [E] Enabling the PCI Device: %d\n", err);
		return err; 
	}

    // err = request_irq(test_priv->irq, &ml605_pci_irq, 0, "ML605_Driver", NULL);
	// if (err <= 0) {
	// 	printk(KERN_INFO "( ML605 PCIe ) [E] Could not allocate interrupt %d : %d\n", test_priv->irq, err);
	// 	return -ENODEV;
	// }


	/* Второй аргумент это один из шести 0-5 base address registers (BARs) */
	mmio_start = pci_resource_start(pdev, 0);
	mmio_end = pci_resource_end(pdev, 0);
	mmio_len = pci_resource_len(pdev, 0);
	mmio_flags = pci_resource_flags(pdev, 0);

	printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 address range: %lX-%lX\n", mmio_start, mmio_end);
	printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 length %ld\n", mmio_len);
	printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 flags %lX\n", mmio_flags);

    /*
    * The following sequence checks if the resource is in the
    * IO/Memory/Interrupt/DMA address space
    */
   
    if(mmio_flags & IORESOURCE_IO) {
        printk("( ML605 PCIe ) [I] IORESOURCE_IO\n");
    }
    else if (mmio_flags & IORESOURCE_MEM) {
		if (mmio_flags & IORESOURCE_CACHEABLE) {
			printk("( ML605 PCIe ) [I] IORESOURCE_MEM (cacheable)\n");
		}
		else {
			printk("( ML605 PCIe ) [I] IORESOURCE_MEM (non-cacheable)\n");
		}
    }
    else if (mmio_flags & IORESOURCE_IRQ) {
        printk("( ML605 PCIe ) [I] IORESOURCE_IRQ\n");
    }
    else if (mmio_flags & IORESOURCE_DMA) {
        printk("( ML605 PCIe ) [I] IORESOURCE_DMA\n");
    }
    else {
        printk("( ML605 PCIe ) [I] NOTHING\n");
    }
	

	/* Запрашиваем необходимый регион памяти, с определенным ранее типом
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

	test_priv = kzalloc(sizeof(struct pci_driver_priv), GFP_KERNEL);
	if (!test_priv) {
		release_device(pdev);
		return -ENOMEM;
	}

	/* отображаем выделенную память к аппаратуре */
	test_priv->base_addr = ioremap(mmio_start, mmio_len);
	if (!test_priv->base_addr) {
		release_device(pdev);
		return -EIO;
	}

	pci_set_drvdata(pdev, test_priv);

    // ****************** NORMAL Device diver *************************
    // register a range of char device numbers
    if (alloc_chrdev_region(&first, 0, 1, PCI_DRIVER_NAME) < 0) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Allocates a range of char device numbers is failed\n");
        return -1;
    }

    // Create class (/sysfs)
    driver_class = class_create(THIS_MODULE, "ML605_DRIVER_CLASS");
    if (driver_class == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Create class failed\n");
        unregister_chrdev_region(first, 1);
        return -1;
    }

	// Y'll see the name in /dev/
    ml605_device = device_create(driver_class, NULL, first, NULL, "ml605");
    if (ml605_device == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Create device failed\n");
        class_destroy(driver_class);
        unregister_chrdev_region(first, 1);
        return -1;
    }

    // Create a character device /dev/tutDevice
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
	struct pci_driver_priv *test_priv = pci_get_drvdata(pdev);
	if (test_priv) { kfree(test_priv); } // освободить память ввода/вывода
	pci_disable_device(pdev);
}



static int ml_open(struct inode *inod, struct file *fil){
    printk(KERN_INFO "( ML605 PCIe ) [I] Char device opened\n");
    return 0;
}

static int ml_close(struct inode *inod, struct file *fil){
    printk(KERN_INFO "( ML605 PCIe ) [I] Char device closed\n");
    return 0;
}

// Just send to the user a string with the value of result
static ssize_t ml_rd(struct file *fil, char *buf, size_t len, loff_t *off){
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

// Parse the input stream ex: "50,2,*" to some operand variables.
static ssize_t ml_wr(struct file *fil, const char *buf, size_t len, loff_t *off){
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
