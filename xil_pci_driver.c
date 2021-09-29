#include "xil_pci_driver.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Kosinov");
MODULE_DESCRIPTION("ML605 PCIe x4");
MODULE_VERSION("0.2");
MODULE_DEVICE_TABLE(pci, pci_id_table);

/*
* This type can hold any valid DMA address for the platform and should be used
* everywhere you hold a DMA address returned from the DMA mapping functions.
*/
static dma_addr_t dma_handle = 0;

/* 
* DMA buffer that shares data for AXIBAR
*/
static void *dmabuf = NULL;

struct pci_driver_priv *axi_dma = NULL;
struct pci_driver_priv *axi_pci = NULL;

/* for char device */
static dev_t  first;
static struct class *driver_class = NULL;
static struct cdev ml605_cdev;
static struct device *ml605_device = NULL;
static struct file_operations fops = {
    .owner   = THIS_MODULE,
    .read	 = ml605_read,
    .write	 = ml605_write,
    .open	 = ml605_open,
    .release = ml605_close,
    .mmap    = ml605_mmap
};

/* Driver registration */
static int __init pci_init(void) { return pci_register_driver(&pci_drv); }

/* Driver deinitialization */
static void __exit pci_exit(void) { pci_unregister_driver(&pci_drv); }

/* Gracefully close device */
void release_device(struct pci_dev *pdev) {
    pci_free_irq_vectors(pdev);
    free_irq(pdev->irq, pdev);
    iounmap(axi_dma->base_addr);
    iounmap(axi_pci->base_addr);
    
    dma_free_coherent(ml605_device, DMA_BUFFER_SIZE, dmabuf, dma_handle); // release memory for DMA

    cdev_del(&ml605_cdev);
    unregister_chrdev_region(first, MINORMASK);
    class_destroy(driver_class);

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
    int dev_major;
    u16 vendor, device, status;
    u8 irq, cashline, lattimer;

    /* DMA TEST */
    // int cnt_err = 0;

    

    axi_dma = kzalloc(sizeof(struct pci_driver_priv), GFP_KERNEL);
    if (!axi_dma) {
        release_device(pdev);
        return -ENOMEM;
    }

    axi_pci = kzalloc(sizeof(struct pci_driver_priv), GFP_KERNEL);
    if (!axi_pci) {
        release_device(pdev);
        return -ENOMEM;
    }

    /*
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

    pci_set_drvdata(pdev, axi_dma);
    pci_set_drvdata(pdev, axi_pci);

    /*
    * Set the DMA mask to inform the kernel about your devices DMA addressing capabilities.
    * This call set the mask for both streaming and coherent APIs together.
    * These calls usually return zero to indicated your device can perform DMA
    * properly on the machine given the address mask you provided, but they might
    * return an error if the mask is too small to be supportable on the given
    * system. If it returns non-zero, your device cannot perform DMA properly on
    * this platform, and attempting to do so will result in undefined behavior.
    * You must not use DMA on this device unless the dma_set_mask family of
    * functions has returned success.
    */
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    if (err) {
        printk(KERN_INFO "( ML605 PCIe ) [E] No suitable DMA available, bit mask error, %d\n", err);
        return -EIO;
    }

    pci_read_config_word(pdev, PCI_VENDOR_ID, &vendor);
    pci_read_config_word(pdev, PCI_DEVICE_ID, &device);
    pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &irq);
    pci_read_config_word(pdev, PCI_STATUS, &status);
    pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cashline);
    pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &lattimer);

    /* Second arg is one of six [0-5] BARs */
    axi_dma->start  = pci_resource_start(pdev,  0);
    axi_dma->end    = pci_resource_end(pdev,    0);
    axi_dma->length = pci_resource_len(pdev,    0);
    axi_dma->flags  = pci_resource_flags(pdev,  0);

    axi_pci->start  = pci_resource_start(pdev,  1);
    axi_pci->end    = pci_resource_end(pdev,    1);
    axi_pci->length = pci_resource_len(pdev,    1);
    axi_pci->flags  = pci_resource_flags(pdev,  1);

    printk(KERN_INFO "( ML605 PCIe ) [I] Initialization process...");
    printk(KERN_INFO "( ML605 PCIe ) [I] VID/PID                        : 0x%04X/0x%04X\n", vendor, device);
    printk(KERN_INFO "( ML605 PCIe ) [I] IRQ Line                       : 0x%02X\n", irq);
    printk(KERN_INFO "( ML605 PCIe ) [I] Status	                        : 0x%04X\n", status);
    printk(KERN_INFO "( ML605 PCIe ) [I] Cash Line                      : 0x%02X\n", cashline);
    printk(KERN_INFO "( ML605 PCIe ) [I] Latency Timer                  : 0x%02X\n", lattimer);
    
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 Flags                     : 0x%llX\n", axi_dma->flags);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 Address Range             : 0x%llX-0x%llX\n", axi_dma->start, axi_dma->end);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR0 Length                    : %lld\n", axi_dma->length);

    printk(KERN_INFO "( ML605 PCIe ) [I] BAR1 Flags                     : 0x%llX\n", axi_pci->flags);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR1 Address Range             : 0x%llX-0x%llX\n", axi_pci->start, axi_pci->end);
    printk(KERN_INFO "( ML605 PCIe ) [I] BAR1 Length                    : %lld\n", axi_pci->length);

    /*
    * The following sequence checks if the resource is in the
    * IO/Memory/Interrupt/DMA address space
    */
    if (!(axi_dma->flags & IORESOURCE_MEM)) {
        printk("( ML605 PCIe ) [E] BAR0 IORESOURCE_MEM flag return zero\n");
        return -ENODEV;
    }
  
    if ((axi_dma->base_addr = ioremap(axi_dma->start, axi_dma->length)) == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Host couldn't remap BAR0\n");
        return -ENOMEM;
    }
  
    if (!(axi_pci->flags & IORESOURCE_MEM)) {
        printk("( ML605 PCIe ) [E] BAR1 IORESOURCE_MEM flag return zero\n");
        return -ENODEV;
    }
  
    if ((axi_pci->base_addr = ioremap(axi_pci->start, axi_pci->length)) == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Host couldn't remap BAR1\n");
        return -ENOMEM;
    }

    /* Enable MSI IRQ */
    if (!pci_enable_msi(pdev)) {
        if (!pdev->msi_enabled) { 
            printk(KERN_INFO "( ML605 PCIe ) [E] MSI interrupt disabled\n");
        }
    }

    /* 
    * Request and setup IRQ
    * Check: cat /proc/interrupts/[PCI_DRIVER_NAME]
    */
    err = request_irq(pdev->irq, ml605_pci_irq, IRQF_SHARED, PCI_DRIVER_NAME, pdev);
    if (err < 0) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Could not request MSI interrupt #%d, error %d\n", pdev->irq, err);
    }

    /*
    * This routine will allocate RAM for that region, so it acts similarly to
    * __get_free_pages() (but takes size instead of a page order).  If your
    * driver needs regions sized smaller than a page, you may prefer using
    * the dma_pool interface, described below.
    * dma_alloc_coherent() returns two values: the virtual address which you
    * can use to access it from the CPU and dma_handle which you pass to the
    * card.
    */
    if ((dmabuf = dma_alloc_coherent(&pdev->dev, DMA_BUFFER_SIZE, &dma_handle, GFP_KERNEL)) == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] DMA Allocates a region of %d bytes of memory failed\n", DMA_BUFFER_SIZE);
        return -ENODEV;
    }

    /* Fill DMA buffer with test values */
    // dmabuf = kmalloc(BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
    // for (i = 0; i < BUFFER_SIZE; i+=4) { *(u32*)(dmabuf + i) = 0xAAAAAAAA; }

    // dma_handle = dma_map_single(&pdev->dev, dmabuf, BUFFER_SIZE, DMA_TO_DEVICE);
    // if (dma_mapping_error(&pdev->dev, dma_handle)) {
    //     printk(KERN_INFO "( ML605 PCIe ) [E] DMA Map Single Error\n");
    //     kfree(dmabuf);
    //     return -ENODEV;
    // }

    printk(KERN_INFO "( ML605 PCIe ) [I] DMA Physical Memory Address    : 0x%p\n", (void *)dma_handle);

    /*
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

    err = pci_request_region(pdev, 1, PCI_DRIVER_NAME);
    if (err) {
        printk(KERN_INFO "( ML605 PCIe ) [E] PCIe request regions failed, %d\n", err);
        pci_disable_device(pdev);
        return -ENODEV;
    }

    pci_set_master(pdev);

    printk(KERN_INFO "( ML605 PCIe ) [I] Read PCI PID/VID Register      : 0x%08X\n", ioread32(axi_pci->base_addr));

    /* 
    * When the BAR is set to a 32-bit address space, then the translation vector should be placed into the
    * AXIBAR2PCIEBAR_nL register where n is the BAR number. When the BAR is set to a 64-bit
    * address space, then the translation’s most significant 32 bits are written into the
    * AXIBAR2PCIEBAR_nU and the least significant 32 bits are written into AXIBAR2PCIEBAR_nL.
    */
    iowrite32( (dma_handle >> 32)        , axi_pci->base_addr + AXIBAR2PCIEBAR_0U);
    iowrite32( (dma_handle  & 0xFFFFFFFF), axi_pci->base_addr + AXIBAR2PCIEBAR_0L);

    printk(KERN_INFO "( ML605 PCIe ) [I] AXI BAR Address (new)          : 0x%08X%08X\n", ioread32(axi_pci->base_addr + AXIBAR2PCIEBAR_0U), 
                                                                                         ioread32(axi_pci->base_addr + AXIBAR2PCIEBAR_0L));

    /*
    // * If you need to use the same streaming DMA region multiple times and touch
    // * the data in between the DMA transfers, the buffer needs to be synced
    // * properly in order for the CPU and device to see the most up-to-date and
    // * correct copy of the DMA buffer.
    // * After each DMA transfer call:
    */
    dma_sync_single_for_cpu(&pdev->dev, dma_handle, DMA_BUFFER_SIZE, DMA_TO_DEVICE);
    
    for(i = 0; i < 64; i+=4){
        printk(KERN_INFO "( ML605 PCIe ) [I] %d : 0x%08X\n", i, *(u32 *)(dmabuf + i));
    }

    /*
    // * Then, if you wish to let the device get at the DMA area again,
    // * finish accessing the data with the CPU, and then before actually
    // * giving the buffer to the hardware call:
    */
    dma_sync_single_for_device(&pdev->dev, dma_handle, DMA_BUFFER_SIZE, DMA_TO_DEVICE);






    /* Char device initialization */

    /* 
    * Allocates a range of char device numbers
    * The major number will be chosen dynamically, and returned (along with the first minor number) in dev
    * Returns zero or a negative error code
    */
    if (alloc_chrdev_region(&first, 0, 1, PCI_DRIVER_NAME) < 0) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Major number allocation is failed\n");
        return -1;
    }

    dev_major = MAJOR(first);

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
    * Initializes cdev, remembering fops, making it ready to add to the system with cdev_add
    */
    cdev_init(&ml605_cdev, &fops);
    if (cdev_add(&ml605_cdev, MKDEV(dev_major, 0), 1) == -1){
        printk(KERN_INFO "( ML605 PCIe ) [E] Create character device failed\n");
        device_destroy(driver_class, first);
        class_destroy(driver_class);
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
    ml605_device = device_create(driver_class, NULL, MKDEV(dev_major, 0), NULL, "ml605_pcie-0");
    if (ml605_device == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Create device failed\n");
        class_destroy(driver_class);
        unregister_chrdev_region(first, 1);
        return -1;
    }

    return 0;
}



/* Clean up */
static void pci_remove(struct pci_dev *pdev) {
    release_device(pdev);
}


/* char device functions */
static int ml605_open(struct inode *inod, struct file *fil){
    printk(KERN_INFO "( ML605 PCIe ) [I] Char device opened\n");
    return 0;
}

static int ml605_close(struct inode *inod, struct file *fil){
    printk(KERN_INFO "( ML605 PCIe ) [I] Char device closed\n");
    return 0;
}

static ssize_t ml605_read(struct file *fil, char *buf, size_t len, loff_t *off){
    unsigned long res;
    dma_sync_single_for_cpu(ml605_device, dma_handle, len, DMA_FROM_DEVICE);
    res = copy_to_user(buf, dmabuf, len);
    dma_sync_single_for_device(ml605_device, dma_handle, len, DMA_FROM_DEVICE);

    if (res != 0) {
        printk( "( ML605 PCIe ) [E] Failed to send characters to user, %ld\n", res);
        return -EFAULT;
    }
    printk(KERN_INFO "( ML605 PCIe ) [I] Performing read operation, %ld\n", res);
    return 0;
}

static ssize_t ml605_write(struct file *fil, const char *buf, size_t len, loff_t *off){
    unsigned long res;
    dma_sync_single_for_cpu(ml605_device, dma_handle, len, DMA_TO_DEVICE); 

    res = copy_from_user(dmabuf, buf, len);

    dma_sync_single_for_device(ml605_device, dma_handle, len, DMA_TO_DEVICE);

    if (res != 0) {
        printk( "( ML605 PCIe ) [E] Failed to send characters to user, %ld\n", res);
        return -EFAULT;
    }

    printk(KERN_INFO "( ML605 PCIe ) [I] Performing write operation, %ld\n", res);
    return 0;
}

static int ml605_mmap(struct file *filp, struct vm_area_struct *vma) {

    unsigned long vsize;
    unsigned long off;
    int res;

    off = vma->vm_pgoff;
    vsize = vma->vm_end - vma->vm_start;

    if (off < 0) {
        printk(KERN_INFO "( ML605 PCIe ) [E] vm_pgoff empty\n");
        return -EINVAL;
    }

    if (vsize > DMA_BUFFER_SIZE) {
        printk(KERN_INFO "( ML605 PCIe ) [E] vsize (%ld) > DMA_BUFFER_SIZE\n", vsize);
        return -EINVAL;
    }

    printk(KERN_INFO "( ML605 PCIe ) [I] Mapping with dma_map_coherent DMA buffer at phys: 0x%p virt 0x%p\n", (void *)dma_handle, dmabuf);
    res = dma_mmap_coherent(ml605_device, vma, dmabuf, dma_handle,  vsize);

    if (res) {
        printk(KERN_INFO "( ML605 PCIe ) [E] Unable to map DMA buffer at phys: 0x%p\n", (void *)dma_handle);
    }
    return res;
}

module_init(pci_init);
module_exit(pci_exit);
