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

static void *dmabuf = NULL;

struct pci_driver_priv *axi_dma = NULL;
struct pci_driver_priv *axi_pci = NULL;

/* for char device */
static dev_t  first;
static struct class *driver_class = NULL;
static struct cdev ml605_cdev;
static struct device *ml605_device;
static struct file_operations fops = {
    .read	 = ml605_read,
    .write	 = ml605_write,
    .open	 = ml605_open,
    .release = ml605_close,
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
    dma_free_coherent(&pdev->dev, DMA_BUFFER_SIZE, dmabuf, dma_handle); // release memory for DMA
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
    u8 irq, cashline, lattimer;

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

    pci_read_config_word(pdev, PCI_VENDOR_ID, &vendor); 		// PCI Vendor ID, Device ID
    pci_read_config_word(pdev, PCI_DEVICE_ID, &device); 		// Регистр ID устройства
    pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &irq);		// Регистр прерываний
    pci_read_config_word(pdev, PCI_STATUS, &status);			// Регистр статуса
    pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cashline);	// Регистр размера кеш-линии
    pci_read_config_byte(pdev, PCI_LATENCY_TIMER, &lattimer);	// Задержка для устройсва

    /* Второй аргумент это один из шести [0-5] базовых адресных регистров (BARs) */
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

    /*
    * This routine will allocate RAM for that region, so it acts similarly to
    * __get_free_pages() (but takes size instead of a page order).  If your
    * driver needs regions sized smaller than a page, you may prefer using
    * the dma_pool interface, described below.
    * dma_alloc_coherent() returns two values: the virtual address which you
    * can use to access it from the CPU and dma_handle which you pass to the
    * card.
    */
    if ((dmabuf = dma_alloc_coherent(&pdev->dev, DMA_BUFFER_SIZE, &dma_handle, GFP_DMA)) == NULL) {
        printk(KERN_INFO "( ML605 PCIe ) [E] DMA Allocates a region of %d bytes of memory failed\n", DMA_BUFFER_SIZE);
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

    err = pci_request_region(pdev, 1, PCI_DRIVER_NAME);
    if (err) {
        printk(KERN_INFO "( ML605 PCIe ) [E] PCIe request regions failed, %d\n", err);
        pci_disable_device(pdev);
        return -ENODEV;
    }

    pci_set_master(pdev);

    // printk(KERN_INFO "( ML605 PCIe ) [I] MM2S DMA Control Register          : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_DMACR));
    // printk(KERN_INFO "( ML605 PCIe ) [I] MM2S DMA Status Register           : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_DMASR));
    // printk(KERN_INFO "( ML605 PCIe ) [I] MM2S Current Descriptor Pointer    : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_CURDESC));
    // printk(KERN_INFO "( ML605 PCIe ) [I] MM2S Tail Descriptor Pointer       : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_MM2S_TAILDESC));
    // printk(KERN_INFO "( ML605 PCIe ) [I] Scatter/Gather User and Cache      : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_SG_CTL));
    // printk(KERN_INFO "( ML605 PCIe ) [I] S2MM DMA Control Register          : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_S2MM_DMACR));
    // printk(KERN_INFO "( ML605 PCIe ) [I] S2MM DMA Status Register           : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_S2MM_DMASR));
    // printk(KERN_INFO "( ML605 PCIe ) [I] S2MM Current Descriptor Pointer    : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_S2MM_CURDESC));
    // printk(KERN_INFO "( ML605 PCIe ) [I] S2MM Tail Descriptor Pointer       : 0x%08X\n", ioread32(axi_dma->base_addr + AXI_DMA_S2MM_TAILDESC));

    printk(KERN_INFO "( ML605 PCIe ) [I] Read PCI PID/VID Register          : 0x%08X\n", ioread32(axi_pci->base_addr));

    printk(KERN_INFO "( ML605 PCIe ) [I] AXI Base Address Translation Configuration\n");
    printk(KERN_INFO "( ML605 PCIe ) [I] Current Lower Address of BAR0      : 0x%08X\n", ioread32(axi_pci->base_addr + AXIBAR2PCIEBAR_0L));
    printk(KERN_INFO "( ML605 PCIe ) [I] Current Upper Address of BAR0      : 0x%08X\n", ioread32(axi_pci->base_addr + AXIBAR2PCIEBAR_0U));

    /* 
    * When the BAR is set to a 32-bit address space, then the translation vector should be placed into the
    * AXIBAR2PCIEBAR_nL register where n is the BAR number. When the BAR is set to a 64-bit
    * address space, then the translation’s most significant 32 bits are written into the
    * AXIBAR2PCIEBAR_nU and the least significant 32 bits are written into AXIBAR2PCIEBAR_nL.
    */

    for (i = 0; i < 256; i+=4) { 
        *(u32*)(dmabuf + i)= 0xDEADBEEF;
    }

    printk(KERN_INFO "( ML605 PCIe ) [I] DMA Virtual Memory Address         : 0x%p\n", dmabuf);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMA Physical Memory Address        : 0x%p\n", (void *)dma_handle);

    iowrite32( (dma_handle >> 32)        , axi_pci->base_addr + AXIBAR2PCIEBAR_0U);
    iowrite32( (dma_handle  & 0xFFFFFFFF), axi_pci->base_addr + AXIBAR2PCIEBAR_0L);

    printk(KERN_INFO "( ML605 PCIe ) [I] Updated Upper Address of BAR0      : 0x%08X\n", ioread32(axi_pci->base_addr + AXIBAR2PCIEBAR_0U));
    printk(KERN_INFO "( ML605 PCIe ) [I] Updated Lower Address of BAR0      : 0x%08X\n", ioread32(axi_pci->base_addr + AXIBAR2PCIEBAR_0L));


    // DMA_Print_DMASR(axi_dma, 0);
    // DMA_Print_DMASR(axi_dma, 1);
    // DMA_Print_DMACR(axi_dma, 0);
    // DMA_Print_DMACR(axi_dma, 1);


    /* Reset MM2S & S2MM DMA */
    // iowrite32(AXI_DMA_DMACR_Reset, axi_dma->base_addr + AXI_DMA_MM2S_DMACR);
    // iowrite32(AXI_DMA_DMACR_Reset, axi_dma->base_addr + AXI_DMA_S2MM_DMACR);

    // iowrite32(AXI_DMA_DMACR_IOC_IrqEn | AXI_DMA_DMACR_ERR_IrqEn, axi_dma->base_addr + AXI_DMA_MM2S_DMACR);
    // iowrite32(AXI_DMA_DMACR_IOC_IrqEn | AXI_DMA_DMACR_ERR_IrqEn, axi_dma->base_addr + AXI_DMA_S2MM_DMACR);

    // DMA_Print_DMASR(axi_dma, 0);
    // DMA_Print_DMASR(axi_dma, 1);
    // DMA_Print_DMACR(axi_dma, 0);
    // DMA_Print_DMACR(axi_dma, 1);

    // /* AXI DMA Stuff */
    dma_sync_single_for_cpu(&pdev->dev, dma_handle, DMA_BUFFER_SIZE, DMA_TO_DEVICE);
    for(i = 0; i < 256; i+=4){
        printk(KERN_INFO "( ML605 PCIe ) [I] %d : 0x%08X\n", i, *(u32 *)(dmabuf + i));
    }
    dma_sync_single_for_device(&pdev->dev, dma_handle, DMA_BUFFER_SIZE, DMA_TO_DEVICE);

    for(i = 0; i < 256; i+=4){
        printk(KERN_INFO "( ML605 PCIe ) [I] %d : 0x%08X\n", i, *(u32 *)(dmabuf + i));
    }

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





/* AXI DMA */
static void DMA_Print_DMASR(struct pci_driver_priv *dma, bool MM2S) {
    u32 dmasr_temp = 0;

    if (MM2S) {
        dmasr_temp = ioread32(axi_dma->base_addr + AXI_DMA_S2MM_DMASR);
        printk(KERN_INFO "( ML605 PCIe ) [I] S2MM:");
    }
    else{
        dmasr_temp = ioread32(axi_dma->base_addr + AXI_DMA_MM2S_DMASR);
        printk(KERN_INFO "( ML605 PCIe ) [I] MM2S:");
    }

    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.Halted    = %d\n", CHECK_BIT(dmasr_temp, 0)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.Idle      = %d\n", CHECK_BIT(dmasr_temp, 1)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.SGIncld   = %d\n", CHECK_BIT(dmasr_temp, 3)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.DMAIntErr = %d\n", CHECK_BIT(dmasr_temp, 4)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.DMASlvErr = %d\n", CHECK_BIT(dmasr_temp, 5)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.DMADecErr = %d\n", CHECK_BIT(dmasr_temp, 6)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.SGIntErr  = %d\n", CHECK_BIT(dmasr_temp, 8)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.SGSlvErr  = %d\n", CHECK_BIT(dmasr_temp, 9)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.SGDecErr  = %d\n", CHECK_BIT(dmasr_temp, 10) ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.IOC_Irq   = %d\n", CHECK_BIT(dmasr_temp, 12) ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.Dly_Irq   = %d\n", CHECK_BIT(dmasr_temp, 13) ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMASR.Err_Irq   = %d\n", CHECK_BIT(dmasr_temp, 14) ? 1 : 0);
}

static void DMA_Print_DMACR(struct pci_driver_priv *dma, bool MM2S) {
    u32 dmacr_temp = 0;

    if (MM2S) {
        dmacr_temp = ioread32(axi_dma->base_addr + AXI_DMA_S2MM_DMACR);
        printk(KERN_INFO "( ML605 PCIe ) [I] S2MM:");
    }
    else{
        dmacr_temp = ioread32(axi_dma->base_addr + AXI_DMA_MM2S_DMACR);
        printk(KERN_INFO "( ML605 PCIe ) [I] MM2S:");
    }

    printk(KERN_INFO "( ML605 PCIe ) [I] DMACR.RS        = %d\n", CHECK_BIT(dmacr_temp, 0)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMACR.Reset     = %d\n", CHECK_BIT(dmacr_temp, 2)  ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMACR.IOC_IrqEn = %d\n", CHECK_BIT(dmacr_temp, 12) ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMACR.Dly_IrqEn = %d\n", CHECK_BIT(dmacr_temp, 13) ? 1 : 0);
    printk(KERN_INFO "( ML605 PCIe ) [I] DMACR.ERR_IrqEn = %d\n", CHECK_BIT(dmacr_temp, 14) ? 1 : 0);
}

// static void DMA_Transfer(struct pci_driver_priv *dma, buffer_t *buff, uint16_t count) {

// 	int descriptor;

// 	for (descriptor = 0; descriptor < count; descriptor++) {
// 		_descriptor_list[descriptor].BUFFER_ADDRESS = (uintptr_t)buff[descriptor].data;
// 		_descriptor_list[descriptor].STATUS = 0;
	
//         if (descriptor == 0) { // Если первый дескриптор - 27 бит TXSOF (Transmit Start of Frame) в 1
// 			_descriptor_list[descriptor].NXTDESC = (uintptr_t) &_descriptor_list[descriptor + 1];
// 			_descriptor_list[descriptor].CONTROL = (buff[descriptor].size| (1 << 27));
//         }
// 		else if (descriptor == count - 1) { // Если последний - 26 бит - TXEOF (Transmit End Of Frame) в 1
// 			_descriptor_list[descriptor].NXTDESC = (uintptr_t) &_descriptor_list[0];
// 			_descriptor_list[descriptor].CONTROL = (buff[descriptor].size | (1 << 26));
// 		}
// 		else {
// 			_descriptor_list[descriptor].NXTDESC = (uintptr_t) &_descriptor_list[descriptor + 1];
// 			_descriptor_list[descriptor].CONTROL = buff[descriptor].size;
// 		}
// 	}

//     iowrite32((uintptr_t)&_descriptor_list[0], dma->base_addr + AXI_DMA_MM2S_CURDESC); // Адрес первого дескрипотора
//     iowrite32((uintptr_t)&_descriptor_list[count - 1], dma->base_addr + AXI_DMA_MM2S_TAILDESC); // Адрес последнего дескриптора
//     iowrite32((count << 16) | 0x1001, dma->base_addr + AXI_DMA_MM2S_DMACR); // IRQThreshold + IOC_IrqEn + RS
// }


/*
void dma_sg_init(struct axi_dma_sg *sg, struct axi_dma_buffer *buffer, size_t pkt_size) {
    int i;
    int size_max = (int) buffer->size/pkt_size;
    void *data = buffer->data;
    for (i = 0; i < size_max; i++) {
        sg[i].nxtdesc = &sg[i + 1]; // Адрес следующего дескриптора
        sg[i].buffer_address = &data; // данные
        iowrite32(pkt_size, sg->control);
        data += pkt_size;
    }
}

void dma_sg_init_sparse(struct axi_dma_sg *sg, struct axi_dma_buffer **buffers, size_t num_buffers, size_t pkt_size){
  int i = 0;
  for(i = 0; i < num_buffers; i++){
    dma_sg_init(sg, buffers[i], pkt_size);
    sg = sg->nxtdesc;
  }
}

void dma_sg_read(struct pci_driver_priv *axi_dma, struct axi_dma_sg *sg) {
    int i = 0;
    int length = sizeof(sg)/sizeof(sg[0]); // Подсчёт количества дескрипторов
    iowrite32((length << 16) | 0x1001, axi_dma->base_addr + AXI_DMA_S2MM_DMACR);    // IRQThreshold + IOC_IrcEn + RS
    iowrite32(sg[length - 1].nxtdesc, axi_dma->base_addr + AXI_DMA_S2MM_TAILDESC); // Адрес последнего дескриптора
    while (i != length) {
        iowrite32(sg[i].buffer_address, axi_dma->base_addr + AXI_DMA_S2MM_CURDESC);
        i++;
    }
}

void dma_sg_write(struct pci_driver_priv *axi_dma, struct axi_dma_sg *sg) {
    int i = 0;
    int length = sizeof(sg)/sizeof(sg[0]); // Number of descriptors
    iowrite32((length << 16) | 0x1001, axi_dma->base_addr + AXI_DMA_MM2S_DMACR);    // IRQThreshold + IOC_IrqEn + RS
    iowrite32(sg[length - 1].nxtdesc, axi_dma->base_addr + AXI_DMA_MM2S_TAILDESC); // Адрес последнего дескриптора
    while (i != length) {
        iowrite32(sg[i].buffer_address, axi_dma->base_addr + AXI_DMA_MM2S_CURDESC);
        i++;
    }
}
*/
/* AXI DMA end */

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
