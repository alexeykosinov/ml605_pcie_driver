// Урок https://habr.com/ru/post/348042/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
// #include <linux/interrupt.h>
// #include <linux/dma-mapping.h>

#define PCI_DRIVER_NAME		"ML605_PCIe"
#define PCI_VID				0x10EE 	/* Xilinx 			*/
#define PCI_PID				0x0505 	/* Device ID 		*/

#define DATA_SIZE			1024 	/* DMA buffer size	*/
#define DDR_BAR		  		0 		/* DDR bar registers 			*/
#define CDMA_BAR		  	2 		/* CDMA bar registers 			*/
#define CDMACR 		  	  	0x00 	/* CDMA control register offset */
#define CDMASR 		  	  	0x04 	/* CDMA status register offset 	*/



#define SA 		          	0x18 	/* LSB source address			*/
#define DA 		          	0x20 	/* LSB destination address		*/

#define BTT 		      	0x28 	/* Transfer byte size			*/

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
	u8 __iomem *hwmem; 
};



/* Регистрации драйвера */
static int __init pci_init(void) {
	printk(KERN_DEBUG "( ML605 PCIe ) Succesfully initialized!");
	return pci_register_driver(&pci_drv); 
}

/* Выгрузка драйвера */
static void __exit pci_exit(void) { pci_unregister_driver(&pci_drv); }

void release_device(struct pci_dev *pdev) {
	pci_release_region(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
	pci_disable_device(pdev);
}







static int pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {

	// int bar, err;
	u16 vendor, device;
	int i = 0;	
	// unsigned long mmio_start, mmio_len;
	struct pci_driver_priv *cdma_priv = NULL;
	struct pci_driver_priv *ddr_priv = NULL;

	unsigned long int cdma_reg_space_start = 0;
	unsigned long int cdma_reg_space_len = 0;
	unsigned long int ddr_start = 0;
	unsigned long int ddr_len = 0;

	dma_addr_t dma_addr;

	struct dma_desc {
		void 	*addr;
		size_t 	size;
	} desc;

	desc.addr = kzalloc(DATA_SIZE, GFP_DMA);
	desc.size = DATA_SIZE;

	/* 
	* Чтение 8-ми битных регистров	: int pci_read_config_byte(struct pci_dev *dev, int where, u8 *ptr);
	* Чтение 16-ти битных регистров : int pci_read_config_word(struct pci_dev *dev, int where, u16 *ptr);
	* Чтение 32-х битных регистров	: int pci_read_config_dword(struct pci_dev *dev, int where, u32 *ptr);
	* Запись 8-ми битных регистров	: int pci_write_config_byte(struct pci_dev *dev, int where, u8 val);
	* Запись 16-ти битных регистров : int pci_write_config_word(struct pci_dev *dev, int where, u16 val);
	* Запись 32-х битных регистров	: int pci_write_config_dword(struct pci_dev *dev, int where, u32 val);
	*/
	pci_read_config_word(pdev, PCI_VENDOR_ID, &vendor); // Чтение фирмы изготовителя устройства
	pci_read_config_word(pdev, PCI_DEVICE_ID, &device); // Чтения ID устройства


	printk(KERN_INFO "( ML605 PCIe ) Device VID/PID: 0x%X/0x%X\n", vendor, device);


	// прикрепление памяти ddr и регистров cdma к pcie на хосте
	ddr_start = pci_resource_start(pdev,DDR_BAR);
	ddr_len = pci_resource_len(pdev,DDR_BAR);
	
	cdma_reg_space_start = pci_resource_start(pdev,CDMA_BAR);
	cdma_reg_space_len = pci_resource_len(pdev,CDMA_BAR);




	printk(KERN_INFO "( ML605 PCIe ) The memory start: 0x%lX\nThe memory length: %ld B\n", cdma_reg_space_start, cdma_reg_space_len);
	printk(KERN_INFO "( ML605 PCIe ) A tested bar is the bar%d\n", CDMA_BAR);

	// прикрепление памяти ddr и регистров cdma к pcie на хосте продолжение
	if (pci_enable_device_mem(pdev) != 0) {
		printk(KERN_INFO "( ML605 PCIe ) PCIe memmory error. Stage 1.\n");
		return -1;
	}

	if ((cdma_priv = kzalloc(sizeof(struct pci_driver_priv), GFP_KERNEL)) == NULL) {
		printk(KERN_INFO "( ML605 PCIe ) PCIe memmory error. Stage 2.0.\n");
		return -1;
	}

	if ((ddr_priv = kzalloc(sizeof(struct pci_driver_priv), GFP_KERNEL)) == NULL) {
		printk(KERN_INFO "( ML605 PCIe ) PCIe memmory error. Stage 2.1.\n");
		return -1;
	}

	if ((cdma_priv->hwmem = ioremap(cdma_reg_space_start,cdma_reg_space_len)) == NULL) {
		printk(KERN_INFO "( ML605 PCIe ) PCIe memmory error. Stage 3.0.\n");
		return -1;
	}

	if ((ddr_priv->hwmem = ioremap(ddr_start,ddr_len)) == NULL) {
		printk(KERN_INFO "( ML605 PCIe ) PCIe memmory error. Stage 3.1.\n");
		return -1;
	}
	
	// заполнение буфера передачи линейным кодом

	// for (i = 0; i < 100; i++) {
	// 	*(desc.addr + i) = i;
	// }

	// чтение регистров cdma и вывод их в журнал
	for (i = 0; i < 11; i++) {
		printk(KERN_INFO "( ML605 PCIe ) %X\n", ioread32(cdma_priv->hwmem + (i*4)));
	}

	// получение адреса на ине pcie буфера для dma
	dma_addr = dma_map_single(&pdev->dev, desc.addr, desc.size, DMA_TO_DEVICE);
	if (dma_mapping_error(&pdev->dev, dma_addr)) {
		printk(KERN_INFO "( ML605 PCIe ) error\n");
	}

	printk(KERN_INFO "( ML605 PCIe ) add %llX\n", dma_addr);

	// уставновка адреса источника 
	iowrite32(dma_addr, cdma_priv->hwmem + SA);
	// устанвка адреса назначеня, адрес ddr в пространстве cdma в ПЛИС
	iowrite32(0xC0000000, cdma_priv->hwmem + DA);
	// передать 100 байт
	iowrite32(0x100, cdma_priv->hwmem + BTT);
	mdelay(10);

	// вывод зачения регистров
	printk(KERN_INFO "( ML605 PCIe ) CR %X\n", ioread32(cdma_priv->hwmem + CDMACR));
	printk(KERN_INFO "( ML605 PCIe ) SR %X\n", ioread32(cdma_priv->hwmem + CDMASR));
	printk(KERN_INFO "( ML605 PCIe ) SA %X\n", ioread32(cdma_priv->hwmem + SA));
	printk(KERN_INFO "( ML605 PCIe ) DA %X\n", ioread32(cdma_priv->hwmem + DA));

	i = 0;
	// ожидание завершения передачи от cdma
	while (!(ioread32(cdma_priv->hwmem + CDMASR) & 0x2)) {
		// если передача так и не завершилось, выдти из ожидания
		if (i == 10000) {
			printk(KERN_INFO "( ML605 PCIe ) ST ERR %X\n", ioread32(cdma_priv->hwmem + CDMASR));
			break;
		}
		else {
			i++;
		}
	}
	
	// проверка на передачу данных в ddr
	for (i = 0; i < 11; i++) {
		printk(KERN_INFO "( ML605 PCIe ) %X\n", ioread8(ddr_priv->hwmem + i));
	}

	// чтение статусного регистра
	printk(KERN_INFO "( ML605 PCIe ) ST %X\n", ioread32(cdma_priv->hwmem + CDMASR));


	dma_unmap_single(&pdev->dev, dma_addr, DATA_SIZE, DMA_TO_DEVICE);
	kfree(desc.addr);
	kfree(cdma_priv);
	kfree(ddr_priv);











////////////////////////////////////////////////////////////////
	/* Чтение/запись памяти устройства */

	/* 
	* Определяем какой именно кусок памяти мы хотим получить, в данном случае этот ресурс ввода/вывода
	* Make BAR mask from the type of resource
	* This helper routine makes bar mask from the type of resource.
	* struct pci_dev * dev 	- The PCI device for which BAR mask is made
	* unsigned long flags	- Resource type mask to be selected
	*/
	// bar = pci_select_bars(pdev, IORESOURCE_MEM);

	/* Инициализируем память устройства
	* Initialize device before it's used by a driver.
	* Ask low-level code to enable Memory resources.
	* Wake up the device if it was suspended.
	* Beware, this function can fail.
	*/
	// err = pci_enable_device_mem(pdev);

	// if (err) { 
	// 	printk(KERN_INFO "( ML605 PCIe ) Enable Device Memory Error/Warning message: %d\n", err);
	// 	return err; 
	// }

	/* Запрашиваем необходимый регион памяти, с определенным ранее типом
	* Mark the PCI region associated with PCI device pdev BAR bar as being reserved by owner res_name.
	* Do not access any address inside the PCI regions unless this call returns successfully.
	* Returns 0 on success, or EBUSY on error.
	* A warning message is also printed on failure.
	*/
	// err = pci_request_region(pdev, bar, PCI_DRIVER_NAME);

	// if (err) {
	// 	printk(KERN_INFO "( ML605 PCIe ) Request Region Error/Warning message: %d\n", err);
	// 	pci_disable_device(pdev);
	// 	return err;
	// }

	/* Получаем адрес начала блока памяти устройства и общую длину этого блока */
	// mmio_start = pci_resource_start(pdev, 0);
	// mmio_len = pci_resource_len(pdev, 0);

	// cdma_priv = kzalloc(sizeof(struct pci_driver_priv), GFP_KERNEL);

	// if (!cdma_priv) {
	// 	release_device(pdev);
	// 	return -ENOMEM;
	// }

	/* Отображаем выделенную память к аппаратуре */
	// cdma_priv->hwmem = ioremap(mmio_start, mmio_len);

	// if (!cdma_priv->hwmem) {
	// 	release_device(pdev);
	// 	return -EIO;
	// }

	// pci_set_drvdata(pdev, cdma_priv);

////////////////////////////////////////////////////////////////


	return 0;

}


/* Освобождение занятых ресурсов */
static void pci_remove(struct pci_dev *pdev) {
	//struct pci_driver_priv *cdma_priv = pci_get_drvdata(pdev);
	//if (cdma_priv) { kfree(cdma_priv); } // освободить память ввода/вывода
	pci_disable_device(pdev);
}














module_init(pci_init);
module_exit(pci_exit);
