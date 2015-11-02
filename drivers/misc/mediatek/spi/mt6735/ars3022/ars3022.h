#ifndef _ARS3022_H
#define _ARS3022_H


#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_spi.h>
#include <cust_eint.h>

//#define ARS3022_SPI_DEBUG
#define CUST_EINT_FP_TYPE 1

#ifdef ARS3022_SPI_DEBUG
#define DEBUG_PRINT(fmt, args...) pr_err(fmt, ## args)
#else
#define DEBUG_PRINT(fmt, args...)
#endif

#define ARS3022_MAJOR				170 /* assigned */
#define N_SPI_MINORS				32  /* ... up to 256 */

#define ARS3022_ADDRESS_0			0x00
#define ARS3022_WRITE_ADDRESS			0xAC
#define ARS3022_READ_DATA			0xAF
#define ARS3022_WRITE_DATA			0xAE

#define ARS3022_EEPROM_WREN_OP			0x06
#define ARS3022_EEPROM_WRDI_OP			0x04
#define ARS3022_EEPROM_RDSR_OP			0x05
#define ARS3022_EEPROM_WRSR_OP			0x01
#define ARS3022_EEPROM_READ_OP			0x03
#define ARS3022_EEPROM_WRITE_OP			0x02

/* ------------------------- Register Definition ------------------------*/
/*
 * Sensor Registers
 */

#define FDATA_ARS3022_ADDR			0x00
#define FSTATUS_ARS3022_ADDR			0x01
/*
 * Detect Define
 */
#define FRAME_READY_MASK			0x01

/* ------------------------- Opcode -------------------------------------*/
#define ARS3022_REGISTER_READ			0x01
#define ARS3022_REGISTER_WRITE			0x02
#define ARS3022_GET_ONE_IMG			0x03
#define ARS3022_SENSOR_RESET			0x04
#define ARS3022_POWER_ONOFF			0x05
#define ARS3022_SET_SPI_CLOCK			0x06
#define ARS3022_RESET_SET			0x07

#define ARS3022_EEPROM_WREN			0x90
#define ARS3022_EEPROM_WRDI			0x91
#define ARS3022_EEPROM_RDSR			0x92
#define ARS3022_EEPROM_WRSR			0x93
#define ARS3022_EEPROM_READ			0x94
#define ARS3022_EEPROM_WRITE			0x95

#define POWER_ON_FPS_DEVICE                0xa0         // Power On FPS Device
#define POWER_OFF_FPS_DEVICE               0xa1         // Power OFF FPS Device

/* trigger signal initial routine*/
#define INT_TRIGGER_INIT			0xa4
/* trigger signal close routine*/
#define INT_TRIGGER_CLOSE			0xa5
/* read trigger status*/
#define INT_TRIGGER_READ			0xa6
/* polling trigger status*/
#define INT_TRIGGER_POLLING			0xa7
/* polling abort*/
#define INT_TRIGGER_ABORT			0xa8

#define ARS3022_DEV_NAME                        "ars3022"

/* ------------------------- Interrupt ------------------------------*/
/* interrupt init */
int ars3022_interrupt_init(int int_num, int detect_period, int detect_threshold);

/* interrupt free */
int ars3022_interrupt_free(void);

void ars3022_interrupt_abort(void);

/* interrupt polling */
unsigned int ars3022_interrupt_poll(
	struct file *file,
	struct poll_table_struct *wait
);
struct interrupt_desc {
	int state;
	int int_count;
	struct timer_list timer;
	bool finger_on;
	int detect_period;
	int detect_threshold;
	struct input_dev       *input_dev;
};

/* ------------------------- Structure ------------------------------*/
struct ars3022_ioc_transfer {
	u8 *tx_buf;
	u8 *rx_buf;

	__u32 len;
	__u32 speed_hz;

	__u16 delay_usecs;
	__u8 bits_per_word;
	__u8 cs_change;
	__u8 opcode;
	__u8 pad[3];

};

#define ARS3022_IOC_MAGIC			'k'
#define ARS3022_MSGSIZE(N) \
	((((N)*(sizeof(struct ars3022_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof(struct ars3022_ioc_transfer))) : 0)
#define ARS3022_IOC_MESSAGE(N) _IOW(ARS3022_IOC_MAGIC, 0, char[ARS3022_MSGSIZE(N)])

struct ars3022_data {
	dev_t devt;
	spinlock_t spi_lock;
	struct spi_device *spi;
	struct list_head device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned users;
	u8 *buffer;
};

#ifdef CUST_EINT_FP_TYPE
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#else
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

/* ------------------------- Data Transfer --------------------------*/
int ars3022_io_read_register(struct ars3022_data *ars3022, u8 *addr, u8 *buf);
int ars3022_io_write_register(struct ars3022_data *ars3022, u8 *buf);
int ars3022_io_get_one_image(struct ars3022_data *ars3022, u8 *buf, u8 *image_buf);
int ars3022_read_register(struct ars3022_data *ars3022, u8 addr, u8 *buf);
int ars3022_mass_read(struct ars3022_data *ars3022, u8 addr, u8 *buf, int read_len);
#endif
