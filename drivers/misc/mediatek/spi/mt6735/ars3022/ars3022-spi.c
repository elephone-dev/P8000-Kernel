 /*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/input.h>
#include <linux/poll.h>
#include "ars3022.h"

//#define ARS3022_VDD18_LDO		MT6323_POWER_LDO_VGP2
//#define ARS3022_VDD18_PIN		(GPIO18 | 0x80000000)
//#define ARS3022_VDD33_LDO		MT6323_POWER_LDO_VIBR
#define ARS3022_VDD33_PIN		(GPIO_FP_VDD33_PIN)
#define ARS3022_RESET_PIN		(GPIO_FP_RST_PIN)

#define ARS3022_VDD18_HIGH		GPIO_OUT_ONE
#define ARS3022_VDD18_LOW		GPIO_OUT_ZERO
#define ARS3022_VDD33_HIGH		GPIO_OUT_ONE
#define ARS3022_VDD33_LOW		GPIO_OUT_ZERO
#define ARS3022_RESET_HIGH		GPIO_OUT_ONE
#define ARS3022_RESET_LOW		GPIO_OUT_ZERO

#if defined(ARS3022_VDD18_LDO) && defined (ARS3022_VDD18_PIN)
#error "Can not define ARS3022_VDD18_LDO & ARS3022_VDD18_PIN both!"
#endif

#if defined(ARS3022_VDD33_LDO) && defined (ARS3022_VDD33_PIN)
#error "Can not define ARS3022_VDD33_LDO & ARS3022_VDD33_PIN both!"
#endif

#define ARS3022_KEY_FINGER_PRESENT	KEY_WAKEUP	

#define SPI_PACKET_SIZE 0x400

#ifdef ARS3022_SPI_DEBUG
#define DEBUG_PRINT(fmt, args...)	pr_err(fmt, ## args)
#else
/* Don't do anything in release builds */
#define DEBUG_PRINT(fmt, args...)
#endif

static DECLARE_BITMAP(minors, N_SPI_MINORS);

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static struct mt_chip_conf spi_conf =
{
	.setuptime = 2,
	.holdtime = 2,
	.high_time = 8,
	.low_time = 8,
	.cs_idletime = 4,
	.ulthgh_thrsh = 0,
	.cpol = 1,
	.cpha = 1,
	.rx_mlsb = SPI_MSB, 
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

/*--------------------------- Data Transfer -----------------------------*/
int ars3022_mass_read(struct ars3022_data *ars3022, u8 addr, u8 *buf, int read_len)
{
	int status;
	struct spi_device *spi;
	struct spi_message m;
	u8 *write_addr;
	u32 spi_transfer_len = (read_len + 3) > SPI_PACKET_SIZE ? (read_len + 3 + SPI_PACKET_SIZE - 1) / SPI_PACKET_SIZE * SPI_PACKET_SIZE : read_len + 3;
	/* Set start address */
	u8 *read_data = kzalloc(spi_transfer_len, GFP_KERNEL);
	struct spi_transfer t_set_addr = {
		.tx_buf = NULL,
		.len = 2,
	};

	/* Write and read data in one data query section */
	struct spi_transfer t_read_data = {
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = spi_transfer_len,
	};

	DEBUG_PRINT("%s", __func__);

	if (read_data == NULL)
		return -ENOMEM;

	write_addr = kzalloc(2, GFP_KERNEL);
	write_addr[0] = ARS3022_WRITE_ADDRESS;
	write_addr[1] = addr;

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);

	read_data[0] = ARS3022_READ_DATA;

	t_set_addr.tx_buf = write_addr;
	t_read_data.tx_buf = t_read_data.rx_buf = read_data;

	spi = ars3022->spi;
	spi_message_init(&m);
	spi_message_add_tail(&t_set_addr, &m);
	spi_message_add_tail(&t_read_data, &m);
	status = spi_sync(spi, &m);

	kfree(write_addr);

	if (status == 0)
		memcpy(buf, read_data + 3, read_len);
	else
		pr_err("%s read data error status = %d\n"
				, __func__, status);
	kfree(read_data);

	return status;
}

/*
 * Read io register
 */
int ars3022_io_read_register(struct ars3022_data *ars3022, u8 *addr, u8 *buf)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;
	int read_len = 1;

	u8 write_addr[4] = {ARS3022_WRITE_ADDRESS, 0x00};
	u8 read_value[4] = {ARS3022_READ_DATA, 0x00};
	u8 result[4] = {0xFF, 0xFF};
	u8 val, addrval;

	struct spi_transfer t_set_addr = {
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t = {
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	DEBUG_PRINT("%s", __func__);

	if (copy_from_user(&addrval, (const u8 __user *) (uintptr_t) addr
		, read_len)) {
		pr_err("%s buffer copy_from_user fail", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);

	spi = ars3022->spi;

	/*Set address*/
	write_addr[1] = addrval;

	/*Start to read*/
	spi_message_init(&m);
	spi_message_add_tail(&t_set_addr, &m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);

	if (status < 0) {
		pr_err("%s read data error status = %d\n"
				, __func__, status);
		return status;
	}

	val = result[1];

	DEBUG_PRINT("%s Read add_val = %x buf = 0x%x\n", __func__,
			addrval, val);

	if (copy_to_user((u8 __user *) (uintptr_t) buf, &val, read_len)) {
		pr_err("%s buffer copy_to_user fail status", __func__);
		status = -EFAULT;
		return status;
	}

	return status;
}

/*
 * Write data to register
 */
int ars3022_io_write_register(struct ars3022_data *ars3022, u8 *buf)
{
	int status = 0;
	struct spi_device *spi;
	int write_len = 2;
	struct spi_message m;

	u8 write_addr[4] = {ARS3022_WRITE_ADDRESS, 0x00};
	u8 write_value[4] = {ARS3022_WRITE_DATA, 0x00};
	u8 val[4];

	DEBUG_PRINT("%s", __func__);

	struct spi_transfer t1 = {
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.tx_buf = write_value,
		.len = 2,
	};

	if (copy_from_user(val, (const u8 __user *) (uintptr_t) buf
		, write_len)) {
		pr_err("%s buffer copy_from_user fail", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s write_len = %d\n", __func__, write_len);
	DEBUG_PRINT("%s address = %x data = %x\n", __func__, val[0], val[1]);

	spi = ars3022->spi;
	
	/*Set address*/
	write_addr[1] = val[0];
	/*Set value*/
	write_value[1] = val[1];

	spi_message_init(&m);
	spi_message_add_tail(&t1, &m);
	spi_message_add_tail(&t2, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %d\n",
				__func__, status);
		return status;
	}

	return status;
}

int ars3022_read_register(struct ars3022_data *ars3022, u8 addr, u8 *buf)
{
	int status;
	struct spi_device *spi;
	struct spi_message m;

	/*Set address*/
	u8 write_addr[4] = {ARS3022_WRITE_ADDRESS, addr};
	u8 read_value[4] = {ARS3022_READ_DATA, 0x00};
	u8 result[4] = {0xFF, 0xFF};

	struct spi_transfer t1 = {
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	DEBUG_PRINT("%s", __func__);

	spi = ars3022->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t1, &m);
	spi_message_add_tail(&t2, &m);
	status = spi_sync(spi, &m);

	if (status == 0) {
		*buf = result[1];
		DEBUG_PRINT("ars3022_read_register address = %x result = %x %x\n"
					, addr, result[0], result[1]);
	} else
		pr_err("%s read data error status = %d\n"
				, __func__, status);

	return status;
}

int ars3022_io_get_one_image(
	struct ars3022_data *ars3022,
	u8 *buf,
	u8 *image_buf
	)
{
	uint8_t read_val,
			*tx_buf = (uint8_t *)buf,
			*work_buf = NULL,
			*val = kzalloc(6, GFP_KERNEL);
	int32_t status;
	uint32_t frame_not_ready_count = 0, read_count;

	DEBUG_PRINT("%s\n", __func__);

	if (val == NULL)
		return -ENOMEM;

	if (copy_from_user(val, (const u8 __user *) (uintptr_t) tx_buf, 6)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto end;
	}

	/* total pixel , width * hight */
	read_count = val[0] * val[1];

	while (1) {
		status = ars3022_read_register
				(ars3022, FSTATUS_ARS3022_ADDR, &read_val);
		if (status < 0)
			goto end;

		if (read_val & FRAME_READY_MASK)
			break;

		if (frame_not_ready_count >= 250) {
			pr_err("frame_not_ready_count = %d\n",
					frame_not_ready_count);
			break;
		}
		frame_not_ready_count++;
	}

	work_buf = kzalloc(read_count, GFP_KERNEL);
	if (work_buf == NULL) {
		status = -ENOMEM;
		goto end;
	}
	status = ars3022_mass_read(ars3022, FDATA_ARS3022_ADDR, work_buf, read_count);
	if (status < 0) {
		pr_err("%s call ars3022_mass_read error status = %d\n"
				, __func__, status);
		goto end;
	}

	if (copy_to_user((u8 __user *) (uintptr_t) image_buf,
		work_buf, read_count)) {
		pr_err("buffer copy_to_user fail status = %d\n", status);
		status = -EFAULT;
	}
end:
	kfree(val);
	kfree(work_buf);
	return status;
}

/*----------------------- EEPROM ------------------------*/

int ars3022_eeprom_wren(struct ars3022_data *ars3022)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 write_data[] = {ARS3022_EEPROM_WREN_OP};
	struct spi_transfer t = {
		.tx_buf = write_data,
		.len = 1,
	};

	DEBUG_PRINT("%s opcode = %x\n", __func__, write_data[0]);

	spi = ars3022->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n",
				__func__, status);
		return status;
	}

	return status;
}

int ars3022_eeprom_wrdi(struct ars3022_data *ars3022)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 write_data[] = {ARS3022_EEPROM_WRDI_OP};
	struct spi_transfer t = {
		.tx_buf = write_data,
		.len = 1,
	};

	DEBUG_PRINT("%s opcode = %x\n", __func__, write_data[0]);

	spi = ars3022->spi;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n",
				__func__, status);
		return status;
	}

	return status;
}

int ars3022_eeprom_rdsr(struct ars3022_data *ars3022, u8 *buf)
{
	int status;
	struct spi_device *spi;
	struct spi_message m;
	u8 val,
	   read_value[] = {ARS3022_EEPROM_RDSR_OP, 0x00},
	   result[] = {0xFF, 0xFF};

	struct spi_transfer t = {
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	spi = ars3022->spi;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n",
				__func__, status);
		return status;
	}

	val = result[1];

	DEBUG_PRINT("%s address = %x buf = %x\n", __func__,
			ARS3022_EEPROM_RDSR_OP, val);

	if (copy_to_user((u8 __user *) (uintptr_t) buf, &val, 1)) {
		pr_err("%s buffer copy_to_user fail status\n", __func__);
		status = -EFAULT;
		return status;
	}

	return status;
}

int ars3022_eeprom_wrsr(struct ars3022_data *ars3022, u8 *buf)
{
	int status;
	struct spi_device *spi;
	struct spi_message m;
	u8 val;

	u8 write_data[] = {ARS3022_EEPROM_WRSR_OP, 0x00};

	struct spi_transfer t = {
		.tx_buf = write_data,
		.len = 2,
	};

	if (copy_from_user(&val, (const u8 __user *) (uintptr_t) buf
		, 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s data = %x\n", __func__, val);

	spi = ars3022->spi;

	write_data[1] = val;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n",
				__func__, status);
		return status;
	}

	return status;
}

int ars3022_eeprom_read(struct ars3022_data *ars3022, u8 *addr, u8 *buf, int read_len)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;
	u8 addrval, *read_value = kzalloc(read_len + 2, GFP_KERNEL);

	struct spi_transfer t = {
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = read_len + 2,
	};

	if (read_value == NULL)
		return -ENOMEM;

	if (copy_from_user(&addrval, (const u8 __user *) (uintptr_t) addr
		, 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto exit;
	}

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);
	DEBUG_PRINT("%s addrval = %x\n", __func__, addrval);

	spi = ars3022->spi;

	read_value[0] = ARS3022_EEPROM_READ_OP;
	read_value[1] = addrval;

	t.tx_buf = t.rx_buf = read_value;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s spi_sync error status = %d\n"
				, __func__, status);
		goto exit;
	}

	if (copy_to_user((u8 __user *) (uintptr_t) buf,
				read_value + 2, read_len)) {
		pr_err("%s buffer copy_to_user fail status\n", __func__);
		status = -EFAULT;
		goto exit;
	}

exit:
	kfree(read_value);

	return status;
}

/*
 * buf - the data wrote to sensor with address info
 * write_len - the length of the data write to memory without address
 */
int ars3022_eeprom_write(struct ars3022_data *ars3022, u8 *buf, int write_len)
{
	int status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 *write_value = kzalloc(write_len + 2, GFP_KERNEL);

	struct spi_transfer t = {
		.tx_buf = NULL,
		.len = write_len + 2,
	};

	if (write_value == NULL)
		return -ENOMEM;

	write_value[0] = ARS3022_EEPROM_WRITE_OP;

	if (copy_from_user(write_value + 1, (const u8 __user *) (uintptr_t) buf
		, write_len + 1)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto exit;
	}

	DEBUG_PRINT("%s write_len = %d\n", __func__, write_len);
	DEBUG_PRINT("%s address = %x\n", __func__, write_value[1]);

	spi = ars3022->spi;

	t.tx_buf = write_value;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s read data error status = %d\n",
				__func__, status);
		goto exit;
	}

exit:
	kfree(write_value);

	return status;
}

/* ------------------------------ Interrupt -----------------------------*/
struct interrupt_desc ars3022_ints;
static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);

/* -------------------------------------------------------------------- */
int  ars3022_input_init(struct interrupt_desc *data)
{
	int error = 0;

	DEBUG_PRINT("%s\n", __func__);

	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		pr_err("Input_allocate_device failed.\n");
		error  = -ENOMEM;
	}

	if (!error) {
		data->input_dev->name = ARS3022_DEV_NAME;

		set_bit(EV_KEY,		data->input_dev->evbit);

		set_bit(ARS3022_KEY_FINGER_PRESENT, data->input_dev->keybit);

		input_set_capability(data->input_dev, EV_KEY, ARS3022_KEY_FINGER_PRESENT );

		error = input_register_device(data->input_dev);
	}

	if (error) {
		pr_err("Input_register_device failed.\n");
		input_free_device(data->input_dev);
		data->input_dev = NULL;
	}

	return error;
}


/* -------------------------------------------------------------------- */
void  ars3022_input_destroy(struct interrupt_desc *data)
{
	DEBUG_PRINT("%s\n", __func__);

	if (data->input_dev != NULL)
	{
		input_free_device(data->input_dev);
		data->input_dev = NULL;
	}
}

/* -------------------------------------------------------------------- */
int ars3022_wakeup_report(struct interrupt_desc *data)
{
	DEBUG_PRINT("%s\n", __func__);
	if (data->input_dev != NULL)	
	{
		input_report_key(data->input_dev, ARS3022_KEY_FINGER_PRESENT, 1);
		input_report_key(data->input_dev, ARS3022_KEY_FINGER_PRESENT, 0);
		input_sync(data->input_dev);
	}
}


/* -------------------------------------------------------------------- */
unsigned int ars3022_interrupt_poll(
	struct file *file,
	struct poll_table_struct *wait
)
{
	unsigned int mask = 0;
	int i = 0;

	ars3022_ints.int_count = 0;
	poll_wait(file, &interrupt_waitq, wait);
	DEBUG_PRINT("%s finger_on = %d\n", __func__, ars3022_ints.finger_on);
	
	if (ars3022_ints.finger_on) {
		mask |= POLLIN | POLLRDNORM;
		ars3022_ints.finger_on = 0;
	}
	return mask;
}

/*
 *	FUNCTION NAME.
 *		ars3022_interrupt_timer_routine
 *
 *	FUNCTIONAL DESCRIPTION.
 *		basic interrupt timer inital routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

void ars3022_interrupt_timer_routine(
	unsigned long _data
)
{
	struct ars3022_data *ars3022;

	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;
	DEBUG_PRINT("ars3022 interrupt count = %d" , bdata->int_count);
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		DEBUG_PRINT("FPS triggered !!!!!!!\n");
		ars3022_wakeup_report(&ars3022_ints);
	} else
		pr_info("FPS not triggered !!!!!!!\n");

	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}


/*
 *	FUNCTION NAME.
 *		ars3022_eint_func
 *
 *	FUNCTIONAL DESCRIPTION.
 *		finger print interrupt callback routine
 *
 *	ENTRY PARAMETERS.
 *		irq
 *		dev_id
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */
static void ars3022_eint_func(void)
{
	//DEBUG_PRINT("%s ars3022_ints.int_count = %d\n", __func__, ars3022_ints.int_count);
	if (!ars3022_ints.int_count)
		mod_timer(&ars3022_ints.timer,
			jiffies + msecs_to_jiffies(ars3022_ints.detect_period));
	ars3022_ints.int_count++;
#if defined(CUST_EINT_FP_NUM)
	#if CUST_EINT_FP_TYPE
		mt_eint_unmask(CUST_EINT_FP_NUM);
	#else
		mt65xx_eint_unmask(CUST_EINT_FP_NUM);
	#endif
#endif
}

/*
 *	FUNCTION NAME.
 *		ars3022_interrupt_init
 *
 *	FUNCTIONAL DESCRIPTION.
 *		button initial routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */
int ars3022_interrupt_init(int int_num, int detect_period, int detect_threshold)
{
	DEBUG_PRINT("%s detect_period = %d detect_threshold = %d\n", __func__, detect_period, detect_threshold);

	if (ars3022_ints.state == 1){
		DEBUG_PRINT("ars3022 has been init!!!!!!!\n");
		return;
		}

	ars3022_ints.state = 1;

	// Setup Interrupt Pin
	mt_set_gpio_mode(GPIO_FP_EINT_PIN, GPIO_FP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_FP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_FP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_FP_EINT_PIN, GPIO_PULL_DOWN);
#if defined(CUST_EINT_FP_NUM)
#if CUST_EINT_FP_TYPE
	mt_eint_set_hw_debounce(CUST_EINT_FP_NUM, CUST_EINT_FP_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_FP_NUM, CUST_EINT_FP_TYPE, ars3022_eint_func, 0);
#else
	mt65xx_eint_set_sens(CUST_EINT_FP_NUM, CUST_EINT_FP_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_FP_NUM, CUST_EINT_FP_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_FP_NUM, CUST_EINT_FP_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_FP_NUM, CUST_EINT_FP_DEBOUNCE_EN, CUST_EINT_FP_POLARITY, ars3022_eint_func, 0);
#endif

#if CUST_EINT_FP_TYPE
	mt_eint_unmask(CUST_EINT_FP_NUM);
#else
	mt65xx_eint_unmask(CUST_EINT_FP_NUM);
#endif
#endif
	ars3022_ints.detect_period = detect_period;
	ars3022_ints.detect_threshold = detect_threshold;
	ars3022_ints.int_count = 0;
	ars3022_ints.finger_on = 0;
	return 0;
}

/*
 *	FUNCTION NAME.
 *		ars3022_interrupt_free
 *
 *	FUNCTIONAL DESCRIPTION.
 *		free all interrupt resource
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */
int ars3022_interrupt_free(void)
{
	DEBUG_PRINT("%s", __func__);

	ars3022_ints.state = 0;
	del_timer_sync(&ars3022_ints.timer);
#if defined(CUST_EINT_FP_NUM)
#if CUST_EINT_FP_TYPE
	mt_eint_mask(CUST_EINT_FP_NUM);
#else
	mt65xx_eint_mask(CUST_EINT_FP_NUM);
#endif
#endif
	return 0;
}

void ars3022_interrupt_abort(void)
{
	DEBUG_PRINT("%s", __func__);
	wake_up_interruptible(&interrupt_waitq);
}

/*-------------------------------------------------------------------------*/

static inline void ars3022_reset(void)
{
	DEBUG_PRINT("%s\n", __func__);
	mt_set_gpio_mode(ARS3022_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(ARS3022_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(ARS3022_RESET_PIN, ARS3022_RESET_LOW);
	msleep(30);
	mt_set_gpio_out(ARS3022_RESET_PIN, ARS3022_RESET_HIGH);
	msleep(20);
}

static inline void ars3022_reset_set(int enable)
{
	DEBUG_PRINT("%s(%s)\n", __func__, enable ? "true" : "false");
	mt_set_gpio_mode(ARS3022_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(ARS3022_RESET_PIN, GPIO_DIR_OUT);
	if (enable == 0) {
		mt_set_gpio_out(ARS3022_RESET_PIN, ARS3022_RESET_LOW);
		msleep(30);
	} else {
		mt_set_gpio_out(ARS3022_RESET_PIN, ARS3022_RESET_HIGH);
		msleep(20);
	}
}

static ssize_t ars3022_read(struct file *filp,
						char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static ssize_t ars3022_write(struct file *filp,
						const char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static long ars3022_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	struct ars3022_data *ars3022;
	struct spi_device *spi;
	u32 tmp;
	struct ars3022_ioc_transfer *ioc = NULL;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != ARS3022_IOC_MAGIC) {
		pr_err("%s _IOC_TYPE(cmd) != ARS3022_IOC_MAGIC", __func__);
		return -ENOTTY;
	}

	/*
	 * Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err) {
		pr_err("%s err", __func__);
		return -EFAULT;
	}

	/*
	 * guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	ars3022 = filp->private_data;
	spin_lock_irq(&ars3022->spi_lock);
	spi = spi_dev_get(ars3022->spi);
	spin_unlock_irq(&ars3022->spi_lock);

	if (spi == NULL) {
		pr_err("%s spi == NULL", __func__);
		return -ESHUTDOWN;
	}

	mutex_lock(&ars3022->buf_lock);

	/* segmented and/or full-duplex I/O request */
	if (_IOC_NR(cmd) != _IOC_NR(ARS3022_IOC_MESSAGE(0))
					|| _IOC_DIR(cmd) != _IOC_WRITE) {
		retval = -ENOTTY;
		goto out;
	}

	tmp = _IOC_SIZE(cmd);
	if ((tmp == 0) || (tmp % sizeof(struct ars3022_ioc_transfer)) != 0) {
		retval = -EINVAL;
		goto out;
	}

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (ioc == NULL) {
		retval = -ENOMEM;
		goto out;
	}
	if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
		retval = -EFAULT;
		goto out;
	}

	DEBUG_PRINT("%s ioc->opcode = %x\n", __func__, ioc->opcode);

	/*
	 * Read register
	 * tx_buf include register address will be read
	 */
	if (ioc->opcode == ARS3022_REGISTER_READ) {
		u8 *address = ioc->tx_buf;
		u8 *result = ioc->rx_buf;
		DEBUG_PRINT("ars3022 ARS3022_REGISTER_READ\n");

		retval = ars3022_io_read_register(ars3022, address, result);
		if (retval < 0)	{
			pr_err("%s ARS3022_REGISTER_READ error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Write data to register
	 * tx_buf includes address and value will be wrote
	 */
	if (ioc->opcode == ARS3022_REGISTER_WRITE) {
		u8 *buf = ioc->tx_buf;
		DEBUG_PRINT("ars3022 ARS3022_REGISTER_WRITE");

		retval = ars3022_io_write_register(ars3022, buf);
		if (retval < 0) {
			pr_err("%s ARS3022_REGISTER_WRITE error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Get one frame data from sensor
	 */
	if (ioc->opcode == ARS3022_GET_ONE_IMG) {
		u8 *buf = ioc->tx_buf;
		u8 *image_buf = ioc->rx_buf;
		DEBUG_PRINT("ars3022 ARS3022_GET_ONE_IMG\n");

		retval = ars3022_io_get_one_image(ars3022, buf, image_buf);
		if (retval < 0) {
			pr_err("%s ARS3022_GET_ONE_IMG error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Power on fps device or trigger something.
	 */
	if(ioc->opcode == POWER_ON_FPS_DEVICE)
	{
#if defined(ARS3022_VDD18_LDO)
		hwPowerOn(ARS3022_VDD18_LDO, VOL_1800, "ars3020");
#elif defined(ARS3022_VDD18_PIN)
		mt_set_gpio_mode(ARS3022_VDD18_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(ARS3022_VDD18_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(ARS3022_VDD18_PIN, ARS3022_VDD18_HIGH);
#endif
#if defined(ARS3022_VDD33_LDO)
		hwPowerOn(ARS3022_VDD33_LDO, VOL_3300, "ars3020");
#elif defined(ARS3022_VDD33_PIN)
		mt_set_gpio_mode(ARS3022_VDD33_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(ARS3022_VDD33_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(ARS3022_VDD33_PIN, ARS3022_VDD33_HIGH);
#endif
		ars3022_reset();
	}

	/*
	 * Power off fps device or trigger something.
	 */
	if(ioc->opcode == POWER_OFF_FPS_DEVICE)
	{
#if defined(ARS3022_VDD18_LDO)
		hwPowerDown(ARS3022_VDD18_LDO, "ars3020");
#elif defined(ARS3022_VDD18_PIN)
		mt_set_gpio_out(ARS3022_VDD18_PIN, ARS3022_VDD18_LOW);
#endif
#if defined(ARS3022_VDD33_LDO)
		hwPowerDown(ARS3022_VDD33_LDO, "ars3020");
#elif defined(ARS3022_VDD33_PIN)
		mt_set_gpio_out(ARS3022_VDD33_PIN, ARS3022_VDD33_LOW);
#endif
	}


	if (ioc->opcode == ARS3022_SENSOR_RESET)
		ars3022_reset();

	if (ioc->opcode == ARS3022_RESET_SET) {
		pr_info("%s ARS3022_SENSOR_RESET\n", __func__);
		pr_info("%s status = %d\n", __func__, ioc->len);
		ars3022_reset_set(ioc->len);
	}

	if (ioc->opcode == ARS3022_SET_SPI_CLOCK) {
		__u32 current_speed = spi->max_speed_hz;
		pr_info("%s ARS3022_SET_SPI_CLOCK\n", __func__);
		pr_info("%s speed_hz = %d\n", __func__, ioc->speed_hz);
		pr_info("%s current_speed = %d\n", __func__, current_speed);

		spi->max_speed_hz = ioc->speed_hz;
		retval = spi_setup(spi);
		if (retval < 0) {
			pr_err("%s spi_setup error %d\n", __func__, retval);
			spi->max_speed_hz = current_speed;
		}
		pr_info("%s spi speed_hz = %d\n", __func__, spi->max_speed_hz);
	}

	if (ioc->opcode == ARS3022_EEPROM_WREN) {
		pr_info("%s ARS3022_EEPROM_WREN\n", __func__);
		ars3022_reset_set(0);
		ars3022_eeprom_wren(ars3022);
		ars3022_reset_set(1);
	}

	if (ioc->opcode == ARS3022_EEPROM_WRDI) {
		pr_info("%s ARS3022_EEPROM_WRDI\n", __func__);
		ars3022_reset_set(0);
		ars3022_eeprom_wrdi(ars3022);
		ars3022_reset_set(1);
	}

	if (ioc->opcode == ARS3022_EEPROM_RDSR) {
		u8 *result = ioc->rx_buf;
		pr_info("%s ARS3022_EEPROM_RDSR\n", __func__);
		ars3022_reset_set(0);
		ars3022_eeprom_rdsr(ars3022, result);
		ars3022_reset_set(1);
	}

	if (ioc->opcode == ARS3022_EEPROM_WRSR) {
		u8 *buf = ioc->tx_buf;
		pr_info("%s ARS3022_EEPROM_WRSR\n", __func__);
		ars3022_reset_set(0);
		ars3022_eeprom_wrsr(ars3022, buf);
		ars3022_reset_set(1);
	}

	if (ioc->opcode == ARS3022_EEPROM_READ) {
		u8 *buf = ioc->tx_buf;
		u8 *result = ioc->rx_buf;
		pr_info("%s ARS3022_EEPROM_READ\n", __func__);
		ars3022_reset_set(0);
		ars3022_eeprom_read(ars3022, buf, result, ioc->len);
		ars3022_reset_set(1);
	}

	if (ioc->opcode == ARS3022_EEPROM_WRITE) {
		u8 *buf = ioc->tx_buf;
		pr_info("%s ARS3022_EEPROM_WRITE\n", __func__);
		ars3022_reset_set(0);
		ars3022_eeprom_write(ars3022, buf, ioc->len);
		ars3022_reset_set(1);
	}

	if (ioc->opcode == ARS3022_POWER_ONOFF)
		pr_info("power control status = %d\n", ioc->len);

	/*
	 * Trigger inital routine
	 */
	if (ioc->opcode == INT_TRIGGER_INIT) {
		pr_info(">>> ars3022 Trigger function init\n");
		u8* trigger_buf = ioc->rx_buf;
		retval = ars3022_interrupt_init(
				(int)ioc->pad[0],
				(int)ioc->pad[1],
				(int)ioc->pad[2]);
		pr_info("trigger init = %d\n", retval);
	}

	/*
	 * trigger
	 */
	if (ioc->opcode == INT_TRIGGER_CLOSE) {
		pr_info("<<< ars3022 Trigger function close\n");
		retval = ars3022_interrupt_free();
		pr_info("trigger close = %d\n", retval);
	}

	/*
	 * read interrupt status
	 */
	if (ioc->opcode == INT_TRIGGER_ABORT)
	{
		pr_info("<<< ars3022 Trigger function close\n");
		ars3022_interrupt_abort();
	}
	
out:
	if (ioc != NULL)
		kfree(ioc);

	mutex_unlock(&ars3022->buf_lock);
	spi_dev_put(spi);
	if (retval < 0)
		pr_err("%s retval = %d", __func__, retval);
	return retval;
}

#ifdef CONFIG_COMPAT
static long ars3022_compat_ioctl(struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	return ars3022_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define ars3022_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int ars3022_open(struct inode *inode, struct file *filp)
{
	struct ars3022_data *ars3022;
	int			status = -ENXIO;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);

	list_for_each_entry(ars3022, &device_list, device_entry)
	{
		if (ars3022->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (ars3022->buffer == NULL) {
			ars3022->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (ars3022->buffer == NULL) {
				dev_dbg(&ars3022->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			ars3022->users++;
			filp->private_data = ars3022;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("ars3022: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int ars3022_release(struct inode *inode, struct file *filp)
{
	struct ars3022_data *ars3022;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);
	ars3022 = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	ars3022->users--;
	if (ars3022->users == 0) {
		int	dofree;

		kfree(ars3022->buffer);
		ars3022->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&ars3022->spi_lock);
		dofree = (ars3022->spi == NULL);
		spin_unlock_irq(&ars3022->spi_lock);

		if (dofree)
			kfree(ars3022);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations ars3022_fops =
{
	.owner = THIS_MODULE,
	.write = ars3022_write,
	.read = ars3022_read,
	.unlocked_ioctl = ars3022_ioctl,
	.compat_ioctl = ars3022_compat_ioctl,
	.open = ars3022_open,
	.release = ars3022_release,
	.llseek = no_llseek,
	.poll = ars3022_interrupt_poll,
};

/*-------------------------------------------------------------------------*/

static struct class *ars3022_class;

/*-------------------------------------------------------------------------*/

static int ars3022_probe(struct spi_device *spi)
{
	struct ars3022_data *ars3022;
	int status;
	unsigned long minor;
	int i;

	DEBUG_PRINT("%s\n", __func__);

	/* Allocate driver data */
	ars3022 = kzalloc(sizeof(*ars3022), GFP_KERNEL);
	if (ars3022 == NULL)
		return -ENOMEM;

	/* Initialize the driver data */
	ars3022->spi = spi;
	spi->controller_data = (void *) &spi_conf;
	spi_setup(spi);
	spin_lock_init(&ars3022->spi_lock);
	mutex_init(&ars3022->buf_lock);

	INIT_LIST_HEAD(&ars3022->device_entry);

	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		ars3022->devt = MKDEV(ARS3022_MAJOR, minor);
		dev = device_create(ars3022_class, &spi->dev, ars3022->devt,
					ars3022, "esfp0");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else{
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&ars3022->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, ars3022);
	else
		kfree(ars3022);

#if defined(ARS3022_VDD18_LDO)
	hwPowerOn(ARS3022_VDD18_LDO, VOL_1800, "ars3022");
#elif defined(ARS3022_VDD18_PIN)
	mt_set_gpio_mode(ARS3022_VDD18_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(ARS3022_VDD18_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(ARS3022_VDD18_PIN, ARS3022_VDD18_HIGH);
#endif
	msleep(20);
#if defined(ARS3022_VDD33_LDO)
	hwPowerOn(ARS3022_VDD33_LDO, VOL_3300, "ars3022");
#elif defined(ARS3022_VDD33_PIN)
	mt_set_gpio_mode(ARS3022_VDD33_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(ARS3022_VDD33_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(ARS3022_VDD33_PIN, ARS3022_VDD33_HIGH);
#endif

	ars3022_reset();

	setup_timer(&ars3022_ints.timer, ars3022_interrupt_timer_routine,
				(unsigned long)&ars3022_ints);
	ars3022_input_init(&ars3022_ints);

	ars3022_ints.state = 0;
	ars3022_interrupt_init(0, 10, 10);
	return status;
}

static int ars3022_remove(struct spi_device *spi)
{
	struct ars3022_data *ars3022 = spi_get_drvdata(spi);
	DEBUG_PRINT("%s\n", __func__);

#if defined(ARS3022_VDD18_LDO)
	hwPowerDown(ARS3022_VDD18_LDO, "ars3022");
#elif defined(ARS3022_VDD18_PIN)
	mt_set_gpio_out(ARS3022_VDD18_PIN, ARS3022_VDD18_LOW);
#endif
#if defined(ARS3022_VDD33_LDO)
	hwPowerDown(ARS3022_VDD33_LDO, "ars3022");
#elif defined(ARS3022_VDD33_PIN)
	mt_set_gpio_out(ARS3022_VDD33_PIN, ARS3022_VDD33_LOW);
#endif

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&ars3022->spi_lock);
	ars3022->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&ars3022->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&ars3022->device_entry);
	device_destroy(ars3022_class, ars3022->devt);
	clear_bit(MINOR(ars3022->devt), minors);
	if (ars3022->users == 0)
		kfree(ars3022);
	mutex_unlock(&device_list_lock);
	
	ars3022_input_destroy(&ars3022_ints);
	ars3022_interrupt_free();
	return 0;
}


static struct spi_driver ars3022_spi_driver = {
	.driver = {
		.name = "ars3022",
		.owner = THIS_MODULE,
	},
	.probe = ars3022_probe,
	.remove = ars3022_remove,
};

static struct spi_board_info ars3022_board_devs[] __initdata = {
	[0] = {
		.modalias = "ars3022",
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_3,
	},
};

/*-------------------------------------------------------------------------*/

static int __init ars3022_init(void)
{
	int status;
	DEBUG_PRINT("%s\n", __func__);

	/*
	 * Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	spi_register_board_info(ars3022_board_devs, ARRAY_SIZE(ars3022_board_devs));
	status = register_chrdev(ARS3022_MAJOR, "spi", &ars3022_fops);
	if (status < 0)
		return status;
	ars3022_class = class_create(THIS_MODULE, "ars3022");
	if (IS_ERR(ars3022_class)) {
		unregister_chrdev(ARS3022_MAJOR, ars3022_spi_driver.driver.name);
		return PTR_ERR(ars3022_class);
	}
	status = spi_register_driver(&ars3022_spi_driver);
	if (status < 0) {
		class_destroy(ars3022_class);
		unregister_chrdev(ARS3022_MAJOR, ars3022_spi_driver.driver.name);
		return status;
	}
	return status;
}
module_init(ars3022_init);

static void __exit ars3022_exit(void)
{
	DEBUG_PRINT("%s\n", __func__);
	spi_unregister_driver(&ars3022_spi_driver);
	class_destroy(ars3022_class);
	unregister_chrdev(ARS3022_MAJOR, ars3022_spi_driver.driver.name);
}
module_exit(ars3022_exit);

MODULE_AUTHOR("wangyk, <wangyk@aratek.com.cn>");
MODULE_DESCRIPTION("SPI Interface for ars3022");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ARS3022");
