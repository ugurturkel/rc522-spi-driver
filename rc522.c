#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <asm/uaccess.h>

#include "rc522.h"

MODULE_DESCRIPTION("SPI Device Driver for RC522 RFID Module");
MODULE_LICENSE("GPL");          
MODULE_AUTHOR("Ugur Turkel");    
MODULE_VERSION("0.1");

/***** data structure****/
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsize = 4096;
static struct class *rc522_class;
static struct spi_master *master;
static struct spi_device *spi;

static const struct file_operations rc522_fops = {
	owner: THIS_MODULE,
	llseek: no_llseek,
	unlocked_ioctl: rc522_ioctl,
	compat_ioctl: rc522_ioctl,
	open: rc522_open,
	release: rc522_release,
};

/***** write, read reg help function *****/
static int rc522_write_reg (struct rc522_info *info, unsigned char reg, unsigned char value)
{
	int ret;
	unsigned char tx_buf[4];
	tx_buf[0] = (reg << 1) & 0x7E;
	tx_buf[1] = value;
	ret = spi_write(info->spi_dev, tx_buf, 2);
	if (ret)
		printk(KERN_ERR "RC522 Driver: failed to write reg: addr=0x%02x, value=0x%02x, ret = %d\n", reg, value, ret);
	return ret;
}
static int rc522_read_reg (struct rc522_info *info, unsigned char reg,unsigned char *value)
{
	unsigned char tx_buf[4];
	unsigned char rx_buf[4];
	int ret;
	tx_buf[0] = ((reg << 1) & 0x7E) | 0x80;
	ret = spi_write_then_read(info->spi_dev, tx_buf, 1, rx_buf, 1);
	if (ret)
		printk(KERN_ERR "RC522 Driver: failed to read reg: addr=0x%02x, value=0x%02x, ret = %d\n", reg, *value, ret);
	else
		*value = rx_buf[0];
	return ret;
}
static int rc522_write_buf(struct rc522_info *info, unsigned char reg, unsigned char *buf, unsigned char len)
{
	int ret;
	unsigned char tx_buf[INFO_BUFFER_SIZE + 1];
	tx_buf[0] = (reg << 1) & 0x7E;
	memcpy(&tx_buf[1], buf, len);
	ret = spi_write(info->spi_dev, tx_buf, len+1);
	if (ret)
		printk(KERN_ERR "RC522 Driver: failed to write buf: addr=0x%02x, value=0x%02x, ret = %d\n", reg, *buf, ret);
	return ret;
} 
/****** the fuction has bug, do not use *****/
static int rc522_read_buf(struct rc522_info *info, unsigned char reg, unsigned char *buf, unsigned char len)
{
	struct spi_message	message;
	uint8_t tx_buf[32] = {0}; 
	int ret;
	tx_buf[0] = ((reg << 1) & 0x7E) | 0x80;
	
	struct spi_transfer	xfer = {
		tx_buf: tx_buf,
		rx_buf: buf,
		len: len,
	};
	
	memset(&tx_buf[1], tx_buf[0], len-1);
	spi_message_init(&message);
	spi_message_add_tail(&xfer, &message);
	ret = spi_async(info->spi_dev, &message);
	if (ret)
		printk(KERN_ERR "RC522 Driver: failed to read buf: addr=0x%02x, value=0x%02x, ret = %d\n", reg,*buf,ret);
	return ret;
} 
static int rc522_set_bitmask (struct rc522_info *info, unsigned char reg, unsigned char mask)
{
	unsigned char tmp = 0x00;
	int ret = 0;
	rc522_read_reg(info, reg, &tmp);
	rc522_write_reg(info, reg, tmp | mask);
	return ret;
} 
static int rc522_clear_bitmask (struct rc522_info *info, unsigned char reg, unsigned char mask)
{
	unsigned char tmp = 0x00;
	int ret = 0;
	rc522_read_reg(info, reg, &tmp);
	rc522_write_reg(info, reg, tmp & ~mask);
	return ret;
}

/***** rc522 help function *****/
static int rc522_reset_chip(struct rc522_info *info)
{
	int ret = 0;
	if(ret = rc522_write_reg(info, COMMAND_REG, PCD_RESETPHASE)) return ret;
	msleep(50);
	rc522_write_reg(info, MODE_REG, 0x3D);
	rc522_write_reg(info, T_RELOAD_REG_L, 30);
	rc522_write_reg(info, T_RELOAD_REG_H, 0);
	rc522_write_reg(info, T_MODE_REG, 0x8D);
	rc522_write_reg(info, T_PRESCALER_REG, 0x3E);
	rc522_write_reg(info, TX_AUTO_REG, 0x40);
	return ret;
}
static int rc522_enable_antenna(struct rc522_info *info)
{
	unsigned char reg_val = 0;
	int ret = 0;
	if(!(reg_val & 0x03))
		rc522_set_bitmask(info,TX_CONTROL_REG,0x03);
	return ret;
}
static int rc522_disable_antenna(struct rc522_info *info)
{	
	int ret = 0;
	rc522_clear_bitmask(info,TX_CONTROL_REG,0x03);
	return ret;
}
static int rc522_config_isotype(struct rc522_info *info, unsigned char type)
{
	int ret = 0;
	if(type == 'A'){
		rc522_clear_bitmask(info,STATUS2_REG, 0x08);
		rc522_write_reg(info, MODE_REG, 0x3D);
		rc522_write_reg(info, RX_SEL_REG, 0x86);
		rc522_write_reg(info, RF_CFG_REG, 0x7F);
		rc522_write_reg(info, T_RELOAD_REG_L, 30);
		rc522_write_reg(info, T_RELOAD_REG_H, 0);
		rc522_write_reg(info, T_MODE_REG, 0x8D);
		rc522_write_reg(info, T_PRESCALER_REG, 0x3E);
		rc522_enable_antenna(info);
	}else{
		printk(KERN_ERR "RC522 Driver: ISO type is not supported.");
		return MI_ERR;
	}
	return ret;
}
static int rc522_init_chip(struct rc522_info *info)
{
	int ret = 0;
	if(ret = rc522_reset_chip(info)) return ret;
	rc522_disable_antenna(info);
	msleep(50);
	rc522_enable_antenna(info);
	rc522_config_isotype(info, 'A');
	return ret;
}



/***** rc522 card ops function *****/
static int rc522_com_card(struct rc522_info *info, unsigned char cmd, unsigned char *tx_buf, unsigned char tx_len, unsigned char *rx_buf, unsigned int  *rx_len_bit)
{
	unsigned char irq_en        = 0x00;
	unsigned char wait_for      = 0x00;
	unsigned char last_bits     = 0x00;
	unsigned char reg_val       = 0x00;
	unsigned char com_irq_reg   = 0x00;
	unsigned int  i;
	unsigned long long cur_usec,last_usec;
	int status = MI_ERR;
	switch(cmd){
	case PCD_AUTHENT:
		irq_en = 0x12;
		wait_for = 0x10;
		break;
	case PCD_TRANSCEIVE:
		irq_en = 0x77;
		wait_for = 0x30;
		break;
	default:
		break;
	}
	rc522_write_reg(info, COM_IEN_REG, irq_en | 0x80);
	rc522_clear_bitmask(info, COM_IRQ_REG, 0x80);
	rc522_write_reg(info, COMMAND_REG, PCD_IDLE);
	rc522_set_bitmask(info, FIFO_LEVEL_REG, 0x80);
 	rc522_write_buf(info, FIFO_DATA_REG, tx_buf, tx_len);
	rc522_write_reg(info, COMMAND_REG, cmd);
	if(cmd == PCD_TRANSCEIVE)
		rc522_set_bitmask(info, BIT_FRAMING_REG, 0x80);
	i =10;
    	do{
		if(status = rc522_read_reg(info, COM_IRQ_REG, &com_irq_reg)) 
			return status;
		msleep(50); // must delay > 25 ms, 
		i--;
	}
	while((i != 0) && !(com_irq_reg & 0x01) && !(com_irq_reg & wait_for));

	rc522_clear_bitmask(info, BIT_FRAMING_REG, 0x80);
	if(i != 0){
		rc522_read_reg(info, ERROR_REG, &reg_val);
		if(!(reg_val & 0x1B)){ 
			status = MI_OK ;
			if(com_irq_reg & irq_en & 0x01) 
				status = MI_NOTAGERR; 
			if(cmd == PCD_TRANSCEIVE){
				rc522_read_reg(info, FIFO_LEVEL_REG, &reg_val);
				rc522_read_reg(info, CONTROL_REG, &last_bits);
				last_bits = last_bits & 0x07;
				if(last_bits)
					*rx_len_bit = (reg_val - 1) * 8 + last_bits;
				else		
					*rx_len_bit = reg_val * 8;
				if(0 == reg_val) 
					reg_val = 1;
				if(reg_val > DEF_FIFO_LENGTH)
					reg_val = 64;
				for(i = 0; i<reg_val; i++)
					rc522_read_reg(info, FIFO_DATA_REG, &rx_buf[i]);
			}
		}
		else{
			printk(KERN_ERR "RC522 Driver: failed to read irq reg: addr=0x%02x, value=0x%02x\n" , COM_IRQ_REG, com_irq_reg);	
			status = MI_ERR;
		}
	}
	rc522_set_bitmask(info, CONTROL_REG, 0x80);
	rc522_write_reg(info, COMMAND_REG, PCD_IDLE);
	return status;
}
static int rc522_calculate_crc(struct rc522_info *info, unsigned char *tx_buf, unsigned char tx_len, unsigned char *rx_buf)
{
	unsigned char reg_val   = 0;
	int status = 0;
	rc522_clear_bitmask(info, DIV_IRQ_REG, 0x04);
	rc522_write_reg(info, COMMAND_REG, PCD_IDLE);
	rc522_set_bitmask(info, FIFO_LEVEL_REG, 0x80);
	rc522_write_buf(info, FIFO_DATA_REG, tx_buf, tx_len);
	rc522_write_reg(info, COMMAND_REG, PCD_CALCCRC);
	int i = 10;
	do{	
		if(status = rc522_read_reg(info, DIV_IRQ_REG, &reg_val)) 
			return status;
		msleep(50);// must delay > 25 ms, 
	}
	while ((i != 0) && !(reg_val & 0x04));
	rc522_read_reg(info, CRC_RESULT_REG_L, &rx_buf[0]);
	rc522_read_reg(info, CRC_RESULT_REG_M, &rx_buf[1]);
	return status;
}

/***** rc522 ops function *****/
static int rc522_halt_chip(struct rc522_info *info)
{
	unsigned char *tx_buf = info->tx_buffer;
	unsigned char *rx_buf = info->rx_buffer;
	unsigned int rx_len_bit;
	int status;
	int ret = 0;
	tx_buf[0] = PICC_HALT;
	tx_buf[1] = 0;
	if(ret = rc522_calculate_crc(info, tx_buf, 2, &tx_buf[2])) 
		return ret;
	status = rc522_com_card(info, PCD_TRANSCEIVE, tx_buf, 2 + CARD_CRC_SIZE, rx_buf, &rx_len_bit);
	return status;
} 
static int rc522_request_card(struct rc522_info *info, struct rc522_ioc_transfer *xfer)
{
	unsigned char *tx_buf = info->tx_buffer;
	unsigned char *rx_buf = info->rx_buffer;
	unsigned int rx_len_bit;
	int status = 0, i = 0;
	tx_buf[0] = PICC_REQIDL;
	rc522_clear_bitmask(info, STATUS2_REG, 0x08);
	rc522_write_reg(info, BIT_FRAMING_REG, 0x07);
	rc522_set_bitmask(info, TX_CONTROL_REG, 0x03);
	status = rc522_com_card(info, PCD_TRANSCEIVE, tx_buf, 1, rx_buf, &rx_len_bit);
	if((status == MI_OK) && (rx_len_bit == 0x10)){
		memcpy(&xfer->txrx_buf[0], rx_buf, CARD_TYPE_SIZE);
        	 /*card type : 0x04,0x00, S50 card */
	}
	else{
		printk(KERN_ERR "RC522 Driver: failed to request card : %d\n" ,  status);	
		for(i=0; i < rx_len_bit/8; i ++){
			printk(KERN_INFO "RC522-Driver: 0x%02x, " ,  rx_buf[i]);	
		}
		printk(KERN_ERR "RC522 Driver: failed to request card : end\n");
   	}
	return status;
} 
static int rc522_anticoll_card(struct rc522_info *info, struct rc522_ioc_transfer *xfer)
{
	unsigned char *tx_buf = info->tx_buffer;
	unsigned char *rx_buf = info->rx_buffer;
	unsigned char id_csum = 0;	
	unsigned int rx_len_bit;
	int status = 0, i = 0;
	tx_buf[0] = PICC_ANTICOLL1;
	tx_buf[1] = 0x20;
	rc522_clear_bitmask(info, STATUS2_REG, 0x08);
   	rc522_write_reg(info, BIT_FRAMING_REG, 0x00);
	rc522_clear_bitmask(info, COLL_REG, 0x80);
	status = rc522_com_card(info, PCD_TRANSCEIVE, tx_buf, 2, rx_buf, &rx_len_bit);
    	if(status == MI_OK){
        	for(i = 0; i < CARD_ID_SIZE; i++) id_csum ^= rx_buf[i];
        	if(id_csum != rx_buf[i]) {
			printk(KERN_ERR "RC522 Driver: failed to check sum card id: %d\n" ,  status);	
			for(i=0; i < rx_len_bit/8; i ++){
				printk(KERN_INFO "RC522 Driver: 0x%02x, " ,  rx_buf[i]);	
			}
			printk(KERN_ERR "RC522 Driver: failed to check sum card id: end\n");	
			status = MI_ERR;
		}
            	memcpy(&xfer->txrx_buf[CARD_TYPE_SIZE], 
				rx_buf, CARD_ID_SIZE + CARD_IDCSUM_SIZE);
	}
	rc522_set_bitmask(info, COLL_REG, 0x80);
	return status;
}
static int rc522_select_card(struct rc522_info *info, struct rc522_ioc_transfer *xfer)
{
	unsigned char *tx_buf = info->tx_buffer;
	unsigned char *rx_buf = info->rx_buffer;
	unsigned int rx_len_bit;
    	int  status = 0, i = 0; 
    	tx_buf[0] = PICC_ANTICOLL1;
    	tx_buf[1] = 0x70;
    	tx_buf[6] = 0;
    	for(i = 0; i < CARD_ID_SIZE; i++){
        	tx_buf[2 + i]  = xfer->txrx_buf[CARD_TYPE_SIZE + i];
        	tx_buf[6]     ^= xfer->txrx_buf[CARD_TYPE_SIZE + i] ;
    	}
	if(status = rc522_calculate_crc(info, tx_buf, 2 + CARD_ID_SIZE + CARD_IDCSUM_SIZE , &tx_buf[2 + CARD_ID_SIZE + CARD_IDCSUM_SIZE])) 
		return status;
	rc522_clear_bitmask(info, STATUS2_REG, 0x08);
	status = rc522_com_card(info, PCD_TRANSCEIVE, tx_buf, 2 + CARD_ID_SIZE + CARD_IDCSUM_SIZE + CARD_CRC_SIZE, rx_buf, &rx_len_bit);
    	if((status == MI_OK) && (rx_len_bit == (unsigned int)0x18)){
		memcpy(&xfer->txrx_buf[CARD_TYPE_SIZE + CARD_ID_SIZE + CARD_IDCSUM_SIZE], rx_buf, CARD_CAP_SIZE + CARD_MSG_SIZE);// card capacity: 8K bits
    	}
    	else{
    		printk(KERN_ERR "RC522 Driver: failed to select card: %d\n" ,  status);	
		for(i=0; i < rx_len_bit/8; i ++){
			printk(KERN_INFO "RC522 Driver: 0x%02x, " ,  rx_buf[i]);	
		}
		printk(KERN_ERR "RC522 Driver: failed to select card: end\n");
		status = MI_ERR;
	}
	return status;
} 

static int rc522_auth_key(struct rc522_info *info, struct rc522_ioc_transfer *xfer)
{
	unsigned char *tx_buf = info->tx_buffer;
	unsigned char *rx_buf = info->rx_buffer;
	unsigned char *key_buf;
	unsigned char   reg_val     = 0;
    	unsigned int rx_len_bit;
	int status = 0, i = 0;
	switch(xfer->key_type){
	case RC522_KEY_A:
		tx_buf[0] = PICC_AUTHENT1A;
		key_buf = xfer->keya_buf;
		break;
	case RC522_KEY_B:
		tx_buf[0] = PICC_AUTHENT1B;
		key_buf = xfer->keyb_buf;
		break;
	default:
		tx_buf[0] = PICC_AUTHENT1A;
		key_buf = xfer->keya_buf;
		break;
	}
   	tx_buf[1] =  (xfer->sect_num * CARD_SECT_NBLK ) + xfer->blk_num;
	memcpy(tx_buf + 2, key_buf, CARD_KEYA_SIZE);
	memcpy(tx_buf+2 + CARD_KEYA_SIZE, xfer->id_buf, CARD_ID_SIZE);
	status = rc522_com_card(info, PCD_AUTHENT, tx_buf, 2 + CARD_KEYA_SIZE + CARD_ID_SIZE, rx_buf, &rx_len_bit);
	rc522_read_reg(info, STATUS2_REG, &reg_val);
	if((status != MI_OK) || (!(reg_val & 0x08))){
		printk(KERN_ERR "RC522 Driver: could not authorized the card: key type=0x%02x,status=%d\n", tx_buf[0], status);	
		printk(KERN_ERR "RC522 Driver: could not authorized the card: end\n");
        status = MI_ERR;
	}
	return status;
}
static int rc522_auth_card(struct rc522_info *info, struct rc522_ioc_transfer *xfer)
{
	int status = 0;
	switch(xfer->key_type){
	case RC522_KEY_A:
	case RC522_KEY_B:
		status = rc522_auth_key(info, xfer);
		break;
	case RC522_KEY_A_B:
		xfer->key_type = RC522_KEY_A;
		status = rc522_auth_key(info, xfer);
		xfer->key_type = RC522_KEY_B;
		status = rc522_auth_key(info, xfer);
		break;
	default:
		xfer->key_type = RC522_KEY_A;
		status = rc522_auth_key(info, xfer);
		break;
	}
	 return status;
} 

static int rc522_read_card(struct rc522_info *info, struct rc522_ioc_transfer *xfer)
{	
	unsigned char *tx_buf = info->tx_buffer;
	unsigned char *rx_buf = info->rx_buffer;
	unsigned int rx_len_bit;
	int   status  = 0, i = 0;
    tx_buf[0] = PICC_READ;
    tx_buf[1] = (xfer->sect_num * CARD_SECT_NBLK ) + xfer->blk_num;
	if(status = rc522_calculate_crc(info, tx_buf, 2, &tx_buf[2])) 
		return status;
    status = rc522_com_card(info, PCD_TRANSCEIVE, tx_buf, 2 + CARD_CRC_SIZE, rx_buf, &rx_len_bit);
    	if((status == MI_OK) && (rx_len_bit == 0x90)){
        	memcpy(xfer->txrx_buf, rx_buf, CARD_BLK_SIZE + CARD_PLUS_SIZE);
    	}
    	else{
    		printk(KERN_ERR "RC522 Driver: failed to read card: %d\n" ,  status);	
		for(i=0; i < rx_len_bit/8; i ++){
			printk(KERN_INFO "RC522 Driver: 0x%02x, " ,  rx_buf[i]);	
		}
		printk(KERN_ERR "RC522 Driver: failed to read card: end\n");
        status = MI_ERR;
    	}
	
    	return status;
} 
static int rc522_write_card(struct rc522_info *info, struct rc522_ioc_transfer *xfer)
{	
	unsigned char *tx_buf = info->tx_buffer;
	unsigned char *rx_buf = info->rx_buffer;
	unsigned int rx_len_bit;
	int   status = 0, i =0;
    tx_buf[0] = PICC_WRITE;
    tx_buf[1] = (xfer->sect_num * CARD_SECT_NBLK ) + xfer->blk_num;
	if(status = rc522_calculate_crc(info, tx_buf, 2, &tx_buf[2])) 
		return status;
	status = rc522_com_card(info, PCD_TRANSCEIVE, tx_buf, 2 + CARD_CRC_SIZE, rx_buf, &rx_len_bit);
    if((status == MI_OK) && (rx_len_bit == 4) && ((rx_buf[0] & 0x0F) == 0x0A)){
			memcpy(tx_buf, xfer->txrx_buf, CARD_BLK_SIZE);
			if(status = rc522_calculate_crc(info, tx_buf, CARD_BLK_SIZE, &tx_buf[CARD_BLK_SIZE])) 
				return status;
			status = rc522_com_card(info,PCD_TRANSCEIVE, tx_buf, CARD_BLK_SIZE + CARD_CRC_SIZE, tx_buf, &rx_len_bit);
			if((status != MI_OK)){
				printk(KERN_ERR "RC522 Driver: failed to write card: %d\n" ,  status);	
				for(i=0; i < rx_len_bit/8; i ++){
					printk(KERN_INFO "RC522-Driver: 0x%02x, " ,  rx_buf[i]);	
			}
				printk(KERN_ERR "RC522 Driver: failed to write card: end\n");
		    	status = MI_ERR;
			}
    	}
    	return status;
}

static int rc522_inc_dec_card(struct rc522_info *info, struct rc522_ioc_transfer *xfer)
{	
	unsigned char *tx_buf = info->tx_buffer;
	unsigned char *rx_buf = info->rx_buffer;
	unsigned int rx_len_bit;
	int status = 0, i = 0;
	switch(xfer->ioc_type){
		case RC522_IOC_INC_CARD:
			tx_buf[0] = PICC_INCREMENT;
			break;
		case RC522_IOC_DEC_CARD:
			tx_buf[0] = PICC_DECREMENT;
			break;
		default:
			printk(KERN_ERR "RC522 Driver: ioc type is not supported.\n");
		    status = MI_ERR;		
			return status;
	}
	tx_buf[1] = (xfer->sect_num * CARD_SECT_NBLK ) + xfer->blk_num;
	if(status = rc522_calculate_crc(info, tx_buf, 2, &tx_buf[2])) 
		return status;
	status = rc522_com_card(info, PCD_TRANSCEIVE, tx_buf, 2 + CARD_CRC_SIZE, rx_buf, &rx_len_bit);
	if ((status == MI_OK) && (rx_len_bit == 4) && ((rx_buf[0] & 0x0F) == 0x0A)){
		memcpy(tx_buf, xfer->txrx_buf, CARD_INC_SIZE);
		if(status = rc522_calculate_crc(info, tx_buf, 
				CARD_INC_SIZE, &tx_buf[CARD_INC_SIZE])) 
			return status;
		status = rc522_com_card(info, PCD_TRANSCEIVE, 
			tx_buf, CARD_INC_SIZE + CARD_CRC_SIZE, 
			rx_buf, &rx_len_bit);
		if(status == MI_OK){
			tx_buf[0] = PICC_TRANSFER;
    		tx_buf[1] = (xfer->sect_num * CARD_SECT_NBLK ) + xfer->blk_num;
    		if(status = rc522_calculate_crc(info, tx_buf, 2, &tx_buf[2])) 
				return status;
    		status = rc522_com_card(info, PCD_TRANSCEIVE, tx_buf,2 + CARD_CRC_SIZE, rx_buf,&rx_len_bit);
			if ((status == MI_OK) && (rx_len_bit == 4) && ((rx_buf[0] & 0x0F) == 0x0A)){   
				status = MI_OK;   
			}
			else{
				printk(KERN_ERR "RC522 Driver: failed to inc_dec card: %d\n" ,  status);	
				for(i=0; i < rx_len_bit/8; i ++){
					printk(KERN_INFO "RC522 Driver: 0x%02x, " ,  rx_buf[i]);	
				}
				printk(KERN_ERR "RC522 Driver: failed to inc_dec card: end\n");
	            status = MI_ERR;
			}
		}
	}
 	return status;
}

static int rc522_read_keya(struct rc522_info *info, struct  rc522_ioc_transfer *xfer)
{
	int status = 0;
	unsigned char buf[CARD_KEYA_SIZE] = {0x00};
	status = rc522_read_card(info, xfer);
	if(status == MI_OK){
		/***** must clean txrx_buf(avoid user to get msg) , then copy old_buf to it . *****/
		memcpy(buf, &xfer->txrx_buf[0], CARD_KEYA_SIZE);
		memset(xfer->txrx_buf, 0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
		memcpy(xfer->txrx_buf, buf, CARD_KEYA_SIZE);
	}
	return status;
}
static int rc522_read_keyb(struct rc522_info *info, struct  rc522_ioc_transfer *xfer)
{
	int status = 0;
	unsigned char buf[CARD_KEYB_SIZE] = {0x00};
	status = rc522_read_card(info, xfer);
	if(status == MI_OK){
		/***** must clean txrx_buf(avoid user to get msg) , then copy old_buf to it . *****/
		memcpy(buf, &xfer->txrx_buf[CARD_KEYA_SIZE + CARD_CTRL_SIZE], CARD_KEYB_SIZE);
		memset(xfer->txrx_buf, 0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
		memcpy(xfer->txrx_buf, buf, CARD_KEYB_SIZE);
	}
	return status;
}
static int rc522_read_ctrl(struct rc522_info *info, struct  rc522_ioc_transfer *xfer)
{
	int status = 0;
	unsigned char buf[CARD_CTRL_SIZE] = {0};
	status = rc522_read_card(info, xfer);
	if(status == MI_OK){
		
		/***** must clean txrx_buf(avoid user to get msg) , then copy old_buf to it . *****/
		memcpy(buf, &xfer->txrx_buf[CARD_KEYA_SIZE], CARD_CTRL_SIZE);
		memset(xfer->txrx_buf, 0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
		memcpy(xfer->txrx_buf, buf, CARD_CTRL_SIZE);
	}
	else{
	}
	return status;
}

static int rc522_write_keya(struct rc522_info *info, struct  rc522_ioc_transfer *xfer)
{
	int status = 0;
	unsigned char buf[CARD_KEYA_SIZE] = {0};
	unsigned char old_buf[CARD_KEYA_SIZE] = {0};
	memcpy(buf, &xfer->txrx_buf[0], CARD_KEYA_SIZE); /* save writed key */
	status = rc522_read_card(info, xfer);
	if(status == MI_OK){
		/***** must clean txrx_buf(avoid user to get msg) , then copy old_buf to it . *****/
		memcpy(old_buf, &xfer->txrx_buf[0], CARD_KEYA_SIZE); /* save old key to return user */
		memcpy(&xfer->txrx_buf[0], buf, CARD_KEYA_SIZE); /* copy writed key to kay space*/
		status = rc522_write_card(info, xfer);
		memset(xfer->txrx_buf, 0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
		if(status == MI_OK){
			memcpy(&xfer->txrx_buf[0], old_buf, CARD_KEYA_SIZE); /* return old key */
		}
	}
	return status;
}
static int rc522_write_keyb(struct rc522_info *info, struct  rc522_ioc_transfer *xfer)
{
	int status = 0;
	unsigned char buf[CARD_KEYB_SIZE] = {0};
	unsigned char old_buf[CARD_KEYB_SIZE] = {0};
	memcpy(buf, &xfer->txrx_buf[0], CARD_KEYB_SIZE); /* save writed key */
	status = rc522_read_card(info, xfer);
	if(status == MI_OK){
		memcpy(old_buf,&xfer->txrx_buf[CARD_KEYA_SIZE + CARD_CTRL_SIZE], CARD_KEYB_SIZE); /* save old key to return user */		
		/***** critical code: because do not read keya from card, 
		****** so must write the keya_buf to keya space .
		****** must clean txrx_buf(avoid user to get msg) , then copy old_buf to it .
		*****/
		memcpy(&xfer->txrx_buf[0], xfer->keya_buf, CARD_KEYA_SIZE);
		memcpy(&xfer->txrx_buf[CARD_KEYA_SIZE + CARD_CTRL_SIZE], buf, CARD_KEYB_SIZE); /* copy writed key to kay space*/
		status = rc522_write_card(info, xfer);
		memset(xfer->txrx_buf, 0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
		if(status == MI_OK){
			memcpy(&xfer->txrx_buf[0], old_buf,CARD_KEYB_SIZE); /* return old key */
		}
	}
	return status;
}

static int rc522_write_ctrl(struct rc522_info *info, struct  rc522_ioc_transfer *xfer)
{
	int status = 0;
	unsigned char buf[CARD_CTRL_SIZE] = {0};
	unsigned char old_buf[CARD_CTRL_SIZE] = {0};
	memcpy(buf, &xfer->txrx_buf[0], CARD_CTRL_SIZE); /* save writed ctrl area */
	status = rc522_read_card(info, xfer);
	if(status == MI_OK){
		memcpy(old_buf, &xfer->txrx_buf[CARD_KEYA_SIZE], CARD_CTRL_SIZE); /* save old ctrl area to return user */
		/***** critical code: because do not read keya from card, 
		****** so must write the keya_buf to keya space .
		****** must clean txrx_buf(avoid user to get msg) , 
		****** then copy old_buf to it .
		*****/
		memcpy(&xfer->txrx_buf[0], xfer->keya_buf, CARD_KEYA_SIZE);
		memcpy(&xfer->txrx_buf[CARD_KEYA_SIZE], buf,CARD_CTRL_SIZE); /* copy writed ctrl to ctrl space*/
		memcpy(&xfer->txrx_buf[CARD_KEYA_SIZE + CARD_CTRL_SIZE], 
		xfer->keyb_buf, CARD_KEYB_SIZE);
		status = rc522_write_card(info, xfer);
		memset(xfer->txrx_buf, 0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
		if(status == MI_OK){
			memcpy(&xfer->txrx_buf[0], old_buf,CARD_CTRL_SIZE); /* return old ctrl area */
		}
	}
	return status;
}

static int rc522_pass_op(struct rc522_info *info, struct  rc522_ioc_transfer *xfer)
{
	int ret = 0;
	int i = 0;
	switch(xfer->ioc_type){
	case RC522_IOC_NONE_0:
		break;
	case RC522_IOC_INIT_CHIP:
		if(info->users >= 2){
			printk(KERN_ERR "RC522 Driver: failed to init chip.\n");
			ret = -EBUSY;
			break;
		}
		ret = rc522_init_chip(info);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to init chip.\n");
			ret = -ENODEV;
		}
		break;
	case RC522_IOC_HALT_CHIP:
		if(info->users >= 2){
			printk(KERN_ERR "RC522 Driver: failed to init chip.\n");
			ret = -EBUSY;
			break;
		}
		ret = rc522_halt_chip(info);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to halt chip.\n");
			ret = -ENODEV;
		}
		break;
	case RC522_IOC_ON_ANTE:
		//ret = rc522_enable_antenna(info);
		break;
	case RC522_IOC_OFF_ANTE:
		//rc522_disable_antenna(info);
		break;
	case RC522_IOC_ON_CHIP:
		
		break;
	case RC522_IOC_OFF_CHIP:
		
		break;

	case RC522_IOC_REQ_CARD:
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_request_card(info,xfer);	/* get card type*/
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to request card\n");
			ret = -ENODEV;
			break;
		}
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_anticoll_card(info, xfer);	/* get card id, id csum*/
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to anticoll card\n");
			ret = -EACCES;
			break;
		}
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_select_card(info, xfer);	/* get card capacity, card plus msg */
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to select card\n");
			ret = -EACCES;
			break;
		}
		memcpy(xfer->id_buf, &xfer->txrx_buf[CARD_TYPE_SIZE], CARD_ID_SIZE);
		break;
		
	case RC522_IOC_READ_CARD:
		if(xfer->sect_num < RC522_SECT_0 || xfer->blk_num < RC522_BLK_0 || xfer->blk_num >= RC522_BLK_3){
			printk(KERN_ERR "RC522 Driver: failed to read ctrl blk(3) data directly\n");
			ret = -EINVAL;
			break;
		}
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_read_card(info,xfer);
		if(ret){printk(KERN_ERR "RC522 Driver: failed to read card\n");
			ret = -EINVAL;
			break;
		}
		break;
	case RC522_IOC_WRITE_CARD:
		if(xfer->sect_num < RC522_SECT_0 || xfer->blk_num < RC522_BLK_0 || xfer->blk_num >= RC522_BLK_3){
			printk(KERN_ERR "RC522 Driver: failed to write ctrl blk(3) data directly\n");
			ret = -EINVAL;
			break;
		}
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_write_card(info,xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to write card\n");
			ret = -EINVAL;
			break;
		}
		break;

	/***** inc and dec card *****/
	case RC522_IOC_INC_CARD:
	case RC522_IOC_DEC_CARD:
		if(xfer->sect_num < RC522_SECT_0 || xfer->blk_num < RC522_BLK_0 || xfer->blk_num >= RC522_BLK_3){
			printk(KERN_ERR "RC522 Driver: failed to inc_dec ctrl blk(3) data directly\n");
			ret = -EINVAL;
			break;
		}
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_inc_dec_card(info,xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to inc_dec card\n");
			ret = -EINVAL;
		}
		break;

	/***** critical ops: key ops, careful !*****/
	case RC522_IOC_READ_KEYA:
		if(xfer->sect_num < RC522_SECT_0 ){
			printk(KERN_ERR "RC522 Driver: failed to get used sector\n");
			ret = -EINVAL;
			break;
		}
		xfer->blk_num = RC522_BLK_3;
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		ret = rc522_read_keya(info,xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to read key A\n");
			ret = -EINVAL;
		}
		break;
	case RC522_IOC_READ_KEYB:
		if(xfer->sect_num < RC522_SECT_0 ){
			printk(KERN_ERR "RC522 Driver: failed to get used sector\n");
			ret = -EINVAL;
			break;
		}
		xfer->blk_num = RC522_BLK_3;
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		ret = rc522_read_keyb(info,xfer);
		if(ret){printk(KERN_ERR "RC522 Driver: failed to read key B\n");
			ret = -EINVAL;
		}
		break;
	case RC522_IOC_READ_CTRL:
		if(xfer->sect_num < RC522_SECT_0){
			printk(KERN_ERR "RC522 Driver: failed to get used sector\n");
			ret = -EINVAL;
			break;
		}
		xfer->blk_num = RC522_BLK_3;
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		ret = rc522_read_ctrl(info,xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to read key A\n");
			ret = -EINVAL;
		}
		break;
		
	/***** critical ops: keya , keyb and ctrl area write ops, careful ! 
	****** please check the writed ctrl area using card reference menuel before do it.
	****** set a lock(sect_lock) to protect sector(0-9:  ctrl blk) write ops , 
	****** the protected sector(0-9)'s ctrl area -using default value is ok .
	****** the implement can reduce the begginner to distroy all sector using bad operation .
	*****/
	case RC522_IOC_WRITE_KEYA:
		if(xfer->sect_num < RC522_SECT_0){
			printk(KERN_ERR "RC522 Driver: failed to get used sector: 0x%02x\n", xfer->sect_num);
			ret = -EINVAL;
			break;	
		}
		if((xfer->sect_num < CARD_LOCK_NSECT) &&(xfer->sect_lock != RC522_SECT_UNLOCK)){
			printk(KERN_ERR "RC522 Driver: failed to get unlocked sector: 0x%02x\n", xfer->sect_num);
			ret = -EINVAL;
			break;
		}
		xfer->blk_num = RC522_BLK_3;
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		ret = rc522_write_keya(info,xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to write key B\n");
			ret = -EINVAL;
		}
		break;
		
	case RC522_IOC_WRITE_KEYB:
		if(xfer->sect_num < RC522_SECT_0){
			printk(KERN_ERR "RC522 Driver: failed to get used sector: 0x%02x\n", xfer->sect_num);
			ret = -EINVAL;
			break;	
		}
		if((xfer->sect_num < CARD_LOCK_NSECT) &&(xfer->sect_lock != RC522_SECT_UNLOCK)){
			printk(KERN_ERR "RC522 Driver: failed to get unlocked sector: 0x%02x\n", xfer->sect_num);
			ret = -EINVAL;
			break;
		}
		xfer->blk_num = RC522_BLK_3;
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		ret = rc522_write_keyb(info,xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to write key B\n");
			ret = -EINVAL;
		}
		break;

	case RC522_IOC_WRITE_CTRL:
		if(xfer->sect_num < RC522_SECT_0){
			printk(KERN_ERR "RC522 Driver: failed to get used sector: 0x%02x\n", xfer->sect_num);
			ret = -EINVAL;
			break;	
		}
		if((xfer->sect_num < CARD_LOCK_NSECT) &&
			 (xfer->sect_lock != RC522_SECT_UNLOCK) ){
			printk(KERN_ERR "RC522 Driver: failed to get unlocked sector: 0x%02x\n", xfer->sect_num);
			ret = -EINVAL;
			break;
		}
		xfer->blk_num = RC522_BLK_3;
		memset(info->tx_buffer, 0x00, INFO_BUFFER_SIZE);
		memset(info->rx_buffer, 0x00, INFO_BUFFER_SIZE);
		ret = rc522_auth_card(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
			ret = -EINVAL;
			break;
		}
		ret = rc522_write_ctrl(info,xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to write ctrl\n");
			ret = -EINVAL;
		}
		break;

	default:
		printk(KERN_ERR "RC522 Driver: failed to get used ioc type: ioctype=%d\n", xfer->ioc_type);
		ret = -EINVAL;
		break;
		break;
	}
	return ret;
}

/***** file ops function *****/
static long rc522_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{	
	struct rc522_info *info;
	struct spi_device	*spi_dev;
	struct rc522_ioc_transfer	*xfer;
	unsigned		n_transfer;
	int	tmp = 0, offset = 0, len = 0;
	int	err = 0, ret = 0;
	
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC){
		printk(KERN_ERR "RC522 Driver: failed to get good magic code: \n", _IOC_TYPE(cmd));
		return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
	(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
	(void __user *)arg, _IOC_SIZE(cmd));
	if (err){
		printk(KERN_ERR "RC522 Driver: failed to get read or write addr permission: \n", err);
		return -EFAULT;
	}
	info = filp->private_data;
	spin_lock_irq(&info->spi_lock);
	spi_dev = spi_dev_get(info->spi_dev);
	spin_unlock_irq(&info->spi_lock);
	if (spi_dev == NULL){
		printk(KERN_ERR "RC522 Driver: failed to get alive spi device : \n");
		return -ESHUTDOWN;
	}
	mutex_lock(&info->buf_lock);
	switch (cmd) {
	default:
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			printk(KERN_ERR "RC522 Driver: failed to get used massage : \n");
			ret = -ENOTTY;
			break;
		}
		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct rc522_ioc_transfer)) != 0) {
			printk(KERN_ERR "RC522 Driver: failed to get aligned massage : \n");
			ret = -EINVAL;
			break;
		}
		n_transfer = tmp / sizeof(struct rc522_ioc_transfer);
		if (n_transfer == 0 || n_transfer > 1){
			printk(KERN_ERR "RC522 Driver: failed to support more massage : \n");
			ret = -EINVAL;
			break;
		}
		xfer = kmalloc(tmp, GFP_KERNEL);
		if (!xfer) {
			printk(KERN_ERR "RC522 Driver: failed to allocate struct rc522_ioc_transfer{}: \n");
			ret = -ENOMEM;
			break;
		}
		if (__copy_from_user(xfer, (void __user *)arg, tmp)) {
			printk(KERN_ERR "RC522 Driver: failed to copy kernel space from user space: \n");
			kfree(xfer);
			ret = -EFAULT;
			break;
		}
		ret = rc522_pass_op(info, xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to pass operation\n");
			ret = -EFAULT;
		}
		else{
			switch(xfer->ioc_type){
				case RC522_IOC_REQ_CARD:
				case RC522_IOC_READ_CARD:
				case RC522_IOC_READ_KEYA:
				case RC522_IOC_READ_KEYB:
				case RC522_IOC_WRITE_KEYA:	/*return old value*/
				case RC522_IOC_WRITE_KEYB:
				case RC522_IOC_READ_CTRL:
				case RC522_IOC_WRITE_CTRL:
					//len = tmp;
					len = CARD_ID_SIZE + CARD_BLK_SIZE + CARD_PLUS_SIZE;
					offset = (tmp - len);	
					if(__copy_to_user((uint8_t __user *)(arg)+offset, ((uint8_t *)xfer) + offset, len )){
						printk(KERN_ERR "RC522 Driver: failed to copy k space to u space: \n");
						ret = -EFAULT;
					}
					break;
				default :
					break;	
			}
			;;; //empty statement terminator
		}
		kfree(xfer);
		break;

	/***** spi inferface mode set *****/
	case SPI_IOC_RD_MODE:
		ret = __put_user(spi_dev->mode & SPI_MODE_MASK,(uint8_t __user *)arg); //u8 means its 1 byte unsigned integer, same as uint8_t
		break;
	case SPI_IOC_RD_LSB_FIRST:
		ret = __put_user((spi_dev->mode & SPI_LSB_FIRST) ?  1 : 0,(uint8_t __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		ret = __put_user(spi_dev->bits_per_word, (uint8_t __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		ret = __put_user(spi_dev->max_speed_hz, (int __user *)arg);
		break;
	case SPI_IOC_WR_MODE:
		ret = __get_user(tmp, (uint8_t __user *)arg);
		if (ret == 0) {
			uint8_t	save = spi_dev->mode;
			if (tmp & ~SPI_MODE_MASK) {
				ret = -EINVAL;
				break;
			}
			tmp |= spi_dev->mode & ~SPI_MODE_MASK;
			spi_dev->mode = (uint8_t)tmp;
			ret = spi_setup(spi_dev);
			if (ret < 0) 
				spi_dev->mode = save;
			else 
				printk(KERN_INFO "RC522 Driver: spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		ret = __get_user(tmp, (uint8_t __user *)arg);
		if (ret == 0) {
			uint8_t	save = spi_dev->mode;
			if (tmp)
				spi_dev->mode |= SPI_LSB_FIRST;
			else 
				spi_dev->mode &= ~SPI_LSB_FIRST;
			ret = spi_setup(spi_dev);
			if (ret < 0) 
				spi_dev->mode = save;
			else 
				printk(KERN_INFO "RC522 Driver: %csb first\n",tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		ret = __get_user(tmp, (uint8_t __user *)arg);
		if (ret == 0) {
			uint8_t	save = spi_dev->bits_per_word;
			spi_dev->bits_per_word = tmp;
			ret = spi_setup(spi_dev);
			if (ret < 0) 
				spi_dev->bits_per_word = save;
			else 
				printk(KERN_INFO "RC522 Driver: %d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		ret = __get_user(tmp, (int __user *)arg);
		if (ret == 0) {
			int	save = spi_dev->max_speed_hz;
			spi_dev->max_speed_hz = tmp;
			ret = spi_setup(spi_dev);
			if (ret < 0) 
				spi_dev->max_speed_hz = save;
			else 
				printk(KERN_INFO "RC522 Driver: %d Hz (max)\n", tmp);
		}
		break;
	}
	mutex_unlock(&info->buf_lock);
	spi_dev_put(spi_dev);
	return ret;
}

static int rc522_open(struct inode *inode, struct file *filp)
{	
	struct rc522_info *info;
	int ret = -ENXIO;
	mutex_lock(&device_list_lock);
	list_for_each_entry(info, &device_list, device_entry) {
		if (info->devt == inode->i_rdev) {
			ret = 0;
			break;
		}
	}
	if (ret == 0) {
		if(info->users == 0){	
			if(ret = rc522_init_chip(info)){
				printk(KERN_ERR "RC522 Driver: failed to open rc522\n");
			}
		}
		info->users++;
		filp->private_data = info;
		nonseekable_open(inode, filp);
	} else 
		printk(KERN_ERR "RC522 Driver: nothing for minor %d\n", iminor(inode));
	mutex_unlock(&device_list_lock);

	return ret;
}
static int rc522_release(struct inode *inode, struct file *filp)
{	
	struct rc522_info *info;
	int ret = 0;
	mutex_lock(&device_list_lock);
	info = filp->private_data;
	filp->private_data = NULL;
	info->users--;
	if (!info->users) {
		int dofree;
		spin_lock_irq(&info->spi_lock);
		dofree = (info->spi_dev == NULL);
		spin_unlock_irq(&info->spi_lock);
		if (dofree) kfree(info);
	}
	mutex_unlock(&device_list_lock);
	return info;
}

/***** probe and remove function ****/
#ifdef RC522_TEST
void rc522_test_card(struct rc522_info *info)
{
	/* only for test mode, normal mode will remove follow code  */
	rc522_init_chip(info);
	int i = 2;
	int ret = 0;
	struct rc522_ioc_transfer xfer={
		key_type: RC522_KEY_A,
		sect_num: RC522_SECT_15,
		blk_num: RC522_BLK_0,
	};
	for(i = 0; i<CARD_KEYA_SIZE; i++){
		xfer.keya_buf[i] = (uint8_t)0xff;
		xfer.keyb_buf[i] = (uint8_t)0xff;
	}
	while(i--){
		ret = rc522_request_card(info, &xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to request card\n");
		}
		else{
			printk(KERN_INFO "RC522 Driver: good to request card: 0x%02x, 0x%02x\n", xfer.txrx_buf[0], xfer.txrx_buf[1]);
			break;
		}
		msleep(50);
	}
	if(i == 0){
		return ;
	}
	// rc522 spi1.0: good to anticoll card : 0x30, 0x93, 0xb7, 0x4f, 0x00
	ret = rc522_anticoll_card(info, &xfer);
		if(ret){
			printk(KERN_ERR "RC522 Driver: failed to anticoll card\n");
		}
		else{
			printk(KERN_INFO "RC522 Driver: good to anticoll card : 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", xfer.txrx_buf[2], xfer.txrx_buf[3], xfer.txrx_buf[4], xfer.txrx_buf[5], xfer.txrx_buf[6]);
		}
	ret = rc522_select_card(info, &xfer);
	if(ret){
		printk(KERN_ERR "RC522 Driver: failed to select card\n");
	}else{
		memcpy(xfer.id_buf, &xfer.txrx_buf[CARD_TYPE_SIZE], CARD_ID_SIZE);
		printk(KERN_INFO "RC522 Driver: good to select card: \n"
				"type = 0x%02x, 0x%02x\n"
				"id = 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n"
				"capacity = 0x%02x Kbits, 0x%02x, 0x%02x\n", 
				xfer.txrx_buf[0], xfer.txrx_buf[1], 
				xfer.txrx_buf[2], xfer.txrx_buf[3], 
				xfer.txrx_buf[4], xfer.txrx_buf[5],
				xfer.txrx_buf[6], 
				xfer.txrx_buf[7],
				xfer.txrx_buf[8], xfer.txrx_buf[9]);
	}
	msleep(50);
	printk(KERN_INFO "RC522 Driver: 1 Read Card:");
	ret = rc522_auth_card(info, &xfer);
	if(ret ){ 
		printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
	}else{ 
		printk(KERN_INFO "RC522 Driver: good to auth card: \n");}
	ret = rc522_read_card(info, &xfer);
	if(ret ){
		printk(KERN_ERR "RC522 Driver: failed to read card\n");
	}else{
		printk(KERN_INFO "RC522 Driver: good to read card: ");
		printk(KERN_INFO "RC522 Driver: sect=0x%02x, blk=0x%02x\n",xfer.sect_num, xfer.blk_num);
		for(i = 0; i< CARD_BLK_SIZE+CARD_PLUS_SIZE; i++){
			if(i%4 == 0) 
				printk(KERN_INFO "RC522 Driver: \n");
			printk(KERN_INFO "RC522 Driver: 0x%02x	",xfer.txrx_buf[i]);
		}
		printk(KERN_INFO "\n");
	}
	printk(KERN_INFO "\n");
	printk(KERN_INFO "RC522 Driver: will write card:  0x%02x, 0x%02x\n",xfer.sect_num, xfer.blk_num );
	for(i = 0; i<CARD_BLK_SIZE; i++){
		if(i%4 == 0)
			printk(KERN_INFO "RC522 Driver: \n");
		printk(KERN_INFO "RC522 Driver: 0x%02x ", i);
		xfer.txrx_buf[i]= (unsigned char) i;
	}
	printk(KERN_INFO "RC522 Driver: \n");
	ret = rc522_write_card(info, &xfer);
	if(ret){
	printk(KERN_ERR "RC522 Driver: failed to read card\n");
	}else{printk(KERN_INFO "RC522 Driver: good to write card: ");
		printk(KERN_INFO "RC522 Driver: sect=0x%02x, blk=0x%02x\n",xfer.sect_num, xfer.blk_num);
		for(i = 0; i< CARD_BLK_SIZE; i++){
			if(i%4 == 0)printk(KERN_INFO "RC522 Driver: \n");
			printk(KERN_INFO "RC522 Driver: 0x%02x	", xfer.txrx_buf[i]);
		}
		printk(KERN_INFO "RC522 Driver: \n");
	}
	msleep(50);
	printk(KERN_INFO "RC522 Driver: 2 Read Card:");
	ret = rc522_auth_card(info, &xfer);
	if(ret){ 
		printk(KERN_ERR "RC522 Driver: could not authorized the card.\n");
	}
	else{
		printk(KERN_INFO "RC522 Driver: good to auth card: \n");}
	ret = rc522_read_card(info, &xfer);
	if(ret){
		printk(KERN_ERR "RC522 Driver: failed to read card\n");
	}
	else{
		printk(KERN_INFO "RC522 Driver: good to read card: ");
		printk(KERN_INFO "RC522 Driver: sect=0x%02x, blk=0x%02x\n",xfer.sect_num, xfer.blk_num);
		for(i = 0; i< CARD_BLK_SIZE+CARD_PLUS_SIZE; i++){
			if(i%4 == 0) printk(KERN_INFO "RC522 Driver: \n");
			printk(KERN_INFO "RC522 Driver: 0x%02x	",xfer.txrx_buf[i]);
		}
		printk(KERN_INFO "RC522 Driver: \n");
	}
}
#else
void rc522_test_card(struct rc522_info *info){}
#endif
static int rc522_probe(struct spi_device *spi_dev)
{	printk(KERN_INFO "RC522 Driver: rc522_probe()\n");
	struct rc522_info *info;
	unsigned long minor;
	int ret;
	info = kzalloc(sizeof(struct rc522_info), GFP_KERNEL);
	if (info == NULL) {
		printk(KERN_ERR "RC522 Driver: failed to allocate struct.\n");
		return -ENOMEM;
	}
	info->spi_dev = spi_dev;
	INIT_LIST_HEAD(&info->device_entry);
	spin_lock_init(&info->spi_lock);
	mutex_init(&info->buf_lock);
	spi_set_drvdata(spi_dev, info);
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		info->devt = MKDEV(SPICHAR_MAJOR, minor);
		dev = device_create(rc522_class, &spi_dev->dev, info->devt, info, "rc522-%d.%d",spi_dev->master->bus_num, spi_dev->chip_select);
		 if(IS_ERR(dev)){
			printk(KERN_ERR "RC522 Driver: failed to create device.\n");
			ret = IS_ERR(dev);
			goto exit_1;
		 }
	} 
	else {
		printk(KERN_ERR "RC522 Driver: failed to allocate minor number.\n");
		ret = -ENODEV;
		goto exit_1;
	}
	set_bit(minor, minors);
	list_add(&info->device_entry, &device_list);
	mutex_unlock(&device_list_lock);
	rc522_test_card(info);
	return 0;
		
	exit_1:
		kfree(info);
		spi_set_drvdata(spi_dev, NULL);
		return ret;
}

static int rc522_remove(struct spi_device *spi_dev)
{
	printk(KERN_INFO "RC522 Driver: removing RC522.\n");
	struct rc522_info *info = spi_get_drvdata(spi_dev);
	if(info == NULL) return 0;
	spin_lock_irq(&info->spi_lock);
	info->spi_dev = NULL;
	spi_set_drvdata(spi_dev, NULL);
	spin_unlock_irq(&info->spi_lock);
	mutex_lock(&device_list_lock);
	list_del(&info->device_entry);
	device_destroy(rc522_class, info->devt);
	clear_bit(MINOR(info->devt), minors);
	if (info->users == 0) kfree(info);
	mutex_unlock(&device_list_lock);
	return 0;
}

/***** spi board info and spi driver structure ****/
static struct spi_board_info rc522_spi_board_info = {
	modalias: SPI_DEVICE_NAME,
	//irq: IRQ_EINT4,
	max_speed_hz: 400000,
	bus_num: 1, //use spi1 in raspberry pi 3, must add "dtoverlay=spi1-1cs, cs0_spidev=disabled" to config.txt
	chip_select: 0, //CS0, GPIO8 on raspberry pi 3
	mode: SPI_MODE_0,
};
static struct spi_driver rc522_spi_driver = {
	driver: {
		name: SPI_DEVICE_NAME,	/* rc522 != spidev*/
		owner: THIS_MODULE,
	},
	probe: rc522_probe,
	remove: rc522_remove,
};

static int __init rc522_init(void)
{	
	int ret;
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	ret = register_chrdev(SPICHAR_MAJOR, SPI_DEVICE_NAME, &rc522_fops);
	if (ret < 0){
		printk(KERN_ERR"RC522 Driver: failed to register char device\n");
		return ret;
	}
	rc522_class = class_create(THIS_MODULE, SPI_DEVICE_NAME);
	if (IS_ERR(rc522_class)) {
		printk(KERN_ERR "RC522 Driver: failed to create class\n");
		ret = PTR_ERR(rc522_class);
		goto exit_1;
	}
	/***** find spi_master from system by bus_num, then create spi_device .
	****** if you insert module again, 
	****** the kernel spi subsystem will make a bug(selectchip busy error).
	****** the follow code can reduce the bug, and get more help for debug .
	****** the condition is kernel has spi master driver surpport .
	*****/
	master = spi_busnum_to_master(rc522_spi_board_info.bus_num);
	if(IS_ERR(master)){
		printk(KERN_ERR "RC522 Driver: failed to find spi master \n");
		ret = PTR_ERR(rc522_class);
		goto exit_2;
	}
	spi = spi_new_device(master, &rc522_spi_board_info);
	if(IS_ERR(master)){
		printk(KERN_ERR "RC522 Driver: failed to create spi device \n");
		ret = PTR_ERR(rc522_class);
		goto exit_3;
	}
	ret = spi_register_driver(&rc522_spi_driver);
	if (ret < 0) {
		printk(KERN_ERR "RC522 Driver: failed to register spi driver\n");
		goto exit_4;
	}
	printk(KERN_INFO "RC522 Driver: RC522 Driver module is loaded.\n");
	return 0;

	exit_4:
		spi_unregister_device(spi);
		spi = NULL;
	exit_3:
		put_device(&master->dev);
		master = NULL;
	exit_2:class_destroy(rc522_class);
		rc522_class = NULL;
	exit_1:
		unregister_chrdev(SPICHAR_MAJOR, rc522_spi_driver.driver.name);
		return ret;
}
static void __exit rc522_exit(void)
{
	spi_unregister_driver(&rc522_spi_driver);
	if(spi != NULL) spi_unregister_device(spi);
	if(master != NULL) put_device(&master->dev);
	class_destroy(rc522_class);
	unregister_chrdev(SPICHAR_MAJOR, rc522_spi_driver.driver.name);
	printk(KERN_INFO "RC522 Driver: RC522 module is unloaded.\n");
}
module_init(rc522_init);
module_exit(rc522_exit);
