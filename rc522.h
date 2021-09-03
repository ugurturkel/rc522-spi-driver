#ifndef __RC522_H
#define __RC522_H
#include <linux/types.h>


/***** rc522 cmd and s50 cmd and regs *****/
#define PCD_IDLE            0x00    /* Cancle the current command */
#define PCD_AUTHENT         0x0E    /* Authentication key */
#define PCD_RECEIVE         0x08    /* Receive data */
#define PCD_TRANSMIT        0x04    /* To send data */
#define PCD_TRANSCEIVE      0x0C    /* To send and receive data */
#define PCD_RESETPHASE      0x0F    /* Reset */ 
#define PCD_CALCCRC         0x03    /* CRC calculation */

#define PICC_REQIDL         0x26    /* Looking for a IC card in the area of antenna, that 
                                       didn't enter a dormant state the area of IC card */
#define PICC_REQALL         0x52    /* Looking for all IC card in the area of antena */
#define PICC_ANTICOLL1      0x93    /* Anticollision */
#define PICC_ANTICOLL2      0x95    /* Anticollision */
#define PICC_AUTHENT1A      0x60    /* Authentication A key */
#define PICC_AUTHENT1B      0x61    /* Authentication B key */
#define PICC_READ           0x30    /* Read block */
#define PICC_WRITE          0xA0    /* Write block */
#define PICC_DECREMENT      0xC0    /* Deductions */
#define PICC_INCREMENT      0xC1    /* Rechange */
#define PICC_RESTORE        0xC2    /* The block of data transferrend to the buffer */
#define PICC_TRANSFER       0xB0    /* Save a data in a buffer */
#define PICC_HALT           0x50    /* dormancy */


#define DEF_FIFO_LENGTH     64      /* FIFO size = 64byte */
#define MAXRLEN             18      /* The maximum length of data received */

/***** PAGE 0 *****/
#define RFU00               0x00
#define COMMAND_REG         0x01
#define COM_IEN_REG         0x02
#define DIVL_EN_REG         0x03
#define COM_IRQ_REG         0x04
#define DIV_IRQ_REG         0x05
#define ERROR_REG           0x06
#define STATUS1_REG         0x07
#define STATUS2_REG         0x08
#define FIFO_DATA_REG       0x09
#define FIFO_LEVEL_REG      0x0A
#define WATER_LEVEL_REG     0x0B
#define CONTROL_REG         0x0C
#define BIT_FRAMING_REG     0x0D
#define COLL_REG            0x0E
#define RFU0F               0x0F

#define RFU10               0x10
#define MODE_REG            0x11
#define TX_MODE_REG         0x12
#define RX_MODE_REG         0x13
#define TX_CONTROL_REG      0x14
#define TX_AUTO_REG         0x15
#define TX_SEL_REG          0x16
#define RX_SEL_REG          0x17
#define RX_THRESHOLD_REG    0x18
#define DEMOD_REG           0x19
#define RFU1A               0x1A
#define RFU1B               0x1B
#define MIFARE_REG          0x1C
#define RFU1D               0x1D
#define RFU1E               0x1E
#define SERIAL_SPEED_REG    0x1F

#define RFU20               0x20  
#define CRC_RESULT_REG_M    0x21
#define CRC_RESULT_REG_L    0x22
#define RFU23               0x23
#define MOD_WIDTH_REG       0x24
#define RFU25               0x25
#define RF_CFG_REG          0x26
#define GS_NREG             0x27
#define CWGS_CFG_REG        0x28
#define MOD_GS_CFG_REG      0x29
#define T_MODE_REG          0x2A
#define T_PRESCALER_REG     0x2B
#define T_RELOAD_REG_H      0x2C
#define T_RELOAD_REG_L      0x2D
#define T_COUN_VALUE_REGH   0x2E
#define T_COUN_VALUE_REGL   0x2F

#define RFU30               0x30
#define TEST_SEL1_REG       0x31
#define TEST_SEL2_REG       0x32
#define TEST_PIN_EN_REG     0x33
#define TEST_PIN_VALUE_REG  0x34
#define TEST_BUS_REG        0x35
#define AUTO_TEST_REG       0x36
#define VERSION_REG         0x37
#define ANALOG_TEST_REG     0x38
#define TEST_DAC1_REG       0x39  
#define TEST_DAC2_REG       0x3A   
#define TEST_ADC_REG        0x3B   
#define RFU3C               0x3C   
#define RFU3D               0x3D   
#define RFU3E               0x3E   
#define RFU3F               0x3F

#define MI_OK               0    
#define MI_NOTAGERR         (-1)
#define MI_ERR              (-2)


/***** device name  *****/

#define SPICHAR_MAJOR			156	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

#define SPI_DEVICE_NAME "rc522"

/***** user transfer data structure *****/
#define CARD_TYPE_SIZE 		2
#define CARD_ID_SIZE 		4
#define CARD_IDCSUM_SIZE 	1
#define CARD_CAP_SIZE 		1
#define CARD_MSG_SIZE 		2

#define CARD_BLK_SIZE		16
#define CARD_PLUS_SIZE 		4

#define CARD_KEYA_SIZE 		6
#define CARD_CTRL_SIZE 		4
#define CARD_KEYB_SIZE 		6

#define CARD_CRC_SIZE 		2

#define CARD_INC_SIZE 		4

#define CARD_SECT_NBLK 		4

/***** user can modify the value to decrease locked sector *****/
#define CARD_LOCK_NSECT 	10

/***** spi mode *****/
#define SPI_CPHA			0x01
#define SPI_CPOL			0x02

#define SPI_MODE_0			(0|0)
#define SPI_MODE_1			(0|SPI_CPHA)
#define SPI_MODE_2			(SPI_CPOL|0)
#define SPI_MODE_3			(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH			0x04
#define SPI_LSB_FIRST		0x08
#define SPI_3WIRE			0x10
#define SPI_LOOP			0x20
#define SPI_NO_CS			0x40
#define SPI_READY			0x80

#define SPI_IOC_MAGIC		'k'

/***** driver data structure *****/
#define INFO_BUFFER_SIZE 32

#define SPI_MSGSIZE(N) \
	((((N)*(sizeof (struct rc522_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct rc522_ioc_transfer))) : 0)
#define SPI_IOC_MESSAGE(N) 			_IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])

#define SPI_IOC_RD_MODE				_IOR(SPI_IOC_MAGIC, 1, uint8_t)
#define SPI_IOC_WR_MODE				_IOW(SPI_IOC_MAGIC, 1, uint8_t)

#define SPI_IOC_RD_LSB_FIRST		_IOR(SPI_IOC_MAGIC, 2, uint8_t)
#define SPI_IOC_WR_LSB_FIRST		_IOW(SPI_IOC_MAGIC, 2, uint8_t)

#define SPI_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOC_MAGIC, 3, uint8_t)
#define SPI_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOC_MAGIC, 3, uint8_t)

#define SPI_IOC_RD_MAX_SPEED_HZ		_IOR(SPI_IOC_MAGIC, 4, uint32_t)
#define SPI_IOC_WR_MAX_SPEED_HZ		_IOW(SPI_IOC_MAGIC, 4, uint32_t)


#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)


enum rc522_ioc_type {
	RC522_IOC_NONE_0 = 0, // avoid bad operation
	RC522_IOC_INIT_CHIP,
	RC522_IOC_HALT_CHIP,
	RC522_IOC_ON_ANTE,
	RC522_IOC_OFF_ANTE,
	RC522_IOC_REQ_CARD,
	RC522_IOC_ON_CHIP,
	RC522_IOC_OFF_CHIP,
	RC522_IOC_NONE_1, // recerve operation
	RC522_IOC_NONE_2, // recerve operation
	
	RC522_IOC_READ_CARD = 10,
	RC522_IOC_WRITE_CARD,

	RC522_IOC_INC_CARD,
	RC522_IOC_DEC_CARD,

	
	RC522_IOC_READ_KEYA,
	RC522_IOC_READ_KEYB,
	
	RC522_IOC_WRITE_KEYA,
	RC522_IOC_WRITE_KEYB,

	RC522_IOC_READ_CTRL,
	RC522_IOC_WRITE_CTRL,
	/* add multi ioc cmd */
};
enum rc522_sect_num {
	RC522_SECT_0 = 0,
	RC522_SECT_1,
	RC522_SECT_2,
	RC522_SECT_3,
	
	RC522_SECT_4,
	RC522_SECT_5,
	RC522_SECT_6,
	RC522_SECT_7,
	
	RC522_SECT_8,
	RC522_SECT_9,
	RC522_SECT_10,
	RC522_SECT_11,
	
	RC522_SECT_12,
	RC522_SECT_13,
	RC522_SECT_14,
	RC522_SECT_15,

};
enum rc522_blk_num {
	RC522_BLK_0 = 0,
	RC522_BLK_1,
	RC522_BLK_2,
	RC522_BLK_3,
};
enum rc522_key_type {
	RC522_KEY_A = 0,
	RC522_KEY_B = 1,
	RC522_KEY_A_B = 2,
};

enum rc522_sect_lock {
	RC522_SECT_LOCK = 0,
	RC522_SECT_UNLOCK = 1,
};

struct rc522_ioc_transfer {
	uint32_t	speed_hz;
	uint32_t	pad;
	uint16_t	delay_usecs;
	uint8_t	bits_per_word;
	uint8_t	cs_change;

	enum rc522_ioc_type ioc_type;	/* io ctrl cmd : req, read, write... */
	enum rc522_key_type key_type;	/* key type: key a, key b, key a and b */
	
	enum rc522_sect_num sect_num;	/* sector number: 0~15 */
	enum rc522_blk_num blk_num;	/* block number: 0~4 */

	enum rc522_sect_lock sect_lock;	/* sect lock: 0=lock, 1=unlock */
	
	uint8_t	keya_buf[CARD_KEYA_SIZE];	/*key a space: 6 Bytes*/
	uint8_t	ctrl_buf[CARD_CTRL_SIZE];	/*ctrl space: 4 Bytes*/
	uint8_t	keyb_buf[CARD_KEYB_SIZE];	/*key b space: 6 Bytes*/
	
	uint8_t	id_buf[CARD_ID_SIZE];	/*id space: 4Bytes for auth card*/
	uint8_t	txrx_buf[CARD_BLK_SIZE + CARD_PLUS_SIZE]; /*w, r space: 16 + 4 Bytes*/
	
};

/***** driver data structure *****/
struct rc522_platdata{
	unsigned long	power_pin;
	int (*setup_pin)(void);
	int (*exit_pin)(void);
};

struct rc522_info{
	struct list_head	device_entry;
	struct spi_device	*spi_dev;
	
	unsigned		users;
	dev_t			devt;
	spinlock_t		spi_lock;
	struct mutex	buf_lock;

	uint8_t			tx_buffer[INFO_BUFFER_SIZE];
	uint8_t			rx_buffer[INFO_BUFFER_SIZE];
};

static long rc522_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int rc522_open(struct inode *inode, struct file *filp);
static int rc522_release(struct inode *inode, struct file *filp);

//#define RC522_TEST //for activating the test code

#endif
