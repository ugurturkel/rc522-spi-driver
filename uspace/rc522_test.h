#ifndef __RC522_TEST_H
#define __RC522_TEST_H
#include <linux/types.h>


/***** user transfer data structure *****/
#define 	CARD_TYPE_SIZE 		2
#define 	CARD_ID_SIZE 		4
#define 	CARD_IDCSUM_SIZE 	1
#define 	CARD_CAP_SIZE 		1
#define 	CARD_MSG_SIZE 		2
#define 	CARD_BLK_SIZE		16
#define 	CARD_PLUS_SIZE 		4
#define 	CARD_KEYA_SIZE 		6
#define 	CARD_CTRL_SIZE 		4
#define 	CARD_KEYB_SIZE 		6
#define 	CARD_CRC_SIZE 		2
#define 	CARD_INC_SIZE 		4
#define 	CARD_SECT_NBLK 		4

/***** user can modify the value to decrease locked sector *****/
#define 	CARD_LOCK_NSECT 	10

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
	uint8_t		bits_per_word;
	uint8_t		cs_change;

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



/***** spi mode *****/
#define 	SPI_CPHA		0x01
#define 	SPI_CPOL		0x02

#define 	SPI_MODE_0		(0|0)
#define 	SPI_MODE_1		(0|SPI_CPHA)
#define 	SPI_MODE_2		(SPI_CPOL|0)
#define 	SPI_MODE_3		(SPI_CPOL|SPI_CPHA)

#define 	SPI_CS_HIGH		0x04
#define 	SPI_LSB_FIRST	0x08
#define 	SPI_3WIRE		0x10
#define 	SPI_LOOP		0x20
#define 	SPI_NO_CS		0x40
#define 	SPI_READY		0x80

#define SPI_IOC_MAGIC		'k'


#define SPI_MSGSIZE(N) \
	((((N)*(sizeof (struct rc522_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct rc522_ioc_transfer))) : 0)
#define SPI_IOC_MESSAGE(N) _IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])

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


#endif
