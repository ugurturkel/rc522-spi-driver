#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#include "rc522_test.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static const char *device = "/dev/rc522-0.1";
static uint8_t mode = SPI_MODE_0;
static uint8_t bits = 8;
static uint32_t speed = 400000;
static uint16_t delay;

static void pabort(const char *s){perror(s);abort();}
static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}
static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 	 0, 0, 	0  },
		};
		int c;
		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);
		if (c == -1)
			break;
		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}


/***** rc522 card ops *****/
static int rc522_operate_card(int fd, struct rc522_ioc_transfer *xfer)
{
	int ret = 0;
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), xfer);
	if (ret == -1)
		printf("ioctl error.\n");
	return ret;
}


/***** rc522 read, write blk ops *****/
static int rc522_read_card(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i =0;

	xfer->ioc_type = RC522_IOC_READ_CARD;
	
	printf("/***** rc522 read card test:  start *****/\n");
	printf("read 0x%02x sector: start\n", xfer->sect_num);
	printf("read 0x%02x blk: start", xfer->blk_num);
	memset(xfer->txrx_buf, (uint8_t)0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_BLK_SIZE; i++){
		if(i%4 == 0) printf("\n");
		printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\nread 0x%02x blk: end\n", xfer->blk_num);
	printf("read 0x%02x sector: end\n", xfer->sect_num);
	printf("/***** rc522 read card test:  end *****/\n");
}

static int rc522_write_card(int fd, struct rc522_ioc_transfer *xfer)
{

	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i =0;

	xfer->ioc_type = RC522_IOC_WRITE_CARD;
	
	printf("/***** rc522 write card test:  start *****/\n");
	printf("write 0x%02x sector: start\n", xfer->sect_num);
	printf("write 0x%02x blk: start", xfer->blk_num);
	memset(xfer->txrx_buf, (uint8_t)(rand()%10), CARD_BLK_SIZE);
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_BLK_SIZE; i++){
		if(i%4 == 0) printf("\n");
			printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\nwrite 0x%02x blk: end\n", xfer->blk_num);
	printf("write 0x%02x sector: end\n", xfer->sect_num);
	printf("/***** rc522 write card test:  end *****/\n");
}

/***** rc522 inc, dec blk ops *****/
static int rc522_inc_card(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i =0;
	xfer->ioc_type = RC522_IOC_INC_CARD;
	printf("/***** rc522 inc card test:  start *****/\n");
	printf("inc 0x%02x sector: start\n", xfer->sect_num);
	printf("inc 0x%02x blk: start", xfer->blk_num);
	for(i=0; i<=3; i++){
		xfer->txrx_buf[i] = 0x00; 
	}
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_INC_SIZE; i++){
		if(i%4 == 0) printf("\n");
		printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\ninc 0x%02x blk: end\n", xfer->blk_num);
	printf("inc 0x%02x sector: end\n", xfer->sect_num);
	printf("/***** rc522 inc card test:  end *****/\n");
}
static int rc522_dec_card(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i = 0;

	xfer->ioc_type = RC522_IOC_DEC_CARD;
	
	printf("/***** rc522 dec card test:  start *****/\n");
	printf("dec 0x%02x sector: start\n", xfer->sect_num);
	printf("dec 0x%02x blk: start", xfer->blk_num);
	xfer->txrx_buf[0] = 0x01; 
	for(i=1; i<=3; i++){
		xfer->txrx_buf[i] = 0x00; 
	}
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_INC_SIZE; i++){
		if(i%4 == 0) printf("\n");
		printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\ndec 0x%02x blk: end\n", xfer->blk_num);
	printf("dec 0x%02x sector: end\n", xfer->sect_num);
	printf("/***** rc522 dec card test:  end *****/\n");
}

/***** rc522 key area ops *****/
static int rc522_read_keya(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i = 0;
	
	xfer->ioc_type = RC522_IOC_READ_KEYA;
	printf("/***** rc522 read keyA test:  start *****/\n");
	printf("read 0x%02x sector keyA: start\n", xfer->sect_num);
	memset(xfer->txrx_buf, (uint8_t)0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_KEYA_SIZE; i++){
		if(i%4 == 0) printf("\n");
		printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\nread 0x%02x sector keya: end\n\n", xfer->sect_num);
	printf("/***** rc522 read keya test:  end *****/\n");
}
static int rc522_read_keyb(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i =0;
	xfer->ioc_type = RC522_IOC_READ_KEYB;
	printf("/***** rc522 read keyb test:  start *****/\n");
	printf("read 0x%02x sector keyb: start\n", xfer->sect_num);
	memset(xfer->txrx_buf, (uint8_t)0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_KEYA_SIZE; i++){
		if(i%4 == 0) printf("\n");
		printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\nread 0x%02x sector keyb: end\n", xfer->sect_num);
	printf("/***** rc522 read keyB test:  end *****/\n");
}
static int rc522_read_ctrl(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i = 0;

	xfer->ioc_type = RC522_IOC_READ_CTRL;
	printf("/***** rc522 read ctrl test:  start *****/\n");
	printf("read 0x%02x sector ctrl: start\n", xfer->sect_num);
	memset(xfer->txrx_buf, (uint8_t)0x00, CARD_BLK_SIZE+CARD_PLUS_SIZE);
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_CTRL_SIZE; i++){
		printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\nread 0x%02x sector ctrl: end\n", xfer->sect_num);
	printf("/***** rc522 read ctrl test:  end *****/\n");
}

static int rc522_write_keya(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	unsigned char rand_key = (uint8_t)(rand()/2);
	int i = 0;
	xfer->ioc_type = RC522_IOC_WRITE_KEYA;

	
	printf("/***** rc522 write keya test:  start *****/\n");
	printf("write 0x%02x sector keya: start\n", xfer->sect_num);
	for(i=0; i<=5; i++){
		xfer->txrx_buf[i] =  (uint8_t)0xFF;
	}
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_KEYA_SIZE; i++){
		printf("0x%02x	", xfer->txrx_buf[i]);
	}	
	printf("\nwrite 0x%02x sector keya: end\n", xfer->sect_num);
	printf("/***** rc522 write keya test:  end *****/\n");
}
static int rc522_write_keyb(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	unsigned char rand_key = (uint8_t)(rand()/2);
	
	int i = 0;
	xfer->ioc_type = RC522_IOC_WRITE_KEYB;


	
	printf("/***** rc522 write keyb test:  start *****/\n");
	printf("write 0x%02x sector keyb: start\n", xfer->sect_num);
	for(i=0; i<=5; i++){
		xfer->txrx_buf[i] =  (uint8_t)0xFF;
	}
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_KEYB_SIZE; i++){
		printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\nwrite 0x%02x sector keyb: end\n", xfer->sect_num);
	printf("/***** rc522 write keyb test:  end *****/\n");
	return 0;
}

static int rc522_write_ctrl(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i = 0;

	xfer->ioc_type = RC522_IOC_WRITE_CTRL;

	printf("rc522 write ctrl test:  start *****/\n");
	printf("write 0x%02x sector ctrl: start\n", xfer->sect_num);
	xfer->txrx_buf[0] =  (uint8_t)0xFF; /* using default value*/
	xfer->txrx_buf[1] =  (uint8_t)0x07; /* using default value*/
	xfer->txrx_buf[2] =  (uint8_t)0x80; /* using default value*/
	xfer->txrx_buf[3] =  (uint8_t)0x69; /* using default value*/
	if(rc522_operate_card(fd, xfer))
		return 0;
	for(i=0; i<CARD_CTRL_SIZE; i++){
		printf("0x%02x	", xfer->txrx_buf[i]);
	}
	printf("\nwrite 0x%02x sector ctrl: end\n", xfer->sect_num);
	printf("/***** rc522 write ctrl test:  end *****/\n");
}


/***** critical ops: the user can have all permission for protected sector(0-9) *****/
static int rc522_unlock_low_sect(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	int i = 0;
	xfer->sect_lock = RC522_SECT_UNLOCK;
	
	printf("/***** rc522 unlock low sect test:  start *****/\n");
	printf("unlock 0x%02x sector: start\n", xfer->sect_num);
	printf("%s: \n%s \nfor %s\n",
	xfer->sect_lock == RC522_SECT_UNLOCK ? 
	"unlock low sector(0-9) mode" : 
	"lock low sector(0-9) mode",
	xfer->sect_lock == RC522_SECT_UNLOCK ? 
	"the user have all permission to write ctrl blk(3)" : 
	"the user have non permission to write ctrl blk(3)",
	xfer->sect_lock == RC522_SECT_UNLOCK ? 
	"all sector(0-15)" : 
	"low sector(0-9)");	
}

static int rc522_lock_low_sect(int fd, struct rc522_ioc_transfer *xfer)
{
	enum rc522_sect_num sect = 0;
	enum rc522_blk_num blk = 0;
	unsigned char rand_key = (uint8_t)(rand()/2);
	int i = 0;
	xfer->sect_lock = RC522_SECT_LOCK;
	printf("unlock 0x%02x sector: end\n", xfer->sect_num);
	printf("/***** rc522 unlock low sect test:  end *****/\n");
	printf("%s: \n%s \nfor %s\n", 
	xfer->sect_lock == RC522_SECT_UNLOCK ? 
	"unlock low sector(0-9) mode" : 
	"lock low sector(0-9) mode",
	xfer->sect_lock == RC522_SECT_UNLOCK ? 
	"the user have all permission to write ctrl blk(3)" : 
	"the user have non permission to write ctrl blk(3)",
	xfer->sect_lock == RC522_SECT_UNLOCK ? 
	"all sector(0-15)" : 
	"low sector(0-9)");	
}
static int rc522_goto_menu(int fd, struct rc522_ioc_transfer *xfer)
{
	return 0;
}
struct {
	void (*function)(int fd, struct rc522_ioc_transfer *xfer);
	char *msg;
	}ioc_type_menu[] = {
	{ rc522_goto_menu,	"rc522 goto before menu: \n" } ,
	{ rc522_read_card,	"rc522 read card test: \n" } ,
	{ rc522_write_card,	"rc522 write card test: \n\n" } ,
	{ rc522_read_keya,	"rc522 read keya test: \n" } ,
	{ rc522_read_keyb,	"rc522 read keyb test: \n" } ,
	{ rc522_read_ctrl, 	"rc522 read ctrl area: \n\n" } ,
	{ rc522_write_keya,	"rc522 write keya test:	\n" } ,
	{ rc522_write_keyb,	"rc522 write keyb test: \n" } ,
	{ rc522_write_ctrl, "rc522 write ctrl area: \n\n" } ,
	{ rc522_lock_low_sect,	"rc522 lock low sect test:	\n" } ,
	{ rc522_unlock_low_sect,"rc522 unlock low sect test:	\n" } ,
	{0, 0}						
};
static int rc522_menu_card(int fd, struct rc522_ioc_transfer *xfer)
{
	int i = 0;
	int select_index = 0;
	uint32_t select_sect = 0;
	
	while(1){
		xfer->sect_lock = RC522_SECT_LOCK;
		printf("RC522 Test Card Menu:\n");
		printf("%s: \n%s \nfor %s\n", 
		xfer->sect_lock == RC522_SECT_UNLOCK ? 
		"unlock low sector(0-9) mode" : 
		"lock low sector(0-9) mode",
		xfer->sect_lock == RC522_SECT_UNLOCK ? 
		"the user have all permission to write ctrl blk(3)" : 
		"the user have non permission to write ctrl blk(3)",
		xfer->sect_lock == RC522_SECT_UNLOCK ? 
		"all sector(0-15)" : 
		"low sector(0-9)");	
		
		printf("\nPlease input the sect number(0-15):");	
		scanf("%u", &select_sect);
		if(select_sect < 0 || select_sect > 15){
			printf("\nYou select unused sect: %d", select_sect);	
			continue;
		}
		xfer->sect_num = (uint8_t)select_sect;
		printf("You select sect: %u\n", xfer->sect_num); 
		while(1)
		{
			printf("\nPlease select function: \n");	
			for(i=0; ioc_type_menu[i].function!=0; i++)
				printf("%d : %s", i, ioc_type_menu[i].msg);
			printf("\nPlease input the select number:");	
			scanf("%d", &select_index) ;	
			printf("You select %d function: %s\n", select_index, ioc_type_menu[select_index].msg);	
			if(0 < select_index && select_index < i)
			{
				(*ioc_type_menu[select_index].function)(fd, xfer);
				sleep(1);
			}
			else{
				break;
			}
		}	
	}
	printf("Do not goto here\n");	
	return i;
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd = -1;
	int i = 0;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");
	/***** RC522 Test 
	****** the key a have all permission ,
	****** the key b have non permission ,
	****** when ctrl area using default value .
	*****/
	struct rc522_ioc_transfer transfer={
		.ioc_type = RC522_IOC_REQ_CARD,
		.blk_num = RC522_BLK_0,
	};
	transfer.key_type = RC522_KEY_A; //RC522_KEY_B
	for(i = 0; i<=5; i++){
		transfer.keya_buf[i] = 0xFF;
	} 
	for(i = 0; i<=5; i++){
		transfer.keyb_buf[i] = 0xFF;
	} 

	memset(transfer.txrx_buf, 0x00, CARD_BLK_SIZE + CARD_PLUS_SIZE);
	i = 10;
	while(i){
		if(rc522_operate_card(fd, &transfer)){
			printf("Please put your card on reader:");
			i--;
			sleep(1);
			continue;
		}
		break;
	}
	if(i == 0){
		printf(" Failed to find your card on reader:\n");
		close(fd);
		return 0;
	}
	
	printf("Request Card: \n");
	printf("type: 0x%02x, 0x%02x, %s \n", 
		transfer.txrx_buf[0],transfer.txrx_buf[1], 
		transfer.txrx_buf[0] == 0x04?"S50 card":
		(transfer.txrx_buf[0] == 0x02?"S70 card":"Other card"));
	printf("id: 0x%02x, 0x%02x, 0x%02x, 0x%02x, idcsum: 0x%02x\n",
			transfer.txrx_buf[2], transfer.txrx_buf[3], 
			transfer.txrx_buf[4], transfer.txrx_buf[5],
			transfer.txrx_buf[6]);
	printf("id_buf: 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
			transfer.id_buf[0], transfer.id_buf[1], 
			transfer.id_buf[2], transfer.id_buf[3]);
	printf("cap: 0x%02x Kbits, msg: 0x%02x, 0x%02x\n",
		transfer.txrx_buf[7], transfer.txrx_buf[8],transfer.txrx_buf[9]);
	printf("Request Card: end\n\n");
	if(transfer.txrx_buf[0] == 0x00){
		printf("Failed to request used card: %s\n","Unused card");
		return 0;
	}
	rc522_menu_card(fd, &transfer);
	close(fd);

	return ret;
}
