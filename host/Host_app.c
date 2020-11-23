/*---------------------------------------------------------------------------
 *
 * serial_uart_host.c
 *
 *---------------------------------------------------------------------------
 */


#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>

struct symstruct
{
	char *key;
	int val;
};
struct termios options,options_old;


#define TID_R_I2C_SC 1
#define TID_Z_I2C_SC 2
#define TID_A_I2C_SC 3
#define TID_Z_I2C_TEMP 4


#define TID_R_TEST 100
#define TID_Z_TEST 200
#define TID_A_TEST 300

#define BADKEY   -1
#define NKEYS (sizeof(lookuptable)/sizeof(struct symstruct))



static struct symstruct lookuptable[] = {
	{ "TID_R_I2C_SC",TID_R_I2C_SC},
	{ "TID_Z_I2C_SC",TID_Z_I2C_SC},
	{ "TID_A_I2C_SC",TID_A_I2C_SC},
	{ "TID_Z_I2C_TEMP",TID_Z_I2C_TEMP},
	{ "TID_Z_TEST",TID_Z_TEST},
	{ "TID_R_TEST",TID_R_TEST},
	{ "TID_A_TEST",TID_A_TEST},
	{ "B2", BADKEY }
};

speed_t set_baud_rate() {
	return B115200;
}



int open_port(char *device)
{
	int fd;
	speed_t baud_spd = 0;

	fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK );
	if (fd == -1) {
		perror("open_port: Unable to open device \n - ");
	}

	tcgetattr(fd, &options_old);
	(void)memcpy(&options,&options_old, sizeof(options));

	memset(options.c_cc, 0, sizeof(options.c_cc));
	options.c_cc[VMIN] = 1;
	options.c_cflag &= ~(PARENB | PARODD | CRTSCTS);
	options.c_iflag &= ~(PARMRK | INPCK);
	options.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD);
	options.c_cflag |= ( CREAD | CLOCAL | CS8 );
	options.c_iflag = options.c_oflag = options.c_lflag = (tcflag_t) 0;
	baud_spd = set_baud_rate();
	cfsetispeed(&options, baud_spd);
	cfsetospeed(&options, baud_spd);

	if (tcsetattr( fd, TCSANOW, &options ) == -1 )
		printf ( "Baudrate set error on fd = %i\n", fd);

	(void)tcflush(fd, TCIOFLUSH); 
	return (fd);

}


int keyfromstring(char *key)
{
	int i;
	for (i=0; i < NKEYS; i++) {
		struct symstruct *sym = &lookuptable[i];
		if (strcmp(sym->key, key) == 0)
			return sym->val;
	}
	return BADKEY;
}


int Run_cmd(char *buff, int fd)
{
	int bytes_sent = 0;
	int len = strlen(buff);
	switch (keyfromstring(buff)) {
		case TID_R_I2C_SC:
		case TID_Z_I2C_SC:
		case TID_A_I2C_SC:
		case TID_Z_TEST:
		case TID_A_TEST:
		case TID_R_TEST:
			bytes_sent = write(fd,buff,len);
			break;
		case TID_Z_I2C_TEMP:
			break;
		case BADKEY:
			break;

	}

	usleep(100000);
	tcdrain(fd);
	return bytes_sent;
}

int main(int argc,char *argv[])
{

	int open_fd;
	int sel_rv;
	fd_set set;
	char tx_buff[100];
	char rx_buff[1024];
	int len = 100;
	int bytes_sent = 0;
	int bytes_recv = 0;
	int bytes_to_read = 0;
	struct timeval timeout;

	if(argc < 2)
	{
		printf("ERROR\n");
		return 0;
	}

	open_fd = open_port("/dev/ttyUSB0");

	memset(tx_buff,0x0,len);
	memcpy(tx_buff,argv[1],strlen(argv[1]));
 
	bytes_sent = Run_cmd(tx_buff, open_fd);

	if(bytes_sent > 0) {
		while(1) {
			FD_ZERO(&set);
        		FD_SET(open_fd, &set);
        		timeout.tv_sec = 2;
        		timeout.tv_usec = 100000;

        		sel_rv = select(1, &set, NULL, NULL, &timeout);

			ioctl(open_fd, FIONREAD, &bytes_to_read);
			if(bytes_to_read > 0) {
				memset(rx_buff,0x0,len);
				bytes_recv = read(open_fd, rx_buff, bytes_to_read);
				printf("%s",rx_buff);
				break;
			}
		}
	} else {
		printf("Wrong command Given \n");
	}
	

	close(open_fd);
	return 0;
}





