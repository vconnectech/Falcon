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

#define TID_C_I2C_EX 5
#define TID_Z_I2C_TEMP_0 6 
#define TID_Z_I2C_TEMP_1 7

#define TID_Z_PMIC_1 8
#define TID_Z_PMIC_2 9


#define TID_R_TEST 100
#define TID_Z_TEST 200
#define TID_A_TEST 300

#define BADKEY   -1
#define NKEYS (sizeof(lookuptable)/sizeof(struct symstruct))



static struct symstruct lookuptable[] = {
	{ "TID_R_I2C_SC",TID_R_I2C_SC},
	{ "TID_Z_I2C_SC",TID_Z_I2C_SC},
	{ "TID_A_I2C_SC",TID_A_I2C_SC},
	{ "TID_Z_I2C_TEMP_0",TID_Z_I2C_TEMP_0},
	{ "TID_Z_I2C_TEMP_2",TID_Z_I2C_TEMP_1},
	{ "TID_Z_PMIC_1",TID_Z_PMIC_1},
	{ "TID_Z_PMIC_2",TID_Z_PMIC_2},
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
#define BUF_SIZE 1024	
	FILE *fp;
	int bytes_sent = 0;
	char result_buf[BUF_SIZE];
	switch (keyfromstring(buff)) {

		case TID_Z_I2C_SC:
			fp = popen("i2cdetect -y 1","r");
			if(fp != NULL) {
				size_t byte_count = fread(result_buf, 1, BUF_SIZE - 1, fp);
				result_buf[byte_count] = 0;
				bytes_sent = write(fd, result_buf, byte_count);
				pclose(fp);
			}
			break;
		case TID_C_I2C_EX:		
			fp = popen("i2cget -y 1 0x42 ","r");
			if(fp != NULL) {
				size_t byte_count = fread(result_buf, 1, BUF_SIZE - 1, fp);
				result_buf[byte_count] = 0;
				bytes_sent = write(fd, result_buf, byte_count);
				pclose(fp);
			}
			break;
		case TID_Z_PMIC_1:		
			fp = popen("i2cget -y 1 0x1A 0x00","r");
			if(fp != NULL) {
				size_t byte_count = fread(result_buf, 1, BUF_SIZE - 1, fp);
				result_buf[byte_count] = 0;
				bytes_sent = write(fd, result_buf, byte_count);
				pclose(fp);
			}
			break;
		case TID_Z_PMIC_2:		
			fp = popen("i2cget -y 1 0x1D 0x00","r");
			if(fp != NULL) {
				size_t byte_count = fread(result_buf, 1, BUF_SIZE - 1, fp);
				result_buf[byte_count] = 0;
				bytes_sent = write(fd, result_buf, byte_count);
				pclose(fp);
			}
			break;
		case TID_Z_I2C_TEMP_1:		
			fp = popen("i2cget -y 1 0x1D 0x00 b","r");
			if(fp != NULL) {
				size_t byte_count = fread(result_buf, 1, BUF_SIZE - 1, fp);
				result_buf[byte_count] = 0;
				bytes_sent = write(fd, result_buf, byte_count);
				pclose(fp);
			}
			break;
		case TID_Z_I2C_TEMP_0:		
			fp = popen("i2cget -y 0 0x1D 0x00 b","r");
			if(fp != NULL) {
				size_t byte_count = fread(result_buf, 1, BUF_SIZE - 1, fp);
				result_buf[byte_count] = 0;
				bytes_sent = write(fd, result_buf, byte_count);
				pclose(fp);
			}
			break;
		
		case TID_A_I2C_SC:
			break;
		case TID_Z_TEST:
			bytes_sent = write(fd,"HELLO FROM ZYNC",strlen("HELLO FROM ZYNC"));
			break;
		case TID_A_TEST:
			break;
		case TID_R_TEST:
			bytes_sent = write(fd,"HELLO FROM PI",strlen("HELLO FROM PI"));
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
	char rx_buff[100];
	int len = 100;
	int bytes_sent = 0;
	int bytes_recv = 0;
	int bytes_to_read = 0;
	struct timeval timeout;

	open_fd = open_port("/dev/ttyPS0");
	memset(rx_buff,0x0,sizeof(rx_buff));
	memset(tx_buff,0x0,sizeof(tx_buff));

	while(1) {

		FD_ZERO(&set);
		FD_SET(open_fd, &set);
		timeout.tv_sec = 1;
		timeout.tv_usec = 100000;

		sel_rv = select(1, &set, NULL, NULL, &timeout);

		bytes_to_read = 0;
		ioctl(open_fd, FIONREAD, &bytes_to_read);
		if(bytes_to_read > 0) {
			memset(rx_buff,0x0,sizeof(rx_buff));
			bytes_recv = read(open_fd, rx_buff, bytes_to_read);
			if(bytes_recv > 0) {
				bytes_sent = Run_cmd(rx_buff, open_fd);
			}
		}

	}
	return 0;
}

