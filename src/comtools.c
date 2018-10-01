/*串口设备无论是在工控领域，还是在嵌入式设备领域，应用都非常广泛。而串口编程也就显得必不可少。
偶然的一次机会，需要使用串口，而且操作系统还要求是Linux，因此，趁着这次机会，综合别人的代码，
进行了一次整理和封装。具体的封装格式为C代码，这样做是为了很好的移植性，使它可以在C和C++环境下，
都可以编译和使用。代码的头文件如下： */

/////////////////////////////////////////////////////////////////////////////// 
//filename:stty.h 
#ifndef __STTY_H__ 
#define __STTY_H__ 

//包含头文件
#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <termios.h> 
#include <errno.h> 
#include <pthread.h> 
// 
// 串口设备信息结构 
typedef struct tty_info_t 
{ 
	int fd; // 串口设备ID 
	pthread_mutex_t mt; // 线程同步互斥对象 
	char name[24]; // 串口设备名称，例："/dev/ttyS0" 
	struct termios ntm; // 新的串口设备选项 
	struct termios otm; // 旧的串口设备选项 
} TTY_INFO; 
// 
// 串口操作函数 
TTY_INFO *readyTTY(int id); 
int setTTYSpeed(TTY_INFO *ptty, int speed); 
int setTTYParity(TTY_INFO *ptty,int databits,int parity,int stopbits); 
int cleanTTY(TTY_INFO *ptty); 
int sendnTTY(TTY_INFO *ptty,char *pbuf,int size); 
int recvnTTY(TTY_INFO *ptty,char *pbuf,int size); 
int lockTTY(TTY_INFO *ptty); 
int unlockTTY(TTY_INFO *ptty); 

#endif 


/*从头文件中的函数定义不难看出，函数的功能，使用过程如下： 
（1） 打开串口设备，调用函数setTTYSpeed（）； 
（2） 设置串口读写的波特率，调用函数setTTYSpeed（）； 
（3） 设置串口的属性，包括停止位、校验位、数据位等，调用函数setTTYParity（）； 
（4） 向串口写入数据，调用函数sendnTTY（）； 
（5） 从串口读出数据，调用函数recvnTTY（）； 
（6） 操作完成后，需要调用函数cleanTTY（）来释放申请的串口信息接口； 
其中，lockTTY（）和unlockTTY（）是为了能够在多线程中使用。在读写操作的前后，需要锁定和释放串口资源。 
具体的使用方法，在代码实现的原文件中，main（）函数中进行了演示。下面就是源代码文件： */

//////////////////////////////////////////////////////////////////////////////// 
//stty.c 

#include <stdio.h> 
#include <sys/ioctl.h> 
#include "stty.h" 

/////////////////////////////////////////////////////////////////////////////// 
// 初始化串口设备并进行原有设置的保存 
TTY_INFO *readyTTY(int id) 
{ 
	TTY_INFO *ptty; 
	
	ptty = (TTY_INFO *)malloc(sizeof(TTY_INFO)); 
	if(ptty == NULL) 
		return NULL; 
	memset(ptty,0,sizeof(TTY_INFO)); 
	pthread_mutex_init(&ptty->mt,NULL); 
	sprintf(ptty->name,"/dev/ttyS%d",id); 
	// 
	// 打开并且设置串口 
	ptty->fd = open(ptty->name, O_RDWR | O_NOCTTY |O_NDELAY); 
	if (ptty->fd <0) 
	{ 
		free(ptty); 
		return NULL; 
	} 
	// 
	// 取得并且保存原来的设置 
	tcgetattr(ptty->fd,&ptty->otm); 
	return ptty; 
} 

/////////////////////////////////////////////////////////////////////////////// 
// 清理串口设备资源 
int cleanTTY(TTY_INFO *ptty) 
{ 
	// 
	// 关闭打开的串口设备 
	if(ptty->fd>0) 
	{ 
		tcsetattr(ptty->fd,TCSANOW,&ptty->otm); 
		close(ptty->fd); 
		ptty->fd = -1; 
		free(ptty); 
		ptty = NULL; 
	} 
	
	return 0; 
} 


/////////////////////////////////////////////////////////////////////////////// 
// 设置串口通信速率 
// ptty 参数类型(TTY_INFO *),已经初始化的串口设备信息结构指针 
// speed 参数类型(int),用来设置串口的波特率 
// return 返回值类型(int),函数执行成功返回零值，否则返回大于零的值 
/////////////////////////////////////////////////////////////////////////////// 
int setTTYSpeed(TTY_INFO *ptty, int speed) 
{ 
	int i; 
	// 
	// 进行新的串口设置,数据位为8位 
	bzero(&ptty->ntm, sizeof(ptty->ntm)); 
	tcgetattr(ptty->fd,&ptty->ntm); 
	ptty->ntm.c_cflag = /*CS8 |*/ CLOCAL | CREAD; 
	
	switch(speed) 
	{ 
	case 300: 
		ptty->ntm.c_cflag |= B300; 
		break; 
	case 1200: 
		ptty->ntm.c_cflag |= B1200; 
		break; 
	case 2400: 
		ptty->ntm.c_cflag |= B2400; 
		break; 
	case 4800: 
		ptty->ntm.c_cflag |= B4800; 
		break; 
	case 9600: 
		ptty->ntm.c_cflag |= B9600; 
		break; 
	case 19200: 
		ptty->ntm.c_cflag |= B19200; 
		break; 
	case 38400: 
		ptty->ntm.c_cflag |= B38400; 
		break; 
	case 115200: 
		ptty->ntm.c_cflag |= B115200; 
		break; 
	} 
	ptty->ntm.c_iflag = IGNPAR; 
	ptty->ntm.c_oflag = 0; 
	// 
	// 
	tcflush(ptty->fd, TCIFLUSH); 
	tcsetattr(ptty->fd,TCSANOW,&ptty->ntm); 
	// 
	// 
	return 0; 
} 
// 设置串口数据位，停止位和效验位 
// ptty 参数类型(TTY_INFO *),已经初始化的串口设备信息结构指针 
// databits 参数类型(int), 数据位,取值为7或者8 
// stopbits 参数类型(int),停止位,取值为1或者2 
// parity 参数类型(int),效验类型 取值为N,E,O,,S 
// return 返回值类型(int),函数执行成功返回零值，否则返回大于零的值 
/////////////////////////////////////////////////////////////////////////////// 
int setTTYParity(TTY_INFO *ptty,int databits,int parity,int stopbits) 
{ 
	// 
	// 取得串口设置 
	if( tcgetattr(ptty->fd,&ptty->ntm) != 0) 
	{ 
		printf("SetupSerial [%s]\n",ptty->name); 
		return 1; 
	} 
	
	bzero(&ptty->ntm, sizeof(ptty->ntm)); 
	ptty->ntm.c_cflag = CS8 | CLOCAL | CREAD; 
	ptty->ntm.c_iflag = IGNPAR; 
	ptty->ntm.c_oflag = 0; 
	// 
	// 设置串口的各种参数 
	ptty->ntm.c_cflag &= ~CSIZE; 
	switch (databits) 
	{ //设置数据位数 
	case 7: 
		ptty->ntm.c_cflag |= CS7; 
		break; 
	case 8: 
		ptty->ntm.c_cflag |= CS8; 
		break; 
	default: 
		printf("Unsupported data size\n"); 
		return 5; 
	} 
	// 
	// 
	switch (parity) 
	{ // 设置奇偶校验位数 
	case n: 
	case N: 
		ptty->ntm.c_cflag &= ~PARENB; /* Clear parity enable */ 
		ptty->ntm.c_iflag &= ~INPCK; /* Enable parity checking */ 
		break; 
	case o: 
	case O: 
		ptty->ntm.c_cflag |= (PARODD|PARENB); /* 设置为奇效验*/ 
		ptty->ntm.c_iflag |= INPCK; /* Disnable parity checking */ 
		break; 
	case e: 
	case E: 
		ptty->ntm.c_cflag |= PARENB; /* Enable parity */ 
		ptty->ntm.c_cflag &= ~PARODD; /* 转换为偶效验*/ 
		ptty->ntm.c_iflag |= INPCK; /* Disnable parity checking */ 
		break; 
	case S: 
	case s: /*as no parity*/ 
		ptty->ntm.c_cflag &= ~PARENB; 
		ptty->ntm.c_cflag &= ~CSTOPB; 
		break; 
	default: 
		printf("Unsupported parity\n"); 
		return 2; 
	} 
	// 
	// 设置停止位 
	switch (stopbits) 
	{ 
	case 1: 
		ptty->ntm.c_cflag &= ~CSTOPB; 
		break; 
	case 2: 
		ptty->ntm.c_cflag |= CSTOPB; 
		break; 
	default: 
		printf("Unsupported stop bits\n"); 
		return 3; 
	} 
	// 
	// 
	ptty->ntm.c_lflag = 0; 
	ptty->ntm.c_cc[VTIME] = 0; // inter-character timer unused 
	ptty->ntm.c_cc[VMIN] = 1; // blocking read until 1 chars received 
	tcflush(ptty->fd, TCIFLUSH); 
	if (tcsetattr(ptty->fd,TCSANOW,&ptty->ntm) != 0) 
	{ 
		printf("SetupSerial \n"); 
		return 4; 
	} 
	
	return 0; 
} 

int recvnTTY(TTY_INFO *ptty,char *pbuf,int size) 
{ 
	int ret,left,bytes; 
	
	left = size; 
	
	while(left>0) 
	{ 
		ret = 0; 
		bytes = 0; 
		
		pthread_mutex_lock(&ptty->mt); 
		ioctl(ptty->fd, FIONREAD, &bytes); 
		if(bytes>0) 
		{ 
			ret = read(ptty->fd,pbuf,left); 
		} 
		pthread_mutex_unlock(&ptty->mt); 
		if(ret >0) 
		{ 
			left -= ret; 
			pbuf += ret; 
		} 
		usleep(100); 
	} 
	
	return size - left; 
} 

int sendnTTY(TTY_INFO *ptty,char *pbuf,int size) 
{ 
	int ret,nleft; 
	char *ptmp; 
	
	ret = 0; 
	nleft = size; 
	ptmp = pbuf; 
	
	while(nleft>0) 
	{ 
		pthread_mutex_lock(&ptty->mt); 
		ret = write(ptty->fd,ptmp,nleft); 
		pthread_mutex_unlock(&ptty->mt); 
		
		if(ret >0) 
		{ 
			nleft -= ret; 
			ptmp += ret; 
		} 
		//usleep(100); 
	} 
	
	return size - nleft; 
} 

int lockTTY(TTY_INFO *ptty) 
{ 
	if(ptty->fd < 0) 
	{ 
		return 1; 
	} 
	
	return flock(ptty->fd,LOCK_EX); 
} 
int unlockTTY(TTY_INFO *ptty) 
{ 
	if(ptty->fd < 0) 
	{ 
		return 1; 
	} 
	
	return flock(ptty->fd,LOCK_UN); 
} 


#ifdef LEAF_TTY_TEST 
/////////////////////////////////////////////////////////////////////////////// 
// 接口测试 
int main(int argc,char **argv) 
{ 
	TTY_INFO *ptty; 
	int nbyte,idx; 
	unsigned char cc[16]; 
	
	ptty = readyTTY(0); 
	if(ptty == NULL) 
	{ 
		printf("readyTTY(0) error\n"); 
		return 1; 
	} 
	// 
	// 
	lockTTY(ptty); 
	if(setTTYSpeed(ptty,9600)>0) 
	{ 
		printf("setTTYSpeed() error\n"); 
		return -1; 
	} 
	if(setTTYParity(ptty,8,N,1)>0) 
	{ 
		printf("setTTYParity() error\n"); 
		return -1; 
	} 
	// 
	idx = 0; 
	while(1) 
	{ 
		cc[0] = 0xFA; 
		sendnTTY(ptty,&cc[0],1); 
		nbyte = recvnTTY(ptty,cc,1); 
		printf("%d:%02X\n",idx++,cc[0]); 
	} 
	
	cleanTTY(ptty); 
	
} 
#endif 