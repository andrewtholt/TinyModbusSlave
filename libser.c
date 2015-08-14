/*
 *  libser.h
 *  
 *
 *  Created by Andrew Holt on 03/06/2010.
 *
 */
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <errno.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <strings.h>
#include <sys/types.h>
#include <sys/stat.h>

void eFlushSerialPort(int );

char *strsave(char *s) {
    char           *p;

    if ((p = (char *) malloc(strlen(s) + 1)) != NULL)
        strcpy(p, s);
    return (p);
}


//-----------------------------------------------------------------------------
// Name: openSerialPort
// 
// Description:	Opens the serial port for comminucation 
//
// Returns: 
//		fd:	The file descriptor for the serial port
//
//-----------------------------------------------------------------------------
int openSerialPort(char *acSerialPortName, speed_t tBaudRate) {	
//	int iSerialPort;
    int fd=-1;

    if( tBaudRate == 0) {
//        tBaudRate = B115200;
        tBaudRate = B9600;
    }

	printf("serial: Opening Serial Port (%s) at %d\n", acSerialPortName,(int)tBaudRate ); 
    fd = open( acSerialPortName , O_NOCTTY | O_RDWR | O_NONBLOCK|O_EXCL);

    if( fd < 0) {
		printf("serial: Can't open serial port - %s\n", acSerialPortName);
        perror("openSerialPort");
		exit (0);
	}

	printf("serial: Opening Serial Port (%d)\n", fd); 

	//-------------------------------------
	// Then initialise UART 
	//-------------------------------------
	iConfigureSerialPort(fd, tBaudRate);

	eFlushSerialPort(fd);
	return fd ;
}

//-----------------------------------------------------------------------------
// Name: 	closeSerialPort()
//
// Description: This function waits for input on socket and the serial port.
//	When something arrives, it gets processed. Then the function returns. 
//	The function should get called again to wait for the next message.
//
// Parameters:
//		iSerialPort: The serial port file descriptor
//
// Returns:
//		
//-----------------------------------------------------------------------------
void closeSerialPort(int iSerialPort) {

	eFlushSerialPort(iSerialPort);
	close(iSerialPort);

}


//-----------------------------------------------------------------------------
// Name: init_serial_port()
//
// Description: This function initialises the serial port for communication with
// 	The Jennic.
//
// Parameters:
//	Fd: The file descriptor for the serial port we are using.
//
// Returns:
//		0
//-----------------------------------------------------------------------------
int iConfigureSerialPort(int iSerialPort, speed_t tSpeed)
{
	int iBits;
	struct termios tty;

	tcflush(iSerialPort, TCIOFLUSH);
	tcgetattr(iSerialPort, &tty);

	iBits = '8';
	cfsetospeed(&tty, tSpeed);
    cfsetispeed(&tty, tSpeed);
 
    printf("Defaults changed, 9600, 8 bit EVEN parity\n");

	switch (iBits) {
		case '5':
		  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS5;
		  break;

		case '6':
		  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS6;
		  break;
		
		case '7':
		  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS7;
		  break;
		
		case '8':
		default:
		  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
		  break;
	}
	
	tty.c_iflag |= IGNBRK;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON|IXOFF|IXANY);

//	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	
	cfmakeraw(&tty);
	
	tcsetattr(iSerialPort, TCSANOW, &tty);

	return 0;
}

void eFlushSerialPort(int iSerialPort) {

	int iBytesReceived;
	char cByte;

	do {
		iBytesReceived = read(iSerialPort, &cByte, 1);
	}
	while (iBytesReceived > 0);
}
