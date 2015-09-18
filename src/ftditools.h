#ifndef FTDITOOLSREC_H
#define FTDITOOLSREC_H

#include <stdio.h>
#include "ftd2xx.h"

#define PURGE_BUFFER 255

#define TXCHANNELS 4
#define RXCHANNELS 20

extern int openFtdi;
extern FT_HANDLE ftHandle;

short open_ftdi(int, char*, int, int);
void close_ftdi();
short send_ftdi(const short *);
short read_ftdi (short *, double*);
unsigned short crc_update (unsigned short, unsigned char);
unsigned short crc16(void*, unsigned short);
#endif