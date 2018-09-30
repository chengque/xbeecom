#include "ftditools.h"
#include "string.h"

#define RECBUFSIZE 256

int openFtdi;
FT_HANDLE ftHandle;

short getlength(short s,short e)
{
    short data_length=0;
    if(s<=e)
    {
        data_length=e-s;
    }
    else
    {
        data_length=e+RECBUFSIZE-s;
    }
    return data_length;
}

short incindex(short num,short inc)
{
    num=num+inc;
    if(num>RECBUFSIZE-1)
    {
        num=num-RECBUFSIZE;
    }
    return num;
}

short decindex(short num,short dec)
{
    num=num-dec;
    if(num<0)
    {
        num=num+RECBUFSIZE;
    }
    return num;
}


short open_ftdi(int BaudRate, char *DeviceName, int SendTimeout, int ReadTimeout)
{
	unsigned long ftStatus;
    
    ftStatus = FT_OpenEx(DeviceName,FT_OPEN_BY_DESCRIPTION,&ftHandle);
    ftStatus = FT_SetBaudRate(ftHandle, (unsigned long)BaudRate);
	ftStatus = FT_SetDataCharacteristics(ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
    ftStatus = FT_SetTimeouts(ftHandle, ReadTimeout, SendTimeout);
	ftStatus = FT_SetFlowControl(ftHandle, FT_FLOW_NONE, 0, 0);
    ftStatus = FT_SetUSBParameters (ftHandle,64, 64);
    ftStatus = FT_SetLatencyTimer (ftHandle, 4);
    //ftStatus = FT_SetLatencyTimer (ftHandle, 2);
	printf("opening device... \n");
	if (ftStatus == FT_OK)
    {
        printf("successful\n");
		return(1);
    }
          // FT_Open OK, use ftHandle to access device } 
    else 
    {
		printf("failed\n");
       
        return(-1); // FT_Open failed }
    }
}

void close_ftdi()
{
    if (openFtdi == 1)
    {
        FT_Close(ftHandle);
        openFtdi=0;
		printf("closed device\n");
    }
}


short read_ftdi (short * data_out)
{
	unsigned long ftStatus;
    int size = 0;
	static char data_in[RECBUFSIZE];
    char data_buf[RECBUFSIZE];
    char recbuf[32];
	short sync = 0;
	short i=0, j=0, k=0;
	int readcount = 0;
    short intstate=0;
    
	int read_tmp = 0;
	char descriptor = 0;
	unsigned short* crc = NULL;
	short* data_ptr = NULL;
	unsigned long RxBytes, BytesReceived;
	static short data_out_store[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static short status_store[] = {0,0};
    //short send_size = 22;
	short send_size = 14;
    short data_new_flag = 1;
    short data_length=0;
    
    static char data_rec_store[256];
    static int data_rec_count=0;
    static short data_sync=0;
    static short starti=0;
    static short endi=0;
    
    size = send_size*2+3+3;
    
    data_length=getlength(starti,endi);
    
    FT_GetQueueStatus(ftHandle,(LPDWORD)&RxBytes);
    if(RxBytes>(RECBUFSIZE-1))
    {
        RxBytes=RECBUFSIZE-1;
    }
    ftStatus = FT_Read(ftHandle,data_buf,(DWORD)RxBytes,(LPDWORD)&BytesReceived);
    if((ftStatus != FT_OK))
    {
        data_new_flag = 0;
    }
    else
    {
        if(data_length+BytesReceived>RECBUFSIZE-1)
        {
            endi=0;starti=0;
        }
        j=0;i=endi;
        for(k=0;k<BytesReceived;k++)
        {
            if(i>RECBUFSIZE-1)
            {
                i=i-RECBUFSIZE;
            }
            data_in[i]=data_buf[j];
            j++;
            i++;
        }
        endi=i;
        data_length=getlength(starti,endi);
        if(data_length<size)
        {
            data_new_flag = 0;
        }
        else
        {
            data_new_flag = 1;
        }
    }	
    
    i=starti;
   
	while ((sync!=3) && (data_new_flag == 1))
	{
		if (sync==0)
		{
		    if (data_in[i]=='>')
            {
                sync=1;
                starti=i;
            }
			else
            {
				sync=0;
            }
		}
		else if (sync==1)
		{
			if (data_in[i]=='*')
            {
				sync=2;
            }
			else
            {
				sync=0;
                incindex(starti,1);
            }
		}
		else if (sync==2)
		{
			if (data_in[i]=='>')
            {
				sync=3;     
            }
			else
            {
				sync=0;
                incindex(starti,1);
            }
		}
        
        data_length=getlength(starti,endi);
        if(sync==3&&data_length<size)
        {
            data_new_flag = 0;
        }
        i=incindex(i,1);
        if(i==endi)
        {
            starti=i;
            data_new_flag=0;
        }
	}
    
	if (data_new_flag == 1)
	{
        j=starti;
        for(k=0;k<size;k++)
        {
            recbuf[k]=data_in[j];
            j++;
            if(j>RECBUFSIZE-1)
            {
                j=j-RECBUFSIZE;
            }
        }
		i=3;
		descriptor = recbuf[i];
		i=i+1;
		data_ptr = (short*) &(recbuf[i]);
        i = i+send_size*2;
		crc = (unsigned short*)&(recbuf[i]);

		if (crc16(data_ptr, send_size*2) == *crc)
		{
            memcpy(data_out, data_ptr, sizeof(data_out_store));
		}
        starti=incindex(starti,size);
	}
	

    
//     FT_GetQueueStatus(ftHandle,(LPDWORD)&RxBytes);
//     
//     if (RxBytes>PURGE_BUFFER)
//     {
//         ftStatus = FT_Purge(ftHandle, FT_PURGE_RX);
//     }
    
    return data_new_flag;
}

short send_ftdi(const short * control_data)
{
	char send_data[TXCHANNELS*2+6];	
	unsigned short crc = 0;
	short j=0, send_switch=1;
    unsigned int BytesSent=0;
    if (send_switch)
    {
        send_data[0] = '>';
        send_data[1] = '*';
        send_data[2] = '>';
        send_data[3] = 'c';
        memcpy(&send_data[4], control_data, TXCHANNELS*2);
        crc = crc16(&send_data[4], TXCHANNELS*2);
        memcpy(&send_data[TXCHANNELS*2+4], &crc, 2);

        FT_Write(ftHandle, send_data, TXCHANNELS*2+6, &BytesSent);
        return BytesSent;
    }
    else
    {
        return 0;
    }
}

unsigned short crc_update (unsigned short crc, unsigned char data)
{
    data ^= (crc & 0xff);
    data ^= data << 4;

    return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
         ^ ((unsigned short )data << 3));
}

 unsigned short crc16(void* data, unsigned short cnt)
{
    unsigned short crc=0xff;
    unsigned char * ptr=(unsigned char *) data;
    int i;

    for (i=0;i<cnt;i++)
    {
        crc=crc_update(crc,*ptr);
        ptr++;
    }
    return crc;

}
 


    

 