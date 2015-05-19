#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <inttypes.h>

// Modbus slave definitions
#define IAM 1   // RTU id
#define BASE 0  // Base address 
#define DATA_SIZE 4 // 32 bits, 4 bytes, or 2 16 bit words of simulated io.

#define OK 0x00
#define ILL_FUNC 0x01
#define ILL_ADDRESS 0x02
#define ILL_VALUE 0x03

unsigned char   uchCRCHi;
unsigned char   uchCRCLo;
unsigned char data[DATA_SIZE]; 

// static unsigned char bitMask[] = {
static uint16_t bitMask[] = {
    0x0001,
    0x0002,
    0x0004,
    0x0008,
    0x0010,
    0x0020,
    0x0040,
    0x0080,
    0x0100,
    0x0200,
    0x0400,
    0x0800,
    0x1000,
    0x2000,
    0x4000,
    0x8000
    
};

static unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
    0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
    0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00,
    0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
    0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
    0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00,
    0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
/* Table of CRC values for low–order byte */
static char     auchCRCLo[] = {0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA,
    0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17,
    0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33,
    0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9,
    0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24,
    0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0,
    0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8,
    0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD,
    0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1,
    0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B,
    0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E,
    0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82,
    0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

void athCalcCRC(unsigned char *puchMsg,unsigned int usDataLen ) {
    /* The function returns the CRC as a unsigned short type  */
    /* *puchMsg message to calculate CRC upon  */
    /* usDataLen  quantity of bytes in message   */
    
    unsigned char *ptr;
    unsigned int len;
    
    ptr=puchMsg;
    len=usDataLen;
    
    unsigned char   uchCRCHi = 0xFF;    /* high byte of CRC
                                         * initialized   */
    unsigned char   uchCRCLo = 0xFF;    /* low byte of CRC
                                         * initialized   */
    unsigned char       uIndex; /* will index into CRC lookup table   */
    
    
    while (usDataLen--) /* pass through message buffer   */
    {
        uIndex = uchCRCLo ^ *puchMsg++; /* calculate the CRC   */
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
        uchCRCHi = auchCRCLo[uIndex];
    }
    
    *(ptr+len)=uchCRCLo;
    *(ptr+len+1)=uchCRCHi;
}

void onTheFlyCalcCrc(unsigned char c) {
    unsigned char uIndex;
    
    uIndex = uchCRCLo ^ c;
    uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
    uchCRCHi = auchCRCLo[uIndex];
    
}

/*
    This needs modifying.
 
    The time to wait needs to be passed as a parameter.
    The returned value needs to be and int.  The bottom byte will contain the char,
    the next byte up is non zero then tger was an error. e.g.
 
    0x0030 - No error ASCII 0
    0x0100 - Exceeded time.
 
    In addition if there is a time out there is no need to add to the CRC.
 
    When used on an MCU a hardware timer would be used and checked.
 
    The response on a time out will be to abandon the current packet, and wait for the t3.5
    inter packet gap.
 */

unsigned char getByte(int tty) {
    unsigned char in[2];
    int cnt=-1;
    
    while(cnt <=0 ) {
        usleep(1);
        cnt=read(tty,in,1);
    }
    onTheFlyCalcCrc( in[0] );
    
    return( in[0] );
}

int readRegisters(int tty, unsigned char rtu, unsigned char func) {
    unsigned tmp=0;
    unsigned int dataCount=0;
    unsigned
    char byteCount;
    int i=0;
    
    unsigned char addressHi=0;
    unsigned char addressLo=0;
    unsigned int address;
    int exception=OK;

    int rc=OK;
    
    uchCRCHi = 0xff;
    uchCRCLo = 0xff;

    onTheFlyCalcCrc( rtu );
    onTheFlyCalcCrc( func );
    printf("entering readRegisters\n");
    
    
    addressHi=getByte(tty);
    address= ( addressHi << 8) & 0xff00;
    
    addressLo=getByte(tty);
    
    address |= ( addressLo & 0xff);
    printf("Address %04x\n", address);
    
    // Get count hi
    //
    tmp=getByte(tty);
    dataCount = (tmp << 8) & 0xff00;
    
    // get count lo
    //
    tmp = getByte(tty);
    
    dataCount |= (tmp & 0xff);
    printf("Count   %04x\n", dataCount);
    
    //
    // Get CRC
    //
    tmp = getByte(tty);
    printf("%02x\n", tmp);
    
    tmp = getByte(tty);
    printf("%02x\n", tmp);
    
    printf("%03d:0x%02x\n", uchCRCLo, uchCRCLo);
    printf("%03d:0x%0x\n", uchCRCHi, uchCRCHi);

    if ((uchCRCHi != 0) & (uchCRCLo != 0)) {
        rc= ILL_FUNC;
    } else {
        printf("Return Holding Registers\n");
        uchCRCHi = 0xff;
        uchCRCLo = 0xff;
        
        tmp=write(tty,&rtu,1);
        onTheFlyCalcCrc( rtu );
        
        tmp=write(tty,&func,1);
        onTheFlyCalcCrc( func );
        
        byteCount = dataCount * 2;
        tmp=write(tty,&byteCount,1);
        onTheFlyCalcCrc( byteCount );
        
        for(i=0; i < byteCount; i++ ) {
            printf("%02d:%02x\n",address+i,data[address+i]);
            //                    tmp=write(tty,&i,1);
            
            tmp=write(tty,&data[address+i],1);
            onTheFlyCalcCrc( data[address+i] );
        }
        
        tmp=write(tty,&uchCRCLo,1);
        tmp=write(tty,&uchCRCHi,1);
        
    }
    return( exception );
}

int writeMultipleRegisters(int tty, unsigned char rtu, unsigned char func) {
    unsigned tmp=0;
    unsigned int dataCount=0;
    unsigned
    char byteCount;
    int i=0;
    unsigned char zero=0;
    unsigned char addressHi=0;
    unsigned char addressLo=0;
    unsigned int address;
    
    int rc=OK;
    
    uchCRCHi = 0xff;
    uchCRCLo = 0xff;
    
    onTheFlyCalcCrc( rtu );
    onTheFlyCalcCrc( func );
    printf("entering readRegisters\n");
    
    
    addressHi=getByte(tty);
    address= ( addressHi << 8) & 0xff00;
    
    addressLo=getByte(tty);
    
    address |= ( addressLo & 0xff);
    printf("Address %04x\n", address);
    
    // Get count hi
    //
    tmp=getByte(tty);
    dataCount = (tmp << 8) & 0xff00;
    
    // get count lo
    //
    tmp = getByte(tty);
    
    dataCount |= (tmp & 0xff);
    printf("Count   %04x\n", dataCount);

    byteCount = getByte(tty);
    printf("Byte Count %02d\n",byteCount);
    
    for (i=0; i<byteCount; i++) {
        // FIX THIS
        // Copy to a holding area until CRC verified.
        //
        data[address+i] = getByte(tty);
        printf(">%02x ",data[address+i] );
    }
    printf("\n");
    tmp = getByte(tty);
    printf("%02x\n", tmp);
    
    tmp = getByte(tty);
    printf("%02x\n", tmp);

    uchCRCHi = 0xff;
    uchCRCLo = 0xff;

    printf("Write Multiple Registers\n");
    tmp=write(tty,&rtu,1);
    onTheFlyCalcCrc( rtu );
    
    tmp=write(tty,&func,1);
    onTheFlyCalcCrc( func );
    
    tmp=write(tty,&addressHi,1);
    onTheFlyCalcCrc( addressHi );
    
    tmp=write(tty,&addressLo,1);
    onTheFlyCalcCrc( addressLo );
    
    tmp=write(tty,&zero,1);
    onTheFlyCalcCrc( zero );
    
    tmp=write(tty,&dataCount,1);
    onTheFlyCalcCrc( dataCount );
    
    printf("O/P %03d:0x%02x\n", uchCRCLo, uchCRCLo);
    printf("O/P %03d:0x%0x\n", uchCRCHi, uchCRCHi);
    
    tmp=write(tty,&uchCRCLo,1);
    //                    onTheFlyCalcCrc( uchCRCLo );
    
    tmp=write(tty,&uchCRCHi,1);
    onTheFlyCalcCrc( uchCRCHi );

    return(rc);
}

int writeSingleCoil(int tty, unsigned char rtu, unsigned char func) {
    unsigned tmp=0;
//    unsigned int dataCount=0;
    unsigned
//    char byteCount;
    int i=0;
//    unsigned char zero=0;
    unsigned char addressHi=0;
    unsigned char addressLo=0;
    
    unsigned char bitAddressHi=0;
    unsigned char bitAddressLo=0;
    
    unsigned char dataHi=0;
    unsigned char dataLo=0;
    unsigned int address;
    unsigned int word=0;
    
    int rc=OK;
    
    uchCRCHi = 0xff;
    uchCRCLo = 0xff;
    
    onTheFlyCalcCrc( rtu );
    onTheFlyCalcCrc( func );
    printf("entering writeSingleCoil\n");
    
    
    addressHi=getByte(tty);
    address= ( addressHi << 8) & 0xff00;
    
    addressLo=getByte(tty);
    
        
    address |= ( addressLo & 0xff);
    printf("Address %04x\n", address);
    //
    // The address is a bit address.
    // First check that it's legal
    //
    // DATA_SIZE is the number of register bytes.
    //
    if (address > (DATA_SIZE * 8)) {
        rc = ILL_ADDRESS;
    } else {        // Address is legal, next two bytes should be 0xff00
                    // re-use addressHi & addressLo
        
        bitAddressHi = (address & 0xff ) >> 4; // this is the word offset ...
        bitAddressLo = address & 0x0f;         // ... and this is the bit.
        
        printf("bitAddressHi=%02x\n",bitAddressHi);
        printf("bitAddressLo=%02x\n",bitAddressLo);
        
        word= (data[ (bitAddressHi *2)+1 ]) | ( data[ bitAddressHi*2] << 8);
        
        
        printf("Word=%04x\n",word);
        
        dataHi = getByte( tty );
        dataLo = getByte(tty );
        tmp=getByte(tty); // Get the CRC
        tmp=getByte(tty);

        if ( (dataLo == 0x00 ) ) {
            switch (dataHi) {
                case 0xff:  // Set the bit.
                    word |= bitMask[ bitAddressLo ];
//                    data[ bitAddressHi ] |= bitMask[ bitAddressLo ];
                    
                    data[ bitAddressHi*2 ] = (word & 0xff00) >> 8;
                    data[ bitAddressHi*2 + 1 ] = word & 0xff;
                    break;
                case 0x00:  // clear the bit
                    word &= ~bitMask[ bitAddressLo ];
                    data[ bitAddressHi*2 ] = (word & 0xff00) >> 8;
                    data[ bitAddressHi*2 + 1 ] = word & 0xff;
//                    data[bitAddressHi] &= ~bitMask[ bitAddressLo ];
                    break;
                default:
                    rc= ILL_VALUE;
                    break;
            }
            
            
        }
        for (i=0; i<DATA_SIZE; i++) {
            printf("%02x:", data[i]);
        }
        printf("\n");
    }

    if (rc == OK) {
        uchCRCHi = 0xff;
        uchCRCLo = 0xff;
        
        printf("Write Single Coil\n");
        tmp=write(tty,&rtu,1);
        onTheFlyCalcCrc( rtu );
        
        tmp=write(tty,&func,1);
        onTheFlyCalcCrc( func );
        
        tmp=write(tty,&addressHi,1);
        onTheFlyCalcCrc( addressHi );
        
        tmp=write(tty,&addressLo,1);
        onTheFlyCalcCrc( addressLo );

        tmp=write(tty,&dataHi,1);
        onTheFlyCalcCrc( dataHi );
        
        tmp=write(tty,&dataLo,1);
        onTheFlyCalcCrc( dataLo );

        tmp=write(tty,&uchCRCLo,1);
        tmp=write(tty,&uchCRCHi,1);
    }
    return( rc );
}
    

int joinModbus( tty ) {
    int rc=OK;
    /*
     The ModBus spec requires that the time between bytes in a packet be <= 1.5 Chars (t1.5) , and the 
     the time between packets be >= 3.5 chars (t3.5).
     
     If the baud rate is greater than 19200 then the inter char timeout is fixed at 750 micro seconds
     and the inter vopacket delay at 1.75 ms (t3.5).
     
     When joining a modbus network a slave must wait until there is at leats t3.5 of silence.
     
     start timer t3.5
     begin
        if charcter recieved then
            discard
            restart timer
        endif
     until timed out
     */
    return( rc );
}

void usage() {
    printf("\n\n\t pmc -h|-p <device>\n\n");
    printf("\t\t-h\t\tHelp\n");
    printf("\t\t-p <device>\tSpecify serial port.\n\n");

    exit(-1);
}

int main(int argc, char *argv[]) {
    int tty=-1;
    
    unsigned char func=-1;
    unsigned char rtu=0;
    unsigned tmp=0;
//    unsigned int dataCount=0;
    unsigned char exception=0;
    int rc;
    int opt;
    extern char *optarg;
    
    char serialDev[255];
    strcpy(serialDev,"/dev/ttyUSB0");
//    char *serialDev="/dev/tty.usbserial-A600drA9";
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF; 
    
    (void)memset( data,0x00, DATA_SIZE );

    while((opt = getopt(argc,argv, "hp:")) != -1) {
        switch(opt) {
            case 'h':
                usage();
                break;
            case 'p':
                strcpy(serialDev, optarg);
                break;
        }
    }
    
    tty=u16OpenSerialPort(serialDev,0);
    
    printf("%d\n",tty);
    
    rc=joinModbus(tty);
    while ( 1 == 1 ) {

        uchCRCHi = 0xff;
        uchCRCLo = 0xff;
        printf("Get RTU id\n");
        rtu=getByte(tty);
        
        printf("RTU=%02x\n",rtu);
        
        if( rtu == (unsigned char)IAM) {
            printf("\tFor me!\n");
        } else {
            printf("\tNot For me!\n");
            /*
             * Should now start a timer that is restarted every time a byte is recieved.
             * On time out go and wait for another id byte.
             */
            exit(0);
        }
        
        printf("Get Func\n");
        func=getByte(tty);
        
        printf("Func=%02x\n",func);

        switch ( func ) {
            case 0x03:
            case 0x04: // Read registers 
                exception = readRegisters(tty,rtu,func);
                break;
            case 0x05: // Write single coil
                exception = writeSingleCoil( tty,rtu, func);
                break;
            case 0x10:
                exception=writeMultipleRegisters(tty,rtu,func);
                break;
        }
        
        printf("Exception=%02d\n",exception);
        if (exception != 0) {
            tmp=write(tty,&rtu,1);
            onTheFlyCalcCrc( rtu );
            
            func |= 0x80;
            tmp=write(tty,&func ,1);
            onTheFlyCalcCrc( func );
            
            tmp=write(tty,&exception ,1);
            onTheFlyCalcCrc( exception );
            
            tmp=write(tty,&uchCRCLo,1);
            tmp=write(tty,&uchCRCHi,1);
            
        }
        usleep( 5000 );
    }
}

