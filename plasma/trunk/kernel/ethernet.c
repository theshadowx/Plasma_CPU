/*--------------------------------------------------------------------
 * TITLE: Plasma Ethernet MAC
 * AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
 * DATE CREATED: 1/12/08
 * FILENAME: ethernet.c
 * PROJECT: Plasma CPU core
 * COPYRIGHT: Software placed into the public domain by the author.
 *    Software 'as is' without warranty.  Author liable for nothing.
 * DESCRIPTION:
 *    Ethernet MAC implementation.
 *    Data is received from the Ethernet PHY four bits at a time. 
 *    After 32-bits are received they are written to 0x13ff0000 + N.  
 *    The data is received LSB first for each byte which requires the
 *    nibbles to be swapped.
 *    Transmit data is read from 0x13fe0000.  Write length/4+1 to
 *    ETHERNET_REG to start transfer.
 *--------------------------------------------------------------------*/
#include "plasma.h"
#include "rtos.h"
#include "tcpip.h"

#define POLYNOMIAL  0x04C11DB7   //CRC bit 33 is truncated
#define TOPBIT      (1<<31)
#define BYTE_EMPTY  0xde         //Data copied into receive buffer
#define COUNT_EMPTY 16           //Count to decide there isn't data
#define INDEX_MASK  0xffff       //Size of receive buffer

//void dump(const unsigned char *data, int length);

static unsigned char gDestMac[]={0x5d, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static unsigned int CrcTable[256];
static unsigned char reflect[256];
static unsigned char reflectNibble[256];
static OS_Semaphore_t *SemEthernet, *SemEthTransmit;
static int gIndex;          //byte index into 0x13ff0000 receive buffer
static int gCheckedBefore;


//Read received data from 0x13ff0000.  Data starts with 0x5d+MACaddress.
//Data is being received while processing the data.  Therefore,
//all errors require waiting and then re-processing the data
//to see if the error is fixed by receiving the rest of the packet.
int EthernetReceive(unsigned char *buffer, int length)
{
   int count;
   int start, i, j, shift, offset, index;
   int byte, byteNext;
   unsigned long crc;
   int byteCrc;
   volatile unsigned char *buf = (unsigned char*)ETHERNET_RECEIVE;
   int packetExpected;

   //Find the start of a frame
   packetExpected = MemoryRead(IRQ_STATUS) & IRQ_ETHERNET_RECEIVE;
   MemoryRead(ETHERNET_REG);        //clear receive interrupt

   //Find dest MAC address
   for(offset = 0; offset <= INDEX_MASK; ++offset)
   {
      index = (gIndex + offset) & INDEX_MASK;
      byte = buf[index];
      if(byte == 0x5d)  //bit pattern 01011101
      {
         for(i = 1; i < sizeof(gDestMac); ++i)
         {
            j = (index + i) & INDEX_MASK;
            byte = buf[j];
            if(byte != 0xff && byte != gDestMac[i])
               break;
         }
         if(i == sizeof(gDestMac))
            break;    //found dest MAC
      }
      else if(byte == BYTE_EMPTY && packetExpected == 0)
         return 0;
   }
   if(offset > INDEX_MASK)
      return 0;
   while(gIndex != index)
   {
      buf[gIndex] = BYTE_EMPTY;
      gIndex = (gIndex + 1) & INDEX_MASK;
   }

   //Found start of frame.  Now find end of frame and check CRC.
   start = gIndex;
   gIndex = (gIndex + 1) & INDEX_MASK;           //skip 0x5d byte
   crc = 0xffffffff;
   for(count = 0; count < length; )
   {
      byte = buf[gIndex];
      gIndex = (gIndex + 1) & INDEX_MASK;

      byte = ((byte << 4) & 0xf0) | (byte >> 4); //swap nibbles
      buffer[count++] = (unsigned char)byte;
      byte = reflect[byte] ^ (crc >> 24);        //calculate CRC32
      crc = CrcTable[byte] ^ (crc << 8);
      if(count >= 40)
      {
         //Check if CRC matches to detect end of frame
         byteCrc = reflectNibble[crc >> 24];
         byteNext = buf[gIndex];
         if(byteCrc == byteNext)
         {
            for(i = 1; i < 4; ++i)
            {
               shift = 24 - (i << 3);
               byteCrc = reflectNibble[(crc >> shift) & 0xff];
               byteNext = buf[(gIndex + i) & 0xffff];
               if(byteCrc != byteNext)
               {
                  //printf("nope %d %d 0x%x 0x%x\n", count, i, byteCrc, byteNext);
                  i = 99;
               }
            }
            if(i == 4)
            {
               //Found end of frame -- set used bytes to BYTE_EMPTY
               //printf("Found it! %d\n", count);
               gIndex = (gIndex + 4) & INDEX_MASK;
               for(i = 0; i < count+5; ++i)
                  buf[(start + i) & INDEX_MASK] = BYTE_EMPTY;
               while(gIndex & 3)
               {
                  buf[gIndex] = BYTE_EMPTY;
                  gIndex = (gIndex + 1) & INDEX_MASK;
               }
               gCheckedBefore = 0;
               return count;
            }
         }
      }
   }
   gIndex = start;
   if(gCheckedBefore++ > 1)
   {
      buf[gIndex] = BYTE_EMPTY;
      gIndex = (gIndex + 1) & INDEX_MASK;
   }
   return 0;        //wait for more data
}


//Copy transmit data to 0x13fe0000 with preamble and CRC32
void EthernetTransmit(unsigned char *buffer, int length)
{
   int i, byte, shift;
   unsigned long crc;
   volatile unsigned char *buf = (unsigned char*)ETHERNET_TRANSMIT;

   OS_SemaphorePend(SemEthTransmit, OS_WAIT_FOREVER);

   //Wait for previous transfer to complete
   for(i = 0; i < 10000; ++i)
   {
      if(MemoryRead(IRQ_STATUS) & IRQ_ETHERNET_TRANSMIT)
         break;
   }
   //if(i > 100)
   //   printf("wait=%d ", i);

   Led(2, 2);
   while(length < 60 || (length & 3) != 0)
      buffer[length++] = 0;

   //Start of Ethernet frame
   for(i = 0; i < 7; ++i)
      buf[i] = 0x55;
   buf[7] = 0x5d;

   //Calculate CRC32
   crc = 0xffffffff;
   for(i = 0; i < length; ++i)
   {
      byte = buffer[i];
      buf[i + 8] = (unsigned char)((byte << 4) | (byte >> 4)); //swap nibbles
      byte = reflect[byte] ^ (crc >> 24);        //calculate CRC32
      crc = CrcTable[byte] ^ (crc << 8);
   }

   //Output CRC32
   for(i = 0; i < 4; ++i)
   {
      shift = 24 - (i << 3);
      byte = reflectNibble[(crc >> shift) & 0xff];
      buf[length + 8 + i] = (unsigned char)byte;
   }

   //Start transfer
   length = (length + 12 + 4) >> 2;
   MemoryWrite(ETHERNET_REG, length);
   Led(2, 0);

   OS_SemaphorePost(SemEthTransmit);
}


void EthernetThread(void *arg)
{
   int length;
   int rc;
   unsigned int ticks, ticksLast=0;
   int ticksWait=50;
   IPFrame *ethFrame=NULL;
   (void)arg;

   for(;;)
   {
      OS_InterruptMaskSet(IRQ_ETHERNET_RECEIVE);      //enable interrupt
      rc = OS_SemaphorePend(SemEthernet, ticksWait);  //wait for interrupt
      if(rc)
         ticksWait = 50;
      else
         ticksWait = 2;

      //Process all received packets
      for(;;)
      {
         if(ethFrame == NULL)
            ethFrame = IPFrameGet(FRAME_COUNT_RCV);
         if(ethFrame == NULL)
            break;
         length = EthernetReceive(ethFrame->packet, PACKET_SIZE);
         if(length == 0)
            break;
         Led(1, 1);
         rc = IPProcessEthernetPacket(ethFrame, length);
         Led(1, 0);
         if(rc)
            ethFrame = NULL;
      }

      ticks = OS_ThreadTime();
      if(ticks - ticksLast >= 50)
      {
         IPTick();
         ticksLast = ticks;
      }
   }
}


void EthernetIsr(void *arg)
{
   (void)arg;
   OS_InterruptMaskClear(IRQ_ETHERNET_RECEIVE);
   OS_SemaphorePost(SemEthernet);
}


/******************* CRC32 calculations **********************
 * The CRC32 code is modified from Michale Barr's article in 
 * Embedded Systems Programming January 2000.
 * A CRC is really modulo-2 binary division.  Substraction means XOR. */
static unsigned int Reflect(unsigned int value, int bits)
{
   unsigned int num=0;
   int i;
   for(i = 0; i < bits; ++i)
   {
      num = (num << 1) | (value & 1);
      value >>= 1;
   }
   return num;
}


static void CrcInit(void)
{
   unsigned int remainder;
   int dividend, bit, i;

   //Compute the remainder of each possible dividend
   for(dividend = 0; dividend < 256; ++dividend)
   {
      //Start with the dividend followed by zeros
      remainder = dividend << 24;
      //Perform modulo-2 division, a bit at a time
      for(bit = 8; bit > 0; --bit)
      {
         //Try to divide the current data bit
         if(remainder & TOPBIT)
            remainder = (remainder << 1) ^ POLYNOMIAL;
         else
            remainder = remainder << 1;
      }
      CrcTable[dividend] = remainder;
   }
   for(i = 0; i < 256; ++i)
   {
      reflect[i] = (unsigned char)Reflect(i, 8);
      reflectNibble[i] = (unsigned char)((Reflect((i >> 4) ^ 0xf, 4) << 4) | 
         Reflect(i ^ 0xf, 4));
   }
}


static void SpinWait(int clocks)
{
   int value = *(volatile int*)COUNTER_REG + clocks;
   while(*(volatile int*)COUNTER_REG - value < 0)
      ;
}


void EthernetInit(unsigned char MacAddress[6])
{
   //Format of SMI data: 0101 A4:A0 R4:R0 00 D15:D0
   unsigned long data=0x5f800100; //SMI R0 = 10Mbps full duplex
   //unsigned long data=0x5f800000; //SMI R0 = 10Mbps half duplex
   int i, value;
   volatile unsigned char *buf = (unsigned char*)ETHERNET_RECEIVE;

   CrcInit();
   if(MacAddress)
   {
      for(i = 0; i < 6; ++i)
      {
         value = MacAddress[i];
         gDestMac[i+1] = (unsigned char)((value >> 4) | (value << 4));
      }
   }

   //Configure Ethernet PHY for 10Mbps full duplex via SMI interface
   MemoryWrite(GPIO0_OUT, ETHERNET_MDIO | ETHERNET_MDIO_WE | ETHERENT_MDC);
   for(i = 0; i < 34; ++i)
   {
      MemoryWrite(GPIO0_OUT, ETHERENT_MDC);    //clock high
      SpinWait(10);
      MemoryWrite(GPIO0_CLEAR, ETHERENT_MDC);  //clock low
      SpinWait(10);
   }
   for(i = 31; i >= 0; --i)
   {
      value = (data >> i) & 1;
      if(value)
         MemoryWrite(GPIO0_OUT, ETHERNET_MDIO);
      else
         MemoryWrite(GPIO0_CLEAR, ETHERNET_MDIO);
      MemoryWrite(GPIO0_OUT, ETHERENT_MDC);    //clock high
      SpinWait(10);
      MemoryWrite(GPIO0_CLEAR, ETHERENT_MDC);  //clock low
      SpinWait(10);
   }
   MemoryWrite(GPIO0_CLEAR, ETHERNET_MDIO_WE | ETHERNET_ENABLE);

   //Clear receive buffer
   for(i = 0; i <= INDEX_MASK; ++i)
      buf[i] = BYTE_EMPTY;

   if(SemEthernet == NULL)
   {
      SemEthernet = OS_SemaphoreCreate("eth", 0);
      SemEthTransmit = OS_SemaphoreCreate("ethT", 1);
      OS_ThreadCreate("eth", EthernetThread, NULL, 240, 0);
   }

   //Setup interrupts for receiving data
   OS_InterruptRegister(IRQ_ETHERNET_RECEIVE, EthernetIsr);

   //Start receive DMA
   MemoryWrite(GPIO0_OUT, ETHERNET_ENABLE);
}
