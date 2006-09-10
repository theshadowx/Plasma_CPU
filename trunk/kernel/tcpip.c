/*--------------------------------------------------------------------
 * TITLE: Plasma TCP/IP Protocol Stack
 * AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
 * DATE CREATED: 4/22/06
 * FILENAME: tcpip.c
 * PROJECT: Plasma CPU core
 * COPYRIGHT: Software placed into the public domain by the author.
 *    Software 'as is' without warranty.  Author liable for nothing.
 * DESCRIPTION:
 *    Plasma TCP/IP Protocol Stack
 *
 *    Possible call stack when receiving a packet:
 *       IPMainThread()
 *          IPProcessEthernetPacket()
 *             IPProcessTCPPacket()
 *                TCPSendPacket()
 *                   IPSendPacket()
 *                      IPChecksum()
 *                      IPSendFrame()
 *                         FrameInsert()
 *--------------------------------------------------------------------*/
#ifdef WIN32
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#define _LIBC
#endif
#include "rtos.h"
#include "tcpip.h"

#ifdef WIN32
#undef OS_CriticalBegin
#undef OS_CriticalEnd
#define OS_CriticalBegin() 0
#define OS_CriticalEnd(A)
#define OS_ThreadCreate(A,B,C,D,E) 0
#define OS_ThreadSleep(A)
#define OS_ThreadTime() 0
#define OS_ThreadSelf() 0
#define OS_MutexCreate(A) NULL
#define OS_MutexPend(A)
#define OS_MutexPost(A)
#define OS_MQueueCreate(A,B,C) 0
#define OS_MQueueSend(A,B) 0
#define OS_MQueueGet(A,B,C) 0
#define UartPacketConfig(A,B,C)
#define UartPacketSend(A,B)
#define UartPrintf printf
#define UartPrintfCritical printf
#define Led(A)
#endif

//ETHER FIELD                 OFFSET   LENGTH   VALUE
#define ETHERNET_DEST         0        //6
#define ETHERNET_SOURCE       6        //6
#define ETHERNET_FRAME_TYPE   12       //2      IP=0x0800; ARP=0x0806

//ARP   FIELD                 OFFSET   LENGTH   VALUE
#define ARP_HARD_TYPE         14       //2      0x0001
#define ARP_PROT_TYPE         16       //2      0x0800
#define ARP_HARD_SIZE         18       //1      0x06
#define ARP_PROT_SIZE         19       //1      0x04
#define ARP_OP                20       //2      ARP=1;ARPreply=2
#define ARP_ETHERNET_SENDER   22       //6
#define ARP_IP_SENDER         28       //4
#define ARP_ETHERNET_TARGET   32       //6
#define ARP_IP_TARGET         38       //4
#define ARP_PAD               42       //18     0

//IP    FIELD                 OFFSET   LENGTH   VALUE
#define IP_VERSION_LENGTH     14       //1      0x45
#define IP_TYPE_OF_SERVICE    15       //1      0x00
#define IP_LENGTH             16       //2
#define IP_ID16               18       //2
#define IP_FRAG_OFFSET        20       //2
#define IP_TIME_TO_LIVE       22       //1      0x80
#define IP_PROTOCOL           23       //1      TCP=0x06;PING=0x01;UDP=0x11
#define IP_CHECKSUM           24       //2
#define IP_SOURCE             26       //4
#define IP_DEST               30       //4

//PSEUDO FIELD                OFFSET   LENGTH   VALUE
#define PSEUDO_IP_SOURCE      0        //4
#define PSEUDO_IP_DEST        4        //4
#define PSEUDO_ZERO           8        //1      0
#define PSEUDO_IP_PROTOCOL    9        //1
#define PSEUDO_LENGTH         10       //2

//UDP   FIELD                 OFFSET   LENGTH   VALUE
#define UDP_SOURCE_PORT       34       //2
#define UDP_DEST_PORT         36       //2
#define UDP_LENGTH            38       //2
#define UDP_CHECKSUM          40       //2
#define UDP_DATA              42

//DHCP  FIELD                 OFFSET   LENGTH   VALUE
#define DHCP_OPCODE           42       //1      REQUEST=1;REPLY=2
#define DHCP_HW_TYPE          43       //1      1
#define DHCP_HW_LEN           44       //1      6
#define DHCP_HOP_COUNT        45       //1      0
#define DHCP_TRANS_ID         46       //4
#define DHCP_NUM_SEC          50       //2      0
#define DHCP_UNUSED           52       //2
#define DHCP_CLIENT_IP        54       //4
#define DHCP_YOUR_IP          58       //4
#define DHCP_SERVER_IP        62       //4
#define DHCP_GATEWAY_IP       66       //4
#define DHCP_CLIENT_ETHERNET  70       //16
#define DHCP_SERVER_NAME      86       //64
#define DHCP_BOOT_FILENAME    150      //128
#define DHCP_MAGIC_COOKIE     278      //4      0x63825363
#define DHCP_OPTIONS          282      //N

#define DHCP_MESSAGE_TYPE     53       //1 type
#define DHCP_DISCOVER         1
#define DHCP_OFFER            2
#define DHCP_REQUEST          3
#define DHCP_ACK              5
#define DHCP_REQUEST_IP       50       //4 ip
#define DHCP_REQUEST_SERV_IP  54       //4 ip
#define DHCP_CLIENT_ID        61       //7 1 ethernet
#define DHCP_HOST_NAME        12       //6 plasma
#define DHCP_PARAMS           55       //4 1=subnet; 15=domain_name; 3=router; 6=dns
#define DHCP_PARAM_SUBNET     1
#define DHCP_PARAM_ROUTER     3
#define DHCP_PARAM_DNS        6
#define DHCP_END_OPTION       0xff

//DHCP  FIELD                 OFFSET   LENGTH   VALUE
#define DNS_ID                0       //2    
#define DNS_FLAGS             2       //2      
#define DNS_NUM_QUESTIONS     4       //2      1 
#define DNS_NUM_ANSWERS_RR    6       //2      0/1
#define DNS_NUM_AUTHORITY_RR  8       //2      0 
#define DNS_NUM_ADDITIONAL_RR 10       //2      0
#define DNS_QUESTIONS         12       //2   

#define DNS_FLAGS_RESPONSE    0x8000
#define DNS_FLAGS_RECURSIVE   0x0100
#define DNS_FLAGS_ERROR       0x0003
#define DNS_FLAGS_OK          0x0000
#define DNS_QUERY_TYPE_IP     1
#define DNS_QUERY_CLASS       1
#define DNS_PORT              53

//TCP   FIELD                 OFFSET   LENGTH   VALUE
#define TCP_SOURCE_PORT       34       //2
#define TCP_DEST_PORT         36       //2
#define TCP_SEQ               38       //4
#define TCP_ACK               42       //4
#define TCP_HEADER_LENGTH     46       //1      0x50
#define TCP_FLAGS             47       //1      SYNC=0x2;ACK=0x10;FIN=0x1
#define TCP_WINDOW_SIZE       48       //2
#define TCP_CHECKSUM          50       //2
#define TCP_URGENT_POINTER    52       //2
#define TCP_DATA              54       //length-N

#define TCP_FLAGS_FIN         1
#define TCP_FLAGS_SYN         2
#define TCP_FLAGS_RST         4
#define TCP_FLAGS_ACK         16

//PING  FIELD                 OFFSET   LENGTH   VALUE
#define PING_TYPE             34       //1      SEND=8;REPLY=0
#define PING_CODE             35       //1      0
#define PING_CHECKSUM         36       //2
#define PING_ID               38       //2
#define PING_SEQUENCE         40       //2
#define PING_DATA             44

static void IPClose2(IPSocket *Socket);

static uint8 ethernetAddressNull[] =    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static uint8 ethernetAddressPlasma[] =  {0x00, 0x10, 0xdd, 0xce, 0x15, 0xd4};
static uint8 ethernetAddressGateway[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static uint8 ipAddressPlasma[] =  {0x9d, 0xfe, 0x28, 10};   //changed by DHCP
static uint8 ipAddressGateway[] = {0xff, 0xff, 0xff, 0xff}; //changed by DHCP
static uint32 ipAddressDns;                                 //changed by DHCP

static OS_Mutex_t *IPMutex;
static int FrameFreeCount=FRAME_COUNT;
static IPFrame *FrameFreeHead;
static IPFrame *FrameSendHead;
static IPFrame *FrameSendTail;
static IPFrame *FrameResendHead;
static IPFrame *FrameResendTail;
static IPSocket *SocketHead;
static uint32 Seconds;
static int DhcpRetrySeconds;
static IPFuncPtr FrameSendFunc;
static OS_MQueue_t *IPMQueue;
static OS_Thread_t *IPThread;
int IPVerbose=1;

static const unsigned char dhcpDiscover[] = {
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff,             //dest
   0x00, 0x10, 0xdd, 0xce, 0x15, 0xd4,             //src
   0x08, 0x00, 
   0x45, 0x00, 0x01, 0x48, 0x2e, 0xf5, 0x00, 0x00, //ip
   0x80, 0x11, 0x0a, 0xb1, 0x00, 0x00, 0x00, 0x00, 
   0xff, 0xff, 0xff, 0xff,
   0x00, 0x44, 0x00, 0x43, 0x01, 0x34, 0x45, 0x66, //udp
   0x01, 0x01, 0x06, 0x00, 0x69, 0x26, 0xb5, 0x52  //dhcp
};

static const unsigned char dhcpOptions[] = {
   0x63, 0x82, 0x53, 0x63,      //cookie
   0x35, 0x01, 0x01,            //DHCP Discover
   0x3d, 0x07, 0x01, 0x00, 0x10, 0xdd, 0xce, 0x15, 0xd4, //Client identifier
   0x0c, 0x06, 'p', 'l', 'a', 's', 'm', 'a',             //Host name
   0x37, 0x03, DHCP_PARAM_SUBNET, DHCP_PARAM_ROUTER, DHCP_PARAM_DNS, //Parameters
   DHCP_END_OPTION
};


//Get a free frame; can be called from an ISR
IPFrame *IPFrameGet(int freeCount)
{
   IPFrame *frame=NULL;
   uint32 state;

   state = OS_CriticalBegin();
   if(FrameFreeCount > freeCount)
   {
      --FrameFreeCount;
      frame = FrameFreeHead;
      if(FrameFreeHead)
         FrameFreeHead = FrameFreeHead->next;
   }
   OS_CriticalEnd(state);
   if(frame)
   {
      assert(frame->state == 0);
      frame->state = 1;
   }
   else if(IPVerbose)
      UartPrintfCritical(":");
   return frame;
}


static void FrameFree(IPFrame *frame)
{
   uint32 state;

   assert(frame->state == 1);
   frame->state = 0;
   state = OS_CriticalBegin();
   frame->next = FrameFreeHead;
   FrameFreeHead = frame;
   ++FrameFreeCount;
   OS_CriticalEnd(state);
}


static void FrameInsert(IPFrame **head, IPFrame **tail, IPFrame *frame)
{
   assert(frame->state == 1);
   frame->state = 2;
   OS_MutexPend(IPMutex);
   frame->prev = NULL;
   frame->next = *head;
   if(*head)
      (*head)->prev = frame;
   *head = frame;
   if(*tail == NULL)
      *tail = frame;
   OS_MutexPost(IPMutex);
}


static void FrameRemove(IPFrame **head, IPFrame **tail, IPFrame *frame)
{
   assert(frame->state == 2);
   frame->state = 1;
   if(frame->prev)
      frame->prev->next = frame->next;
   else
      *head = frame->next;
   if(frame->next)
      frame->next->prev = frame->prev;
   else
      *tail = frame->prev;
   frame->prev = NULL;
   frame->next = NULL;
}


static int IPChecksum(int checksum, const unsigned char *data, int length)
{
   int i;
   checksum = ~checksum & 0xffff;
   for(i = 0; i < length-1; i += 2)
   {
      checksum += (data[i] << 8) | data[i+1];
   }
   if(i < length)
      checksum += data[i] << 8;
   while(checksum >> 16)
      checksum = (checksum & 0xffff) + (checksum >> 16);
   checksum = ~checksum & 0xffff;
   return checksum;
}


static int EthernetVerifyChecksums(const unsigned char *packet, int length)
{
   int checksum, length2;
   unsigned char pseudo[12];

   //Calculate checksums
   if(packet[ETHERNET_FRAME_TYPE+1] == 0x00)  //IP
   {
      checksum = IPChecksum(0xffff, packet+IP_VERSION_LENGTH, 20);
      if(checksum)
         return -1;
      if(packet[IP_PROTOCOL] == 0x01)         //PING
      {
         checksum = IPChecksum(0xffff, packet+PING_TYPE, length-PING_TYPE);
      }
      else if(packet[IP_PROTOCOL] == 0x11)    //UDP
      {
         if(packet[UDP_CHECKSUM] == 0 && packet[UDP_CHECKSUM+1] == 0)
            return 0;
         memcpy(pseudo+PSEUDO_IP_SOURCE, packet+IP_SOURCE, 4);
         memcpy(pseudo+PSEUDO_IP_DEST, packet+IP_DEST, 4);
         pseudo[PSEUDO_ZERO] = 0;
         pseudo[PSEUDO_IP_PROTOCOL] = packet[IP_PROTOCOL];
         memcpy(pseudo+PSEUDO_LENGTH, packet+UDP_LENGTH, 2);
         checksum = IPChecksum(0xffff, pseudo, 12);
         length2 = (packet[UDP_LENGTH] << 8) + packet[UDP_LENGTH+1];
         checksum = IPChecksum(checksum, packet+UDP_SOURCE_PORT, length);
      }
      else if(packet[IP_PROTOCOL] == 0x06)    //TCP
      {
         memcpy(pseudo+PSEUDO_IP_SOURCE, packet+IP_SOURCE, 4);
         memcpy(pseudo+PSEUDO_IP_DEST, packet+IP_DEST, 4);
         pseudo[PSEUDO_ZERO] = 0;
         pseudo[PSEUDO_IP_PROTOCOL] = packet[IP_PROTOCOL];
         length = (packet[IP_LENGTH] << 8) + packet[IP_LENGTH+1];
         length2 = length - 20;
         pseudo[PSEUDO_LENGTH] = (unsigned char)(length2 >> 8);
         pseudo[PSEUDO_LENGTH+1] = (unsigned char)length2;
         checksum = IPChecksum(0xffff, pseudo, 12);
         checksum = IPChecksum(checksum, packet+TCP_SOURCE_PORT, length2);
      }
      if(checksum)
         return -1;
   }
   return 0;
}


static void IPFrameReschedule(IPFrame *frame)
{
   int length;
   length = frame->length - TCP_DATA;
   if(frame->packet[TCP_FLAGS] & (TCP_FLAGS_FIN | TCP_FLAGS_SYN))
      ++length;
   if(frame->socket == NULL || frame->socket->state == IP_UDP || length == 0 ||
      ++frame->retryCnt > 4)
   {
      FrameFree(frame);     //can't be ACK'ed
   }
   else
   {
      //Put on resend list until TCP ACK'ed
      frame->timeout = RETRANSMIT_TIME;
      FrameInsert(&FrameResendHead, &FrameResendTail, frame);
   }
}


static void IPSendFrame(IPFrame *frame)
{
   uint32 message[4];

   if(FrameSendFunc)
   {
      //Single threaded
      FrameSendFunc(frame->packet, frame->length);
      IPFrameReschedule(frame);
   }
   else
   {
      //Add Packet to send queue
      FrameInsert(&FrameSendHead, &FrameSendTail, frame);

      //Wakeup sender thread
      message[0] = 2;
      OS_MQueueSend(IPMQueue, message);
   }
}


static void IPSendPacket(IPSocket *socket, IPFrame *frame, int length)
{
   int checksum, length2=length;
   unsigned char pseudo[12], *packet=frame->packet;

   frame->length = (uint16)length;

   //Calculate checksums
   if(packet[ETHERNET_FRAME_TYPE+1] == 0x00)  //IP
   {
      length2 = length - IP_VERSION_LENGTH;
      packet[IP_LENGTH] = (uint8)(length2 >> 8);
      packet[IP_LENGTH+1] = (uint8)length2;
      memset(packet+IP_CHECKSUM, 0, 2);
      checksum = IPChecksum(0xffff, packet+IP_VERSION_LENGTH, 20);
      packet[IP_CHECKSUM] = (unsigned char)(checksum >> 8);
      packet[IP_CHECKSUM+1] = (unsigned char)checksum;
      if(packet[IP_PROTOCOL] == 0x01)         //PING
      {
         memset(packet+PING_CHECKSUM, 0, 2);
         checksum = IPChecksum(0xffff, packet+PING_TYPE, length-PING_TYPE);
         packet[PING_CHECKSUM] = (unsigned char)(checksum >> 8);
         packet[PING_CHECKSUM+1] = (unsigned char)checksum;
      }
      else if(packet[IP_PROTOCOL] == 0x11)    //UDP
      {
         length2 = length - UDP_SOURCE_PORT;
         packet[UDP_LENGTH] = (uint8)(length2 >> 8);
         packet[UDP_LENGTH+1] = (uint8)length2;
         memcpy(pseudo+PSEUDO_IP_SOURCE, packet+IP_SOURCE, 4);
         memcpy(pseudo+PSEUDO_IP_DEST, packet+IP_DEST, 4);
         pseudo[PSEUDO_ZERO] = 0;
         pseudo[PSEUDO_IP_PROTOCOL] = packet[IP_PROTOCOL];
         memcpy(pseudo+PSEUDO_LENGTH, packet+UDP_LENGTH, 2);
         checksum = IPChecksum(0xffff, pseudo, 12);
         memset(packet+UDP_CHECKSUM, 0, 2);
         length2 = (packet[UDP_LENGTH] << 8) + packet[UDP_LENGTH+1];
         checksum = IPChecksum(checksum, packet+UDP_SOURCE_PORT, length2);
         packet[UDP_CHECKSUM] = (unsigned char)(checksum >> 8);
         packet[UDP_CHECKSUM+1] = (unsigned char)checksum;
      }
      else if(packet[IP_PROTOCOL] == 0x06)    //TCP
      {
         memcpy(pseudo+PSEUDO_IP_SOURCE, packet+IP_SOURCE, 4);
         memcpy(pseudo+PSEUDO_IP_DEST, packet+IP_DEST, 4);
         pseudo[PSEUDO_ZERO] = 0;
         pseudo[PSEUDO_IP_PROTOCOL] = packet[IP_PROTOCOL];
         length2 = (packet[IP_LENGTH] << 8) + packet[IP_LENGTH+1];
         length2 = length2 - 20;
         pseudo[PSEUDO_LENGTH] = (unsigned char)(length2 >> 8);
         pseudo[PSEUDO_LENGTH+1] = (unsigned char)length2;
         checksum = IPChecksum(0xffff, pseudo, 12);
         memset(packet+TCP_CHECKSUM, 0, 2);
         checksum = IPChecksum(checksum, packet+TCP_SOURCE_PORT, length2);
         packet[TCP_CHECKSUM] = (unsigned char)(checksum >> 8);
         packet[TCP_CHECKSUM+1] = (unsigned char)checksum;
      }
   }

   length2 = length - TCP_DATA;
   if(socket && (packet[TCP_FLAGS] & (TCP_FLAGS_FIN | TCP_FLAGS_SYN)))
      length2 = 1;
   frame->socket = socket;
   frame->timeout = 0;
   frame->retryCnt = 0;
   if(socket)
      frame->seqEnd = socket->seq + length2;
   IPSendFrame(frame);
}


static void TCPSendPacket(IPSocket *socket, IPFrame *frame, int length)
{
   uint8 *packet = frame->packet;
   int flags, count;

   flags = packet[TCP_FLAGS];
   memcpy(packet, socket->headerSend, TCP_SEQ);
   packet[TCP_FLAGS] = (uint8)flags;
   if(flags & TCP_FLAGS_SYN)
      packet[TCP_HEADER_LENGTH] = 0x60;  //set maximum segment size
   else
      packet[TCP_HEADER_LENGTH] = 0x50;
   packet[TCP_SEQ]   = (uint8)(socket->seq >> 24);
   packet[TCP_SEQ+1] = (uint8)(socket->seq >> 16);
   packet[TCP_SEQ+2] = (uint8)(socket->seq >> 8);
   packet[TCP_SEQ+3] = (uint8)socket->seq;
   packet[TCP_ACK]   = (uint8)(socket->ack >> 24);
   packet[TCP_ACK+1] = (uint8)(socket->ack >> 16);
   packet[TCP_ACK+2] = (uint8)(socket->ack >> 8);
   packet[TCP_ACK+3] = (uint8)socket->ack;
   count = FrameFreeCount - FRAME_COUNT_WINDOW;
   if(count < 1)
      count = 1;
   count *= 512;
   packet[TCP_WINDOW_SIZE] = (uint8)(count >> 8);
   packet[TCP_WINDOW_SIZE+1] = (uint8)count;
   packet[TCP_URGENT_POINTER] = 0;
   packet[TCP_URGENT_POINTER+1] = 0;
   IPSendPacket(socket, frame, length);
}


static void EthernetCreateResponse(unsigned char *packetOut,
                                   const unsigned char *packet,
                                   int length)
{
   //Swap destination and source fields
   memcpy(packetOut, packet, length);
   memcpy(packetOut+ETHERNET_DEST, packet+ETHERNET_SOURCE, 6);
   memcpy(packetOut+ETHERNET_SOURCE, packet+ETHERNET_DEST, 6);
   if(packet[ETHERNET_FRAME_TYPE+1] == 0x00)  //IP
   {
      memcpy(packetOut+IP_SOURCE, packet+IP_DEST, 4);
      memcpy(packetOut+IP_DEST, packet+IP_SOURCE, 4);
      if(packet[IP_PROTOCOL] == 0x06 || packet[IP_PROTOCOL] == 0x11)   //TCP/UDP
      {
         memcpy(packetOut+TCP_SOURCE_PORT, packet+TCP_DEST_PORT, 2);
         memcpy(packetOut+TCP_DEST_PORT, packet+TCP_SOURCE_PORT, 2);
      }
   }
}


static void IPDhcp(const unsigned char *packet, int length, int state)
{
   uint8 *packetOut, *ptr;
   const uint8 *ptr2;
   IPFrame *frame;
   static int request=0;

   if(state == 1)
   {
      //Create DHCP Discover
      frame = IPFrameGet(0);
      if(frame == NULL)
         return;
      packetOut = frame->packet;
      memset(packetOut, 0, 512);
      memcpy(packetOut, dhcpDiscover, sizeof(dhcpDiscover));
      memcpy(packetOut+ETHERNET_SOURCE, ethernetAddressPlasma, 6);
      memcpy(packetOut+DHCP_CLIENT_ETHERNET, ethernetAddressPlasma, 6);
      memcpy(packetOut+DHCP_MAGIC_COOKIE, dhcpOptions, sizeof(dhcpOptions));
      memcpy(packetOut+DHCP_MAGIC_COOKIE+10, ethernetAddressPlasma, 6);
      IPSendPacket(NULL, frame, 400);
      request = DHCP_DISCOVER;
      DhcpRetrySeconds = RETRANSMIT_TIME;
   }
   else if(state == 2 && memcmp(packet+DHCP_CLIENT_ETHERNET, ethernetAddressPlasma, 6) == 0)
   {
      if(packet[DHCP_MAGIC_COOKIE+6] == DHCP_OFFER && request == DHCP_DISCOVER)
      {
         //Process DHCP Offer and send DHCP Request
         frame = IPFrameGet(0);
         if(frame == NULL)
            return;
         packetOut = frame->packet;
         memset(packetOut, 0, 512);
         memcpy(packetOut, dhcpDiscover, sizeof(dhcpDiscover));
         memcpy(packetOut+ETHERNET_SOURCE, ethernetAddressPlasma, 6);
         memcpy(packetOut+DHCP_CLIENT_ETHERNET, ethernetAddressPlasma, 6);
         memcpy(packetOut+DHCP_MAGIC_COOKIE, dhcpOptions, sizeof(dhcpOptions));
         //memcpy(packetOut+DHCP_MAGIC_COOKIE+10, ethernetAddressPlasma, 6);
         request = DHCP_REQUEST;
         packetOut[DHCP_MAGIC_COOKIE+6] = DHCP_REQUEST;
         ptr = packetOut+DHCP_MAGIC_COOKIE+sizeof(dhcpOptions)-1;
         ptr[0] = DHCP_REQUEST_IP;
         ptr[1] = 4;
         memcpy(ptr+2, packet+DHCP_YOUR_IP, 4);
         ptr[6] = DHCP_REQUEST_SERV_IP;
         ptr[7] = 4;
         memcpy(ptr+8, packet+DHCP_SERVER_IP, 4);
         ptr[12] = DHCP_END_OPTION;
         IPSendPacket(NULL, frame, 400);
      }
      else if(packet[DHCP_MAGIC_COOKIE+6] == DHCP_ACK && request == DHCP_REQUEST)
      {
         //Process DHCP Ack
         request = 0;
         DhcpRetrySeconds = 3600*4;
         memcpy(ipAddressPlasma, packet+DHCP_YOUR_IP, 4);
         printf("IP=%d.%d.%d.%d ", ipAddressPlasma[0], ipAddressPlasma[1],
            ipAddressPlasma[2], ipAddressPlasma[3]);
         memcpy(ipAddressGateway, packet+DHCP_GATEWAY_IP, 4);
         if(ipAddressGateway[0] == 0 && ipAddressGateway[1] == 0 &&
            ipAddressGateway[2] == 0 && ipAddressGateway[3] == 0)
            memcpy(ipAddressGateway, packet+DHCP_SERVER_IP, 4);
         printf("GW=%d.%d.%d.%d ", ipAddressGateway[0], ipAddressGateway[1],
            ipAddressGateway[2], ipAddressGateway[3]);
         memcpy(ethernetAddressGateway, packet+ETHERNET_SOURCE, 6);
         ptr2 = packet+DHCP_MAGIC_COOKIE+4;
         while(ptr2[0] != DHCP_END_OPTION && (int)(ptr2 - packet) < length)
         {
            if(ptr2[0] == DHCP_PARAM_DNS)
            {
               ipAddressDns = (ptr2[2] << 24) | (ptr2[3] << 16) | (ptr2[4] << 8) | ptr2[5];
               printf("DNS=%d.%d.%d.%d ", ptr2[2], ptr2[3], ptr2[4], ptr2[5]);
            }
            ptr2 += ptr2[1] + 2;
         }

         //Check if DHCP reply came from gateway
         if(memcmp(packet+IP_SOURCE, ipAddressGateway, 4))
         {
            //Send ARP to gateway
            frame = IPFrameGet(0);
            if(frame == NULL)
               return;
            packetOut = frame->packet;
            memset(packetOut, 0, 512);
            memset(packetOut+ETHERNET_DEST, 0xff, 6);
            memcpy(packetOut+ETHERNET_SOURCE, ethernetAddressPlasma, 6);
            packetOut[ETHERNET_FRAME_TYPE] = 0x08;
            packetOut[ETHERNET_FRAME_TYPE+1] = 0x06;
            packetOut[ARP_HARD_TYPE+1] = 0x01;
            packetOut[ARP_PROT_TYPE] = 0x08;
            packetOut[ARP_HARD_SIZE] = 0x06;
            packetOut[ARP_PROT_SIZE] = 0x04;
            packetOut[ARP_OP+1] = 1;
            memcpy(packetOut+ARP_ETHERNET_SENDER, ethernetAddressPlasma, 6);
            memcpy(packetOut+ARP_IP_SENDER, ipAddressPlasma, 4);
            memcpy(packetOut+ARP_IP_TARGET, ipAddressGateway, 4);
            IPSendPacket(NULL, frame, 60);
         }
      }
   }
}


static int IPProcessTCPPacket(IPFrame *frameIn)
{
   uint32 seq, ack;
   int length, ip_length, bytes;
   IPSocket *socket, *socketNew;
   IPFrame *frameOut, *frame2, *framePrev;
   uint8 *packet, *packetOut;

   packet = frameIn->packet;
   length = frameIn->length;

   ip_length = (packet[IP_LENGTH] << 8) | packet[IP_LENGTH+1];
   seq = (packet[TCP_SEQ] << 24) | (packet[TCP_SEQ+1] << 16) | 
         (packet[TCP_SEQ+2] << 8) | packet[TCP_SEQ+3];
   ack = (packet[TCP_ACK] << 24) | (packet[TCP_ACK+1] << 16) | 
         (packet[TCP_ACK+2] << 8) | packet[TCP_ACK+3];

   //Check if start of connection
   if((packet[TCP_FLAGS] & (TCP_FLAGS_SYN | TCP_FLAGS_ACK)) == TCP_FLAGS_SYN)
   {
      if(IPVerbose)
         printf("S");
      //Check if duplicate SYN
      for(socket = SocketHead; socket; socket = socket->next)
      {
         if(socket->state != IP_LISTEN &&
            packet[IP_PROTOCOL] == socket->headerRcv[IP_PROTOCOL] &&
            memcmp(packet+IP_SOURCE, socket->headerRcv+IP_SOURCE, 8) == 0 &&
            memcmp(packet+TCP_SOURCE_PORT, socket->headerRcv+TCP_SOURCE_PORT, 4) == 0)
         {
            if(IPVerbose)
               printf("s");
            return 0;
         }
      }

      //Find an open port
      for(socket = SocketHead; socket; socket = socket->next)
      {
         if(socket->state == IP_LISTEN &&
            packet[IP_PROTOCOL] == socket->headerRcv[IP_PROTOCOL] &&
            memcmp(packet+TCP_DEST_PORT, socket->headerRcv+TCP_DEST_PORT, 2) == 0)
         {
            //Create a new socket
            frameOut = IPFrameGet(FRAME_COUNT_SEND);
            if(frameOut == NULL)
               return 0;
            socketNew = (IPSocket*)malloc(sizeof(IPSocket));
            if(socketNew == NULL)
               return 0;
            memcpy(socketNew, socket, sizeof(IPSocket));
            socketNew->state = IP_TCP;
            socketNew->timeout = RETRANSMIT_TIME * 3;
            socketNew->ack = seq;
            socketNew->seq = socketNew->ack + 0x12345678;
            socketNew->seqReceived = socketNew->seq;

            //Send ACK
            packetOut = frameOut->packet;
            EthernetCreateResponse(packetOut, packet, length);
            memcpy(socketNew->headerRcv, packet, TCP_SEQ);
            memcpy(socketNew->headerSend, packetOut, TCP_SEQ);
            packetOut[TCP_FLAGS] = TCP_FLAGS_SYN | TCP_FLAGS_ACK;
            ++socketNew->ack;
            packetOut[TCP_DATA] = 2;    //maximum segment size = 536
            packetOut[TCP_DATA+1] = 4;
            packetOut[TCP_DATA+2] = 2;
            packetOut[TCP_DATA+3] = 24;
            TCPSendPacket(socketNew, frameOut, TCP_DATA+4);
            ++socketNew->seq;

            //Add socket to linked list
            OS_MutexPend(IPMutex);
            socketNew->next = SocketHead;
            socketNew->prev = NULL;
            if(SocketHead)
               SocketHead->prev = socketNew;
            SocketHead = socketNew;
            OS_MutexPost(IPMutex);
            return 0;
         }
      }
      return 0;
   }

   //Find an open socket
   for(socket = SocketHead; socket; socket = socket->next)
   {
      if(packet[IP_PROTOCOL] == socket->headerRcv[IP_PROTOCOL] &&
         memcmp(packet+IP_SOURCE, socket->headerRcv+IP_SOURCE, 8) == 0 &&
         memcmp(packet+TCP_SOURCE_PORT, socket->headerRcv+TCP_SOURCE_PORT, 4) == 0)
      {
         break;
      }
   }
   if(socket == NULL)
      return 0;

   //Check if FIN flag set
   if(packet[TCP_FLAGS] & TCP_FLAGS_FIN)
   {
      socket->timeout = SOCKET_TIMEOUT;
      if(IPVerbose)
         printf("F");
      frameOut = IPFrameGet(0);
      if(frameOut == NULL)
         return 0;
      packetOut = frameOut->packet;
      packetOut[TCP_FLAGS] = TCP_FLAGS_ACK;
      ++socket->ack;
      TCPSendPacket(socket, frameOut, TCP_DATA);
      if(socket->state == IP_FIN_SERVER)
         IPClose2(socket);
      else
      {
         socket->state = IP_FIN_CLIENT;
         if(socket->funcPtr)
            socket->funcPtr(socket);
      }
   }
   else if(packet[TCP_FLAGS] & TCP_FLAGS_RST)
   {
      if(socket->state == IP_FIN_SERVER)
         IPClose2(socket);
      else
      {
         socket->state = IP_FIN_CLIENT;
         if(socket->funcPtr)
            socket->funcPtr(socket);
      }
   }
   else
   {
      //Check if packets can be removed from retransmition list
      if(ack != socket->seqReceived)
      {
         OS_MutexPend(IPMutex);
         for(frame2 = FrameResendHead; frame2; )
         {
            framePrev = frame2;
            frame2 = frame2->next;
            if(framePrev->socket == socket && (int)(ack - framePrev->seqEnd) >= 0)
            {
               //Remove packet from retransmition queue
               if(socket->timeout)
                  socket->timeout = SOCKET_TIMEOUT;
               FrameRemove(&FrameResendHead, &FrameResendTail, framePrev);
               FrameFree(framePrev);
            }
         }
         OS_MutexPost(IPMutex);
         socket->seqReceived = ack;
      }

      bytes = ip_length - (TCP_DATA - IP_VERSION_LENGTH);

      //Check if SYN/ACK
      if((packet[TCP_FLAGS] & (TCP_FLAGS_SYN | TCP_FLAGS_ACK)) == 
         (TCP_FLAGS_SYN | TCP_FLAGS_ACK))
      {
         //Ack SYN/ACK
         socket->ack = seq + 1;
         frameOut = IPFrameGet(FRAME_COUNT_SEND);
         if(frameOut)
         {
            frameOut->packet[TCP_FLAGS] = TCP_FLAGS_ACK;
            TCPSendPacket(socket, frameOut, TCP_DATA);
         }
         if(socket->funcPtr)
            socket->funcPtr(socket);
         return 0;
      }
      else if(packet[TCP_HEADER_LENGTH] != 0x50)
      {
         if(IPVerbose)
            printf("length error\n");
         return 0;
      }

      //Copy packet into socket
      if(socket->ack == seq && bytes > 0)
      {
         //Insert packet into socket linked list
         if(socket->timeout)
            socket->timeout = SOCKET_TIMEOUT;
         if(IPVerbose)
            printf("D");
         FrameInsert(&socket->frameReadHead, &socket->frameReadTail, frameIn);
         socket->ack += bytes;

         //Ack data
         frameOut = IPFrameGet(FRAME_COUNT_SEND);
         if(frameOut)
         {
            frameOut->packet[TCP_FLAGS] = TCP_FLAGS_ACK;
            TCPSendPacket(socket, frameOut, TCP_DATA);
         }

         //Notify application
         if(socket->funcPtr)
            socket->funcPtr(socket);
         //Using frame
         return 1;     
      }

      if(bytes)
      {
         //Ack with current offset since data missing
         frameOut = IPFrameGet(FRAME_COUNT_SEND);
         if(frameOut)
         {
            frameOut->packet[TCP_FLAGS] = TCP_FLAGS_ACK;
            TCPSendPacket(socket, frameOut, TCP_DATA);
         }
      }
   }
   return 0;
}


int IPProcessEthernetPacket(IPFrame *frameIn)
{
   int ip_length, rc;
   IPSocket *socket;
   IPFrame *frameOut;
   uint8 *packet, *packetOut;

   packet = frameIn->packet;

   if(packet[ETHERNET_FRAME_TYPE] != 0x08 || frameIn->length > PACKET_SIZE)
      return 0;  //wrong ethernet type, packet not used

   //ARP?
   if(packet[ETHERNET_FRAME_TYPE+1] == 0x06)
   {
      //Check if ARP reply
      if(memcmp(packet+ETHERNET_DEST, ethernetAddressPlasma, 6) == 0 &&
         packet[ARP_OP+1] == 2 && memcmp(packet+ARP_IP_SENDER, ipAddressGateway, 4) == 0)
      {
         //Found MAC address for gateway
         memcpy(ethernetAddressGateway, packet+ARP_ETHERNET_SENDER, 6);
         return 0;
      }

      //Check if ARP request
      if(memcmp(packet+ETHERNET_DEST, ethernetAddressNull, 6) ||
         packet[ARP_OP] != 0 || packet[ARP_OP+1] != 1 ||  
         memcmp(packet+ARP_IP_TARGET, ipAddressPlasma, 4))
         return 0;
      //Create ARP response
      frameOut = IPFrameGet(0);
      if(frameOut == NULL)
         return 0;
      packetOut = frameOut->packet;
      memcpy(packetOut, packet, frameIn->length);
      memcpy(packetOut+ETHERNET_DEST, packet+ETHERNET_SOURCE, 6);
      memcpy(packetOut+ETHERNET_SOURCE, ethernetAddressPlasma, 6);
      packetOut[ARP_OP+1] = 2; //ARP reply
      memcpy(packetOut+ARP_ETHERNET_SENDER, ethernetAddressPlasma, 6);
      memcpy(packetOut+ARP_IP_SENDER, packet+ARP_IP_TARGET, 4);
      memcpy(packetOut+ARP_ETHERNET_TARGET, packet+ARP_ETHERNET_SENDER, 6);
      memcpy(packetOut+ARP_IP_TARGET, packet+ARP_IP_SENDER, 4);
      IPSendPacket(NULL, frameOut, frameIn->length);
      return 0;
   }

   //Check if proper type of packet
   ip_length = (packet[IP_LENGTH] << 8) | packet[IP_LENGTH+1];
   if(frameIn->length < UDP_DATA || ip_length > frameIn->length - IP_VERSION_LENGTH)
      return 0;
   if(packet[ETHERNET_FRAME_TYPE+1] != 0x00 ||
      packet[IP_VERSION_LENGTH] != 0x45)
      return 0;

   //Check if DHCP reply
   if(packet[IP_PROTOCOL] == 0x11 &&
      packet[UDP_SOURCE_PORT] == 0 && packet[UDP_SOURCE_PORT+1] == 67 &&
      packet[UDP_DEST_PORT] == 0 && packet[UDP_DEST_PORT+1] == 68)
   {
      IPDhcp(packet, frameIn->length, 2);            //DHCP reply
      return 0;
   }

   //Check if correct destination address
   if(memcmp(packet+ETHERNET_DEST, ethernetAddressPlasma, 6) ||
      memcmp(packet+IP_DEST, ipAddressPlasma, 4))
      return 0;
   rc = EthernetVerifyChecksums(packet, frameIn->length);
   //if(rc)
   //{
   //   printf("C ");
   //   return;
   //}

   //PING request?
   if(packet[IP_PROTOCOL] == 1)
   {
      if(packet[PING_TYPE] != 8)
         return 0;
      frameOut = IPFrameGet(FRAME_COUNT_SEND);
      if(frameOut == NULL)
         return 0;
      packetOut = frameOut->packet;
      EthernetCreateResponse(packetOut, packet, frameIn->length);
      frameOut->packet[PING_TYPE] = 0;       //PING reply
      IPSendPacket(NULL, frameOut, frameIn->length);
      return 0;
   }

   //TCP packet?
   if(packet[IP_PROTOCOL] == 0x06)
   {
      return IPProcessTCPPacket(frameIn);
   }

   //UDP packet?
   if(packet[IP_PROTOCOL] == 0x11)
   {
      //Find open socket
      for(socket = SocketHead; socket; socket = socket->next)
      {
         if(packet[IP_PROTOCOL] == socket->headerRcv[IP_PROTOCOL] &&
            memcmp(packet+IP_SOURCE, socket->headerRcv+IP_SOURCE, 8) == 0 &&
            memcmp(packet+UDP_SOURCE_PORT, socket->headerRcv+UDP_SOURCE_PORT, 2) == 0)
         {
            break;
         }
      }

      if(socket == NULL)
      {
         //Find listening socket
         for(socket = SocketHead; socket; socket = socket->next)
         {
            if(packet[IP_PROTOCOL] == socket->headerRcv[IP_PROTOCOL] &&
               memcmp(packet+UDP_DEST_PORT, socket->headerRcv+UDP_DEST_PORT, 2) == 0)
            {
               break;
            }
         }
      }

      if(socket)
      {
         if(IPVerbose)
            printf("U");
         FrameInsert(&socket->frameReadHead, &socket->frameReadTail, frameIn);
         socket->funcPtr(socket);
         return 1;
      }
   }
   return 0;
}


void IPMainThread(void *Arg)
{
   uint32 message[4];
   int rc;
   IPFrame *frame, *frameOut=NULL;
   uint32 ticks, ticksLast;
   (void)Arg;

   ticksLast = OS_ThreadTime();
   memset(message, 0, sizeof(message));

   for(;;)
   {
      Led(0);
      rc = OS_MQueueGet(IPMQueue, message, 10);
      if(rc == 0)
      {
         frame = (IPFrame*)message[1];
         if(message[0] == 0)       //frame received
         {
            Led(1);
            frame->length = (uint16)message[2];
            rc = IPProcessEthernetPacket(frame);
            if(rc == 0)
               FrameFree(frame);
         }
         else if(message[0] == 1)  //frame sent
         {
            Led(2);
            assert(frame == frameOut);
            IPFrameReschedule(frame);
            frameOut = NULL;
         }
         else if(message[0] == 2)  //frame ready to send
         {
         }
      }

      if(frameOut == NULL)
      {
         OS_MutexPend(IPMutex);
         frameOut = FrameSendTail;
         if(frameOut)
            FrameRemove(&FrameSendHead, &FrameSendTail, frameOut);
         OS_MutexPost(IPMutex);
         if(frameOut)
         {
            Led(4);
            UartPacketSend(frameOut->packet, frameOut->length);
         }
      }

      ticks = OS_ThreadTime();
      if(ticks - ticksLast > 100)
      {
         IPTick();
         ticksLast = ticks;
      }
   }
}


uint8 *MyPacketGet(void)
{
   return (uint8*)IPFrameGet(FRAME_COUNT_RCV);
}


//Set FrameSendFunction only if single threaded
void IPInit(IPFuncPtr FrameSendFunction)
{
   int i;
   IPFrame *frame;

   FrameSendFunc = FrameSendFunction;
   IPMutex = OS_MutexCreate("IPSem");
   IPMQueue = OS_MQueueCreate("IPMQ", FRAME_COUNT*2, 32);
   for(i = 0; i < FRAME_COUNT; ++i)
   {
      frame = (IPFrame*)malloc(sizeof(IPFrame));
      memset(frame, 0, sizeof(IPFrame));
      frame->next = FrameFreeHead;
      frame->prev = NULL;
      FrameFreeHead = frame;
   }
   UartPacketConfig(MyPacketGet, PACKET_SIZE, IPMQueue);
   if(FrameSendFunction == NULL)
      IPThread = OS_ThreadCreate("TCP/IP", IPMainThread, NULL, 240, 6000);

   IPDhcp(NULL, 360, 1);        //Send DHCP request
}


//To open a socket for listen set IPAddress to 0
IPSocket *IPOpen(IPMode_e Mode, uint32 IPAddress, uint32 Port, IPFuncPtr funcPtr)
{
   IPSocket *socket;
   uint8 *ptrSend, *ptrRcv;
   IPFrame *frame;
   static int portSource=0x1007;
   (void)Mode;
   (void)IPAddress;

   socket = (IPSocket*)malloc(sizeof(IPSocket));
   if(socket == NULL)
      return socket;
   memset(socket, 0, sizeof(IPSocket));
   socket->prev = NULL;
   socket->state = IP_LISTEN;
   socket->timeout = 0;
   socket->frameReadHead = NULL;
   socket->frameReadTail = NULL;
   socket->readOffset = 0;
   socket->funcPtr = funcPtr;
   socket->userData = 0;
   socket->userFunc = NULL;
   socket->userPtr = NULL;
   ptrSend = socket->headerSend;
   ptrRcv = socket->headerRcv;

   if(IPAddress == 0)
   {
      //Setup listing port
      socket->headerRcv[TCP_DEST_PORT] = (uint8)(Port >> 8);
      socket->headerRcv[TCP_DEST_PORT+1] = (uint8)Port;
   }
   else
   {
      //Setup sending packet
      memset(ptrSend, 0, UDP_LENGTH);
      memset(ptrRcv, 0, UDP_LENGTH);

      //Setup Ethernet
      memcpy(ptrSend+ETHERNET_DEST, ethernetAddressGateway, 6);
      memcpy(ptrSend+ETHERNET_SOURCE, ethernetAddressPlasma, 6);
      ptrSend[ETHERNET_FRAME_TYPE] = 0x08;

      //Setup IP
      ptrSend[IP_VERSION_LENGTH] = 0x45;
      ptrSend[IP_TIME_TO_LIVE] = 0x80;

      //Setup IP addresses
      memcpy(ptrSend+IP_SOURCE, ipAddressPlasma, 4);
      ptrSend[IP_DEST] = (uint8)(IPAddress >> 24);
      ptrSend[IP_DEST+1] = (uint8)(IPAddress >> 16);
      ptrSend[IP_DEST+2] = (uint8)(IPAddress >> 8);
      ptrSend[IP_DEST+3] = (uint8)IPAddress;
      ptrRcv[IP_SOURCE] = (uint8)(IPAddress >> 24);
      ptrRcv[IP_SOURCE+1] = (uint8)(IPAddress >> 16);
      ptrRcv[IP_SOURCE+2] = (uint8)(IPAddress >> 8);
      ptrRcv[IP_SOURCE+3] = (uint8)IPAddress;
      memcpy(ptrRcv+IP_DEST, ipAddressPlasma, 4);

      //Setup ports
      ptrSend[TCP_SOURCE_PORT] = (uint8)(portSource >> 8);
      ptrSend[TCP_SOURCE_PORT+1] = (uint8)portSource;
      ptrSend[TCP_DEST_PORT] = (uint8)(Port >> 8);
      ptrSend[TCP_DEST_PORT+1] = (uint8)Port;
      ptrRcv[TCP_SOURCE_PORT] = (uint8)(Port >> 8);
      ptrRcv[TCP_SOURCE_PORT+1] = (uint8)Port;
      ptrRcv[TCP_DEST_PORT] = (uint8)(portSource >> 8);
      ptrRcv[TCP_DEST_PORT+1] = (uint8)portSource;
      ++portSource;
   }

   if(Mode == IP_MODE_TCP)
   {
      if(IPAddress)
         socket->state = IP_TCP;
      else
         socket->state = IP_LISTEN;
      ptrSend[IP_PROTOCOL] = 0x06;  //TCP
      ptrRcv[IP_PROTOCOL] = 0x06;
   }
   else if(Mode == IP_MODE_UDP)
   {
      socket->state = IP_UDP;
      ptrSend[IP_PROTOCOL] = 0x11;  //UDP
      ptrRcv[IP_PROTOCOL] = 0x11; 
   }

   //Add socket to linked list
   OS_MutexPend(IPMutex);
   socket->next = SocketHead;
   socket->prev = NULL;
   if(SocketHead)
      SocketHead->prev = socket;
   SocketHead = socket;
   OS_MutexPost(IPMutex);

   if(Mode == IP_MODE_TCP && IPAddress)
   {
      //Send TCP SYN
      frame = IPFrameGet(0);
      if(frame)
      {
         frame->packet[TCP_FLAGS] = TCP_FLAGS_SYN;
         frame->packet[TCP_DATA] = 2;    //maximum segment size = 536
         frame->packet[TCP_DATA+1] = 4;
         frame->packet[TCP_DATA+2] = 2;
         frame->packet[TCP_DATA+3] = 24;
         TCPSendPacket(socket, frame, TCP_DATA+4);
         ++socket->seq;
      }
   }
   return socket;
}


void IPWriteFlush(IPSocket *Socket)
{
   uint8 *packetOut;
   if(Socket->frameSend && Socket->state != IP_UDP)
   {
      packetOut = Socket->frameSend->packet;
      packetOut[TCP_FLAGS] = TCP_FLAGS_ACK;
      TCPSendPacket(Socket, Socket->frameSend, TCP_DATA + Socket->sendOffset);
      Socket->seq += Socket->sendOffset;
      Socket->frameSend = NULL;
      Socket->sendOffset = 0;
   }
}


uint32 IPWrite(IPSocket *Socket, const uint8 *Buf, uint32 Length)
{
   IPFrame *frameOut;
   uint8 *packetOut;
   uint32 bytes, count=0;
   int offset;

   //printf("IPWrite(0x%x, %d)", Socket, Length);
   while(Length)
   {
      if(Socket->frameSend == NULL)
      {
         Socket->frameSend = IPFrameGet(FRAME_COUNT_SEND);
         Socket->sendOffset = 0;
      }
      frameOut = Socket->frameSend;
      offset = Socket->sendOffset;
      if(frameOut == NULL)
         break;
      packetOut = frameOut->packet;
      bytes = 512 - offset;
      if(bytes > Length)
         bytes = Length;
      Socket->sendOffset += bytes;

      if(Socket->state != IP_UDP)
      {
         memcpy(packetOut+TCP_DATA+offset, Buf, bytes);
         if(Socket->sendOffset >= 512)
            IPWriteFlush(Socket);
      }
      else  //UDP
      {
         memcpy(packetOut+UDP_DATA+offset, Buf, bytes);
         memcpy(packetOut, Socket->headerSend, UDP_LENGTH);
         IPSendPacket(Socket, Socket->frameSend, UDP_DATA + Socket->sendOffset);
         Socket->frameSend = NULL;
      }
      count += bytes;
      Buf += bytes;
      Length -= bytes;
   }
   return count;
}


void IPWritePend(IPSocket *Socket, uint8 *Buf, uint32 Length)
{
   int bytes;
   OS_Thread_t *self;

   self = OS_ThreadSelf();
   assert(self != IPThread);
   while(Length)
   {
      bytes = IPWrite(Socket, Buf, Length);
      Buf += bytes;
      Length -= bytes;
      if(Length)
         OS_ThreadSleep(1);
   }
}


uint32 IPRead(IPSocket *Socket, uint8 *Buf, uint32 Length)
{
   IPFrame *frame, *frame2;
   int count=0, bytes, offset;

   if(Socket->state == IP_TCP)
      offset = TCP_DATA;
   else
      offset = UDP_DATA;

   OS_MutexPend(IPMutex);
   for(frame = Socket->frameReadTail; Length && frame; )
   {
      bytes = frame->length - offset - Socket->readOffset;
      if(bytes > (int)Length)
         bytes = Length;
      memcpy(Buf, frame->packet + offset + Socket->readOffset, bytes);
      Buf += bytes;
      Socket->readOffset += bytes;
      Length -= bytes;
      count += bytes;

      //Check if done with packet
      frame2 = frame;
      frame = frame->prev;
      if(Socket->readOffset == frame2->length - offset)
      {
         //Remove packet from socket linked list
         Socket->readOffset = 0;
         FrameRemove(&Socket->frameReadHead, &Socket->frameReadTail, frame2);
         FrameFree(frame2);
      }
   }
   OS_MutexPost(IPMutex);
   return count;
}


static void IPClose2(IPSocket *Socket)
{
   IPFrame *frame, *framePrev;

   OS_MutexPend(IPMutex);

   //Mark packets as don't retransmit
   for(frame = FrameSendHead; frame; frame = frame->next)
   {
      if(frame->socket == Socket)
         frame->socket = NULL;
   }

   //Remove packets from retransmision list
   for(frame = FrameResendHead; frame; )
   {
      framePrev = frame;
      frame = frame->next;
      if(framePrev->socket == Socket)
      {
         FrameRemove(&FrameResendHead, &FrameResendTail, framePrev);
         FrameFree(framePrev);
      }
   }

   //Remove packets from socket read linked list
   for(frame = Socket->frameReadHead; frame; )
   {
      framePrev = frame;
      frame = frame->next;
      FrameRemove(&Socket->frameReadHead, &Socket->frameReadTail, framePrev);
      FrameFree(framePrev);
   }

   //Remove socket
   if(Socket->prev == NULL)
      SocketHead = Socket->next;
   else
      Socket->prev->next = Socket->next;
   if(Socket->next)
      Socket->next->prev = Socket->prev;
   free(Socket);
   OS_MutexPost(IPMutex);
}


void IPClose(IPSocket *Socket)
{
   IPFrame *frameOut;

   IPWriteFlush(Socket);
   if(Socket->state == IP_UDP)
   {
      IPClose2(Socket);
      return;
   }
   frameOut = IPFrameGet(0);
   if(frameOut == 0)
      return;
   frameOut->packet[TCP_FLAGS] = TCP_FLAGS_FIN | TCP_FLAGS_ACK;
   TCPSendPacket(Socket, frameOut, TCP_DATA);
   ++Socket->seq;
   if(Socket->state == IP_FIN_CLIENT)
      IPClose2(Socket);
   else
      Socket->state = IP_FIN_SERVER;
}


//static void ShowIP(IPSocket *socket, uint32 ipAddress)
//{
//   (void)socket;
//   printf("IP=0x%x\n", ipAddress);
//}


void IPTick(void)
{
   IPFrame *frame, *frame2;
   IPSocket *socket, *socket2;

   if(IPVerbose && (Seconds % 60) == 0)
   {
      if(FrameFreeCount == FRAME_COUNT)
         printf("T");
      else
         printf("T(%d)", FrameFreeCount);
   }
   ++Seconds;
   if(--DhcpRetrySeconds <= 0)
      IPDhcp(NULL, 400, 1);   //DHCP request
   //if(Seconds == 10)
   //   IPResolve("plasmacpu.no-ip.org", ShowIP);

   OS_MutexPend(IPMutex);

   //Retransmit timeout packets
   for(frame = FrameResendHead; frame; )
   {
      frame2 = frame;
      frame = frame->next;
      if(--frame2->timeout == 0)
      {
         if(IPVerbose)
            printf("r");
         FrameRemove(&FrameResendHead, &FrameResendTail, frame2);
         IPSendFrame(frame2);
      }
   }

   //Close timed out sockets
   for(socket = SocketHead; socket; )
   {
      socket2 = socket;
      socket = socket->next;
      if(socket2->timeout && --socket2->timeout == 0)
      {
         socket2->timeout = 10;
         if(IPVerbose)
            printf("t(%d)", socket2->state);
         if(socket2->state == IP_TCP)
            IPClose(socket2);
         else if(socket2->state == IP_FIN_CLIENT)
            IPClose(socket2);
         else
            IPClose2(socket2);
      }
   }
   OS_MutexPost(IPMutex);
}


static void DnsCallback(IPSocket *socket)
{
   uint8 buf[200], *ptr;
   uint32 ipAddress;
   int i, length;

   memset(buf, 0, sizeof(buf));
   IPRead(socket, buf, sizeof(buf));
   if(buf[DNS_NUM_ANSWERS_RR+1])
   {
      ptr = buf + DNS_QUESTIONS;
      while(*ptr)
         ++ptr;
      ++ptr;
      ptr += 4;
      for(i = 0; (int)(ptr - buf) < sizeof(buf) && i < buf[DNS_NUM_ANSWERS_RR+1]; ++i)
      {
         if(ptr[2] == 0 && ptr[3] == 1 && ptr[4] == 0 && ptr[5] == 1)
         {
            ipAddress = (ptr[12] << 24) | (ptr[13] << 16) | (ptr[14] << 8) | ptr[15];
            printf("ipAddress = %d.%d.%d.%d\n", ptr[12], ptr[13], ptr[14], ptr[16]);
            socket->userData = ipAddress;
            if(socket->userFunc)
            {
               socket->userFunc(socket, ipAddress);
            }
            break;
         }
         length = (ptr[10] << 8) | ptr[11];
         ptr += 12 + length;
      }
   }
   if(FrameSendFunc)
      IPClose(socket);
}


uint32 IPResolve(char *Name, IPFuncPtr resolvedFunc)
{
   uint8 buf[200], *ptr;
   int length, i;
   IPSocket *socket;
   uint32 ipAddress=0;

   socket = IPOpen(IP_MODE_UDP, ipAddressDns, DNS_PORT, DnsCallback);
   memset(buf, 0, sizeof(buf));
   buf[DNS_ID+1] = 1;
   buf[DNS_FLAGS] = 1;
   buf[DNS_NUM_QUESTIONS+1] = 1;

   //Setup name
   ptr = buf + DNS_QUESTIONS;
   strncpy((char*)ptr+1, Name, 100);
   ptr[0] = 1;
   while(ptr[0])
   {
      for(i = 0; i < 100; ++i)
      {
         if(ptr[i+1] == '.' || ptr[i+1] == 0)
         {
            ptr[0] = (uint8)i;
            ptr += i+1;
            break;
         }
      }
   }
   ++ptr;
   ptr[1] = DNS_QUERY_TYPE_IP;
   ptr[3] = DNS_QUERY_CLASS;
   length = (int)(ptr - buf) + 4;
   if(length < 60)
      length = 60;

   socket->userFunc = (IPFuncPtr)resolvedFunc;
   socket->userData = 0;
   IPWrite(socket, buf, length);

   if(FrameSendFunc == NULL)
   {
      for(i = 0; i < 1000 && socket->userData == 0; ++i)
         OS_ThreadSleep(1);
      ipAddress = socket->userData;
      IPClose(socket);
   }
   return ipAddress;
}
