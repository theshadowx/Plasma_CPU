/*--------------------------------------------------------------------
 * TITLE: Plasma Real Time Operating System
 * AUTHOR: Steve Rhoads (rhoadss@yahoo.com)
 * DATE CREATED: 12/17/05
 * FILENAME: rtos.c
 * PROJECT: Plasma CPU core
 * COPYRIGHT: Software placed into the public domain by the author.
 *    Software 'as is' without warranty.  Author liable for nothing.
 * DESCRIPTION:
 *    Plasma Real Time Operating System
 *    Fully pre-emptive RTOS with support for:
 *       Heaps, Threads, Semaphores, Mutexes, Message Queues, and Timers.
 *    This file tries to be hardware independent except for calls to:
 *       MemoryRead() and MemoryWrite() for interrupts.
 *    Partial support for multiple CPUs using symmetric multiprocessing.
 *--------------------------------------------------------------------*/
#include "plasma.h"
#include "rtos.h"

#define HEAP_MAGIC 0x1234abcd
#define THREAD_MAGIC 0x4321abcd
#define SEM_RESERVED_COUNT 2
#define HEAP_COUNT 8


/*************** Structures ***************/
#ifdef WIN32
#define setjmp _setjmp
   //x86 registers
   typedef struct jmp_buf2 {  
      uint32 Ebp, Ebx, Edi, Esi, sp, pc, extra[10];
   } jmp_buf2;
#else  
   //Plasma registers
   typedef struct jmp_buf2 {  
      uint32 s[9], gp, sp, pc;
   } jmp_buf2;
#endif

typedef struct HeapNode_s {
   struct HeapNode_s *next;
   int size;
} HeapNode_t;

struct OS_Heap_s {
   uint32 magic;
   const char *name;
   OS_Semaphore_t *semaphore;
   HeapNode_t *available;
   HeapNode_t base;
   struct OS_Heap_s *alternate;
};
//typedef struct OS_Heap_s OS_Heap_t;

typedef enum {
   THREAD_PEND    = 0,
   THREAD_READY   = 1,
   THREAD_RUNNING = 2
} OS_ThreadState_e;

struct OS_Thread_s {
   const char *name;
   OS_ThreadState_e state;
   int cpuLock;
   jmp_buf env;
   OS_FuncPtr_t funcPtr;
   void *arg;
   uint32 priority;
   uint32 ticksTimeout;
   void *info;
   OS_Semaphore_t *semaphorePending;
   int returnCode;
   uint32 spinLocks;
   struct OS_Thread_s *next, *prev;
   struct OS_Thread_s *nextTimeout, *prevTimeout;
   uint32 magic[1];
};
//typedef struct OS_Thread_s OS_Thread_t;

struct OS_Semaphore_s {
   const char *name;
   struct OS_Thread_s *threadHead;
   int count;
};
//typedef struct OS_Semaphore_s OS_Semaphore_t;

struct OS_Mutex_s {
   OS_Semaphore_t *semaphore;
   OS_Thread_t *thread;
   int count;
}; 
//typedef struct OS_Mutex_s OS_Mutex_t;

struct OS_MQueue_s {
   const char *name;
   OS_Semaphore_t *semaphore;
   int count, size, used, read, write;
};
//typedef struct OS_MQueue_s OS_MQueue_t;

struct OS_Timer_s {
   const char *name;
   struct OS_Timer_s *next, *prev;
   uint32 ticksTimeout;
   uint32 ticksRestart;
   int active;
   OS_MQueue_t *mqueue;
   uint32 info;
}; 
//typedef struct OS_Timer_s OS_Timer_t;


/*************** Globals ******************/
static OS_Heap_t *HeapArray[HEAP_COUNT];
static OS_Semaphore_t *SemaphoreSleep;
static OS_Semaphore_t *SemaphoreLock;
static int ThreadSwapEnabled;
static int ThreadNeedReschedule;
static uint32 ThreadTime;
static OS_Thread_t *ThreadHead;   //Linked list of threads sorted by priority
static OS_Thread_t *TimeoutHead;  //Linked list of threads sorted by timeout
static OS_Thread_t *ThreadCurrent[OS_CPU_COUNT];
static void *NeedToFree;
static OS_Semaphore_t SemaphoreReserved[SEM_RESERVED_COUNT];
static OS_Timer_t *TimerHead;     //Linked list of timers sorted by timeout
static OS_Semaphore_t *SemaphoreTimer;
static OS_FuncPtr_t Isr[32];
static int InterruptInside;


/***************** Heap *******************/
/******************************************/
OS_Heap_t *OS_HeapCreate(const char *Name, void *Memory, uint32 Size)
{
   OS_Heap_t *heap;

   assert(((uint32)Memory & 3) == 0);
   heap = (OS_Heap_t*)Memory;
   heap->magic = HEAP_MAGIC;
   heap->name = Name;
   heap->semaphore = OS_SemaphoreCreate(Name, 1);
   heap->available = (HeapNode_t*)(heap + 1);
   heap->available->next = &heap->base;
   heap->available->size = (Size - sizeof(OS_Heap_t)) / sizeof(HeapNode_t);
   heap->base.next = heap->available;
   heap->base.size = 0;
   return heap;
}


/******************************************/
void OS_HeapDestroy(OS_Heap_t *Heap)
{
   OS_SemaphoreDelete(Heap->semaphore);
}


/******************************************/
//Modified from K&R
void *OS_HeapMalloc(OS_Heap_t *Heap, int Bytes)
{
   HeapNode_t *node, *prevp;
   int nunits;

   if((int)Heap < HEAP_COUNT)
      Heap = HeapArray[(int)Heap];
   //printf("OS_HeapMalloc(%s, %d)\n", Heap->name, Bytes);
   nunits = (Bytes + sizeof(HeapNode_t) - 1) / sizeof(HeapNode_t) + 1;
   OS_SemaphorePend(Heap->semaphore, OS_WAIT_FOREVER);
   prevp = Heap->available;
   for(node = prevp->next; ; prevp = node, node = node->next)
   {
      if(node->size >= nunits)       //Big enough?
      {
         if(node->size == nunits)    //Exactly
            prevp->next = node->next;
         else
         {                           //Allocate tail end
            node->size -= nunits;
            node += node->size;
            node->size = nunits;
         }
         Heap->available = prevp;
         node->next = (HeapNode_t*)Heap;
         OS_SemaphorePost(Heap->semaphore);
         //printf("ptr=0x%x\n", (uint32)(node + 1));
         return (void*)(node + 1);
      }
      if(node == Heap->available)   //Wrapped around free list
      {
         OS_SemaphorePost(Heap->semaphore);
         if(Heap->alternate)
            return OS_HeapMalloc(Heap->alternate, Bytes);
         return NULL;
      }
   }
}


/******************************************/
//Modified from K&R
void OS_HeapFree(void *Block)
{
   OS_Heap_t *heap;
   HeapNode_t *bp, *node;

   assert(Block);
   //printf("OS_HeapFree(0x%x)\n", Block);
   bp = (HeapNode_t*)Block - 1;   //point to block header
   heap = (OS_Heap_t*)bp->next;
   assert(heap->magic == HEAP_MAGIC);
   if(heap->magic != HEAP_MAGIC)
      return;
   OS_SemaphorePend(heap->semaphore, OS_WAIT_FOREVER);
   for(node = heap->available; !(node < bp && bp < node->next); node = node->next)
   {
      if(node >= node->next && (bp > node || bp < node->next))
         break;               //freed block at start or end of area
   }

   if(bp + bp->size == node->next)   //join to upper
   {
      bp->size += node->next->size;
      bp->next = node->next->next;
   }
   else
   {
      bp->next = node->next;
   }

   if(node + node->size == bp)       //join to lower
   {
      node->size += bp->size;
      node->next = bp->next;
   }
   else
      node->next = bp;
   heap->available = node;
   OS_SemaphorePost(heap->semaphore);
}


/******************************************/
void OS_HeapAlternate(OS_Heap_t *Heap, OS_Heap_t *Alternate)
{
   Heap->alternate = Alternate;
}


/******************************************/
void OS_HeapRegister(void *Index, OS_Heap_t *Heap)
{
   if((int)Index < HEAP_COUNT)
      HeapArray[(int)Index] = Heap;
}



/***************** Thread *****************/
/******************************************/
//Linked list of threads sorted by priority
//Must be called with interrupts disabled
static void OS_ThreadPriorityInsert(OS_Thread_t **head, OS_Thread_t *thread)
{
   OS_Thread_t *node, *prev;

   prev = NULL;
   for(node = *head; node; node = node->next)
   {
      if(node->priority <= thread->priority)
         break;
      prev = node;
   }

   if(prev == NULL)
   {
      thread->next = *head;
      thread->prev = NULL;
      *head = thread;
   }
   else
   {
      if(prev->next)
         prev->next->prev = thread;
      thread->next = prev->next;
      thread->prev = prev;
      prev->next = thread;
   }
   assert(ThreadHead);
   if(*head == ThreadHead)
      thread->state = THREAD_READY;
}


/******************************************/
//Must be called with interrupts disabled
static void OS_ThreadPriorityRemove(OS_Thread_t **head, OS_Thread_t *thread)
{
   assert(thread->magic[0] == THREAD_MAGIC);  //check stack overflow
   if(thread->prev == NULL)
      *head = thread->next;
   else
      thread->prev->next = thread->next;
   if(thread->next)
      thread->next->prev = thread->prev;
   thread->next = NULL;
   thread->prev = NULL;
   thread->state = THREAD_PEND;
}


/******************************************/
//Linked list of threads sorted by timeout value
//Must be called with interrupts disabled
static void OS_ThreadTimeoutInsert(OS_Thread_t *thread)
{
   OS_Thread_t *node, *prev;
   int diff;

   prev = NULL;
   for(node = TimeoutHead; node; node = node->nextTimeout)
   {
      diff = thread->ticksTimeout - node->ticksTimeout;
      if(diff <= 0)
         break;
      prev = node;
   }

   if(prev == NULL)
   {
      thread->nextTimeout = TimeoutHead;
      thread->prevTimeout = NULL;
      if(TimeoutHead)
         TimeoutHead->prevTimeout = thread;
      TimeoutHead = thread;
   }
   else
   {
      if(prev->nextTimeout)
         prev->nextTimeout->prevTimeout = thread;
      thread->nextTimeout = prev->nextTimeout;
      thread->prevTimeout = prev;
      prev->nextTimeout = thread;
   }
}


/******************************************/
//Must be called with interrupts disabled
static void OS_ThreadTimeoutRemove(OS_Thread_t *thread)
{
   if(thread->prevTimeout == NULL && TimeoutHead != thread)
      return;         //not in list
   if(thread->prevTimeout == NULL)
      TimeoutHead = thread->nextTimeout;
   else
      thread->prevTimeout->nextTimeout = thread->nextTimeout;
   if(thread->nextTimeout)
      thread->nextTimeout->prevTimeout = thread->prevTimeout;
   thread->nextTimeout = NULL;
   thread->prevTimeout = NULL;
}


#if OS_CPU_COUNT <= 1
/******************************************/
//Loads a new thread 
//Must be called with interrupts disabled
static void OS_ThreadReschedule(int RoundRobin)
{
   OS_Thread_t *threadNext, *threadCurrent, *threadTry;
   int rc;

   if(ThreadSwapEnabled == 0 || InterruptInside)
   {
      ThreadNeedReschedule |= 2 + RoundRobin;
      return;
   }

   //Determine which thread should run
   threadCurrent = ThreadCurrent[0];
   threadNext = threadCurrent;
   if(threadCurrent == NULL || threadCurrent->state == THREAD_PEND)
      threadNext = ThreadHead;
   else if(threadCurrent->priority < ThreadHead->priority)
      threadNext = ThreadHead;
   else if(RoundRobin)
   {
      //Determine next ready thread with same priority
      threadTry = threadCurrent->next;
      if(threadTry && threadTry->priority == threadCurrent->priority)
         threadNext = threadTry;
      else
         threadNext = ThreadHead;
   }

   if(threadNext != threadCurrent)
   {
      //Swap threads
      ThreadCurrent[0] = threadNext;
      assert(threadNext);
      if(threadCurrent)
      {
         assert(threadCurrent->magic[0] == THREAD_MAGIC); //check stack overflow
         //printf("OS_ThreadRescheduleSave(%s)\n", threadPrev->name);
         rc = setjmp(threadCurrent->env);  //ANSI C call to save registers
         if(rc)
         {
            //Returned from longjmp()
            return;
         }
      }

      threadNext = ThreadCurrent[0];       //removed warning
      //printf("OS_ThreadRescheduleRestore(%s)\n", threadNext->name);
      longjmp(threadNext->env, 1);         //ANSI C call to restore registers
   }
}

#else //#if OS_CPU_COUNT <= 1

/******************************************/
//Loads a new thread in a multiprocessor environment
//Must be called with interrupts disabled
static void OS_ThreadReschedule(int RoundRobin)
{
   OS_Thread_t *threadNext, *threadCurrent, *threadBest, *threadAlt;
   uint32 i, cpuLow, priorityLow, cpuIndex = OS_CpuIndex();
   int rc;
   static volatile int CpuReschedule;    //remove warning

   if(ThreadSwapEnabled == 0 || InterruptInside)
   {
      ThreadNeedReschedule |= 2 + RoundRobin;
      return;
   }

   //Find which CPU is running the lowest priority thread
   CpuReschedule = -1;
   cpuLow = 0;
   priorityLow = 0xffffffff;
   for(i = 0; i < OS_CPU_COUNT; ++i)
   {
      if(i != cpuIndex && (ThreadCurrent[i] == NULL || 
         ThreadCurrent[i]->priority < priorityLow))
      {
         cpuLow = i;
         if(ThreadCurrent[i])
            priorityLow = ThreadCurrent[i]->priority;
         else
            priorityLow = 0;
      }
   }

   //Determine highest priority ready threads
   for(threadBest = ThreadHead; threadBest; threadBest = threadBest->next)
   {
      if(threadBest->state == THREAD_READY && 
         (threadBest->cpuLock == -1 || threadBest->cpuLock == (int)cpuIndex))
         break;
   }
   threadAlt = NULL;
   if(threadBest)
   {
      for(threadAlt = threadBest->next; threadAlt; threadAlt = threadAlt->next)
      {
         if(threadAlt->state == THREAD_READY &&
            (threadAlt->cpuLock == -1 || threadAlt->cpuLock == (int)cpuIndex))
            break;
      }
   }

   //Determine if a CPU needs to reschedule
   if(threadAlt && threadAlt->priority > priorityLow)
      CpuReschedule = cpuLow;

   //Determine which thread should run
   threadCurrent = ThreadCurrent[cpuIndex];
   threadNext = threadCurrent;
   if(threadCurrent == NULL || threadCurrent->state == THREAD_PEND)
   {
      threadNext = threadBest;
   }
   else if(threadBest && threadCurrent->priority < threadBest->priority)
   {
      threadNext = threadBest;

      if(threadAlt == NULL || threadCurrent->priority > threadAlt->priority)
      {
         CpuReschedule = cpuLow;

         //Only enable this if there really are multiple CPUs
         //Try to minimize thread swapping by detecting if the other
         //CPU would just start running threadCurrent
//       if(threadCurrent->priority > priorityLow)
//          threadNext = threadCurrent;
      }
   }
   else if(RoundRobin)
   {
      //Find the next ready thread
      for(threadAlt = threadCurrent->next; threadAlt; threadAlt = threadAlt->next)
      {
         if(threadAlt->state == THREAD_READY &&
            (threadAlt->cpuLock == -1 || threadAlt->cpuLock == (int)cpuIndex))
            break;
      }
      if(threadAlt && threadAlt->priority == threadCurrent->priority)
         threadNext = threadAlt;
      else if(threadBest && threadBest->priority >= threadCurrent->priority)
         threadNext = threadBest;
   }

   if(threadNext != threadCurrent)
   {
      //Swap threads
      ThreadCurrent[cpuIndex] = threadNext;
      assert(threadNext);
      if(threadCurrent)
      {
         assert(threadCurrent->magic[0] == THREAD_MAGIC); //check stack overflow
         threadCurrent->state = THREAD_READY;
         threadCurrent->spinLocks = OS_SpinCountGet();

         //printf("OS_ThreadRescheduleSave(%s)\n", threadPrev->name);
         rc = setjmp(threadCurrent->env);   //ANSI C call to save registers
         if(rc)
         {
            //Returned from longjmp()
            if(CpuReschedule >= 0)
               OS_CpuInterrupt(CpuReschedule, 1);
            return;
         }
      }

      //Restore spin lock count
      cpuIndex = OS_CpuIndex();             //removed warning
      threadNext = ThreadCurrent[cpuIndex]; //removed warning
      threadNext->state = THREAD_RUNNING;
      OS_SpinCountSet(threadNext->spinLocks);

      //printf("OS_ThreadRescheduleRestore(%s)\n", threadNext->name);
      longjmp(threadNext->env, 1);          //ANSI C call to restore registers
   }

   //Check if a different CPU needs to swap threads
   if(CpuReschedule >= 0)
      OS_CpuInterrupt(CpuReschedule, 1);
}


void OS_ThreadCpuLock(OS_Thread_t *Thread, int CpuIndex)
{
   Thread->cpuLock = CpuIndex;
   if(Thread == OS_ThreadSelf() && CpuIndex != (int)OS_CpuIndex())
      OS_ThreadSleep(1);
}

#endif //#if OS_CPU_COUNT <= 1


/******************************************/
static void OS_ThreadInit(void *Arg)
{
   uint32 cpuIndex = OS_CpuIndex();
   (void)Arg;

   OS_CriticalEnd(1);
   ThreadCurrent[cpuIndex]->funcPtr(ThreadCurrent[cpuIndex]->arg);
   OS_ThreadExit();
}


/******************************************/
//Stops warning "argument X might be clobbered by `longjmp'"
static void OS_ThreadRegsInit(jmp_buf env)
{
   setjmp(env); //ANSI C call to save registers
}


/******************************************/
OS_Thread_t *OS_ThreadCreate(const char *Name,
                             OS_FuncPtr_t FuncPtr, 
                             void *Arg, 
                             uint32 Priority, 
                             uint32 StackSize)
{
   OS_Thread_t *thread;
   uint8 *stack;
   jmp_buf2 *env;
   uint32 state;

   OS_SemaphorePend(SemaphoreLock, OS_WAIT_FOREVER);
   if(NeedToFree)
      OS_HeapFree(NeedToFree);
   NeedToFree = NULL;
   OS_SemaphorePost(SemaphoreLock);

   if(StackSize == 0)
      StackSize = STACK_SIZE_DEFAULT;
   if(StackSize < STACK_SIZE_MINIMUM)
      StackSize = STACK_SIZE_MINIMUM;
   thread = (OS_Thread_t*)OS_HeapMalloc(NULL, sizeof(OS_Thread_t) + StackSize);
   assert(thread);
   if(thread == NULL)
      return NULL;
   stack = (uint8*)(thread + 1);
   memset(stack, 0xcd, StackSize);

   thread->name = Name;
   thread->state = THREAD_READY;
   thread->cpuLock = -1;
   thread->funcPtr = FuncPtr;
   thread->arg = Arg;
   thread->priority = Priority;
   thread->info = NULL;
   thread->semaphorePending = NULL;
   thread->returnCode = 0;
   thread->spinLocks = 1;
   thread->next = NULL;
   thread->prev = NULL;
   thread->nextTimeout = NULL;
   thread->prevTimeout = NULL;
   thread->magic[0] = THREAD_MAGIC;

   OS_ThreadRegsInit(thread->env);
   env = (jmp_buf2*)thread->env;
   env->sp = (uint32)stack + StackSize - 4;
   env->pc = (uint32)OS_ThreadInit;

   state = OS_CriticalBegin();
   OS_ThreadPriorityInsert(&ThreadHead, thread);
   OS_ThreadReschedule(0);
   OS_CriticalEnd(state);
   return thread;
}


/******************************************/
void OS_ThreadExit(void)
{
   uint32 state, cpuIndex = OS_CpuIndex();
   OS_SemaphorePend(SemaphoreLock, OS_WAIT_FOREVER);
   if(NeedToFree)
      OS_HeapFree(NeedToFree);
   NeedToFree = NULL;
   OS_SemaphorePost(SemaphoreLock);

   state = OS_CriticalBegin();
   OS_ThreadPriorityRemove(&ThreadHead, ThreadCurrent[cpuIndex]);
   NeedToFree = ThreadCurrent[cpuIndex];
   OS_ThreadReschedule(0);
   assert(ThreadHead == NULL);
   OS_CriticalEnd(state);
}


/******************************************/
OS_Thread_t *OS_ThreadSelf(void)
{
   return ThreadCurrent[OS_CpuIndex()];
}


/******************************************/
void OS_ThreadSleep(int Ticks)
{
   OS_SemaphorePend(SemaphoreSleep, Ticks);
}


/******************************************/
uint32 OS_ThreadTime(void)
{
   return ThreadTime;
}


/******************************************/
void OS_ThreadInfoSet(OS_Thread_t *Thread, void *Info)
{
   Thread->info = Info;
}


/******************************************/
void *OS_ThreadInfoGet(OS_Thread_t *Thread)
{
   return Thread->info;
}


/******************************************/
uint32 OS_ThreadPriorityGet(OS_Thread_t *Thread)
{
   return Thread->priority;
}


/******************************************/
void OS_ThreadPrioritySet(OS_Thread_t *Thread, uint32 Priority)
{
   uint32 state;
   state = OS_CriticalBegin();
   Thread->priority = Priority;
   if(Thread->state != THREAD_PEND)
   {
      OS_ThreadPriorityRemove(&ThreadHead, Thread);
      OS_ThreadPriorityInsert(&ThreadHead, Thread);
      OS_ThreadReschedule(0);
   }
   OS_CriticalEnd(state);
}


/******************************************/
//Must be called with interrupts disabled
void OS_ThreadTick(void *Arg)
{
   OS_Thread_t *thread;
   OS_Semaphore_t *semaphore;
   int diff;
   (void)Arg;

   ++ThreadTime;
   while(TimeoutHead)
   {
      thread = TimeoutHead;
      diff = ThreadTime - thread->ticksTimeout;
      if(diff < 0)
         break;
      OS_ThreadTimeoutRemove(thread);
      semaphore = thread->semaphorePending;
      ++semaphore->count;
      thread->semaphorePending = NULL;
      thread->returnCode = -1;
      OS_ThreadPriorityRemove(&semaphore->threadHead, thread);
      OS_ThreadPriorityInsert(&ThreadHead, thread);
   }
   OS_ThreadReschedule(1);
}



/***************** Semaphore **************/
/******************************************/
OS_Semaphore_t *OS_SemaphoreCreate(const char *Name, uint32 Count)
{
   OS_Semaphore_t *semaphore;
   static int semCount;

   if(semCount < SEM_RESERVED_COUNT)
      semaphore = &SemaphoreReserved[semCount++];
   else
      semaphore = (OS_Semaphore_t*)OS_HeapMalloc(NULL, sizeof(OS_Semaphore_t));
   assert(semaphore);
   if(semaphore == NULL)
      return NULL;

   semaphore->name = Name;
   semaphore->threadHead = NULL;
   semaphore->count = Count;
   return semaphore;
}


/******************************************/
void OS_SemaphoreDelete(OS_Semaphore_t *Semaphore)
{
   while(Semaphore->threadHead)
      OS_SemaphorePost(Semaphore);
   OS_HeapFree(Semaphore);
}


/******************************************/
int OS_SemaphorePend(OS_Semaphore_t *Semaphore, int Ticks)
{
   uint32 state, cpuIndex;
   OS_Thread_t *thread;
   int returnCode=0;

   //assert(OS_SpinCountGet() == 0);
   assert(Semaphore);
   assert(InterruptInside == 0);
   state = OS_CriticalBegin();
   if(--Semaphore->count < 0)
   {
      cpuIndex = OS_CpuIndex();
      assert(ThreadCurrent[cpuIndex]);
      if(Ticks == 0)
      {
         ++Semaphore->count;
         OS_CriticalEnd(state);
         return -1;
      }
      thread = ThreadCurrent[cpuIndex];
      thread->semaphorePending = Semaphore;
      thread->ticksTimeout = Ticks + OS_ThreadTime();
      OS_ThreadPriorityRemove(&ThreadHead, thread);
      OS_ThreadPriorityInsert(&Semaphore->threadHead, thread);
      if(Ticks != OS_WAIT_FOREVER)
         OS_ThreadTimeoutInsert(thread);
      assert(ThreadHead);
      OS_ThreadReschedule(0);
      returnCode = thread->returnCode;
   }
   OS_CriticalEnd(state);
   //assert(OS_SpinCountGet() == 0);
   return returnCode;
}


/******************************************/
void OS_SemaphorePost(OS_Semaphore_t *Semaphore)
{
   uint32 state;
   OS_Thread_t *thread;

   assert(Semaphore);
   state = OS_CriticalBegin();
   if(++Semaphore->count <= 0)
   {
      thread = Semaphore->threadHead;
      OS_ThreadTimeoutRemove(thread);
      OS_ThreadPriorityRemove(&Semaphore->threadHead, thread);
      OS_ThreadPriorityInsert(&ThreadHead, thread);
      thread->semaphorePending = NULL;
      thread->returnCode = 0;
      OS_ThreadReschedule(0);
   }
   OS_CriticalEnd(state);
}



/***************** Mutex ******************/
/******************************************/
OS_Mutex_t *OS_MutexCreate(const char *Name)
{
   OS_Mutex_t *mutex;

   mutex = (OS_Mutex_t*)OS_HeapMalloc(NULL, sizeof(OS_Mutex_t));
   if(mutex == NULL)
      return NULL;
   mutex->semaphore = OS_SemaphoreCreate(Name, 1);
   if(mutex->semaphore == NULL)
      return NULL;
   mutex->thread = NULL;
   mutex->count = 0;
   return mutex;
}


/******************************************/
void OS_MutexDelete(OS_Mutex_t *Mutex)
{
   OS_SemaphoreDelete(Mutex->semaphore);
   OS_HeapFree(Mutex);
}


/******************************************/
void OS_MutexPend(OS_Mutex_t *Mutex)
{
   OS_Thread_t *thread;

   assert(Mutex);
   thread = OS_ThreadSelf();
   if(thread == Mutex->thread)
   {
      ++Mutex->count;
      return;
   }
   OS_SemaphorePend(Mutex->semaphore, OS_WAIT_FOREVER);
   Mutex->thread = thread;
   Mutex->count = 1;
}


/******************************************/
void OS_MutexPost(OS_Mutex_t *Mutex)
{
   assert(Mutex);
   assert(Mutex->thread == OS_ThreadSelf());
   assert(Mutex->count > 0);
   if(--Mutex->count <= 0)
   {
      Mutex->thread = NULL;
      OS_SemaphorePost(Mutex->semaphore);
   }
}



/***************** MQueue *****************/
/******************************************/
OS_MQueue_t *OS_MQueueCreate(const char *Name,
                             int MessageCount,
                             int MessageBytes)
{
   OS_MQueue_t *queue;
   int size;

   size = MessageBytes / sizeof(uint32);
   queue = (OS_MQueue_t*)OS_HeapMalloc(NULL, sizeof(OS_MQueue_t) + 
      MessageCount * size * 4);
   if(queue == NULL)
      return queue;
   queue->name = Name;
   queue->semaphore = OS_SemaphoreCreate(Name, 0);
   if(queue->semaphore == NULL)
      return NULL;
   queue->count = MessageCount;
   queue->size = size;
   queue->used = 0;
   queue->read = 0;
   queue->write = 0;
   return queue;
}


/******************************************/
void OS_MQueueDelete(OS_MQueue_t *MQueue)
{
   OS_SemaphoreDelete(MQueue->semaphore);
   OS_HeapFree(MQueue);
}


/******************************************/
int OS_MQueueSend(OS_MQueue_t *MQueue, void *Message)
{
   uint32 state, *dst, *src;
   int i;

   assert(MQueue);
   src = (uint32*)Message;
   state = OS_CriticalBegin();
   if(++MQueue->used > MQueue->count)
   {
      --MQueue->used;
      OS_CriticalEnd(state);
      return -1;
   }
   dst = (uint32*)(MQueue + 1) + MQueue->write * MQueue->size;
   for(i = 0; i < MQueue->size; ++i)
      dst[i] = src[i];
   if(++MQueue->write >= MQueue->count)
      MQueue->write = 0;
   OS_CriticalEnd(state);
   OS_SemaphorePost(MQueue->semaphore);
   return 0;
}


/******************************************/
int OS_MQueueGet(OS_MQueue_t *MQueue, void *Message, int Ticks)
{
   uint32 state, *dst, *src;
   int i, rc;

   assert(MQueue);
   dst = (uint32*)Message;
   rc = OS_SemaphorePend(MQueue->semaphore, Ticks);
   if(rc)
      return rc;
   state = OS_CriticalBegin();
   --MQueue->used;
   src = (uint32*)(MQueue + 1) + MQueue->read * MQueue->size;
   for(i = 0; i < MQueue->size; ++i)
      dst[i] = src[i];
   if(++MQueue->read >= MQueue->count)
      MQueue->read = 0;
   OS_CriticalEnd(state);
   return 0;
}



/***************** Timer ******************/
/******************************************/
static void OS_TimerThread(void *Arg)
{
   uint32 timeNow;
   int diff, ticks;
   uint32 message[8];
   OS_Timer_t *timer;
   (void)Arg;

   timeNow = OS_ThreadTime();
   for(;;)
   {
      //Determine how long to sleep
      OS_SemaphorePend(SemaphoreLock, OS_WAIT_FOREVER);
      if(TimerHead)
         ticks = TimerHead->ticksTimeout - timeNow;
      else
         ticks = OS_WAIT_FOREVER;
      OS_SemaphorePost(SemaphoreLock);
      OS_SemaphorePend(SemaphoreTimer, ticks);

      //Send messages for all timed out timers
      timeNow = OS_ThreadTime();
      for(;;)
      {
         OS_SemaphorePend(SemaphoreLock, OS_WAIT_FOREVER);
         if(TimerHead == NULL)
         {
            OS_SemaphorePost(SemaphoreLock);
            break;
         }
         diff = timeNow - TimerHead->ticksTimeout;
         if(diff < 0)
         {
            OS_SemaphorePost(SemaphoreLock);
            break;
         }
         timer = TimerHead;
         OS_SemaphorePost(SemaphoreLock);
         if(timer->ticksRestart)
            OS_TimerStart(timer, timer->ticksRestart, timer->ticksRestart);
         else
            OS_TimerStop(timer);

         //Send message
         message[0] = MESSAGE_TYPE_TIMER;
         message[1] = (uint32)timer;
         message[2] = (uint32)timer->info;
         OS_MQueueSend(timer->mqueue, message);
      }
   }
}


/******************************************/
OS_Timer_t *OS_TimerCreate(const char *Name, OS_MQueue_t *MQueue, uint32 Info)
{
   OS_Timer_t *timer;
   int startThread=0;

   OS_SemaphorePend(SemaphoreLock, OS_WAIT_FOREVER);
   if(SemaphoreTimer == NULL)
   {
      SemaphoreTimer = OS_SemaphoreCreate("Timer", 0);
      startThread = 1;
   }
   OS_SemaphorePost(SemaphoreLock);
   if(startThread)
      OS_ThreadCreate("Timer", OS_TimerThread, NULL, 250, 2000);

   timer = (OS_Timer_t*)OS_HeapMalloc(NULL, sizeof(OS_Timer_t));
   if(timer == NULL)
      return NULL;
   timer->name = Name;
   timer->mqueue = MQueue;
   timer->next = NULL;
   timer->prev = NULL;
   timer->info = Info;
   timer->active = 0;
   return timer;
}


/******************************************/
void OS_TimerDelete(OS_Timer_t *Timer)
{
   OS_TimerStop(Timer);
   OS_HeapFree(Timer);
}


/******************************************/
//Must not be called from an ISR
void OS_TimerStart(OS_Timer_t *Timer, uint32 Ticks, uint32 TicksRestart)
{
   OS_Timer_t *node, *prev;
   int diff, check=0;

   assert(Timer);
   assert(InterruptInside == 0);
   Ticks += OS_ThreadTime();
   if(Timer->active)
      OS_TimerStop(Timer);
   OS_SemaphorePend(SemaphoreLock, OS_WAIT_FOREVER);
   Timer->ticksTimeout = Ticks;
   Timer->ticksRestart = TicksRestart;
   Timer->active = 1;
   prev = NULL;
   for(node = TimerHead; node; node = node->next)
   {
      diff = Ticks - node->ticksTimeout;
      if(diff <= 0)
         break;
      prev = node;
   }
   Timer->next = node;
   Timer->prev = prev;
   if(node)
      node->prev = Timer;
   if(prev == NULL)
   {
      TimerHead = Timer;
      check = 1;
   }
   else
      prev->next = Timer;
   OS_SemaphorePost(SemaphoreLock);
   if(check)
      OS_SemaphorePost(SemaphoreTimer);
}


/******************************************/
//Must not be called from an ISR
void OS_TimerStop(OS_Timer_t *Timer)
{
   assert(Timer);
   assert(InterruptInside == 0);
   OS_SemaphorePend(SemaphoreLock, OS_WAIT_FOREVER);
   if(Timer->active)
   {
      Timer->active = 0;
      if(Timer->prev == NULL)
         TimerHead = Timer->next;
      else
         Timer->prev->next = Timer->next;
      if(Timer->next)
         Timer->next->prev = Timer->prev;
   }
   OS_SemaphorePost(SemaphoreLock);
}


/***************** ISR ********************/
/******************************************/
void OS_InterruptServiceRoutine(uint32 Status, uint32 *Stack)
{
   int i;
   uint32 state;

   //MemoryWrite(GPIO0_OUT, Status);  //Change LEDs
   InterruptInside = 1;
   if(Stack && (Stack[STACK_EPC] & 1) && Isr[31]) //BREAK or SYSCALL
      Isr[31](Stack);
   i = 0;
   do
   {   
      if(Status & 1)
      {
         if(Isr[i])
            Isr[i]((uint32*)i);
         else
            OS_InterruptMaskClear(1 << i);
      }
      Status >>= 1;
      ++i;
   } while(Status);
   InterruptInside = 0;
   //MemoryWrite(GPIO0_OUT, 0);

   state = OS_SpinLock();
   if(ThreadNeedReschedule)
      OS_ThreadReschedule(ThreadNeedReschedule & 1);
   OS_SpinUnlock(state);
}


/******************************************/
void OS_InterruptRegister(uint32 Mask, OS_FuncPtr_t FuncPtr)
{
   int i;

   for(i = 0; i < 32; ++i)
   {
      if(Mask & (1 << i))
         Isr[i] = FuncPtr;
   }
}


/******************************************/
//Plasma hardware dependent
uint32 OS_InterruptStatus(void)
{
   return MemoryRead(IRQ_STATUS);
}


/******************************************/
//Plasma hardware dependent
uint32 OS_InterruptMaskSet(uint32 Mask)
{
   uint32 mask, state;
   state = OS_CriticalBegin();
   mask = MemoryRead(IRQ_MASK) | Mask;
   MemoryWrite(IRQ_MASK, mask);
   OS_CriticalEnd(state);
   return mask;
}


/******************************************/
//Plasma hardware dependent
uint32 OS_InterruptMaskClear(uint32 Mask)
{
   uint32 mask, state;
   state = OS_CriticalBegin();
   mask = MemoryRead(IRQ_MASK) & ~Mask;
   MemoryWrite(IRQ_MASK, mask);
   OS_CriticalEnd(state);
   return mask;
}


/**************** Init ********************/
/******************************************/
static volatile uint32 IdleCount;
static void OS_IdleThread(void *Arg)
{
   (void)Arg;

   //Don't block in the idle thread!
   for(;;)
   {
      ++IdleCount;
   }
}


/******************************************/
#ifndef DISABLE_IRQ_SIM
static void OS_IdleSimulateIsr(void *Arg)
{
   uint32 count=0, value;
   (void)Arg;

   for(;;)
   {
      MemoryRead(IRQ_MASK + 4);       //calls Sleep(10)
#if WIN32
      while(OS_InterruptMaskSet(0) & IRQ_UART_WRITE_AVAILABLE)
         OS_InterruptServiceRoutine(IRQ_UART_WRITE_AVAILABLE, 0);
#endif
      value = OS_InterruptMaskSet(0);
      OS_InterruptServiceRoutine(value, 0);
      ++count;
   }
}
#endif //DISABLE_IRQ_SIM


/******************************************/
//Plasma hardware dependent
void OS_ThreadTick2(void *Arg)
{
   uint32 status, mask, state;

   state = OS_SpinLock();
   status = MemoryRead(IRQ_STATUS) & (IRQ_COUNTER18 | IRQ_COUNTER18_NOT);
   mask = MemoryRead(IRQ_MASK) | IRQ_COUNTER18 | IRQ_COUNTER18_NOT;
   mask &= ~status;
   MemoryWrite(IRQ_MASK, mask);
   OS_ThreadTick(Arg);
   OS_SpinUnlock(state);
}


/******************************************/
void OS_Init(uint32 *HeapStorage, uint32 Bytes)
{
   int i;
   OS_AsmInterruptInit();               //Patch interrupt vector
   OS_InterruptMaskClear(0xffffffff);   //Disable interrupts
   HeapArray[0] = OS_HeapCreate("Default", HeapStorage, Bytes);
   SemaphoreSleep = OS_SemaphoreCreate("Sleep", 0);
   SemaphoreLock = OS_SemaphoreCreate("Lock", 1);
   for(i = 0; i < OS_CPU_COUNT; ++i)
      OS_ThreadCreate("Idle", OS_IdleThread, NULL, 0, 256);
#ifndef DISABLE_IRQ_SIM
   if((OS_InterruptStatus() & (IRQ_COUNTER18 | IRQ_COUNTER18_NOT)) == 0)
   {
      //Detected that running in simulator so create SimIsr thread
      UartPrintfCritical("SimIsr\n");
      OS_ThreadCreate("SimIsr", OS_IdleSimulateIsr, NULL, 1, 0);
   }
#endif //DISABLE_IRQ_SIM

   //Plasma hardware dependent
   OS_InterruptRegister(IRQ_COUNTER18 | IRQ_COUNTER18_NOT, OS_ThreadTick2);
   OS_InterruptMaskSet(IRQ_COUNTER18 | IRQ_COUNTER18_NOT);
}


/******************************************/
void OS_Start(void)
{
   ThreadSwapEnabled = 1;
   (void)OS_SpinLock();
   OS_ThreadReschedule(1);
}


/******************************************/
//Place breakpoint here
void OS_Assert(void)
{
}


#if OS_CPU_COUNT > 1
static uint8 SpinLockArray[OS_CPU_COUNT];
/******************************************/
uint32 OS_CpuIndex(void)
{
   return 0; //0 to OS_CPU_COUNT-1
}


/******************************************/
//Symmetric Multiprocessing Spin Lock Mutex
uint32 OS_SpinLock(void)
{
   uint32 state, cpuIndex, i, j, ok, delay;

   cpuIndex = OS_CpuIndex();
   delay = cpuIndex + 8;
   state = OS_AsmInterruptEnable(0);
   do
   {
      ok = 1;
      if(++SpinLockArray[cpuIndex] == 1)
      {
         for(i = 0; i < OS_CPU_COUNT; ++i)
         {
            if(i != cpuIndex && SpinLockArray[i])
               ok = 0;
         }
         if(ok == 0)
         {
            SpinLockArray[cpuIndex] = 0;
            for(j = 0; j < delay; ++j)  //wait a bit
               ++i;
            if(delay < 128)
               delay <<= 1;
         }
      }
   } while(ok == 0);
   return state;
}


/******************************************/
void OS_SpinUnlock(uint32 state)
{
   uint32 cpuIndex;
   cpuIndex = OS_CpuIndex();
   if(--SpinLockArray[cpuIndex] == 0)
      OS_AsmInterruptEnable(state);

   assert(SpinLockArray[cpuIndex] < 10);
   assert(!(state && SpinLockArray[cpuIndex]));
}


/******************************************/
//Must be called with interrupts disabled and spin locked
uint32 OS_SpinCountGet(void)
{
   uint32 cpuIndex, count;
   cpuIndex = OS_CpuIndex();
   count = SpinLockArray[cpuIndex];
   //assert(count);
   return count;
}


/******************************************/
//Must be called with interrupts disabled and spin locked
void OS_SpinCountSet(uint32 count)
{
   uint32 cpuIndex;
   cpuIndex = OS_CpuIndex();
   SpinLockArray[cpuIndex] = (uint8)count;
   assert(count);
}


/******************************************/
void OS_CpuInterrupt(uint32 cpuIndex, uint32 bitfield)
{
   (void)cpuIndex;
   (void)bitfield;
}


/******************************************/
void OS_CpuInterruptServiceRoutine(void *Arg)
{
   uint32 state;
   (void)Arg;
   state = OS_SpinLock();
   OS_ThreadReschedule(0);
   OS_SpinUnlock(state);
}
#endif


/************** WIN32 Support *************/
#ifdef WIN32
//Support RTOS inside Windows
extern int kbhit();
extern int getch(void);
extern int putch(int);
extern void __stdcall Sleep(unsigned long value);

static uint32 Memory[8];

uint32 MemoryRead(uint32 Address)
{
   Memory[2] |= IRQ_UART_WRITE_AVAILABLE;
   switch(Address)
   {
   case UART_READ: 
      if(kbhit())
         Memory[0] = getch();
      Memory[2] &= ~IRQ_UART_READ_AVAILABLE; //clear bit
      return Memory[0];
   case IRQ_MASK: 
      return Memory[1];
   case IRQ_MASK + 4:
      Sleep(10);
      return 0;
   case IRQ_STATUS: 
      if(kbhit())
         Memory[2] |= IRQ_UART_READ_AVAILABLE;
      return Memory[2];
   }
   return 0;
}

void MemoryWrite(uint32 Address, uint32 Value)
{
   switch(Address)
   {
   case UART_WRITE: 
      putch(Value); 
      break;
   case IRQ_MASK:   
      Memory[1] = Value; 
      break;
   case IRQ_STATUS: 
      Memory[2] = Value; 
      break;
   }
}

uint32 OS_AsmInterruptEnable(uint32 EnableInterrupt)
{
   return EnableInterrupt;
}

void OS_AsmInterruptInit(void)
{
}
#endif  //WIN32


/**************** Example *****************/
#ifndef NO_MAIN
static uint8 HeapSpace[1024*512];

int main(void)
{
   UartPrintfCritical("Starting RTOS\n");
   OS_Init((uint32*)HeapSpace, sizeof(HeapSpace));
   UartInit();
   OS_ThreadCreate("Main", MainThread, 0, 100, 64000);
   OS_Start();
   return 0;
}
#endif

