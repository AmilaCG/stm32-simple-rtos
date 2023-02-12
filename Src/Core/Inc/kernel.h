/*
 * kernel.h
 *
 *  Created on: 12 Feb 2023
 *      Author: Amila Abeygunasekara
 */

#ifndef INC_KERNEL_H_
#define INC_KERNEL_H_

#include <stdint.h>

// Thread Control Block (TCB)
typedef struct
{
  void* sp; // Stack pointer
} OSThread;

typedef void (*OSThreadHandler)();

void OS_init(void);

// This function MUST be called with interrupts DISABLED
void OS_sched(void);

void OSThread_start(
    OSThread* thread,
    OSThreadHandler threadHandler,
    void* stkSto, uint32_t stkSize);

#endif /* INC_KERNEL_H_ */
