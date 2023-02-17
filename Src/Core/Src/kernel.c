/*
 * kernel.c
 *
 *  Created on: 12 Feb 2023
 *      Author: Amila Abeygunasekara
 */

#include "kernel.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_cortex.h"

OSThread* volatile osCurr; // Pointer to the current thread
OSThread* volatile osNext; // Pointer to the next thread

void OS_init(void)
{
  // Updating the System handler priority register 3 (SHPR3) register
  NVIC_SetPriority(SysTick_IRQn, 0x00); // SysTick priority = 0 (high)
  NVIC_SetPriority(PendSV_IRQn, 0x0F);  // PendSV priority = 15 (low)
}

void OS_sched(void)
{
  extern OSThread blinky1;
  extern OSThread blinky2;

  if (osCurr == &blinky1)
  {
    osNext = &blinky2;
  }
  else
  {
    osNext = &blinky1;
  }

  if (osNext != osCurr)
  {
    // Trigger the PendSV interrupt
    *(uint32_t volatile *)0xE000ED04 = (1U << 28);
  }
}

void OSThread_start(
    OSThread* thread,
    OSThreadHandler threadHandler,
    void* stkSto, uint32_t stkSize)
{
  uint32_t* sp = (uint32_t*)((((uint32_t)stkSto + stkSize) / 8) * 8);
  uint32_t* stkLimit;

  // Fabricate Cortex-M ISR stack frame for blinky1
  *(--sp) = (1U << 24); // xPSR
  *(--sp) = (uint32_t)threadHandler; //PC
  *(--sp) = 0x0000000EU; // LR
  *(--sp) = 0x0000000CU; // R12
  *(--sp) = 0x00000003U; // R3
  *(--sp) = 0x00000002U; // R2
  *(--sp) = 0x00000001U; // R1
  *(--sp) = 0x00000000U; // R0
  // Additionally, fake registers for R4-R11
  *(--sp) = 0x0000000BU; // R11
  *(--sp) = 0x0000000AU; // R10
  *(--sp) = 0x00000009U; // R9
  *(--sp) = 0x00000008U; // R8
  *(--sp) = 0x00000007U; // R7
  *(--sp) = 0x00000006U; // R6
  *(--sp) = 0x00000005U; // R5
  *(--sp) = 0x00000004U; // R4

  // Save the top of the stack in the thread's attribute
  thread->sp = sp;

  // Round up the bottom of the stack to the 8-byte boundary
  stkLimit = (uint32_t*)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

  // Pre-fill the unused part of the stack with 0xDEADBEEF
  for (sp = sp - 1U; sp >= stkLimit; --sp)
  {
    *sp = 0xDEADBEEFU;
  }
}

__attribute__ ((naked))
void PendSV_Handler(void)
{
__asm volatile (
  // __disable_irq();
  "  cpsid        i                 \n"

  // if (osCurr != (OSThread *)0) {
  "  ldr          r1,=osCurr        \n"
  "  ldr          r1,[r1,#0x00]     \n"
  "  cbz          r1,PendSV_restore \n"

  // Push registers r4-r11 on the stack
  "  push         {r4-r11}          \n"

  // osCurr->sp = sp;
  "  ldr          r1,=osCurr        \n"
  "  ldr          r1,[r1,#0x00]     \n"
  "  str          sp,[r1,#0x00]     \n"
  // }

  "PendSV_restore:                  \n"
  // sp = osNext->sp;
  "  ldr          r1,=osNext        \n"
  "  ldr          r1,[r1,#0x00]     \n"
  "  ldr          sp,[r1,#0x00]     \n"

  // osCurr = osNext;
  "  ldr          r1,=osNext        \n"
  "  ldr          r1,[r1,#0x00]     \n"
  "  ldr          r2,=osCurr        \n"
  "  str          r1,[r2,#0x00]     \n"

  // pop registers r4-r11
  "  pop          {r4-r11}          \n"

  // __enable_irq();
  "  cpsie        i                 \n"

  // return to the next thread
  "  bx           lr                \n");
}
