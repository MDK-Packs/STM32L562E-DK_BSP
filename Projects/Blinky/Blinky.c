/*------------------------------------------------------------------------------
 * Copyright (c) 2017 - 2020 Arm Limited (or its affiliates). All
 * rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2.Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   3.Neither the name of Arm nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *------------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 *----------------------------------------------------------------------------*/

#include "main.h"


static osThreadId_t tid_thrLED;         // Thread id of thread: LED
static osThreadId_t tid_thrButton;      // Thread id of thread: Button

/*------------------------------------------------------------------------------
  thrLED: blink LED
 *----------------------------------------------------------------------------*/
__NO_RETURN static void thrLED (void *arg) {
  uint32_t active_flag = 0U;

  (void)arg;

  for (;;) {
    if (osThreadFlagsWait (1U, osFlagsWaitAny, 0U) == 1U) {
      active_flag ^= 1U;
    }

    if (active_flag == 1U) {
      vioSetSignal (vioLED0, vioLEDoff);          // Switch LED0 off
      vioSetSignal (vioLED1, vioLEDon);           // Switch LED1 on
      osDelay (100U);                             // Delay 100 ms
      vioSetSignal (vioLED0, vioLEDon);           // Switch LEDs
      vioSetSignal (vioLED1, vioLEDoff);
      osDelay (100U);                             // Delay 100 ms
    }
    else {
      vioSetSignal (vioLED0, vioLEDon);           // Switch LED0 on
      osDelay (500U);                             // Delay 500 ms
      vioSetSignal (vioLED0, vioLEDoff);          // Switch LED0 off
      osDelay (500U);                             // Delay 500 ms
    }
  }
}

/*------------------------------------------------------------------------------
  thrButton: check Button state
 *----------------------------------------------------------------------------*/
__NO_RETURN static void thrButton(void *arg) {
  uint32_t last = 0U;
  uint32_t state;

  (void)arg;

  for (;;) {
    state = (vioGetSignal (vioBUTTON0));          // Get pressed Button state
    if (state != last) {
      if (state == 1) {
        osThreadFlagsSet (tid_thrLED, 1U);        // Set flag to thrLED
      }
      last = state;
    }
    osDelay (100U);
  }
}

/*------------------------------------------------------------------------------
 * Application main thread
 *----------------------------------------------------------------------------*/
__NO_RETURN void app_main (void *arg) {
  (void)arg;

  tid_thrLED = osThreadNew (thrLED, NULL, NULL);        // Create LED thread
  if (tid_thrLED == NULL) { /* add error handling */ }

  tid_thrButton = osThreadNew (thrButton, NULL, NULL);  // Create Button thread
  if (tid_thrButton == NULL) { /* add error handling */ }

  osThreadExit ();
}
