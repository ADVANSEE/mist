/*
 * Copyright (c) 2013, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "mist.h"
#include "propple-socket.h"

#include "node-id.h"

static struct propple_socket s;

PROCESS(propple_example_process, "Propple example");
AUTOSTART_PROCESSES(&propple_example_process);
/*---------------------------------------------------------------------------*/
static void
receive_callback(struct propple_socket *s,
                 void *ptr,
                 uint8_t seqno,
                 const uint8_t *data,
                 uint16_t datalen)
{
  printf("propple-example: received data '%s' with seqno %d datalen %d\n",
         data,
         seqno, datalen);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(propple_example_process, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();
  propple_socket_open(&s, NULL, receive_callback,
                      7878,
                      CLOCK_SECOND * 8,
                      CLOCK_SECOND * 64,
                      1);

  if(node_id == 1) {
    static struct etimer e;
    char buf[10];
    static int count;

    while(1) {
      snprintf(buf, sizeof(buf), "hej %d", count++);
      propple_socket_send(&s, (uint8_t *)buf, strlen(buf) + 1);

      etimer_set(&e, CLOCK_SECOND * 60);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&e));
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
