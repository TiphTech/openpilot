// Exercise the production TX/forwarding hooks with independent timer clocks.
#include <stdbool.h>
#include <string.h>
#include "fake_stm.h"
#include "can.h"
void putui(uint32_t value) { (void)value; }
bool safety_tx_buffered_for_fwd = false;
#include "faults.h"
#include "safety.h"

#define EXPORT

EXPORT void alt2_test_init(int param) {
  set_safety_hooks(SAFETY_HYUNDAI_CANFD, param);
  relay_malfunction = false;
}

EXPORT int cluster_test_packet(int address, int bus, int length, uint8_t *data, uint32_t now,
                               bool tx, bool extended, bool relay_fault) {
  timer.CNT = now;
  relay_malfunction = relay_fault;
  CANPacket_t pkt = {0};
  pkt.addr = address;
  pkt.bus = bus;
  pkt.fd = 1U;
  pkt.extended = extended;
  for (unsigned int i = 0U; i < 16U; i++) {
    if (dlc_to_len[i] == length) pkt.data_len_code = i;
  }
  memcpy(pkt.data, data, length);
  int result = tx ? (int)safety_tx_hook(&pkt) : safety_fwd_hook(&pkt);
  memcpy(data, pkt.data, length);
  return result;
}

EXPORT void fwd_timer_set_tick(uint32_t tick) {
  safety_mode_cnt = tick;
}
