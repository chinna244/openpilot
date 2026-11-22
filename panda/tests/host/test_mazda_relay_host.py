"""Host checks of the Mazda relay sequencing in panda firmware: the real set_safety_mode, can_init and
can_init_all_checked source compiled with stubbed hardware (develop dd6a2c75d relay-after-CAN-init, Mazda only)."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
pytestmark = pytest.mark.skipif(shutil.which("gcc") is None, reason="needs a host gcc")


def func(path, sig):
  s = Path(path).read_text()
  i = s.index(sig)
  return s[i:s.index("\n}\n", i) + 3]


def run_c(tmp_path, source):
  c, exe = tmp_path / "t.c", tmp_path / "t"
  c.write_text(source)
  subprocess.run(["gcc", "-std=gnu11", "-Wall", "-Wno-unused-function", "-Wno-unused-variable", "-o", str(exe), str(c)], check=True)
  subprocess.run([str(exe)], check=True)


RELAY_PRELUDE = r'''#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#define SAFETY_SILENT 0U
#define SAFETY_NOOUTPUT 19U
#define SAFETY_ELM327 3U
#define SAFETY_TOYOTA 2U
#define SAFETY_MAZDA 13U
#define PERMANENT_FAULTS 0U
#define CAN_MODE_NORMAL 0U
#define CAN_MODE_OBD_CAN2 1U
#define print(x) ((void)0)
#define assert_fatal(c, m) assert(c)
static uint32_t faults = 0U;
static uint16_t current_safety_mode = 0xffffU;
static int set_safety_hooks(uint16_t mode, uint16_t param) { (void)param; current_safety_mode = mode; return 0; }
static bool relay_closed;
static void set_intercept_relay(bool intercept, bool ign) { (void)ign; relay_closed = intercept; }
typedef struct { void (*set_can_mode)(uint8_t); } board;
static void scm(uint8_t m) { (void)m; }
static board b = {scm}; static board *current_board = &b;
static bool can_silent, heartbeat_lost; static uint32_t heartbeat_counter, safety_tx_blocked, safety_rx_invalid, safety_mode_cnt;
static void can_clear_send(void *c, int n) { (void)c; (void)n; }
#define CANIF_FROM_CAN_NUM(n) ((void *)0)
static int init_failures;  /* how many calls of can_init_all_checked fail before one succeeds */
static int init_calls;
static bool init_silent[8];  /* can_silent as each init pass applied it to the controllers */
static bool init_relay[8];   /* the relay as each init pass ran */
static bool can_init_all_checked(void) { init_silent[init_calls] = can_silent; init_relay[init_calls] = relay_closed; init_calls++; return init_calls > init_failures; }
'''
RELAY_MAIN = r'''
static void run(uint16_t mode, bool relay_before, int failures) {
  relay_closed = relay_before; init_failures = failures; init_calls = 0; can_silent = false; current_safety_mode = 0xffffU;
  set_safety_mode(mode, 0U);
}
int main(void) {
  for (int before = 0; before < 2; before++) {
    run(SAFETY_MAZDA, before, 0); assert(relay_closed && current_safety_mode == SAFETY_MAZDA && !can_silent);
    assert(!init_relay[0]);  /* factory passthrough while the cores re-init, whatever the relay was */
    run(SAFETY_MAZDA, before, 1); assert(relay_closed && current_safety_mode == SAFETY_MAZDA && init_calls == 2);  /* retry works */
    run(SAFETY_MAZDA, before, 2); assert(!relay_closed && current_safety_mode == SAFETY_SILENT && can_silent);   /* fail safe */
    assert(init_calls == 3 && !init_silent[0] && !init_silent[1] && init_silent[2]);  /* the controllers get silent mode */
    assert(!init_relay[0] && !init_relay[1] && !init_relay[2]);  /* never closed during a failing init */
    run(SAFETY_MAZDA, before, 9); assert(!relay_closed && current_safety_mode == SAFETY_SILENT && init_calls == 3);  /* bounded */
    run(SAFETY_TOYOTA, before, 5); assert(relay_closed && current_safety_mode == SAFETY_TOYOTA && init_calls == 1); /* base behaviour */
    run(SAFETY_ELM327, before, 0); assert(!relay_closed);
  }
  printf("set_safety_mode relay harness OK\n");
  return 0;
}
'''


def test_mazda_relay_closes_after_init_and_fails_safe(tmp_path):
  """Both starting relay positions: Mazda closes after the cores come up (one retry), falls back to SILENT with the
  relay open when they do not; Toyota keeps base behaviour; ELM327 opens."""
  body = func(ROOT / "panda/board/main.c", "void set_safety_mode(uint16_t mode, uint16_t param) {")
  run_c(tmp_path, RELAY_PRELUDE + body + RELAY_MAIN)


@pytest.mark.parametrize("drv, typ", [("fdcan.h", "FDCAN_GlobalTypeDef"), ("bxcan.h", "CAN_TypeDef")])
def test_can_init_reports_success_and_failure(tmp_path, drv, typ):
  body = func(ROOT / f"panda/board/drivers/{drv}", "bool can_init(uint8_t can_number) {")
  checked = func(ROOT / "panda/board/drivers/can_common.h", "bool can_init_all_checked(void) {")
  source = f"""#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#define PANDA_CAN_CNT 3U
#define REGISTER_INTERRUPT(...)
typedef struct {{ int x; }} {typ};
static {typ} cores[3];
#define CANIF_FROM_CAN_NUM(n) (&cores[(n)])
typedef struct {{ bool canfd_enabled; }} bus_config_t;
static bus_config_t bus_config[3];
static int can_queues[3];
static void can_clear(int q) {{ (void)q; }}
static bool power_save_enabled = false;
static void set_power_save_state(bool b) {{ (void)b; }}
static void process_can(uint8_t n) {{ (void)n; }}
static int fail_core = -1;
static bool can_set_speed(uint8_t n) {{ (void)n; return true; }}
static bool llcan_init({typ} *c) {{ return (c - cores) != fail_core; }}
{body}
{checked}
int main(void) {{
  fail_core = -1; assert(can_init(0) && can_init(2)); assert(can_init_all_checked());
  fail_core = 1; assert(!can_init(1) && can_init(0)); assert(!can_init_all_checked());
  printf("{drv}: can_init + can_init_all_checked OK\\n");
  return 0;
}}
"""
  run_c(tmp_path, source)
