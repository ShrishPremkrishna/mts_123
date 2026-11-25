#!/usr/bin/env bash
# i2c_recover.sh — free a hung I2C-1 bus (BNO085 0x4B, battery ADC 0x48, GT911 touch) on the Pi 5
# without a power cycle. Called as `fcctl.sh i2c-recover [--check] [--force]` (fcctl refuses while a
# robot stack runs unless --force); usable alone with the same options.
#
# Why (mts 2026-11-25, fc/notes/imu-filter.md "v3"): the kernel logged "i2c_designware ...:
# SDA stuck at low", then every transfer failed ("timeout waiting for bus ready"); GPIO2/3 read low
# for minutes and a stack restart (driver soft reset over I2C) could not help.
#
# What it does (stack stopped):
#   1. reads SDA (GPIO2) / SCL (GPIO3); both high = idle bus: nothing to do (still probes 0x4B).
#   2. unbinds the RP1 DesignWare I2C controller (sudo), so the kernel stops driving the pins;
#   3. SCL as input with pull-up: still low => a device holds SCL (clock stretch hang): pulses
#      cannot help, a power cycle (or the device's reset pin) is needed; reported, pins restored;
#   4. otherwise the standard recovery: up to 16 SCL pulses (open-drain emulation via pinctrl) until
#      SDA is released, then a STOP condition;
#   5. restores GPIO2/3 to a3 (I2C1) with pull-ups, rebinds the controller (re-initialises it),
#      waits for /dev/i2c-1, reads the levels and probes 0x4B with one 4-byte SHTP header read.
# Prints log lines on stderr and one JSON line on stdout. Exit 0 = bus idle and 0x4B answers.
#
# UNTESTED on a hung bus as of 2026-11-25 (written during the live session; the bus had already
# been power-cycled). --check is read-only. Env overrides for tests: FC_PINCTRL, FC_SUDO,
# FC_I2C_DRV (sysfs driver dir), FC_I2C_CTRL, FC_I2C_DEVNODE, FC_I2CTRANSFER, FC_I2C_SLEEP.
set -uo pipefail

PINCTRL=${FC_PINCTRL:-pinctrl}
SUDO=${FC_SUDO:-sudo -n}
DRV=${FC_I2C_DRV:-/sys/bus/platform/drivers/i2c_designware}
CTRL=${FC_I2C_CTRL:-1f00074000.i2c}
DEVNODE=${FC_I2C_DEVNODE:-/dev/i2c-1}
I2CTRANSFER=${FC_I2CTRANSFER:-i2ctransfer}
NAP=${FC_I2C_SLEEP:-0.001}
SDA=2
SCL=3
ADDR=0x4b
CHECK=false
FORCE=false
for a in "$@"; do
  case "$a" in
    --check) CHECK=true ;;
    --force) FORCE=true ;;
    *) echo "usage: i2c_recover.sh [--check] [--force]" >&2; exit 2 ;;
  esac
done

log() { echo "[i2c-recover] $*" >&2; }
lev() { "$PINCTRL" lev "$1" 2>/dev/null | tr -dc '01' | head -c1; }
pset() { "$PINCTRL" set "$@" >/dev/null 2>&1; }
probe() {  # 0x4B answers a 4-byte read (an SHTP header; harmless for the BNO)
  command -v "$I2CTRANSFER" > /dev/null 2>&1 || { echo null; return; }
  if "$I2CTRANSFER" -y 1 "r4@$ADDR" > /dev/null 2>&1; then echo true; else echo false; fi
}
emit() {  # ok action msg
  printf '{"ok":%s,"action":"%s","before":{"sda":%s,"scl":%s},"after":{"sda":%s,"scl":%s},"pulses":%s,"scl_held":%s,"rebound":%s,"bno_ack":%s,"msg":"%s"}\n' \
    "$1" "$2" "${B_SDA:-null}" "${B_SCL:-null}" "${A_SDA:-null}" "${A_SCL:-null}" "${PULSES:-0}" \
    "${SCL_HELD:-null}" "${REBOUND:-false}" "${ACK:-null}" "$3"
}

if ! $FORCE && pgrep -f 'ros2_control_node|persistent_session.py' > /dev/null 2>&1; then
  emit false refused "a robot stack is running (it holds the IMU); stop it first or pass --force"
  exit 1
fi

B_SDA=$(lev $SDA); B_SCL=$(lev $SCL)
[ -n "$B_SDA" ] && [ -n "$B_SCL" ] || { emit false error "cannot read GPIO levels with $PINCTRL"; exit 1; }
log "before: SDA=$B_SDA SCL=$B_SCL"
if [ "$B_SDA" = 1 ] && [ "$B_SCL" = 1 ]; then
  A_SDA=$B_SDA; A_SCL=$B_SCL; ACK=$(probe)
  if [ "$ACK" = false ]; then emit false none "bus idle but 0x4B does not answer: power cycle the robot"; exit 1; fi
  emit true none "bus idle"; exit 0
fi
if $CHECK; then
  A_SDA=$B_SDA; A_SCL=$B_SCL
  emit false check "bus not idle (a line reads low); run without --check to recover (stack stopped)"; exit 1
fi

if [ -e "$DRV/$CTRL" ]; then
  echo "$CTRL" | $SUDO tee "$DRV/unbind" > /dev/null || { emit false error "unbind failed (sudo?)"; exit 1; }
  log "controller $CTRL unbound"
fi
restore() {
  pset $SDA,$SCL a3 pu
  echo "$CTRL" | $SUDO tee "$DRV/bind" > /dev/null 2>&1 && REBOUND=true
  for _ in $(seq 1 30); do [ -e "$DEVNODE" ] && break; sleep 0.1; done
  A_SDA=$(lev $SDA); A_SCL=$(lev $SCL)
}

pset $SDA ip pu
pset $SCL ip pu
sleep 0.01
if [ "$(lev $SCL)" = 0 ]; then
  SCL_HELD=true
  log "SCL stays low as an input: a device holds the clock; pulses cannot free it"
  restore; ACK=$(probe)
  emit false scl_held "a device holds SCL low: power cycle the robot (BNO reset)"; exit 1
fi
SCL_HELD=false
PULSES=0
pset $SCL op dh
for i in $(seq 1 16); do
  pset $SCL dl; sleep "$NAP"; pset $SCL dh; sleep "$NAP"
  PULSES=$i
  [ "$(lev $SDA)" = 1 ] && break
done
log "$PULSES SCL pulse(s); SDA=$(lev $SDA)"
pset $SCL dl; pset $SDA op dl; sleep "$NAP"; pset $SCL dh; sleep "$NAP"; pset $SDA dh; sleep "$NAP"  # STOP
pset $SDA ip pu; pset $SCL ip pu
restore
ACK=$(probe)
log "after: SDA=$A_SDA SCL=$A_SCL rebound=$REBOUND bno_ack=$ACK"
if [ "$A_SDA" = 1 ] && [ "$A_SCL" = 1 ] && [ "$ACK" != false ]; then
  emit true recovered "bus freed; restart the stack (or let a v3 driver re-init) to resume the IMU"; exit 0
fi
emit false failed "bus still not idle or 0x4B silent: power cycle the robot"; exit 1
