#!/usr/bin/env bash
# Host-to-host pamodbus discovery loopback test over a socat pty pair.
#
# Requires: socat, and the host tools built (make in this directory).
#
# Usage:  bash loopback_test.sh
# Result: the SLAVE is discovered and assigned an ID (PASS). The master-side
#         verify confirmation is pacing-sensitive; see README.md finding #3.
cd "$(dirname "$0")" || exit 2
rm -f /tmp/ttyA /tmp/ttyB /tmp/sl.log /tmp/m.log /tmp/soc.log

socat pty,raw,echo=0,link=/tmp/ttyA pty,raw,echo=0,link=/tmp/ttyB >/tmp/soc.log 2>&1 &
SPID=$!
sleep 1

# Start the slave first and warm it up so it is listening before the master scans.
./slave_disco   /tmp/ttyA 115200 15 >/tmp/sl.log 2>&1 & SP=$!
sleep 2
./master_disco  /tmp/ttyB 115200 10 >/tmp/m.log 2>&1 & MP=$!

wait "$SP"; RC1=$?
wait "$MP"; RC2=$?
kill "$SPID" 2>/dev/null

echo "rc_slave=$RC1 rc_master=$RC2"
echo "=== SLAVE (ttyA) ==="; cat /tmp/sl.log
echo "=== MASTER (ttyB) ==="; cat /tmp/m.log
exit $(( RC1 || RC2 ))