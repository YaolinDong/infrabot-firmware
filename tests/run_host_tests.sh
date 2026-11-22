#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# Firmware host tests: the control modules compiled for the PC with mocked
# drives, no HAL, network or RTOS.  Everything they need is in this
# repository.
#   tests/run_host_tests.sh                # all
#   tests/run_host_tests.sh test_safety    # one or more by name
set -eu
cd "$(dirname "$0")/.."
CC=${CC:-cc}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
CFLAGS="-std=c11 -Wall -Wextra -Werror ${CFLAGS_EXTRA:-}"
wanted() {
  [ $# -eq 0 ] && return 0
  for name in "$@"; do [ "$name" = "$CURRENT" ] && return 0; done
  return 1
}

run() {
  CURRENT=$1
  shift
  if wanted $SELECTED; then
    echo "== $CURRENT"
    # shellcheck disable=SC2086
    $CC $CFLAGS "$@" "$OUT/state.a" -lm -o "$OUT/$CURRENT"
    if [ -n "${LCD_SCREENSHOTS:-}" ] && { [ "$CURRENT" = test_lcd_ui ] || [ "$CURRENT" = test_lcd_ui_43 ]; }; then
      panel=5; [ "$CURRENT" = test_lcd_ui_43 ] && panel=43
      mkdir -p "$LCD_SCREENSHOTS/$panel"
      "$OUT/$CURRENT" "$LCD_SCREENSHOTS/$panel"
    elif [ "$CURRENT" = test_dm_protocol_equivalence ]; then
      gzip -dc tests/data/dm_protocol_vectors.bin.gz > "$OUT/vectors"
      "$OUT/$CURRENT" < "$OUT/vectors"
    else
      "$OUT/$CURRENT"
    fi
  fi
}

# Co-located production headers must not hide mock headers for quoted includes.
# Expose just the sources/inlined fragments together; header search stays per test.
HOST_SRC="$OUT/src"
mkdir "$HOST_SRC"
for file in app/*.c control/*.c control/*.cpp control/*.inc drivers/*/*.c ui/*.c ui/*.inc; do
  ln -s "$PWD/$file" "$HOST_SRC/"
done
INCLUDES="-I$HOST_SRC -Iapp -Icontrol -Idrivers/damiao -Idrivers/can -Idrivers/net -Idrivers/display -Iui -Iprotocol -Iboards/fire-h743-challenger-v2"
MOTION="$HOST_SRC/arm_motion.c $HOST_SRC/infrabot_ik_mcu.c"
for storage in drivers/can/bus_state.c control/control_stats.c drivers/damiao/dm_runtime.c drivers/net/eth_state.c app/uart_tx_state.c; do
  object="$OUT/$(basename "$storage" .c).o"
  $CC $CFLAGS $INCLUDES -c "$storage" -o "$object"
  ar rcs "$OUT/state.a" "$object"
done
SELECTED="$*"
CURRENT=test_app_admittance
if wanted $SELECTED || { CURRENT=test_app_gravity; wanted $SELECTED; } || { CURRENT=test_app_impedance; wanted $SELECTED; } || { CURRENT=test_local_control; wanted $SELECTED; } ||
   { CURRENT=test_gravity_handover; wanted $SELECTED; } ||
   { CURRENT=test_gravity_diagnostics; wanted $SELECTED; } ||
   { CURRENT=test_gravity_governor; wanted $SELECTED; }; then
  ${CXX:-c++} ${CFLAGS_EXTRA:-} -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti $INCLUDES \
    -c $HOST_SRC/arm_gravity.cpp -o "$OUT/gravity.o"
fi
run test_protocol_info -Itests/mocks/info $INCLUDES drivers/net/app_info.c tests/info_fixture.c tests/test_protocol_info.c
run test_dm_protocol -Idrivers/damiao drivers/damiao/dm_protocol.c tests/test_dm_protocol.c
run test_dm_protocol_equivalence -ffp-contract=off -Idrivers/damiao drivers/damiao/dm_protocol.c tests/test_dm_protocol_equivalence.c
run test_admittance_release_fk -O2 $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_release_fk.c
run test_admittance_safety $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_safety.c
run test_admittance_ff_relay -O2 $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_ff_relay.c
run test_admittance_estimator $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_estimator.c
run test_admittance_dynamics $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_dynamics.c
run test_admittance_nullspace $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_nullspace.c
run test_admittance_shoulder_push -O2 $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_shoulder_push.c
run test_admittance_pullback -DARM_ADM_TEST_SPEED_CAP=.05f $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_pullback.c
run test_admittance_reference $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_reference.c
run test_admittance_joint $INCLUDES $HOST_SRC/arm_admittance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_admittance_joint.c
run test_impedance_parameters -Itests/mocks $INCLUDES $HOST_SRC/app_impedance.c tests/test_impedance_parameters.c
run test_impedance_math $INCLUDES $HOST_SRC/arm_impedance.c $HOST_SRC/infrabot_ik_mcu.c tests/test_impedance_math.c
run test_app_admittance -DAPP_GRAVITY_REAL_TEST -Itests/mocks $INCLUDES $MOTION \
  $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c $HOST_SRC/app_control_mode.c $HOST_SRC/app_gravity.c $HOST_SRC/app_admittance.c $HOST_SRC/arm_admittance_jitter.c $HOST_SRC/app_admittance_parameters.c $HOST_SRC/arm_admittance.c $HOST_SRC/arm_impedance.c $HOST_SRC/app_impedance.c \
  tests/test_app_admittance.c "$OUT/gravity.o"
run test_app_admittance_parameters -Itests/mocks $INCLUDES -ffunction-sections -fdata-sections $HOST_SRC/app_admittance.c $HOST_SRC/arm_admittance_jitter.c $HOST_SRC/app_admittance_parameters.c tests/test_app_admittance_parameters.c -Wl,--gc-sections
run test_app_impedance -DAPP_GRAVITY_REAL_TEST -Itests/mocks $INCLUDES $MOTION \
  $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c $HOST_SRC/app_control_mode.c $HOST_SRC/app_gravity.c $HOST_SRC/app_admittance.c $HOST_SRC/arm_admittance_jitter.c $HOST_SRC/app_admittance_parameters.c $HOST_SRC/arm_admittance.c $HOST_SRC/arm_impedance.c $HOST_SRC/app_impedance.c \
  tests/test_app_impedance.c "$OUT/gravity.o"
run test_app_gravity -DAPP_GRAVITY_REAL_TEST -Itests/mocks $INCLUDES $MOTION \
  $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c $HOST_SRC/app_control_mode.c $HOST_SRC/app_gravity.c $HOST_SRC/app_admittance.c $HOST_SRC/arm_admittance_jitter.c $HOST_SRC/app_admittance_parameters.c $HOST_SRC/arm_admittance.c $HOST_SRC/arm_impedance.c $HOST_SRC/app_impedance.c \
  tests/test_app_gravity.c "$OUT/gravity.o"
run test_gravity_handover -DAPP_GRAVITY_REAL_TEST -Itests/mocks $INCLUDES $MOTION \
  $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c $HOST_SRC/app_control_mode.c $HOST_SRC/app_gravity.c $HOST_SRC/app_admittance.c $HOST_SRC/arm_admittance_jitter.c $HOST_SRC/app_admittance_parameters.c $HOST_SRC/arm_admittance.c $HOST_SRC/arm_impedance.c $HOST_SRC/app_impedance.c \
  tests/test_gravity_handover.c "$OUT/gravity.o"
run test_gravity_diagnostics -DAPP_GRAVITY_REAL_TEST -Itests/mocks $INCLUDES $MOTION \
  $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c $HOST_SRC/app_control_mode.c $HOST_SRC/app_gravity.c $HOST_SRC/app_admittance.c $HOST_SRC/arm_admittance_jitter.c $HOST_SRC/app_admittance_parameters.c $HOST_SRC/arm_admittance.c $HOST_SRC/arm_impedance.c $HOST_SRC/app_impedance.c \
  tests/test_gravity_diagnostics.c "$OUT/gravity.o"
run test_gravity_governor -DAPP_GRAVITY_REAL_TEST -Itests/mocks $INCLUDES $MOTION \
  $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c $HOST_SRC/app_control_mode.c $HOST_SRC/app_gravity.c $HOST_SRC/app_admittance.c $HOST_SRC/arm_admittance_jitter.c $HOST_SRC/app_admittance_parameters.c $HOST_SRC/arm_admittance.c $HOST_SRC/arm_impedance.c $HOST_SRC/app_impedance.c \
  tests/test_gravity_governor.c "$OUT/gravity.o"
run test_local_control -DAPP_GRAVITY_REAL_TEST -Itests/mocks $INCLUDES $MOTION \
  $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c $HOST_SRC/app_control_mode.c $HOST_SRC/app_gravity.c $HOST_SRC/app_admittance.c $HOST_SRC/arm_admittance_jitter.c $HOST_SRC/app_admittance_parameters.c $HOST_SRC/arm_admittance.c $HOST_SRC/arm_impedance.c $HOST_SRC/app_impedance.c \
  $HOST_SRC/app_local_control.c tests/test_local_control.c "$OUT/gravity.o"
run test_lcd_slider $INCLUDES tests/test_lcd_slider.c
run test_persist -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast -Itests/mocks/persist -Itests/mocks $INCLUDES tests/test_persist.c
run test_touch_probe -Itests/mocks/touch -Itests/mocks/lcd $INCLUDES $HOST_SRC/app_lcd_panel.c tests/test_touch_probe.c
run test_lcd_panel $INCLUDES $HOST_SRC/app_lcd_panel.c tests/test_lcd_panel.c
run test_panel $INCLUDES tests/test_panel.c
run test_lcd_boot -O2 -Itests/mocks/boot $INCLUDES tests/test_lcd_boot.c
run test_lcd_ui -DFW_NETBOOT_APP -Ibootloader -O2 -Wno-format-truncation -ffunction-sections -fdata-sections \
  -Itests/mocks/lcd -Itests/mocks $INCLUDES tests/test_lcd_ui.c \
  $HOST_SRC/app_lcd_font.c $HOST_SRC/app_lcd_zh.c $HOST_SRC/app_lcd_panel.c $HOST_SRC/infrabot_ik_mcu.c $HOST_SRC/app_impedance.c $HOST_SRC/app_admittance_parameters.c -Wl,--gc-sections
run test_lcd_ui_43 -DFW_NETBOOT_APP -Ibootloader -DTEST_LCD_43 -O2 -Wno-format-truncation -ffunction-sections -fdata-sections \
  -Itests/mocks/lcd -Itests/mocks $INCLUDES tests/test_lcd_ui.c \
  $HOST_SRC/app_lcd_font.c $HOST_SRC/app_lcd_zh.c $HOST_SRC/app_lcd_panel.c $HOST_SRC/infrabot_ik_mcu.c $HOST_SRC/app_impedance.c $HOST_SRC/app_admittance_parameters.c -Wl,--gc-sections
run test_arm_motion $INCLUDES $MOTION tests/test_arm_motion.c
run test_cartesian_speed_cap -O2 $INCLUDES $MOTION tests/test_cartesian_speed_cap.c
run test_app_motion -Itests/mocks $INCLUDES $MOTION $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c tests/test_app_motion.c
run test_control_mode -Itests/mocks $INCLUDES $MOTION $HOST_SRC/app_motion.c $HOST_SRC/app_safety.c \
  $HOST_SRC/app_control_mode.c tests/test_control_mode.c
run test_arm_power -Itests/mocks $INCLUDES $HOST_SRC/app_arm_power.c tests/test_arm_power.c
run test_enable -Itests/mocks $INCLUDES $HOST_SRC/app_enable.c $HOST_SRC/app_gripper.c tests/test_enable.c
run test_safety -Itests/mocks $INCLUDES $MOTION $HOST_SRC/app_safety.c tests/test_safety.c
run test_app_log $INCLUDES $HOST_SRC/app_log.c tests/test_app_log.c
run test_selfcheck -Itests/mocks $INCLUDES $HOST_SRC/app_damiao.c tests/test_selfcheck.c
run test_lcd_schedule -Itests/mocks/lcd $INCLUDES tests/test_lcd_schedule.c
run test_lcd_3d -O2 -ffast-math -Itests/mocks/lcd -Itests/mocks $INCLUDES \
  $HOST_SRC/infrabot_ik_mcu.c $HOST_SRC/app_lcd_mesh.c tests/test_lcd_3d.c
run test_lcd_3d_uncached -O2 -ffast-math -DAPP_LCD_SCRATCH_CACHE=0 \
  -Itests/mocks/lcd -Itests/mocks $INCLUDES \
  $HOST_SRC/infrabot_ik_mcu.c $HOST_SRC/app_lcd_mesh.c tests/test_lcd_3d.c
run test_lcd_3d_full -O2 -ffast-math -DAPP_LCD_PARTIAL_REFRESH=0 \
  -Itests/mocks/lcd -Itests/mocks $INCLUDES \
  $HOST_SRC/infrabot_ik_mcu.c $HOST_SRC/app_lcd_mesh.c tests/test_lcd_3d.c
run test_lcd_3d_sdram -O2 -ffast-math -DAPP_LCD_FAST_BUFFER=0 \
  -Itests/mocks/lcd -Itests/mocks $INCLUDES \
  $HOST_SRC/infrabot_ik_mcu.c $HOST_SRC/app_lcd_mesh.c tests/test_lcd_3d.c
run test_lcd_3d_sdram_uncached -O2 -ffast-math -DAPP_LCD_FAST_BUFFER=0 -DAPP_LCD_SCRATCH_CACHE=0 \
  -Itests/mocks/lcd -Itests/mocks $INCLUDES \
  $HOST_SRC/infrabot_ik_mcu.c $HOST_SRC/app_lcd_mesh.c tests/test_lcd_3d.c

# The MCU never removes power by itself (docs/control-modes.md, 受控停止).
# These are all the places that can: an LCD DISABLE ARM, a PC's DISABLE (v3/v4, a group, before
# zeroing), the rollback of a failed enable, boot, a register read that
# finds a drive in another control mode (app_damiao.c), and the arm found
# without power (app_arm_power.c: bus faulted, J1-J7 silent; the drives are
# off already, only the MCU's bookkeeping follows).  A new one fails
# here until it is reviewed and added on purpose.
CURRENT=test_no_self_disable
if wanted $SELECTED; then
  echo "== $CURRENT"
  check_sites() {
    found=$(grep -c "$1" app/*.c control/*.c drivers/*/*.c ui/*.c boards/fire-h743-challenger-v2/*.c | grep -v ':0$' | LC_ALL=C sort | tr '\n' ' ')
    if [ "$found" != "$2" ]; then
      echo "power-removing sites of '$1' changed" >&2
      echo "  expected: $2" >&2
      echo "  found:    $found" >&2
      exit 1
    fi
  }
  check_sites 'eth_udp_command\.control_request *= *DM_ETH_CMD_DISABLE' 'control/app_motion.c:1 drivers/damiao/app_damiao.c:1 '
  check_sites 'AppEnable_Disable(' 'app/app_tasks.c:3 control/app_arm_power.c:1 control/app_enable.c:2 control/app_local_control.c:1 '
  check_sites 'platform_drive_disable(' 'control/app_enable.c:1 '
  check_sites 'DM_Motor_Disable(' \
    'app/app_startup.c:2 drivers/damiao/app_damiao.c:1 drivers/damiao/dm_motor.c:1 '
  check_sites 'dm_motor_runtime\.enabled\[[^]]*\] *= *0' 'app/app_tasks.c:1 control/app_enable.c:1 drivers/damiao/app_damiao.c:1 '
  # dm_control_enabled = 0 stops every CAN frame (app_damiao.c recomputes it
  # from the enabled drives).
  check_sites 'dm_motor_runtime\.control_enabled *= *0' 'app/app_tasks.c:1 control/app_motion.c:1 drivers/damiao/app_damiao.c:1 '
  echo "no self-disable: power removal is an explicit PC/LCD request, enable rollback, boot, drive-mode mismatch or arm power already lost"
fi
CURRENT=test_layers
if wanted $SELECTED; then
  echo "== $CURRENT"
  python3 -B tools/check_layers.py
fi
echo "host tests passed"
