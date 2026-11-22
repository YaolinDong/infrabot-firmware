#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Hermetic baseline/current host replay of unchanged local controllers (no device IO)."""
from pathlib import Path
import argparse,hashlib,io,shutil,subprocess,tarfile,tempfile
ROOT=Path(__file__).resolve().parents[1]
TRACE=r'''
#define GRAVITY_HANDOVER_ENTRY ignored_original_main
#include "test_gravity_handover.c"
#undef GRAVITY_HANDOVER_ENTRY
int main(int argc,char **argv){
 (void)ignored_original_main;assert(argc==2);FILE *out=fopen(argv[1],"wb");assert(out);
 for(unsigned mode=0;mode<=8;mode++){
  setup(.4f);assert(AppControlMode_Switch(mode)==0);
  for(unsigned n=0;n<1500;n++){
   for(unsigned i=0;i<7;i++){
    float q=pose[i]+(n>1100?.012f*sinf(.013f*n+i):0);
    dm_motors[i].position=dm_motor_zero_offset[i]+DM_MOTOR_DIRECTION(i)*q;
    dm_motors[i].velocity=n>1100?DM_MOTOR_DIRECTION(i)*.02f*cosf(.013f*n+i):0;
    dm_motors[i].feedback_count++;
   }
   step();
   for(unsigned i=0;i<8;i++){
    float fields[]={dm_command_position[i],dm_eth_udp_target_velocity[i],dm_eth_udp_target_kp[i],dm_eth_udp_target_kd[i],dm_eth_udp_target_torque[i]};
    assert(fwrite(fields,sizeof(fields),1,out)==1);
   }
   uint8_t fields[]={dm_eth_control_mode,dm_control_enabled,AppControlMode_Telemetry()};
   assert(fwrite(fields,sizeof(fields),1,out)==1);
  }
 }
 fclose(out);return 0;
}'''
COMMON=["arm_motion.c","infrabot_ik_mcu.c","app_motion.c","app_safety.c","app_control_mode.c","app_gravity.c","arm_impedance.c","app_impedance.c"]
NEW=["app_admittance.c","app_admittance_parameters.c","arm_admittance.c","arm_admittance_jitter.c"]
def run(tree,work,output):
    work.mkdir()
    if (tree/"Core"/"Src"/"app_motion.c").is_file():
        source = tree/"Core"/"Src"
        includes = [tree/"Core"/"Inc"]
    else:
        source = tree/"control"
        includes = [source, tree/"app", tree/"protocol", tree/"drivers/damiao",
                    tree/"drivers/net", tree/"drivers/can", tree/"boards/fire-h743-challenger-v2"]
    # Compile an isolated copy so quoted includes in a controller C file
    # still select the host mocks before co-located hardware headers.
    inputs = []
    for name in COMMON + [n for n in NEW if (source/n).is_file()]:
        inputs.append(shutil.copy(source/name, work/name))
    trace=TRACE
    if (tree/"drivers/net/eth_state.h").exists():
        state_sources = (
            "drivers/can/bus_state.c",
            "control/control_stats.c",
            "drivers/damiao/dm_runtime.c",
            "drivers/net/eth_state.c",
            "app/uart_tx_state.c",
        )
        inputs.extend(tree/name for name in state_sources)
        fields = {
            "dm_control_enabled": "dm_motor_runtime.control_enabled",
            "dm_motor_zero_offset": "dm_motor_runtime.zero_offset",
            "dm_command_position": "dm_motor_runtime.command_position",
            "dm_eth_control_mode": "eth_udp_command.control_mode",
            "dm_eth_udp_target_velocity": "eth_udp_command.target_velocity",
            "dm_eth_udp_target_kp": "eth_udp_command.target_kp",
            "dm_eth_udp_target_kd": "eth_udp_command.target_kd",
            "dm_eth_udp_target_torque": "eth_udp_command.target_torque",
        }
        for old,field in fields.items():
            trace=trace.replace(old,field)
    src=work/"legacy_trace.c";src.write_text(trace)
    inc=["-DAPP_GRAVITY_REAL_TEST", "-I"+str(tree/"tests/mocks"), "-I"+str(tree/"tests")]
    inc.extend("-I"+str(folder) for folder in [*includes, source])
    gravity=work/"gravity.o"
    subprocess.run(["c++","-std=c++17","-O2",*inc,"-c",str(source/"arm_gravity.cpp"),"-o",str(gravity)],check=True)
    exe=work/"trace"
    subprocess.run(["cc","-std=c11","-O2",*inc,*inputs,str(src),str(gravity),"-lm","-lstdc++","-o",str(exe)],check=True)
    subprocess.run([str(exe),str(output)],check=True,stdout=subprocess.DEVNULL)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", help="optional available Git revision to replay against the current tree")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        current = root / "current.bin"
        run(ROOT, root / "new", current)
        data = current.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if args.baseline:
            base = root / "base"
            base.mkdir()
            archive = subprocess.check_output(["git", "archive", args.baseline], cwd=ROOT)
            with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
                tar.extractall(base, filter="data")
            old = root / "baseline.bin"
            run(base, root / "old", old)
            if data != old.read_bytes():
                raise SystemExit("Legacy actuator trace changed")
            print("Legacy mode-entry / local-controller replay 0..8: byte-identical", len(data), "bytes SHA256", digest)
        else:
            if len(data) != 2200500 or digest != "2330ad4a67a0a90e4aa6c338039d64c64a44c1aa0d3a039926296274ce8216f8":
                raise SystemExit(f"Legacy actuator trace changed: {len(data)} bytes SHA256 {digest}")
            print("Legacy modes 0..8 match the reference:", len(data), "bytes SHA256", digest)

if __name__ == "__main__":
    main()
