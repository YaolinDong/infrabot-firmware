#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
"""Generate the conditional stability region for the folded admittance model."""
import argparse
import csv
import ctypes
import itertools
import json
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",type=Path,default=ROOT/"build/admittance-simulation")
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        library=Path(tmp)/"coupled.so"
        subprocess.run(["cc","-std=c11","-O2","-shared","-fPIC","-I"+str(ROOT/"control"),
            str(ROOT/"tests/admittance_coupled.c"),str(ROOT/"control/arm_admittance.c"),
            str(ROOT/"control/infrabot_ik_mcu.c"),"-lm","-o",str(library)],check=True)
        lib=ctypes.CDLL(str(library))
        v9=lib.adm_coupled_v9
        v9.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.c_char_p,ctypes.POINTER(ctypes.c_double)]
        traces={}
        for scenario,learning,name in [(1,0,"bias-off"),(1,1,"bias-on"),(2,1,"force-ramp")]:
            output=(ctypes.c_double*16)()
            path=a.out/(name+".csv")
            if not v9(scenario,learning,str(path).encode(),output):raise OSError(path)
            traces[name]=dict(fault=int(output[0]),freeze_ticks=int(output[1]),drift_ticks=int(output[9]),
                virtual_peak_mm=1000*output[12],final_virtual_mm=1000*output[11],
                final_actual_mm=1000*output[5],peak_external_Nm=output[13],final_bias_Nm=output[14],
                settle_s=output[7],overshoot_mm=1000*output[8],max_reverse_step_mm=1000*output[15])
        print("V9 scenarios:",traces,flush=True)
        (a.out/"v9-scenarios.json").write_text(json.dumps(traces,indent=2)+"\n")
        off,on,ramp=(traces[k] for k in ("bias-off","bias-on","force-ramp"))
        v9_pass=(all(not (r["fault"] or r["freeze_ticks"] or r["drift_ticks"]) for r in traces.values())
                 and off["virtual_peak_mm"]<50 and on["virtual_peak_mm"]<5
                 and on["peak_external_Nm"]<1 and .28<on["final_bias_Nm"]<=.30001
                 and 0<=ramp["settle_s"]<1.5 and abs(ramp["final_actual_mm"])<2
                 and ramp["overshoot_mm"]<=.001 and ramp["max_reverse_step_mm"]<=.001
                 and ramp["virtual_peak_mm"]>10)
        run=lib.adm_coupled_run
        run.argtypes=[ctypes.c_double]*7+[ctypes.c_int,ctypes.POINTER(ctypes.c_double)]
        def case(mass,zeta,mh,kh,dh,mp,estimate=1,release=0):
            result=(ctypes.c_double*12)();run(mass,zeta,mh,kh,dh,mp,estimate,release,result)
            stable=result[0]==0 and result[1]==0 and result[9]==0 and result[4]<.001 and abs(result[6])<.005
            return dict(mass=mass,zeta=zeta,human_mass=mh,human_k=kh,human_d=dh,physical_mass=mp,
                friction_estimate=estimate,stable=stable,fault=int(result[0]),freeze_ticks=int(result[1]),
                peak_mm=result[2]*1000,peak_mm_s=result[3]*1000,tail_range_mm=result[4]*1000,
                final_mm=result[5]*1000,final_mm_s=result[6]*1000,settle_s=result[7],
                overshoot_mm=result[8]*1000,drift_frozen_ticks=int(result[9]))
        rows=[];regions=[]
        hands=list(itertools.product((1,2),(300,1000,3000),(10,25,40),(1,2,3.75)))
        for mass,zeta in itertools.product((1,1.5,2,3,4,6),(.6,.8,1,1.2,1.5,2)):
            tests=[case(mass,zeta,*h) for h in hands];rows.extend(tests)
            for physical in (1,2,3.75):
                subset=[r for r in tests if r["physical_mass"]==physical]
                regions.append(dict(physical_mass=physical,mass=mass,zeta=zeta,stable=sum(r["stable"] for r in subset),total=len(subset),
                                    all_stable=all(r["stable"] for r in subset)))
        releases=[case(3,1,0,0,0,mp,estimate,1) for mp,estimate in itertools.product((1,2,3.75),(1,1.3))]
    for name,data in (("cases.csv",rows),("stability-region.csv",regions),("release.csv",releases)):
        with (a.out/name).open("w",newline="") as f:
            w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    summary=dict(regions=regions,release=releases,
        model="x=.2q, Kp=300 Kd=2.5, Fc=.65 Fs=1.7Fc Fv=.117; 10kHz plant /1kHz MCU; torque 12bit ±54Nm, delayed2ms; reference16bit floor; baseline FF slew, speed brake/gross trip and torque supervisor; reflected human mass",
        stability="No stop/freeze, last1s position range<1mm and final speed<5mm/s; not a hardware certificate")
    (a.out/"report.json").write_text(json.dumps(summary,indent=2)+"\n")
    required_all=[r for r in rows if r["mass"] in (1.5,3) and r["zeta"]==1 and r["human_k"]==3000]
    required=[r for r in required_all if r["physical_mass"]<=2]
    stress=[r for r in required_all if r["physical_mass"]==3.75]
    print("coupled cases",len(rows),"stable",sum(r["stable"] for r in rows))
    print("required default/half-mass stiff hand:",sum(r["stable"] for r in required),"/",len(required))
    print("additional 3.75kg default/half-mass stress:",sum(r["stable"] for r in stress),"/",len(stress))
    for r in releases:print("release",r)
    print(a.out/"stability-region.csv")
    # Report acceptance gaps explicitly; never relabel bounded limit cycles stable.
    acceptance=dict(v9_scenarios_pass=v9_pass,declared_physical_mass_kg=[1,2],stress_physical_mass_kg=3.75,
        default_stiff_hand_pass=all(r["stable"] for r in required),
        stress_stiff_hand_pass=all(r["stable"] for r in stress),
        release_pass=all(r["stable"] and r["settle_s"]>=0 and r["settle_s"]<1.5 and r["overshoot_mm"]<2 and abs(r["final_mm"])<1 for r in releases))
    (a.out/"acceptance.json").write_text(json.dumps(acceptance,indent=2)+"\n")
    if not acceptance["stress_stiff_hand_pass"]:
        print("LIMITATION: extra 3.75kg half-mass stiff-hand stress is not universally stable.")
    if not v9_pass or not acceptance["default_stiff_hand_pass"] or not acceptance["release_pass"]:
        raise SystemExit("Declared primary-model acceptance failed; see cases.csv")

if __name__=="__main__":main()

