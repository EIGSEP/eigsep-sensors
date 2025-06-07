#!/usr/bin/env python3
import argparse
from picoser import open_port, send_cmd

def main() -> None:
    ap  = argparse.ArgumentParser(prog="picoctl")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("stop")
    sub.add_parser("resume")
    sub.add_parser("bootsel")

    p_set  = sub.add_parser("set")
    p_set.add_argument("temp", type=float, help="target temperature (°C)")

    p_hyst = sub.add_parser("hyst")
    p_hyst.add_argument("band", type=float, help="hysteresis band (°C)")

    args = ap.parse_args()
    cmd  = args.cmd.lower()          

  
    if   cmd == "set":
        cmd_str = f"SET,{args.temp}"
    elif cmd == "hyst":
        cmd_str = f"HYST,{args.band}"
    elif cmd == "bootsel":
        cmd_str = "BOOTSEL"
    elif cmd == "stop":
        cmd_str = "STOP"
    elif cmd == "resume":
        cmd_str = "RESUME"
    else:
        ap.error(f"unknown command {cmd}")  

    with open_port() as ser:          
        reply = send_cmd(ser, cmd_str)
        print(reply)

if __name__ == "__main__":
    main()
