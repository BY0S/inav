#!/usr/bin/env python3
"""
Generates a pre-configured eeprom.bin for INAV SITL + X-Plane 12 testing.

Configures:
- UART2 (TCP 5761) for MSP companion control alongside UART1 (TCP 5760)
- receiver_type = MSP (so MSP_SET_RAW_RC works without ARMING_DISABLED_RC_LINK)
- init_gyro_cal = OFF, pitot_hardware = NONE, inav_gravity_cal_tolerance = 250
- nav_wp_max_safe_distance = 500 (allows 200m+ first waypoints without arming block)
- AUX modes for ARM, ANGLE, NAV ALTHOLD, and NAV WP (Waypoint Mission)
- Motor and Servo mixers (M01 throttle + S01..S03 Roll/Pitch/Yaw for X-Plane)
"""

import argparse
import os
import socket
import subprocess
import time


def send_cli(commands, host="127.0.0.1", port=5760):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3.0)
    for _ in range(30):
        try:
            s.connect((host, port))
            break
        except Exception:
            time.sleep(0.25)
    else:
        raise RuntimeError(f"Could not connect to INAV SITL CLI on {host}:{port}")

    s.sendall(b"#")
    time.sleep(0.4)
    try:
        s.recv(4096)
    except Exception:
        pass

    out = ""
    for cmd in commands:
        s.sendall((cmd + "\r\n").encode())
        time.sleep(0.12)
        try:
            out += s.recv(8192).decode(errors="ignore")
        except Exception:
            pass
    s.close()
    return out


def main():
    parser = argparse.ArgumentParser(description="Configure INAV SITL eeprom.bin for X-Plane 12 testing")
    default_sitl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../inav/build-nix-sitl"))
    parser.add_argument("--sitl-dir", default=default_sitl_dir, help="Directory where eeprom.bin will be written")
    parser.add_argument("--sitl-bin", default=None, help="Path to inav_*_SITL binary")
    args = parser.parse_args()

    sitl_dir = os.path.abspath(args.sitl_dir)
    os.makedirs(sitl_dir, exist_ok=True)

    if args.sitl_bin:
        sitl_bin = os.path.abspath(args.sitl_bin)
    else:
        candidate_bin = os.path.join(sitl_dir, "bin", "inav_9.1.0_SITL")
        fallback_bin = os.path.join(sitl_dir, "inav_9.1.0_SITL")
        sitl_bin = candidate_bin if os.path.exists(candidate_bin) else fallback_bin

    if not os.path.exists(sitl_bin):
        raise FileNotFoundError(f"SITL binary not found at: {sitl_bin}")

    eeprom_path = os.path.join(sitl_dir, "eeprom.bin")
    print(f"Configuring {eeprom_path} using {sitl_bin}...")

    subprocess.run(["pkill", "-x", "inav_9.1.0_SITL"], check=False)
    time.sleep(0.3)

    proc = subprocess.Popen(
        [sitl_bin, f"--path={eeprom_path}"],
        cwd=sitl_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        time.sleep(1.0)
        cmds = [
            "serial 1 1 115200 115200 0 115200",
            "feature GPS",
            "feature PWM_OUTPUT_ENABLE",
            "set platform_type = AIRPLANE",
            "set model_preview_type = 14",
            "set receiver_type = MSP",
            "set gps_provider = UBLOX",
            "set mag_hardware = FAKE",
            "set baro_hardware = FAKE",
            "set pitot_hardware = NONE",
            "set init_gyro_cal = OFF",
            "set inav_gravity_cal_tolerance = 250",
            "set inav_w_z_baro_p = 5.000",
            "set nav_extra_arming_safety = ALLOW_BYPASS",
            "set nav_wp_max_safe_distance = 500",
            "set nav_wp_radius = 1500",
            "set nav_fw_cruise_thr = 1900",
            "set nav_fw_min_thr = 1700",
            "set nav_fw_max_thr = 2000",
            "aux 0 0 0 1700 2100",
            "aux 1 1 1 1300 1700",
            "aux 2 3 1 1700 2100",
            "aux 3 28 2 1700 2100",
            "mmix reset",
            "mmix 0 1.000 0.000 0.000 0.000",
            "smix reset",
            "smix 0 1 0 100 0 -1",
            "smix 1 2 1 -100 0 -1",
            "smix 2 3 2 -100 0 -1",
            "smix 3 3 0 60 0 -1",
            "save",
        ]
        send_cli(cmds)
        time.sleep(1.5)
        verify = send_cli(["get receiver_type", "get nav_wp_max_safe_distance", "smix", "aux"])
        print(verify)
        print(f"Successfully configured {eeprom_path}")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except Exception:
            proc.kill()


if __name__ == "__main__":
    main()
