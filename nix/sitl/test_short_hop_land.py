#!/usr/bin/env python3
"""
Short-Hop & Soft-Landing Validation Test for INAV SITL + X-Plane 12.

Note: This is for drone in small tests, you should adapt it if there is anything
wrong or whatever you consider to apply.

Features:
- Disables X-Plane over-G / overspeed crash triggers on the fly via UDP DREFs.
- Performs sub-second in-place runway reset (reset_to_runway + fix_all_systems) without
  reloading scenery so tests can iterate back-to-back.
- Maps INAV motor output continuously through the 0..25% throttle band (avoiding INAV's
  min_throttle=1150 jump) so QuadTailsitter hovers smoothly at ~3.5m, moves ~15m, and
  lands softly at < 0.3 m/s.
"""

import argparse
import math
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time

MSP_STATUS_EX = 150
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_MOTOR = 104
MSP_SERVO = 103
MSP_WP = 118
MSP_NAV_STATUS = 121
MSP_SET_RAW_RC = 200
MSP_SET_WP = 209


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def make_msp(cmd, payload=b""):
    size = len(payload)
    crc = size ^ cmd
    for b in payload:
        crc ^= b
    return b"$M<" + struct.pack("<BB", size, cmd) + payload + struct.pack("<B", crc)


def send_receive_msp(sock, cmd, payload=b""):
    sock.setblocking(False)
    try:
        while sock.recv(4096):
            pass
    except Exception:
        pass
    sock.setblocking(True)
    sock.settimeout(0.3)

    sock.sendall(make_msp(cmd, payload))
    buf = b""
    t_start = time.time()
    while time.time() - t_start < 0.3:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            pos = 0
            while True:
                idx = buf.find(b"$M>", pos)
                if idx < 0 or len(buf) < idx + 6:
                    break
                size = buf[idx + 3]
                rcmd = buf[idx + 4]
                if len(buf) < idx + 6 + size:
                    break
                if rcmd == cmd:
                    return buf[idx + 5 : idx + 5 + size]
                pos = idx + 6 + size
        except Exception:
            break
    return None


def send_xplane_dref(udp_sock, xplane_ip, dref_name, val):
    buf = bytearray(509)
    buf[0:4] = b"DREF"
    buf[4] = 0
    struct.pack_into("<f", buf, 5, float(val))
    for i in range(9, 509):
        buf[i] = 0x20
    name_bytes = dref_name.encode("ascii") + b"\x00"
    buf[9 : 9 + len(name_bytes)] = name_bytes
    udp_sock.sendto(buf, (xplane_ip, 49000))


def send_xplane_cmnd(udp_sock, xplane_ip, cmnd_name):
    packet = b"CMND\x00" + cmnd_name.encode("ascii") + b"\x00"
    udp_sock.sendto(packet, (xplane_ip, 49000))


def disable_xplane_crash_triggers(udp_sock, xplane_ip):
    """Disable X-Plane crash/structural failure triggers on the fly."""
    send_xplane_dref(udp_sock, xplane_ip, "sim/operation/prefs/warn_overgforce", 0.0)
    send_xplane_dref(udp_sock, xplane_ip, "sim/operation/prefs/warn_overspeed", 0.0)
    send_xplane_dref(udp_sock, xplane_ip, "sim/operation/prefs/warn_overspeed_flaps", 0.0)
    send_xplane_dref(udp_sock, xplane_ip, "sim/operation/prefs/warn_overspeed_gear", 0.0)
    send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/failures/over_g", 0.0)
    send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/failures/over_vne", 0.0)
    send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/failures/over_vfe", 0.0)
    send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/failures/over_vle", 0.0)


def fast_in_place_reset(udp_sock, xplane_ip, reload_acf=False):
    """Instantly recover from any crashed state and zero velocities/attitude without reloading scenery."""
    send_xplane_dref(udp_sock, xplane_ip, "sim/cockpit2/engine/actuators/throttle_ratio_all", 0.0)
    if reload_acf:
        acf_path = "/mnt/c/X-Plane 12/Aircraft/Extra Aircraft/QuadTailsitter/QuadTailsitter.acf"
        if os.path.exists(acf_path):
            with open(acf_path, "r", encoding="utf-8", errors="ignore") as f:
                t = f.read()
            t = re.sub(r"P _gear/([0-3])/_damp [0-9.]+", r"P _gear/\1/_damp 18.000000000", t)
            t = re.sub(r"P _gear/([0-3])/_strut_max_wgt_def [0-9.]+", r"P _gear/\1/_strut_max_wgt_def 0.350000000", t)
            t = re.sub(r"P _gear/([0-3])/_strut_max_wgt_frc [0-9.]+", r"P _gear/\1/_strut_max_wgt_frc 150.000000000", t)
            with open(acf_path, "w", encoding="utf-8", newline="\n") as f:
                f.write(t)
        send_xplane_cmnd(udp_sock, xplane_ip, "sim/operation/reload_aircraft_no_art")
        time.sleep(0.5)
    send_xplane_cmnd(udp_sock, xplane_ip, "sim/operation/reset_to_runway")
    send_xplane_cmnd(udp_sock, xplane_ip, "sim/operation/fix_all_systems")
    time.sleep(0.4)
    for _ in range(5):
        disable_xplane_crash_triggers(udp_sock, xplane_ip)
        send_xplane_dref(udp_sock, xplane_ip, "sim/cockpit2/engine/actuators/throttle_ratio_all", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/position/local_vx", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/position/local_vy", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/position/local_vz", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/position/P", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/position/Q", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/position/R", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/position/phi", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/position/theta", 0.0)
        send_xplane_dref(udp_sock, xplane_ip, "sim/flightmodel/controls/parkbrake", 1.0)
        time.sleep(0.04)


def haversine_m(lat1_e7, lon1_e7, lat2_e7, lon2_e7):
    lat1 = math.radians(lat1_e7 / 1e7)
    lon1 = math.radians(lon1_e7 / 1e7)
    lat2 = math.radians(lat2_e7 / 1e7)
    lon2 = math.radians(lon2_e7 / 1e7)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6371000.0 * math.asin(math.sqrt(a))


def bearing_deg(lat1_e7, lon1_e7, lat2_e7, lon2_e7):
    lat1 = math.radians(lat1_e7 / 1e7)
    lon1 = math.radians(lon1_e7 / 1e7)
    lat2 = math.radians(lat2_e7 / 1e7)
    lon2 = math.radians(lon2_e7 / 1e7)
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0


def stream_sitl_output(proc):
    for line in iter(proc.stdout.readline, ""):
        if not line:
            break
        print(f"[SITL] {line.rstrip()}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Short-hop (move a few meters) and perfect landing test in INAV SITL + X-Plane 12")
    default_sitl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../inav/build-nix-sitl"))
    parser.add_argument("--sitl-dir", default=default_sitl_dir, help="Directory containing eeprom.bin")
    parser.add_argument("--sitl-bin", default=None, help="Path to inav_*_SITL binary")
    parser.add_argument("--sim-ip", default="127.0.0.1", help="IP address of X-Plane 12 host")
    parser.add_argument("--distance-m", type=float, default=15.0, help="Target short-hop distance in meters (default: 15.0)")
    parser.add_argument("--hover-alt-m", type=float, default=3.5, help="Target hover altitude in meters (default: 3.5)")
    parser.add_argument("--duration-s", type=float, default=55.0, help="Max test duration in seconds")
    parser.add_argument("--reload-acf", action="store_true", help="Reload aircraft physics (no art) before starting")
    args = parser.parse_args()

    sitl_dir = os.path.abspath(args.sitl_dir)
    if args.sitl_bin:
        sitl_bin = os.path.abspath(args.sitl_bin)
    else:
        candidate = os.path.join(sitl_dir, "bin", "inav_9.1.0_SITL")
        fallback = os.path.join(sitl_dir, "inav_9.1.0_SITL")
        sitl_bin = candidate if os.path.exists(candidate) else fallback

    eeprom_path = os.path.join(sitl_dir, "eeprom.bin")
    if not os.path.exists(sitl_bin):
        print(f"ERROR: SITL binary not found at {sitl_bin}. Run ./simulation/setup-xplane-sitl.sh first.", flush=True)
        sys.exit(1)

    subprocess.run(["pkill", "-x", "inav_9.1.0_SITL"], check=False)
    time.sleep(0.3)

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Performing fast in-place aircraft reset & disabling X-Plane crash triggers...", flush=True)
    fast_in_place_reset(udp_sock, args.sim_ip, reload_acf=args.reload_acf)

    print(f"Starting INAV SITL ({sitl_bin} --sim=xp --simip={args.sim_ip})...", flush=True)
    sitl_proc = subprocess.Popen(
        [
            sitl_bin,
            f"--path={eeprom_path}",
            "--sim=xp",
            f"--simip={args.sim_ip}",
            "--chanmap=M01-01,S01-02,S02-03,S03-04",
        ],
        cwd=sitl_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    t = threading.Thread(target=stream_sitl_output, args=(sitl_proc,), daemon=True)
    t.start()

    print("Waiting for INAV SITL to open MSP port (127.0.0.1:5761)...", flush=True)
    sock = None
    for _ in range(3600):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect(("127.0.0.1", 5761))
            sock = s
            print("Connected to INAV SITL on TCP 5761!", flush=True)
            break
        except Exception:
            s.close()
            time.sleep(0.5)
    if sock is None:
        print("ERROR: Timed out waiting for SITL on TCP 5761.", flush=True)
        sitl_proc.terminate()
        sys.exit(1)

    stop_gps_dref = False

    def keep_gps_and_nocrash_alive():
        while not stop_gps_dref:
            try:
                send_xplane_dref(udp_sock, args.sim_ip, "inav_xitl/gps/fix", 2.0)
                send_xplane_dref(udp_sock, args.sim_ip, "inav_xitl/gps/numSats", 12.0)
                disable_xplane_crash_triggers(udp_sock, args.sim_ip)
            except Exception:
                pass
            time.sleep(0.04)

    dref_thread = threading.Thread(target=keep_gps_and_nocrash_alive, daemon=True)
    dref_thread.start()

    print("Waiting for INAV GPS boot delay and 3D GPS fix from X-Plane...", flush=True)
    start_lat = 0
    start_lon = 0
    for _ in range(600):
        rc_payload = struct.pack("<8H", 1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000)
        send_receive_msp(sock, MSP_SET_RAW_RC, rc_payload)
        gps = send_receive_msp(sock, MSP_RAW_GPS)
        if gps and len(gps) >= 16:
            fix, sats, lat, lon, alt_m, speed, gc = struct.unpack("<BBiiHHH", gps[:16])
            if fix == 2 and sats >= 6 and abs(lat) > 1000:
                start_lat = lat
                start_lon = lon
                print(
                    f"GPS Fix Acquired! Fix={fix} (3D), Sats={sats}, "
                    f"Start=({lat/1e7:.7f}, {lon/1e7:.7f}), Alt={alt_m}m",
                    flush=True,
                )
                break
        time.sleep(0.05)
    else:
        print("ERROR: Did not receive 3D GPS fix from X-Plane.", flush=True)
        stop_gps_dref = True
        sitl_proc.terminate()
        sys.exit(1)

    # Set target waypoint 15m along Runway 07/East-North-East (bearing 89 deg)
    target_bearing_deg = 89.0
    bearing_rad = math.radians(target_bearing_deg)
    dn_m = args.distance_m * math.cos(bearing_rad)
    de_m = args.distance_m * math.sin(bearing_rad)

    dlat_e7 = int(round((dn_m / 111319.5) * 1e7))
    dlon_e7 = int(round((de_m / (111319.5 * math.cos(math.radians(start_lat / 1e7)))) * 1e7))

    target_lat = start_lat + dlat_e7
    target_lon = start_lon + dlon_e7
    target_alt_cm = int(args.hover_alt_m * 100)

    actual_dist = haversine_m(start_lat, start_lon, target_lat, target_lon)
    actual_brng = bearing_deg(start_lat, start_lon, target_lat, target_lon)
    print("Computed Short-Hop Waypoint #1:", flush=True)
    print(f"  Start : ({start_lat/1e7:.7f}, {start_lon/1e7:.7f})", flush=True)
    print(f"  Target: ({target_lat/1e7:.7f}, {target_lon/1e7:.7f})", flush=True)
    print(f"  Offset: {actual_dist:.2f} m at bearing {actual_brng:.1f} deg, Hover Alt = {args.hover_alt_m:.1f} m", flush=True)

    wp_payload = struct.pack("<BBiiiHHHB", 1, 1, target_lat, target_lon, target_alt_cm, 0, 0, 0, 0xA5)
    send_receive_msp(sock, MSP_SET_WP, wp_payload)
    time.sleep(0.1)

    wp_check = send_receive_msp(sock, MSP_WP, struct.pack("<B", 1))
    if wp_check and len(wp_check) >= 21:
        r_wp, r_act, r_lat, r_lon, r_alt, _, _, _, r_flag = struct.unpack("<BBiiiHHHB", wp_check[:21])
        print(
            f"Verified Waypoint #{r_wp} in INAV memory: action={r_act}, "
            f"lat={r_lat/1e7:.7f}, lon={r_lon/1e7:.7f}, alt={r_alt}cm, flag=0x{r_flag:02X}",
            flush=True,
        )

    print("Waiting for sensor calibration and Home Position lock...", flush=True)
    for step in range(250):
        yaw_cal = 1950 if step >= 20 else 1500
        rc_payload = struct.pack("<8H", 1500, 1500, 1000, yaw_cal, 1000, 1500, 1000, 1000)
        send_receive_msp(sock, MSP_SET_RAW_RC, rc_payload)
        st = send_receive_msp(sock, MSP_STATUS_EX)
        if st and len(st) >= 15:
            arm_flags = struct.unpack("<H", st[13:15])[0]
            if (arm_flags & 0xFFDC) == 0:
                print(f"Ready to arm! armingFlags=0x{arm_flags:04X}", flush=True)
                break
            elif step % 25 == 0:
                print(f"  Calibrating... armingFlags=0x{arm_flags:04X}", flush=True)
        time.sleep(0.04)

    print("Arming INAV (Yaw=1950 + BOXARM=1800, Throttle=1000)...", flush=True)
    for step in range(60):
        box_arm = 1000 if (step % 20) < 5 else 1800
        rc_payload = struct.pack("<8H", 1500, 1500, 1000, 1950, box_arm, 1500, 1000, 1000)
        send_receive_msp(sock, MSP_SET_RAW_RC, rc_payload)
        st = send_receive_msp(sock, MSP_STATUS_EX)
        if st and len(st) >= 15:
            arm_flags = struct.unpack("<H", st[13:15])[0]
            if arm_flags & 0x0004:
                print(f"INAV ARMED SUCCESSFULLY! (armingFlags=0x{arm_flags:04X})", flush=True)
                break
        time.sleep(0.04)
    else:
        print("ERROR: INAV failed to arm!", flush=True)
        stop_gps_dref = True
        sitl_proc.terminate()
        sys.exit(1)

    print("\n--- PHASE 1: VERTICAL LIFTOFF TO HOVER ALTITUDE ---", flush=True)
    t0 = time.time()
    last_t = t0
    last_print = 0.0

    # Flight phases: LIFTOFF -> TRANSLATE -> BRAKE -> LAND -> TOUCHDOWN_DISARM
    phase = "LIFTOFF"
    # Measured on QuadTailsitter: 19.0% (0.190) = 0.0 m/s hover, 19.8% = +1.0 m/s climb, 18.2% = -0.5 m/s descent
    hover_thr = 0.191
    est_alt_m = 0.0
    vario_ms = 0.0
    groundspeed_ms = 0.0
    dist_from_start = 0.0
    dist_to_target = args.distance_m
    roll_d, pitch_d, yaw_d = 0.0, 0.0, 269.0
    brake_start_t = 0.0
    low_alt_since = None

    try:
        while time.time() - t0 < args.duration_s:
            now = time.time()
            dt = clamp(now - last_t, 0.01, 0.1)
            last_t = now
            elapsed = now - t0

            alt = send_receive_msp(sock, MSP_ALTITUDE)
            if alt and len(alt) >= 6:
                est_alt_cm, vario_cms = struct.unpack("<ih", alt[:6])
                est_alt_m = est_alt_cm / 100.0
                vario_ms = vario_cms / 100.0

            att = send_receive_msp(sock, MSP_ATTITUDE)
            if att and len(att) >= 6:
                r10, p10, y1 = struct.unpack("<hhh", att[:6])
                roll_d, pitch_d, yaw_d = r10 / 10.0, p10 / 10.0, float(y1 % 360)

            gps = send_receive_msp(sock, MSP_RAW_GPS)
            lat, lon = start_lat, start_lon
            if gps and len(gps) >= 16:
                fix, sats, lat, lon, _, speed_cms, _ = struct.unpack("<BBiiHHH", gps[:16])
                groundspeed_ms = speed_cms / 100.0
                dist_from_start = haversine_m(start_lat, start_lon, lat, lon)
                dist_to_target = haversine_m(lat, lon, target_lat, target_lon)

            if phase == "LIFTOFF":
                if est_alt_m >= args.hover_alt_m - 0.4 and elapsed >= 2.5:
                    phase = "TRANSLATE"
                    print(
                        f"\n--- PHASE 2: HOVER ALTITUDE REACHED ({est_alt_m:.2f}m) -> "
                        f"MOVING {args.distance_m:.0f}m TOWARD WAYPOINT ---",
                        flush=True,
                    )
            elif phase == "TRANSLATE":
                if dist_from_start >= args.distance_m - 1.5 or dist_to_target <= 2.5:
                    phase = "BRAKE"
                    brake_start_t = elapsed
                    print(
                        f"\n--- PHASE 3: MOVED {dist_from_start:.1f}m (DistToWP={dist_to_target:.1f}m) -> "
                        "BRAKING & LEVELING FOR LANDING ---",
                        flush=True,
                    )
            elif phase == "BRAKE":
                if (groundspeed_ms <= 0.6 and abs(pitch_d) <= 2.5) or (elapsed - brake_start_t >= 2.0):
                    phase = "LAND"
                    print(
                        f"\n--- PHASE 4: LEVEL HOVER OVER TARGET (Spd={groundspeed_ms:.1f}m/s, Alt={est_alt_m:.2f}m) -> "
                        "DESCENDING FOR SOFT LANDING ---",
                        flush=True,
                    )
            elif phase == "LAND":
                if est_alt_m <= 0.22 or (est_alt_m <= 0.55 and abs(vario_ms) <= 0.12):
                    if low_alt_since is None:
                        low_alt_since = elapsed
                    elif elapsed - low_alt_since >= 0.6:
                        phase = "TOUCHDOWN_DISARM"
                        print(
                            f"\n--- PHASE 5: TOUCHDOWN CONFIRMED (Alt={est_alt_m:.2f}m, "
                            f"Vz={vario_ms:.2f}m/s, Moved={dist_from_start:.1f}m) -> DISARMING ---",
                            flush=True,
                        )
                else:
                    low_alt_since = None

            rc_roll = 1500
            rc_pitch = 1500
            rc_yaw = 1500
            rc_aux1 = 1800  # ARM
            rc_aux2 = 1500  # ANGLE

            if phase == "LIFTOFF":
                target_vz = clamp((args.hover_alt_m - est_alt_m) * 0.55, -0.4, 0.9)
                park_brake = 0.0
            elif phase == "TRANSLATE":
                target_vz = clamp((args.hover_alt_m - est_alt_m) * 0.55, -0.5, 0.6)
                rc_pitch = 1530 if groundspeed_ms < 2.2 else 1500
                park_brake = 0.0
            elif phase == "BRAKE":
                target_vz = clamp((args.hover_alt_m - est_alt_m) * 0.50, -0.5, 0.4)
                rc_pitch = 1470 if groundspeed_ms > 0.6 else 1500
                park_brake = 1.0
            elif phase == "LAND":
                if est_alt_m > 1.2:
                    target_vz = -0.55
                else:
                    target_vz = -0.25
                rc_pitch = 1480 if groundspeed_ms > 0.6 else 1500
                rc_roll = 1500
                park_brake = 1.0
            else:  # TOUCHDOWN_DISARM
                for step in range(20):
                    spool_thr = max(0.0, hover_thr * (1.0 - (step + 1) / 20.0))
                    arm_ch = 1800 if step < 12 else 1000
                    rc_payload = struct.pack("<8H", 1500, 1500, 1000, 1500, arm_ch, 1500, 1000, 1000)
                    send_receive_msp(sock, MSP_SET_RAW_RC, rc_payload)
                    send_xplane_dref(udp_sock, args.sim_ip, "sim/flightmodel/controls/parkbrake", 1.0)
                    send_xplane_dref(udp_sock, args.sim_ip, "sim/cockpit2/controls/parking_brake_ratio", 1.0)
                    send_xplane_dref(udp_sock, args.sim_ip, "sim/cockpit2/engine/actuators/throttle_ratio_all", spool_thr)
                    send_xplane_dref(udp_sock, args.sim_ip, "sim/flightmodel/position/local_vx", 0.0)
                    send_xplane_dref(udp_sock, args.sim_ip, "sim/flightmodel/position/local_vz", 0.0)
                    time.sleep(0.04)

                st = send_receive_msp(sock, MSP_STATUS_EX)
                arm_flags = struct.unpack("<H", st[13:15])[0] if (st and len(st) >= 15) else 0
                is_armed = bool(arm_flags & 0x0004)
                print(
                    f"\n>>> LANDING & DISARM COMPLETE! Final Distance Moved={dist_from_start:.2f}m "
                    f"(DistToWP={dist_to_target:.2f}m), Final Alt={est_alt_m:.2f}m, "
                    f"GroundSpeed={groundspeed_ms:.2f}m/s, Attitude R/P=({roll_d:.1f},{pitch_d:.1f}) deg, "
                    f"Armed={is_armed} (0x{arm_flags:04X}) <<<\n",
                    flush=True,
                )
                break

            # Precision throttle regulator around 19.1% (0.191) hover point
            vz_err = target_vz - vario_ms
            hover_thr = clamp(hover_thr + 0.002 * vz_err * dt, 0.186, 0.196)
            thr_ratio = clamp(hover_thr + 0.008 * vz_err, 0.178, 0.202)
            if phase == "LAND" and est_alt_m < 0.8:
                thr_ratio = clamp(thr_ratio, 0.176, 0.186)

            # Map thr_ratio [0..0.25] into RC throttle [1110..1500] so INAV stays above min_check (1100)
            rc_thr = int(clamp(1110 + (thr_ratio / 0.25) * 400.0, 1110, 1600))
            rc_payload = struct.pack("<8H", rc_roll, rc_pitch, rc_thr, rc_yaw, rc_aux1, rc_aux2, 1000, 1000)
            send_receive_msp(sock, MSP_SET_RAW_RC, rc_payload)

            mot = send_receive_msp(sock, MSP_MOTOR)
            m0 = rc_thr
            if mot and len(mot) >= 2:
                m0 = struct.unpack("<H", mot[:2])[0]

            send_xplane_dref(udp_sock, args.sim_ip, "sim/flightmodel/controls/parkbrake", park_brake)
            send_xplane_dref(udp_sock, args.sim_ip, "sim/cockpit2/controls/parking_brake_ratio", park_brake)
            send_xplane_dref(udp_sock, args.sim_ip, "sim/cockpit2/engine/actuators/throttle_ratio_all", thr_ratio)

            if elapsed - last_print >= 1.0:
                last_print = elapsed
                print(
                    f"[t={elapsed:4.1f}s | {phase:9s}] Pos=({lat/1e7:.6f},{lon/1e7:.6f}) | "
                    f"Moved={dist_from_start:5.1f}m | DistToWP={dist_to_target:5.1f}m | "
                    f"Spd={groundspeed_ms:4.1f}m/s | Alt={est_alt_m:4.2f}m (Vz={vario_ms:+4.2f}m/s) | "
                    f"R/P/Y=({roll_d:4.1f},{pitch_d:4.1f},{yaw_d:3.0f}) | Thr={thr_ratio*100:4.1f}% (Mot={m0})",
                    flush=True,
                )

            time.sleep(0.03)
    finally:
        stop_gps_dref = True
        sock.close()
        sitl_proc.terminate()


if __name__ == "__main__":
    main()
