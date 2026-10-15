## Building SITL

### Compiler requirements

* Modern GCC. Must be a *real* GCC, unless you're using MacOS; faking it with clang will not work. gcc13 or later is recommended.
* Unix sockets networking. Cygwin is required on Windows (vice `winsock`).
* Pthreads


### Nix / NixOS / WSL2 (Reproducible Build):

This repository includes a reproducible Nix build environment (`shell.nix`) with helper scripts under `nix/`:

```bash
# Configure and compile the INAV SITL binary (inav_9.1.0_SITL) in ./build-nix-sitl/
./nix/build-nix.sh SITL
```

You can also run the CMake configuration step explicitly:

```bash
./nix/configure-nix.sh --sitl
./nix/build-nix.sh SITL
```

The resulting binary is generated at `./build-nix-sitl/bin/inav_9.1.0_SITL`.


### Linux and FreeBSD:

Almost like normal, ruby, cmake and make are also required.
With cmake, the option "-DSITL=ON" must be specified.

```
mkdir build_SITL
cd build_SITL
cmake -DSITL=ON ..
make
```

### Windows:

Compile under cygwin, using the Linux instructions. Note that depending on the Cygwin packaging _du jour_ of `ruby` it _may_ also be necessary to:

* Install the `rubygems` package.
* run `gem install getoptlong`

Copy cygwin1.dll into the directory, or include cygwin's /bin/ directory in the environment variable PATH.

If the build fails (segfault, possibly out of memory), adding `-DCMAKE_BUILD_TYPE=MinRelSize` to the `cmake` command may help.

### Build manager

`ninja` may also be used (parallel builds without `-j $(nproc)`):

```
cmake -GNinja -DSITL=ON ..
ninja
```

### Supported environments

* Linux on x86_64, ia-32, Aarch64 (e.g. Rpi), RISCV64 (e.g. VisionFive2)
* Windows on x86_64
* FreeBSD x86_64 (at least).
* MacOS on x86_64 and Aarch64

---

## Running SITL with X-Plane 12 & Autonomous Waypoint Testing (`test_nav_200m_ne.py`)

> [!IMPORTANT]
> **This is for drone in small tests, you should adapt it if there is anything wrong or whatever you consider to apply.**

When testing INAV navigation (`MSP_SET_WP`, `MSP_SET_RAW_RC`, `NAV WP`, or custom navigation adjustments) against **X-Plane 12**:

1. **Install the X-Plane Plugin (`INAV-X-Plane-XITL`)**:
   Download the latest release zip from [`Scavanger/INAV-X-Plane-XITL/releases`](https://github.com/Scavanger/INAV-X-Plane-XITL/releases) and extract `INAV-X-Plane-XITL` into `<X-Plane 12>/Resources/plugins/`.
2. **Pre-configure the SITL `eeprom.bin`**:
   Run [`./nix/sitl/configure_sitl_eeprom.py`](../../nix/sitl/configure_sitl_eeprom.py) so that:
   * `UART2` (TCP port `5761`) is enabled for MSP companion commands while `UART1` (TCP port `5760`) remains available for the X-Plane plugin/CLI.
   * `receiver_type = MSP` is enabled instead of `SIM` (so `MSP_SET_RAW_RC` commands are accepted and `ARMING_DISABLED_RC_LINK` is cleared).
   * `nav_wp_max_safe_distance = 500` is set (allowing 200m+ first waypoints without triggering `ARMING_DISABLED_NAVIGATION_UNSAFE`).
   * Motor (`M01`) and Servo (`S01..S03`) mixers match X-Plane's yoke sign conventions (`--chanmap=M01-01,S01-02,S02-03,S03-04`).
3. **Run the 200m North-East Autonomous Navigation Test (`test_nav_200m_ne.py`)**:
   With X-Plane 12 open and an aircraft loaded on the ground, run:
   ```bash
   python3 ./nix/sitl/test_nav_200m_ne.py
   ```
   Or pass a custom X-Plane host IP / distance / bearing:
   ```bash
   python3 ./nix/sitl/test_nav_200m_ne.py --sim-ip 127.0.0.1 --distance-m 200 --bearing-deg 55
   ```

### Reference Script: `test_nav_200m_ne.py`

> **Note:** This is for drone in small tests, you should adapt it if there is anything wrong or whatever you consider to apply.

The full script is located at [`nix/sitl/test_nav_200m_ne.py`](../../nix/sitl/test_nav_200m_ne.py) and shown below for reference:

```python
#!/usr/bin/env python3
"""
Autonomous 200m North-East GPS Waypoint Navigation Test for INAV SITL + X-Plane 12.

Note: This is for drone in small tests, you should adapt it if there is anything
wrong or whatever you consider to apply.

Connects to INAV SITL over MSP (TCP 5761), reads the current GPS position from X-Plane,
computes a target waypoint 200 meters North-East (bearing 55 deg), uploads Waypoint #1
via MSP_SET_WP, arms INAV, engages NAV WP mode, and logs live flight progress.
"""

import argparse
import math
import os
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
    parser = argparse.ArgumentParser(description="Run 200m North-East GPS waypoint test in INAV SITL + X-Plane 12")
    default_sitl_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../build-nix-sitl"))
    parser.add_argument("--sitl-dir", default=default_sitl_dir, help="Directory containing eeprom.bin")
    parser.add_argument("--sitl-bin", default=None, help="Path to inav_*_SITL binary")
    parser.add_argument("--sim-ip", default="127.0.0.1", help="IP address of X-Plane 12 host")
    parser.add_argument("--distance-m", type=float, default=200.0, help="Target distance in meters (default: 200.0)")
    parser.add_argument("--bearing-deg", type=float, default=55.0, help="Target bearing in degrees (default: 55.0 NE)")
    parser.add_argument("--duration-s", type=float, default=90.0, help="Max test duration in seconds")
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
        print(f"ERROR: SITL binary not found at {sitl_bin}. Run ./nix/build-nix.sh SITL first.", flush=True)
        sys.exit(1)

    subprocess.run(["pkill", "-x", "inav_9.1.0_SITL"], check=False)
    time.sleep(0.5)

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

    print("Waiting for X-Plane 12 flight to load and INAV SITL to open MSP port (127.0.0.1:5761)...", flush=True)
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

    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop_gps_dref = False

    def keep_gps_dref_alive():
        while not stop_gps_dref:
            try:
                send_xplane_dref(udp_sock, args.sim_ip, "inav_xitl/gps/fix", 2.0)
                send_xplane_dref(udp_sock, args.sim_ip, "inav_xitl/gps/numSats", 12.0)
            except Exception:
                pass
            time.sleep(0.02)

    dref_thread = threading.Thread(target=keep_gps_dref_alive, daemon=True)
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

    bearing_rad = math.radians(args.bearing_deg)
    dn_m = args.distance_m * math.cos(bearing_rad)
    de_m = args.distance_m * math.sin(bearing_rad)

    dlat_e7 = int(round((dn_m / 111319.5) * 1e7))
    dlon_e7 = int(round((de_m / (111319.5 * math.cos(math.radians(start_lat / 1e7)))) * 1e7))

    target_lat = start_lat + dlat_e7
    target_lon = start_lon + dlon_e7
    target_alt_cm = 3500

    actual_dist = haversine_m(start_lat, start_lon, target_lat, target_lon)
    actual_brng = bearing_deg(start_lat, start_lon, target_lat, target_lon)
    print("Computed Target Waypoint #1:", flush=True)
    print(f"  Start : ({start_lat/1e7:.7f}, {start_lon/1e7:.7f})", flush=True)
    print(f"  Target: ({target_lat/1e7:.7f}, {target_lon/1e7:.7f})", flush=True)
    print(f"  Offset: {actual_dist:.2f} m at bearing {actual_brng:.1f} deg, Rel Alt = {target_alt_cm/100:.1f} m", flush=True)

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
        rc_payload = struct.pack("<8H", 1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000)
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

    print("Arming INAV (BOXARM=1800, Throttle=1000)...", flush=True)
    for step in range(50):
        yaw_arm = 1950 if step >= 10 else 1500
        rc_payload = struct.pack("<8H", 1500, 1500, 1000, yaw_arm, 1800, 1500, 1000, 1000)
        send_receive_msp(sock, MSP_SET_RAW_RC, rc_payload)
        st = send_receive_msp(sock, MSP_STATUS_EX)
        if st and len(st) >= 15:
            arm_flags = struct.unpack("<H", st[13:15])[0]
            if arm_flags & 0x0004:
                print(f"INAV ARMED SUCCESSFULLY! (armingFlags=0x{arm_flags:04X})", flush=True)
                break
        time.sleep(0.04)

    print("\n--- TAKING OFF & ENGAGING NAV WP TOWARD TARGET ---", flush=True)
    t0 = time.time()
    last_print = 0.0
    reached_target = False

    try:
        while time.time() - t0 < args.duration_s:
            elapsed = time.time() - t0
            if not reached_target:
                rc_payload = struct.pack("<8H", 1500, 1500, 1950, 1500, 1800, 1500, 1800, 1000)
            else:
                rc_payload = struct.pack("<8H", 1500, 1500, 1100, 1500, 1800, 1500, 1000, 1000)

            send_receive_msp(sock, MSP_SET_RAW_RC, rc_payload)

            mot = send_receive_msp(sock, MSP_MOTOR)
            m0 = 1000
            if mot and len(mot) >= 2:
                m0 = struct.unpack("<H", mot[:2])[0]
                thr_ratio = max(0.0, min(1.0, (m0 - 1000.0) / 1000.0))
                send_xplane_dref(udp_sock, args.sim_ip, "sim/flightmodel/controls/parkbrake", 0.0)
                send_xplane_dref(udp_sock, args.sim_ip, "sim/cockpit2/controls/parking_brake_ratio", 0.0)
                send_xplane_dref(udp_sock, args.sim_ip, "sim/cockpit2/engine/actuators/throttle_ratio_all", thr_ratio)

            if elapsed - last_print >= 1.0:
                last_print = elapsed
                gps = send_receive_msp(sock, MSP_RAW_GPS)
                att = send_receive_msp(sock, MSP_ATTITUDE)
                alt = send_receive_msp(sock, MSP_ALTITUDE)
                nav = send_receive_msp(sock, MSP_NAV_STATUS)

                if gps and len(gps) >= 16:
                    fix, sats, lat, lon, alt_m, speed_cms, gc_deg10 = struct.unpack("<BBiiHHH", gps[:16])
                    dist_from_start = haversine_m(start_lat, start_lon, lat, lon)
                    brng_from_start = bearing_deg(start_lat, start_lon, lat, lon)
                    dist_to_target = haversine_m(lat, lon, target_lat, target_lon)

                    roll_d, pitch_d, yaw_d = (0.0, 0.0, 0.0)
                    if att and len(att) >= 6:
                        r10, p10, y1 = struct.unpack("<hhh", att[:6])
                        roll_d, pitch_d, yaw_d = r10 / 10.0, p10 / 10.0, y1

                    est_alt_m = 0.0
                    if alt and len(alt) >= 4:
                        est_alt_m = struct.unpack("<i", alt[:4])[0] / 100.0

                    nav_state, nav_err = (0, 0)
                    if nav and len(nav) >= 7:
                        _, nav_state, _, _, nav_err, _ = struct.unpack("<BBBBBH", nav[:7])

                    print(
                        f"[t={elapsed:4.1f}s] Pos=({lat/1e7:.6f},{lon/1e7:.6f}) | "
                        f"Moved={dist_from_start:5.1f}m (brng {brng_from_start:5.1f} deg) | "
                        f"DistToTarget={dist_to_target:5.1f}m | Spd={speed_cms/100.0:4.1f}m/s | "
                        f"Alt={est_alt_m:4.1f}m | R/P/Y=({roll_d:4.1f},{pitch_d:4.1f},{yaw_d:3.0f}) | "
                        f"Mot={m0} | NavState={nav_state} (Err={nav_err})",
                        flush=True,
                    )

                    if not reached_target and dist_to_target <= 25.0:
                        reached_target = True
                        print(
                            f"\n>>> TARGET REACHED (Within {dist_to_target:.1f}m of 200m NE waypoint)! "
                            "Cutting throttle for landing... <<<\n",
                            flush=True,
                        )
            time.sleep(0.04)
    finally:
        stop_gps_dref = True
        sock.close()
        sitl_proc.terminate()


if __name__ == "__main__":
    main()
```
