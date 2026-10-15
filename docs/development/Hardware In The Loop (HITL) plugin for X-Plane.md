# Hardware In The Loop (HITL) & Software In The Loop (SITL) plugin for X-Plane 11/12

**Hardware-in-the-loop (HITL) simulation**, is a technique that is used in the development and testing of complex real-time embedded systems. 

**X-Plane** is a flight simulation engine series developed and published by Laminar Research https://www.x-plane.com/

**INAV-X-Plane-HITL** is plugin for **X-Plane** for testing and developing flight controllers with **INAV flight controller firmware** 

https://github.com/RomanLut/INAV-X-Plane-HITL

or for latest HITL / SITL features (INAV >= 9.0)

 **INAV-X-Plane-XITL** 
 
 https://github.com/Scavanger/INAV-X-Plane-XITL

HITL technique can be used to test features during development. Please check page above for installation instructions.

---

## Software-In-The-Loop (SITL) Small Drone Testing with X-Plane 12

> **Note:** This is for drone in small tests, you should adapt it if there is anything wrong or whatever you consider to apply.

You can also run INAV as a native Linux/WSL2 executable (`inav_9.1.0_SITL`) connected directly to **X-Plane 12** (`--sim=xp`) without physical flight controller hardware:

1. Build SITL via Nix:
   ```bash
   ./nix/build-nix.sh SITL
   ```
2. Configure the SITL virtual EEPROM (`./build-nix-sitl/eeprom.bin`):
   ```bash
   python3 ./nix/sitl/configure_sitl_eeprom.py
   ```
3. Run the 200m North-East autonomous GPS waypoint test script against X-Plane 12:
   ```bash
   python3 ./nix/sitl/test_nav_200m_ne.py
   ```

See **[Building SITL.md](Building%20SITL.md)** for full details and the complete `test_nav_200m_ne.py` script.
