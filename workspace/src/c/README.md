# Low-Level C Control Code

This directory contains low-level C programs for direct drone control using MAVLink.

## Files

- `simple_takeoff.c` - Basic takeoff command
- `simple_landing.c` - Landing command
- `rethome.c` - Return to home/launch
- `waypoints.c` - Waypoint navigation

## Compilation

Compile with MAVLink headers:

```bash
gcc -I../libs/mavlink -o ../bin/program_name program_name.c
```

## Usage

Programs communicate directly with the drone via MAVLink protocol on UDP port 14550.
