import asyncio
from drone_interface import Drone
from mavsdk.offboard import (OffboardError, Attitude)


async def main():
    # !! IMPORTANT: Close QGC before running — it locks the serial port !!

    # Initialize drone connection
    drone = Drone(system_address="serial:///dev/ttyACM0:115200")

    # Connect to the drone
    await drone.connect()

    # Check pre-flight health
    if not await drone.check_health():
        print("!! Pre-flight checks failed. Aborting.")
        return

    # Disable GPS requirements
    print("--> Disabling GPS requirements...")
    await drone.drone.param.set_param_int("COM_ARM_WO_GPS", 1)
    await drone.drone.param.set_param_int("SYS_HAS_GPS", 0)
    await drone.drone.param.set_param_int("EKF2_GPS_CTRL", 0)
    await drone.drone.param.set_param_int("EKF2_HGT_REF", 0)
    await drone.drone.param.set_param_int("EKF2_BARO_CTRL", 1)

    # --- Offboard with Attitude control (works without GPS/EKF position) ---
    # Attitude(roll_deg, pitch_deg, yaw_deg, thrust_value)
    # thrust_value: 0.0 = no thrust, 1.0 = full thrust

    # Step 1: Set initial setpoint BEFORE starting offboard
    print("--> Setting initial attitude setpoint (zero thrust)...")
    await drone.drone.offboard.set_attitude(
        Attitude(0.0, 0.0, 0.0, 0.0)  # Level attitude, no thrust
    )

    # Step 2: Start offboard mode
    try:
        await drone.drone.offboard.start()
        print("--> Offboard mode ACTIVE")
    except OffboardError as e:
        print(f"!! Failed to start offboard mode: {e}")
        return

    # Step 3: Arm
    print("--> Arming...")
    await drone.drone.action.arm()
    await asyncio.sleep(1)

    # Step 4: Spin motors at low thrust (DO NOT FLY — test on bench first!)
    # 0.3 = ~30% thrust — enough to spin props, not enough to lift
    print("--> Motors LOW thrust (30%) — bench test...")
    await drone.drone.offboard.set_attitude(
        Attitude(0.0, 0.0, 0.0, 0.3)
    )
    await asyncio.sleep(5)

    # Step 5: Stop motors
    print("--> Stopping motors...")
    await drone.drone.offboard.set_attitude(
        Attitude(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(2)

    # Stop offboard and disarm
    try:
        await drone.drone.offboard.stop()
    except:
        pass

    print("--> Disarming...")
    await drone.drone.action.disarm()
    print("--> Mission complete!")


if __name__ == "__main__":
    asyncio.run(main())
