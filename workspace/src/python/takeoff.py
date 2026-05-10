import asyncio
from drone_interface import Drone


async def main():
    # Initialize drone connection
    drone = Drone(system_address="serial:///dev/ttyACM0:115200")  #drone = Drone(system_address="udp://:14540")
    
    # Connect to the drone
    await drone.connect()
    
    # Check pre-flight health
    if not await drone.check_health():
        print("!! Pre-flight checks failed. Aborting.")
        return
    
    # Arm and takeoff to 10 meters
    await drone.arm_and_takeoff(altitude_meters=10)
    print("--> Mission complete!")


if __name__ == "__main__":
    asyncio.run(main())
