import asyncio
from drone_interface import Drone


async def main():
    # Initialize drone connection
    drone = Drone(system_address="udp://:14540")
    
    # Connect to the drone
    await drone.connect()
    
    # Land the drone
    await drone.land()
    
    print("--> Mission complete!")


if __name__ == "__main__":
    asyncio.run(main())
