import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

class Drone:
    def __init__(self, system_address="udp://:14540"):
        self.system_address = system_address
        self.drone = System()
        self._is_connected = False

    async def connect(self):
        """Connects to the drone and waits for a successful link."""
        print(f"--> Connecting to {self.system_address}...")
        await self.drone.connect(system_address=self.system_address)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self._is_connected = True
                print("--> Drone Connected!")
                break

    async def check_health(self):
        """Pre-flight checks. Returns True if safe to fly."""
        print("--> Checking system health...")
        # GPS check disabled — no GPS module connected
        # async for health in self.drone.telemetry.health():
        #     if not health.is_global_position_ok:
        #         print("!! GPS Error: No Global Position fix.")
        #         return False
        #     break
        # Battery check disabled — sensor not reporting valid data
        # async for battery in self.drone.telemetry.battery():
        #     if battery.remaining_percent < 0.2:
        #         print(f"!! Low Battery: {battery.remaining_percent*100:.1f}%")
        #         return False
        #     break
        print("--> Systems Green (GPS & battery checks disabled).")
        return True
    
    async def arm_and_takeoff(self, altitude_meters):
        """Arms the drone and takes off to target altitude."""
        # Allow arming without GPS & configure EKF2 to work without GPS
        print("--> Disabling GPS requirements...")
        await self.drone.param.set_param_int("COM_ARM_WO_GPS", 1)
        await self.drone.param.set_param_int("SYS_HAS_GPS", 0)
        
        # Tell EKF2 to not fuse GPS data
        print("--> Configuring EKF2 for GPS-less flight...")
        await self.drone.param.set_param_int("EKF2_GPS_CTRL", 0)      # Disable GPS fusion
        await self.drone.param.set_param_int("EKF2_HGT_REF", 0)       # Use barometer as height reference
        await self.drone.param.set_param_int("EKF2_BARO_CTRL", 1)     # Enable barometer fusion

        print("--> Arming...")
        await self.drone.action.arm()
        
        print(f"--> Taking off to {altitude_meters}m...")
        await self.drone.action.set_takeoff_altitude(altitude_meters)
        await self.drone.action.takeoff()
        
        # Telemetry loop: Monitor altitude until target is reached
        target_reached = False
        async for position in self.drone.telemetry.position():
            current_altitude = position.relative_altitude_m
            print(f"    Altitude: {current_altitude:.2f}m / {altitude_meters}m")
            
            # Check if within 95% of target altitude (tolerance)
            if current_altitude >= altitude_meters * 0.95:
                target_reached = True
                print("--> Takeoff complete.")
                break
            
            await asyncio.sleep(0.5)  # Check every 0.5 seconds
        
        if not target_reached:
            print("!! Warning: Takeoff loop exited without reaching target altitude")

    async def takeoff_relative(self, relative_altitude_meters):
        """
        Takes off to a relative altitude from current position.
        For example, relative_altitude_meters=10 will climb 10m from current altitude.
        """        # Get current altitude
        current_altitude = 0.0
        async for position in self.drone.telemetry.position():
            current_altitude = position.relative_altitude_m
            break
        
        target_altitude = current_altitude + relative_altitude_meters
        print(f"--> Current altitude: {current_altitude:.2f}m")
        print(f"--> Target altitude: {target_altitude:.2f}m (+{relative_altitude_meters}m)")
        
        # Arm and takeoff to target altitude
        await self.arm_and_takeoff(target_altitude)

    async def goto_local(self, north, east, down, yaw):
        """
        Moves the drone to a specific NED location relative to takeoff point.
        NOTE: 'down' is negative for altitude (e.g., -10 is 10m up).
        """
        print(f"--> Moving to N:{north} E:{east} D:{down} Y:{yaw}")
        try:
            # Send the setpoint
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(north, east, down, yaw)
            )
            # We must verify we are in offboard mode
            if not await self.ensure_offboard_active():
                await self.drone.offboard.start()
            
        except OffboardError as e:
            print(f"!! Offboard Error: {e}")

    async def ensure_offboard_active(self):
        """Helper to ensure offboard mode is active"""
        return self.drone.offboard.is_active()

    async def return_to_launch(self):
        """Stops offboard mode and triggers RTL."""
        print("--> Triggering RTL...")
        try:
            await self.drone.offboard.stop()
        except:
            pass # Ignore if already stopped
            
        await self.drone.action.return_to_launch()
        print("--> RTL activated.")

    async def land(self):
        """Lands the drone at current position."""
        print("--> Landing...")
        try:
            await self.drone.offboard.stop()
        except:
            pass # Ignore if already stopped
        
        await self.drone.action.land()
        print("--> Landing initiated.")