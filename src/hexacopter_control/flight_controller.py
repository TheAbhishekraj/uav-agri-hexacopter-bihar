#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem

class AgriculturalHexacopter:
    def __init__(self):
        self.drone = System()
    
    async def connect(self):
        print("🔗 Connecting to PX4 SITL...")
        await self.drone.connect(system_address="udp://:14540")
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Connected to PX4 SITL")
                break
    
    async def spray_grid_mission(self):
        await self.connect()
        
        print("🛫 Taking off...")
        await self.drone.action.set_takeoff_altitude(5.0)
        await self.drone.action.takeoff()
        
        async for in_air in self.drone.telemetry.in_air():
            if in_air:
                print("✈️ Drone is airborne")
                break
        
        # Generate spray mission
        mission_items = self._generate_spray_pattern()
        
        print("📤 Uploading mission...")
        await self.drone.mission.set_mission_items(mission_items)
        
        print("🔓 Arming...")
        await self.drone.action.arm()
        
        print("🌾 Starting mission...")
        await self.drone.mission.start_mission()
        
        async for progress in self.drone.mission.mission_progress():
            print(f"📍 Mission progress: {progress.current}/{progress.total}")
            if progress.current == progress.total:
                print("✅ Mission done. Landing...")
                break
        
        await self.drone.action.land()
        print("🛬 Landed. Mission complete.")
    
    def _generate_spray_pattern(self):
        """Generate agricultural grid spray pattern"""
        mission_items = [
            # Gauripur, Bihar field coordinates
            MissionItem(25.6723, 89.9876, 5.0, 3.0, False, 0.0, 0.0, MissionItem.CameraAction.NONE),
            MissionItem(25.6723, 89.9890, 5.0, 3.0, True,  0.0, 0.0, MissionItem.CameraAction.NONE),
            MissionItem(25.6733, 89.9890, 5.0, 3.0, False, 0.0, 0.0, MissionItem.CameraAction.NONE),
            MissionItem(25.6733, 89.9876, 5.0, 3.0, True,  0.0, 0.0, MissionItem.CameraAction.NONE),
        ]
        return mission_items

async def main():
    print("🚜 Agricultural Hexacopter – Autonomous Spraying Mission")
    controller = AgriculturalHexacopter()
    await controller.spray_grid_mission()

if __name__ == "__main__":
    asyncio.run(main())

