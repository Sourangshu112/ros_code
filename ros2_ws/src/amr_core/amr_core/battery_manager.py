import math

class BatteryManager:
    def __init__(self, node):
        self.node = node
        
        # Attach the timer natively to the ROS 2 node
        self.timer = self.node.create_timer(1.0, self.battery_tick)

    def battery_tick(self):
        """Simulates battery dynamics at 1 Hz."""
        dist_to_station = math.dist([self.node.current_x, self.node.current_y],[self.node.offset_x, self.node.offset_y])
        # 1. Charging State
        
        if self.node.agent.is_charging_task_active() and dist_to_station < 0.5:
            self.node.battery += self.node.c_rate
            if self.node.battery > 100:
                self.node.battery = 100
                
        # 2. Discharging State
        else:
            if self.node.is_driving:
                # Drain fast while actively executing a physical path
                self.node.battery -= self.node.e_rate
            else:
                # Drain slow while idling
                self.node.battery -= self.node.e_rate_standstill
                
            # Prevent negative battery values
            if self.node.battery < 0.0:
                self.node.battery = 0.0