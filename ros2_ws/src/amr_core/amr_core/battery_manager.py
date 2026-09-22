class BatteryManager:
    def __init__(self, node, driving_multiplier=3.0):
        self.node = node
        self.driving_multiplier = driving_multiplier
        
        # Attach the timer natively to the ROS 2 node
        self.timer = self.node.create_timer(1.0, self.battery_tick)

    def battery_tick(self):
        """Simulates battery dynamics at 1 Hz."""
        # 1. Charging State
        if self.node.agent.is_charging_task_active():
            self.node.battery += self.node.c_rate
            if self.node.battery > self.node.battery_full:
                self.node.battery = self.node.battery_full
                
        # 2. Discharging State
        else:
            if self.node.is_driving:
                # Drain fast while actively executing a physical path
                self.node.battery -= (self.node.e_rate * self.driving_multiplier)
            else:
                # Drain slow while idling
                self.node.battery -= self.node.e_rate
                
            # Prevent negative battery values
            if self.node.battery < 0.0:
                self.node.battery = 0.0