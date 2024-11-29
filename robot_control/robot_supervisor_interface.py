# a class representing the available interactions a supervisor may have with a robot
class RobotSupervisorInterface:
    def __init__(self, robot):
        self.robot = robot

    # read the proximity sensors
    def read_proximity_sensors(self):
        return [s.read() for s in self.robot.ir_sensors]

    # read the wheel encoders
    def read_wheel_encoders(self):
        return [e.read() for e in self.robot.wheel_encoders]

    # apply wheel drive command
    def set_wheel_drive_rates(self, v_l, v_r):
        self.robot.set_wheel_drive_rates(v_l, v_r)
    
    #read the sensors to check if they're sensing another robot # i will probably delete this dude since we have the function below
    def read_robot_detection_array(self):
        return [s.detecting_robot for s in self.robot.ir_sensors]
    
    def read_robot_neighbors_pose(self):
        nearby_bots = set()
        for sensor in self.robot.ir_sensors:
            for bot_pose in sensor.detected_robots_pose:
                nearby_bots.add(bot_pose)
        return list(nearby_bots)
