
from gpiozero import Motor

class AutonomousController(object):
    def __init__(self, motor1_pins, motor2_pins, motor3_pins, motor4_pins, motor5_pins, motor6_pins):
        self.__motor1 = Motor(forward= motor1_pins[0], backward= motor1_pins[1], enable= motor1_pins[2], pwm=True) #left front
        self.__motor2 = Motor(forward= motor2_pins[0], backward= motor2_pins[1], enable= motor2_pins[2], pwm=True) #left back
        self.__motor3 = Motor(forward= motor3_pins[0], backward= motor3_pins[1], enable= motor3_pins[2], pwm=True) #right front
        self.__motor4 = Motor(forward= motor4_pins[0], backward= motor4_pins[1], enable= motor4_pins[2], pwm=True) #right back
        self.__motor5 = Motor(forward= motor5_pins[0], backward= motor5_pins[1], enable= motor5_pins[2], pwm=False) #intake left
        self.__motor6 = Motor(forward= motor6_pins[0], backward= motor6_pins[1], enable= motor6_pins[2], pwm=False) #intake right
        
        self.__heading = None
        self.__desired_heading = None
        # self.__speed = None
        # self.__position = None

        pass

    def initialize(self, robot_state):
        #initialize heading
        self.__heading = robot_state['heading']
        self.__speed = robot_state['speed']
        self.__position = robot_state['position']
        self.__desired_heading = robot_state['heading']

    def __repr__(self):
        pass
    def update():
        pass
        
    def drive_motors(self, left_speed:float=0.0, right_speed:float=0.0):
        assert (left_speed >= -100) & (left_speed <= 100) & (right_speed >= -100) & (right_speed <= 100), "Motor Error: Speeds out of bounds (-100 to 100 percent)"
        
        if (left_speed > 0):
            self.__motor1.forward(abs(left_speed)/100)
            self.__motor2.forward(abs(left_speed)/100)
        elif(left_speed < 0):
            self.__motor1.backward(abs(left_speed)/100)
            self.__motor2.backward(abs(left_speed)/100)
        else: 
            self.__motor1.stop()
            self.__motor2.stop()

        if (right_speed > 0):
            self.__motor3.forward(abs(right_speed)/100)
            self.__motor4.forward(abs(right_speed)/100)
        elif(right_speed < 0):
            self.__motor3.backward(abs(right_speed)/100)
            self.__motor4.backward(abs(right_speed)/100)
        else: 
            self.__motor3.stop()
            self.__motor4.stop()
    
    def run_motor(self, motor, speed:float=0.0, reverse:bool=False):
        assert (speed >= -100) & (speed <= 100), "ERR: Motor Speed Out of Bounds."
        if (speed > 0):
            motor.forward(abs(speed)/100)
        elif(left_speed < 0):
            motor.backward(abs(speed)/100)
        else: 
            motor.stop()

    def drive_fwd_continuosly(self, speed):
        drive_motors(left_speed=speed,right_speed=speed)

    def turn_continously(self, turn_dir:str="clockwise", speed, *args, **kwargs)
        """
        Turn robot continuously (tank/pivot turn)
        """
        turn_dir = turn_dir.lower().strip()
        assert (turn_dir == "clockwise") or (turn_dir == "counterclockwise"), "ERR: Invalid Turn Direction Parameter."
        if (turn_dir == "clockwise"):
            drive_motors(left_speed=speed, right_speed=-speed)
        elif(turn_dir == "counterclockwise"):
            drive_motors(left_speed=-speed, right_speed=speed)
        else:
            print("ERR: something went wrong")

    def stop(self)
        drive_motors(0.0,0.0)
    
    # def run_avoidance_check(self)->bool:

    def get_desired_heading(self):
        return self.__desired_heading
    
    #private member functions (the __ before the variable or function denotes it as private)
    def __heading_to_position(self, target_center):
        tgt_hdg = np.mod(np.degrees(np.arctan2(target_center[0]-self.__position[0],
                                               target_center[1]-self.__position[1]))+360,360)
        return tgt_hdg
    

    def __heading_to_angle(self, target_angles):
        #account for multiple targets? targets would be ping pong balls in this case
        if len(target_angles=0)
            #no targets detected to turn to, in this case, keep going at current heading
            return self.__heading
        
        relative_angle = 0
        angle_difference = 0
        for i in range(0, min(len(target_angles))):
            if angle_difference < abs(target_angles[i]):
                relative_angle = target_angles[i]
                angle_difference = abs(target_angles[i])

        tgt_hdg = self.__heading + relative_angle
        return tgt_hdg
    def __select_action(self):
        delta_angle = max(self.__desired_heading, self.__heading) - min(self.__desired_heading, self.__heading)
        
        # determine the angle between current and desired heading
        delta_angle = max(self.__desired_heading, self.__heading) - min(self.__desired_heading, self.__heading)

        if delta_angle > 0:
            if self.__heading > self.__desired_heading:
                #tank turn left until heading is correct
            elif self.__heading < self.__desired_heading:
                #tank turn right until heading is correct
            else:
                #drive straight as heading is correct

    def decide(self):
        while(True): #replace with while switch is on when switch enabled.
            #TODO implement detect april tags, find angles to them?
            #TODO implement ping pong ball detection, find angles to them
            #TODO check if heading is correct, if not turn. else drive forward
            self.__heading = #get heading from adcs system
            #check time, if time is running out use self.__heading_to_position(insert center of arena position here? whatever the final drop off is)
            print(f"The heading of the robot is {self.__heading}")
            self.__desired_heading = self.__heading_to_angle(targets) #TODO implement targets (ping pong balls? fiducial/april tag)
            self.__select_action() #make this return a command?
            drive_fwd_continuosly(speed=100)
            # turn_continuously(turn_dir="clockwise",speed=100)

        pass
if __name__ == "__main__":
    autonomousController = AutonomousController(motor1_pins=(8,7,12),
                                                motor2_pins=(26,20,19),
                                                motor3_pins=(14,15,18),
                                                motor4_pins=(5,6,13),
                                                motor5_pins=(17,27,22),
                                                motor6_pins=(10,9,11)
                                                )
    autonomousController.decide()