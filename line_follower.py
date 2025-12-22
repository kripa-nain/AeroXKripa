'''
Task 2.2 - 

A car is loaded. You have to develop a PID controller for that car such that it runs along the line y = 0.
The line is also visible on the plane.
Callibrate the PID gains such that car gets to the line as fast as possible and follows it without much disturbance.
Refer to the past two taks and their codes for hints.



INSTRUCTIONS -
    Select the simulation window and Press ENTER to execute


'''




import numpy as np 
import pybullet as p 
import time
import math
import cv2

p_id = p.connect(p.GUI)                #Loading the simulation
p.setGravity(0, 0, -10)                #Setting the gravity

plane = p.loadURDF("plane.urdf")        #Loading the plane
carPos = [0,3,0.1]                      #This is where the car will spawn, this is constant. Don't change

m = 0                           #Declaring the slope of the required line y = mx + c
c = 0                           #Declaring the contsnat of the reuired line  y = mx + c
angle = math.atan(m)

car = p.loadURDF("car1.urdf", carPos, p.getQuaternionFromEuler([0,0,angle]))  #Loading the car with head parallel to the given line


def printLine(m, c):                        #This functions draws a line that we need to follow
    angle = math.atan(m)
    z = 0.02
    origin = [0,c,z]
    line = p.loadURDF("line.urdf", origin, p.getQuaternionFromEuler([0,0,angle]))

printLine(m, c)                    #Calling the function to print the line


num = p.getNumJoints(car)                  #Getting the total number of joints in the car
for i in range(num):
    print(p.getJointInfo(car, i))           #Printing the information of each joint to get the motor joints


#These are the 4 motor joints that we need to manipulate, we declare them here.

fl = 2        #Front Left wheel        
fr = 3        #Front Right wheel
bl = 4        #Back Left wheel
br = 5        #Back Right wheel

p.setJointMotorControlArray(car, [fl, bl, fr, br], p.VELOCITY_CONTROL, forces = [0,0,0,0])   #This is done to enable torque control in wheels of the car
p.stepSimulation()



'''
Above this is the loading code, make no changes to it
Below this is the code that you need to work with.
'''
Kp = 3.0
Ki = 0.0
Kd = 7000.0
base_torque = 6
last_error = 0

def moveCar(car_id, action):
    # Differential drive: torque difference
    l_torque = base_torque - action
    r_torque = base_torque + action
    p.setJointMotorControlArray(car_id, [fl, bl, fr, br], p.TORQUE_CONTROL, forces=[l_torque, l_torque, r_torque, r_torque])

def calc_error(car_id):
    global last_error
    pos, _ = p.getBasePositionAndOrientation(car_id)
    error = 0 - pos[1]  # Line is at y=0
    
    # PID: P + D (since Ki is 0)
    action = (Kp * error) + (Kd * (error - last_error))
    last_error = error
    return action

# --- 3. THE LOOP ---
print("Press ENTER in the simulation window to start.")

while True:
    keys = p.getKeyboardEvents()
    
    # Check if ENTER is pressed
    if keys.get(p.B3G_RETURN) == 1:
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        
        # Load Files 
        p.loadURDF("plane.urdf")
        car = p.loadURDF("car1.urdf", carPos, p.getQuaternionFromEuler([0, 0, 0]))
        
        # Enable torque control
        p.setJointMotorControlArray(car, [fl, bl, fr, br], p.VELOCITY_CONTROL, forces=[0,0,0,0])
        
        # Reset memory
        last_error = 0

        # ACTIVE DRIVING LOOP
        while True:
            p.stepSimulation()
            p.resetDebugVisualizerCamera(7, -90, -45, p.getBasePositionAndOrientation(car)[0])
            
            # Apply PID
            steering_action = calc_error(car)
            moveCar(car, steering_action)
            
            time.sleep(1./240.)
            
            # Reset check
            inner_keys = p.getKeyboardEvents()
            if inner_keys.get(p.B3G_RETURN) == 1:
                break
    
    time.sleep(0.1) # Keeps the script from closing