#########################################################
# Pioner P3DX - Robot Sensors and Motor Test
# (c) 2022 Djoko Purwanto, djoko@ee.its.ac.id
# Task by Muhammad Juniarto
########################################################

import sim
import sys
import time
import keyboard
import numpy as np

# == Function Definition ==
# -------------------------
def connectSimulator():
    # Close Connection
    sim.simxFinish(-1)
    # Connect to CoppeliaSIm Scene Simulation
    clientID = sim.simxStart('127.0.0.1', 19999, True, True,5000, 5)
    print(clientID)
    if clientID != -1 : print('Connected to remote API server')
    else:
        print('Connection unsuccessful, program ended.')
        sys.exit()
    return clientID

def getMotorHandle(clientID):
    _, motorLeftHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, motorRightHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
    return motorLeftHandle,motorRightHandle

def getSensorsHandle(clientID):
    sensorsHandle = np.array([])
    for i in range(16):
        _, sensorHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i+1),sim.simx_opmode_blocking)
        # Ignore
        _,_,_,_,_ = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        sensorsHandle = np.append(sensorsHandle, sensorHandle)
    return sensorsHandle

def getDistance(clientID, sensors):
    distances = np.array([])
    for i in range(16):
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, np.int(sensors[i]), sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        if detectionState == False:
            distance = 10.0
        distances = np.append(distances, distance)
    return distances

def setRobotMotion(clientID, motors, veloCmd):
    error1 = sim.simxSetJointTargetVelocity(clientID, motors[0], 0, sim.simx_opmode_streaming)
    error2 = sim.simxSetJointTargetVelocity(clientID, motors[1], 0, sim.simx_opmode_streaming)
    return error1,error2


# == Main Program ==
print('Program Started')
# Connection to Coppelia Simulator
client_id = connectSimulator()
# -- Object Handle
motor_left_handle, motor_right_handle = getMotorHandle(client_id)
print('-----------------------')
print(motor_left_handle)
print('-----------------------')

sensors_handle = getSensorsHandle(client_id)

# -- Simulation Process
samp_time = 0.1
n = 1.
velo_zero = (0,0)
time_start = time.time()

while(True):
    t_now = time.time()-time_start
    # Command Processing
    if t_now >= samp_time*n:
        # Sensors
        object_distances = getDistance(client_id, sensors_handle)
        # Robot Motion Simulation
        #setRobotMotion(client_id,(motor_left_handle, motor_right_handle), velo_zero)
        # Update Sample
        n += 1.
        # Show Info
        print('t= ', round(t_now,2), 'front side object distance=',object_distances[3], object_distances[4])
        print('------------------------')
        print('[1st sensor] Left Sensor= ',object_distances[0])
        print('[2nd sensor] Left Sensor= ',object_distances[1])
        print('[3rd sensor] Left Sensor= ',object_distances[2])
        print('[4th sensor] Left Middle Sensor= ',object_distances[3])
        print('[5th sensor] Left Middle Sensor= ',object_distances[4])
        print('[6th sensor] Left Middle Sensor= ',object_distances[5])
        print('[7th sensor] Lef Middle Sensor= ',object_distances[6])
        print('[8th sensor] Middle Sensor= ',object_distances[7])
        print('[9th sensor] Middle Sensor= ',object_distances[8])
        print('[10th sensor] Middle Sensor= ',object_distances[9])
        print('[11th sensor] Right Middle Sensor= ',object_distances[10])
        print('[12th sensor] Right Middle Sensor= ',object_distances[11])
        print('[13th sensor] Right Middle Sensor= ',object_distances[12])
        print('[14th sensor] Right Sensor= ',object_distances[13])
        print('[15th sensor] Right Sensor= ',object_distances[14])
        print('[16th sensor] Right Sensor= ',object_distances[15])
        print('\n')

    if keyboard.is_pressed('esc'):
        err1, err2 = setRobotMotion(client_id,(motor_left_handle, motor_right_handle), velo_zero)
        _ = sim.simxStopSimulation(client_id,sim.simx_opmode_oneshot)
        print(err1,err2)
        break
    if keyboard.is_pressed('tab'):
        print("Manual Override!")

        pass

# -- Simulation Finished
sim.simxFinish(client_id)
print('Program Ended\n')