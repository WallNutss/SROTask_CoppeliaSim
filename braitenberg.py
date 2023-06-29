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
    return (motorLeftHandle,motorRightHandle)

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
    detectionPoint = np.array([])
    for i in range(16):
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, np.int(sensors[i]), sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        if detectionState == False:
            distance = 10.0
        distances = np.append(distances, distance)
        detectionPoint = np.append(detectionPoint, bool(detectionState))
    return distances, detectedPoint

def setRobotMotion(clientID, motors, veloCmd):
    error1 = sim.simxSetJointTargetVelocity(clientID, motors[0], 0, sim.simx_opmode_streaming)
    error2 = sim.simxSetJointTargetVelocity(clientID, motors[1], 0, sim.simx_opmode_streaming)
    return error1,error2


# == Main Program ==
print('Program Started')
# Connection to Coppelia Simulator
client_id = connectSimulator()
# -- Object Handle
motors = getMotorHandle(client_id)

# -- Braitenberg Algorithm
detect = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
noDetection = 0.5
detectionMax = 0.2
braitenbergL = [-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
braitenbergR = [-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

sensors_handle = getSensorsHandle(client_id)

# -- Simulation Process
samp_time = 0.1
n = 1.
velo = 1
velo_zero = (0,0)
time_start = time.time()

while(True):
    t_now = time.time()-time_start
    # Command Processing
    if t_now >= samp_time*n:
        # Sensors
        object_distances, object_state = getDistance(client_id, sensors_handle)
        print(object_state)
        # Robot Motion Simulation
        #setRobotMotion(client_id,(motor_left_handle, motor_right_handle), velo_zero)
        # Update Sample
        n += 1.

        # Sensor Processing dn
        for i in range(16):
            if object_distances[i] < noDetection and object_state[i]:
                dist = object_distances[i]
                if(object_distances[i] < detectionMax):
                    dist = detectionMax
                detect[i] = 1 - ((dist-detectionMax)/(noDetection-detectionMax))
                print(detect[i])
            else:
                detect[i] = 0
            
        sumL = 0
        sumR = 0
        for i in range(16):
            sumL = sumL + (braitenbergL[i] * detect[i])
            sumR = sumR + (braitenbergR[i] * detect[i])

        vLeft = velo
        vRight = velo
        # Braitenberg Algorithm Actuation
        vLeft = vLeft + sumL
        vRight = vRight + sumR
        
        # Motors
        _ = sim.simxSetJointTargetVelocity(client_id, motors[0], vLeft, sim.simx_opmode_streaming)
        _ = sim.simxSetJointTargetVelocity(client_id, motors[1], vRight, sim.simx_opmode_streaming)
        
        # Show Info
        print('t= ', round(t_now,2), 'front side object distance=',object_distances[2], object_distances[4])


    if keyboard.is_pressed('esc'):
        err1, err2 = setRobotMotion(client_id,motors, velo_zero)
        _ = sim.simxStopSimulation(client_id,sim.simx_opmode_oneshot)
        print(err1,err2)
        break

# -- Simulation Finished
sim.simxFinish(client_id)
print('Program Ended\n')