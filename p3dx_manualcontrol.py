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
import math

# == Function Definition ==
# -------------------------
def connectSimulator():
    # Close Connection
    sim.simxFinish(-1)
    # Connect to CoppeliaSIm Scene Simulation
    clientID = sim.simxStart('127.0.0.1', 19999, True, True,5000, 5)
    if clientID != -1 : print('Connected to remote API server')
    else:
        print('Connection unsuccessful, program ended.')
        sys.exit()
    return clientID

def getMotorHandle(clientID):
    _, motorLeftHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    _, motorRightHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
    return motorLeftHandle,motorRightHandle

def keyboardPressed():
    if keyboard.is_pressed('w'):
        velo = (1,1)
    elif keyboard.is_pressed('s'):
        velo = (-1,-1)
    elif keyboard.is_pressed('a'):
        velo = (0.5,1)
    elif keyboard.is_pressed('d'):
        velo = (1,0.5)
    else:
        velo = (0,0)
    err1,err2 = setRobotMotion(client_id,motors,velo)

def inversKinematics(speed,angle):
    r = 0.195 # wheel radii
    l = 0.190 # distance to wheel   

    u = np.array([[speed],
                 [angle]])
    
    M = np.array([[r/2,  r/2],
                  [r/(2*l),  -r/(2*l)]])
    
    M_ = np.linalg.inv(M)
    phi = np.dot(M_, u)
    return phi[0]*0.5, phi[1]*  0.5 

def inversKinematicsPose():
    r = 0.195 # wheel radii
    l = 0.190 # distance to wheel   
    pass

def setObjectHandler(clientID, path):
    objectHandler = sim.simxGetObjectHandle(clientID, path, sim.simx_opmode_blocking)[1]
    return objectHandler

def getObjectPose(clientID, objectHandle, block=None):
    if(block == None):
        position = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_oneshot)[1]
        orientation = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_oneshot)[1]
    else:
        position = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_blocking)[1]
        orientation = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_blocking)[1]

     # print("Pose of: x = {:.2f} y = {:.2f} theta = {:.2f}".format(position[0], position[1], orientation[2]*(180/np.pi)))
    return position[0], position[1], orientation[2]

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
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, int(sensors[i]), sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        if detectionState == False:
            distance = 10.0
        distances = np.append(distances, distance)
    return distances

def setRobotMotion(clientID, motors, veloCmd):
    error1 = sim.simxSetJointTargetVelocity(clientID, motors[0], veloCmd[0], sim.simx_opmode_streaming)
    error2 = sim.simxSetJointTargetVelocity(clientID, motors[1], veloCmd[1], sim.simx_opmode_streaming)
    return error1,error2

def pathTracking(robotPose, waypoints, offset):
    length = len(waypoints)
    i = pathTracking.i
    
    err_x = waypoints[i][0] - robotPose[0]
    err_y = waypoints[i][1] - robotPose[1]

    print("wp = {} wp_target = {:.2f} {:.2f} {:.2f} err_x = {:.2f} err_y = {:.2f}".format(i, waypoints[i][0], waypoints[i][1], waypoints[i][2], err_x, err_y))

    if(abs(err_x) <= offset and abs(err_y) <= offset and pathTracking.i < length-1):
        pathTracking.i = pathTracking.i + 1
        return waypoints[i][0], waypoints[i][1], waypoints[i][2]
    else:
        return waypoints[i][0], waypoints[i][1], waypoints[i][2]

def objectFollower(s3,s4,s5,s6):
    distance_ref = 0.25
    angle_ref = 0
    Kp_trans = 2
    Kp_rot = 0.2

    # Current Situation
    distance = min(s4,s5)
    angle = s6 - s3

    # Control Action Input
    du = distance_ref - distance
    thetau = angle_ref - angle

    # Control
    v = Kp_trans*du
    omega = Kp_rot*thetau
    phiR,phiL =  inversKinematics(v,omega)
    return phiR,phiL

def poseControl(ref,act,etol):
    K = [1.5, 1.2, 0.8]
    ex = ref[0] - act[0]
    ey = ref[1] - act[1]
    egamma = ref[2] - act[2]

    theta = np.arctan2(ey, ex)

    if abs(ex) <= etol and abs(ey) <= etol:
        xc = 0
        yc = 0
        gammac = K[2]*(egamma)
    else:
        xc = K[0]*ex
        yc = K[1]*ey
        gammac = theta - act[2]* (180 / np.pi)
    
    return xc,yc,gammac,theta

def poseControl2(ref,act,etol):
    K = [1.5, 1.2, 0.2]
    ex = ref[0] - act[0]
    ey = ref[1] - act[1]
    egamma = ref[2] - act[2]
    theta = np.arctan2(ey, ex)
    
    gammac = theta - act[2]

    # Search nearest angle
    if(abs(egamma) >= np.pi):
        if(egamma > 0):
            egamma = egamma - 2*np.pi
        else:
            egamma = egamma + 2*np.pi

    if(abs(gammac)>= np.pi):
        if(gammac >0):
            gammac = gammac - 2*np.pi
        else:
            gammac = gammac + 2*np.pi
    
    # Check Position or Oriental Control
    if(abs(np.sqrt(ex**2+ey**2))>= etol):
        vtrans = K[0]*np.sqrt(ex**2+ey**2)
        vrot = -K[1]*gammac
    else:
        if(egamma >=0.05):
            vrot = -K[1]*egamma
        else:
            pass
    
    return vtrans,vrot


def velocityGenerator(xc,yc,gammac,theta):
    r = 0.195 # wheel radii
    l = 0.190 # distance to wheel 

    M = np.array([[(r/2)*math.cos(theta),  (r/2)*math.cos(theta)],
                  [(r/2)*math.sin(theta),  (r/2)*math.sin(theta)],
                  [(r/2)*l, (-r/2)*l]])
    
    u = np.array([[xc],
                 [yc],
                 [gammac]])
    
    #M_ = np.transpose(M)
    #M_ = M.reshape(2,3)
    M_ = np.linalg.pinv(M)
    
    phi = np.dot(M_, u)

    return phi[0],phi[1]

def velocityNormalization(thetaR, thetaL):
    thetaMax = max(thetaR,thetaL)
    thetaMin = min(thetaR,thetaL)
    #a = np.array([thetaR,thetaL])
    #thetaNorm = np.linalg.norm(a)

    thetaRN = (2/thetaMax)*thetaR
    thetaLN = (2/thetaMax)*thetaL

    return thetaRN, thetaLN
    
def limitSpeednoCrazy(v, limR,limL):
    vR = v[0][0]
    vL = v[1][0]
    if(vR >= limR):
       vR = limR
    if(vL >= limL):
       vL = limL
    return vR,vL

# == Main Program ==
print('Program Started')
# Connection to Coppelia Simulator
client_id = connectSimulator()
# -- Object Handle
motor_left_handle, motor_right_handle = getMotorHandle(client_id)
motors = (motor_left_handle,motor_right_handle)

#Sensor Handle
sensors_handle = getSensorsHandle(client_id)

#Robot Handle
robotHandle = setObjectHandler(client_id, "/PioneerP3DX")
waypoints = [None]*4
for i in range(4):
    waypoints[i] = setObjectHandler(client_id, "/Cylinder[{}]".format(i))

pathTracking.i = 0

# -- Simulation Process
mode = 4
samp_time = 0.1
n = 1.
velo_zero = (0,0)
time_start = time.time()

# Getting Waypoints Information
waypointsPose = [None]*4
for i in range(4):
    waypointsPose[i] = getObjectPose(client_id, waypoints[i], block=True)

while(True):
    t_now = time.time()-time_start
    # Command Processing
    if t_now >= samp_time*n:
        # Sensors
        object_distances = getDistance(client_id, sensors_handle)
        # Update Sample
        n += 1.
        # Show Info Mode

        # Mode Manual Control (TASK 2)
        if(mode == 1):
            keyboardPressed()

        # Mode Object Follower DONE :') (TASK 3)
        if(mode == 2 ):
            # ALWAYS ON
            s3,s4, s5, s6 =  (object_distances[2], object_distances[3], object_distances[4], object_distances[5])
            vR,vL = limitSpeednoCrazy(objectFollower(s3,s4,s5,s6), 1.0, 1.0)
            setRobotMotion(client_id,motors,(vR,vL))
            #objectFollower()
        
        # Mode Localization to Origin (TASK 4)
        if(mode ==3):
            # Masih belum selesai [Yang pose Control pdf not WORK]

            # Getting information about the Origin
            origin = setObjectHandler(client_id,'/Cylinder') # Getting ID Tag on Cylinder
            ref = getObjectPose(client_id,origin)
                     
            # Getting Information about the robot position
            robot = setObjectHandler(client_id, '/PioneerP3DX')
            act = getObjectPose(client_id, robot,block=True)

            # Ready for Pose Control
            xc, yc, gammac, theta = poseControl(ref, act,etol=0.1)
            thetaR, thetaL = velocityGenerator(xc,yc,gammac, theta)
            
            # Comment ini untuk menjalankan algoritma dari alvan
            thetaRN, thetaLN = velocityNormalization(thetaR,thetaL)
            setRobotMotion(client_id,motors,(thetaR, thetaL))     

            # Algoritma dari Alvan [WORK]/ Uncomment Code Below to see it work // Ya nggak juga sih, kalo digeser jadi hancur juga
            #vtrans,vrot = poseControl2(ref,act,etol=0.1)
            #v1,v2 = inversKinematics(vtrans,vrot)
            #setRobotMotion(client_id,motors,(v1, v2))
            

        # Mode Multiple Waypoints/Path Tracking
        if (mode==4):
            # Getting robot information
            robot = setObjectHandler(client_id, '/PioneerP3DX')
            robotPose = getObjectPose(client_id, robot,block=True)
            
            waypointsPose[3] = getObjectPose(client_id, waypoints[3])
            wp_x, wp_y, wp_z = pathTracking(robotPose, waypointsPose, 1)

            #Akar masalahnya disini, yang masalahnya juga muncul di assignment 4 :')
            vtrans, vrot = poseControl2([wp_x, wp_y, wp_z], robotPose, etol=0.1)
            #Inverse Kinematics All Good
            v1,v2 = inversKinematics(vtrans,vrot)
            setRobotMotion(client_id,motors,(float(v1), float(v2)))

        # Keyboard Interuption
        if keyboard.is_pressed('esc'):
            err1, err2 = setRobotMotion(client_id,motors, (0,0))
            _ = sim.simxStopSimulation(client_id,sim.simx_opmode_oneshot)
            break
        #print('t= ', round(t_now,2), 'front side object distance=',object_distances[3], object_distances[4])

# -- Simulation Finished
sim.simxFinish(client_id)
print('Program Ended\n')