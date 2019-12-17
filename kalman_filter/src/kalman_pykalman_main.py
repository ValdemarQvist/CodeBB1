#!/usr/bin/python
##########################################################################################
# This ros node takes in xy coordinates and the covariance from two different sensors.
# The two senors are a camera and a two lidars working in sync, when a face is detected
# by the camra or the legs of a person is detected by the lidars, a message is published
# on ehiter /facedetector or /people_tracker_measurements.
# These vaules are then fed into a kalman filter, in this case we used a premade filter
# called pykalman. When the filter is updated with new infomation from one of the topics
# or a pre set time has gone whit out any updates from the sensors, there is a need to update
# the kalman filter whitout any new infomation.
#
#   This is made by group 19gr566 anno 2019 at AaU fall semester
# for a project in comunication between sensors that is located in a ROS enviroment
#
##########################################################################################

# these import is the same as include
import rospy
import numpy as np
from geometry_msgs.msg import Point
from pykalman import KalmanFilter
import matplotlib.pyplot as plt
from people_msgs.msg import PositionMeasurementArray
from faceid.msg import face_detector
import math
import datetime
SensorID = []
X_pos = []
Y_pos = []
# Holder for the seqencial id for the msg to
# the controller
seq_id = 0

# Holder for the time steps in the array of previous messaurments
# this will be reset every time a new person is deteced
counter = 0




# Initial position before finding a person
initial_xy = [-3.0, 0.0]

# A holder for the change in time
dt = 0.0

# Setting up matrices to hold the estemated position over time
# aswell as prediction over time
n_timesteps = 3000
n_dim_state = 4
x_est = np.zeros((n_timesteps, n_dim_state))
P_k = np.zeros((n_timesteps, n_dim_state, n_dim_state))

# Filter Configuration
# Transition_matrix
F = [[1,  0,  dt,   0],
     [0,  1,   0,  dt],
     [0,  0,   1,   0],
     [0,  0,   0,   1]]

# Observation_matrix
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]

# Transition_covariance
Q = [[1e-4,     0,     0,     0],
     [   0,  1e-4,     0,     0],
     [   0,     0,  1e-4,     0],
     [   0,     0,     0,  1e-4]]

# Initial observation_covariance
R0 = [[0.1, 0],
     [0, 0.1]]

# Initial estemation
X0 = [initial_xy[0],
      initial_xy[1],
      0,
      0]

# Initial prediction matric
# There is a need to a assumed a bigger uncertainty in initial velocity
P0 = [[  0,    0,   0,   0],
      [  0,    0,   0,   0],
      [  0,    0,   1,   0],
      [  0,    0,   0,   1]]

# Defining the first kalman filter setup, when a new person is detected this will reset
kf = KalmanFilter(transition_matrices = F,
    observation_matrices = H,
    transition_covariance = Q,
    observation_covariance = R0, # the covariance will be adapted depending on Sensor_ID
    initial_state_mean = X0,
    initial_state_covariance = P0)



x_est[counter] = kf.initial_state_mean
P_k[counter] = kf.initial_state_covariance
counter +=1
# This class is used as a helper class to store
# the values of the detected person i.e xy coordinates,
# covariance matrice and name
class personInfo:
    # Defining a person whit is information gather form masseges recived
    def __init__(self,name,leg_x,leg_y,leg_cov,face_x,face_y,face_cov):
        self.persName = name
        self.persLeg_x = leg_x
        self.persLeg_y = leg_y
        self.persLeg_cov = leg_cov
        self.persFace_x = face_x
        self.persFace_y = face_y
        self.persFace_cov = face_cov
    # This method returns true if the two objects has the same names
    def __eq__(self, other):
        return self.persName == other.perName
    # This method copies one object attributes to another object
    def set(self,other):
        other.persName = self.persName
        other.persLeg_x = self.persLeg_x
        other.persLeg_y = self.persLeg_y
        other.persLeg_cov = self.persLeg_cov
        other.persFace_x = self.persFace_x
        other.persFace_y = self.persFace_y
        other.persFace_cov = self.persFace_cov


# This class kalman is where the program is "runnig" this class holds all of the prossecis
# that is need to filter and predict througe KalmanFilter and it wraps ROS around this kalman filter





# This method updates the filter based on the what topic
def filterAndPub(xy,xy_cov):
    global msgTransmit
    global Point
    global counter
    global kf
    global oldTime
    global x_est
    global P_k
    #global point_pub
    #means, covariances = kf.filter(measurements)
    x_est[counter], P_k[counter] = kf.filter_update(x_est[counter-1],P_k[counter-1],observation = xy,observation_covariance = xy_cov)
    point_msgs = Point()
    # Establizing the message for velocity controller
    point_msgs.x = x_est[counter+1,0]
    point_msgs.y = x_est[counter+1,2]
    # publishing the messsage
    point_pub.publish(point_msgs)
    # Setting the old time to new time
    oldTime = rospy.Time.now()

    counter +=1
    # Spin the set time to ensure that the dt is more or less consisten
    #print(counter)
    #print("here")
    #rospy.spin() to ensure that the message has been prossed correctly




# This method is called when a message rerivecs on the topic /facedetector
def faceCallback(face,arg):
    global msgTransmit
    global counter
    global SensorID
    global X_pos
    global Y_pos
    # This bool is used to ensure that only one set of data is updated in the
    # kalman filter per time step
    msgTransmit = True
    # There can be detected mutliple persons on this topic so we need to look througe all
    # of them and store them
    # Setting the covariance matrices of xy coordinates from the facedetector
    test = float("inf")


    if np.isnan(face.pos[0].pose.covariance[0]).any()  and np.isnan(face.pos[0].pose.covariance[7]).any() and np.isinf(face.pos[0].pose.covariance[0]).any() and np.isinf(face.pos[0].pose.covariance[7]).any():
        R = [[face.pos[0].pose.covariance[0], 0],[0, face.pos[0].pose.covariance[7]]]
    else:
        R = [[50.0, 0],[0, 50.0]]
    # obs and obs_cov has to be put in the correct structure for pykalman
    # thi is why we us the numpy to translate the matrice in to the correct structure
    _X = face.pos[0].pose.pose.position.x
    _Y = face.pos[0].pose.pose.position.y
    obs = [_X,_Y]
    obs_cov = np.asarray(R)
    SensorID.append(1)
    X_pos.append(_X)
    Y_pos.append(_Y)
    now = rospy.get_rostime()

    # Updateing the kalman filter and publishing

    filterAndPub(obs,obs_cov)

# This method is called when a message rerivecs on the topic /people_tracker_measurements
def legCallback(leg,arg):
    global msgTransmit
    global counter
    global SensorID
    global X_pos
    global Y_pos
    # This bool is used to ensure that only one set of data is updated in the
    # kalman filter per time step
    msgTransmit = True
    # There can be detected mutliple persons on this topic so we need to look througe all
    # of them and store them

    # /people_tracker_measurements all so provides a name of the tracked objects
    # if the knownPerson has the same name as unknownPerson it is the same persons
    # so the kalman filter only needs to be updated updates
    test = float("inf")
    _X = leg.people[0].pos.x
    _Y = leg.people[0].pos.y
    if leg.people[0].covariance[0] != test and leg.people[0].covariance[7] != test:
        R = [[leg.people[0].covariance[0], 0],[0, leg.people[0].covariance[7]]]
    else:
        R = [[50.0, 0],[0, 50.0]]
    obs = [_X,_Y]
    obs_cov = np.asarray(R)
    SensorID.append(2)
    X_pos.append(_X)
    Y_pos.append(_Y)
    now = rospy.get_rostime()
    #Sensor_update_index[counter,0] = rospy.get_rostime()
    # Updateing the kalman filter and publishing
    filterAndPub(obs,obs_cov)



print("running")
# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously
rospy.init_node('kalman_filter', anonymous=True)
oldTime = rospy.Time.now()
# Defining Publisher
point_pub = rospy.Publisher('/sensorPosition', Point, queue_size=10)
# When sleep() is inwoke it will in that the node is running at
# rate of 10 Hz
#rate = rospy.Rate(10) # 10hz
# While we have not closed the node this loop will run
while not rospy.is_shutdown():
    #calculating the difference in time
    dt = rospy.Time.now() - oldTime

    leg_detection_sub = rospy.Subscriber("/leg_detector",PositionMeasurementArray,legCallback,1)

    face_detection_sub = rospy.Subscriber("/facedetector", face_detector,faceCallback,1)


    #rate.sleep()
    rospy.spin()


print("dropped")
x = datetime.datetime.now()


'''
np.savetxt('legx.csv',Sensor_leg_update_X,delimiter = ',')
np.savetxt('legy.csv',Sensor_leg_update_Y,delimiter = ',')
'''
np.savetxt('X_pos'+ x.strftime("%d-%m-%y-%H-%M-%S") +'.csv',X_pos,delimiter =',')
np.savetxt('Y_pos'+ x.strftime("%d-%m-%y-%H-%M-%S") +'.csv',Y_pos,delimiter = ',')
np.savetxt('SensorID'+ x.strftime("%d-%m-%y-%H-%M-%S") +'.csv',SensorID,delimiter = ',')
np.savetxt('est_x'+ x.strftime("%d-%m-%y-%H-%M-%S") +'.csv',x_est,delimiter =',')
#np.savetxt('predict.csv',P_k,delimiter =',')

print('done saveing')
# This section is only for plotting and saving the values of the track person
if False :

    # extracting the Sensor update points for the plot
    Sensor_face_update_index = [i for i , x in enumerate(SensorID) if x == 1]
    Sensor_leg_update_index = [i for i , x in enumerate(SensorID) if x == 2]

    Sensor_face_update_X = [ X_pos[i] for i in Sensor_face_update_index ]
    Sensor_face_update_Y = [ Y_pos[i] for i in Sensor_face_update_index ]

    Sensor_leg_update_X = [ X_pos[i] for i in Sensor_leg_update_index ]
    Sensor_leg_update_Y = [ Y_pos[i] for i in Sensor_leg_update_index ]

    # plot of the resulted trajectory
    plt.plot(Sensor_face_update_X, Sensor_face_update_Y, "ro", label="Sensor face point")
    plt.plot(Sensor_leg_update_X, Sensor_leg_update_Y, "bo", label="Sensor leg point")
    plt.plot(x_est[:, 0], x_est[:, 2], "g.", label="Filtered point")
    plt.grid()
    plt.legend(loc="upper left")
    plt.show()
