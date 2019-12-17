
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
n_timesteps = 10000
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
def filterAndPub(xy,xy_cov,counter):
    x_est[counter], P_k[counter] = (
    kf.filter_update(
    x_est[counter-1],
    P_k[counter-1],
    observation = xy,
    observation_covariance = xy_cov)
    )
    #msg = Point
    # Establizing the message for velocity controller
    Point.x = x_est[counter][0]
    Point.y = x_est[counter][3]
    # publishing the messsage
    point_pub.publish(Point)
    # Setting the onld time to
    oldTime = rospy.Time.now
    # Sleep the set time to ensure that the dt is more or less consisten
    rate.sleep()
    # rospy.spin() to ensure that the message has been prossed correctly
    rospy.spin()
    counter +=1

    # This method is called when a message rerivecs on the topic /facedetector
def faceCallback(face,self):
    print('person face')
        # This bool is used to ensure that only one set of data is updated in the
        # kalman filter per time step
    msgTransmit = True
        # There can be detected mutliple persons on this topic so we need to look througe all
        # of them and store them
    knownPerson = personInfo('none',0,0,0,0,0,0)
    for f in range(len(face.pos)):
        _face_x = face.pos[f].pose.position.x
        _face_y = face.pos[f].pose.position.y
        _face_cov =[[face.pos[f].covariance[0],face.pos[f].covariance[1],face.pos[f].covariance[2]],
        [face.pos[f].covariance[6],face.pos[f].covariance[7],face.pos[f].covariance[8]],
        [face.pos[f].covariance[12],face.pos[f].covariance[13],face.pos[f].covariance[14]]]
        knownPerson[f] = personInfo(_name,0,0,0,_face_x,_face_y,_face_cov)
    #unknownPerson[0].set(knownPerson)
    # Setting the covariance matrices of xy coordinates from the facedetector
    R = [[knownPerson[0].persFace_cov[1], 0],[0, knownPerson[0].persFace_cov[4]]]
        # obs and obs_cov has to be put in the correct structure for pykalman
        # thi is why we us the numpy to translate the matrice in to the correct structure
    obs = [knownPerson[0].persFace_x,knownPerson[0].persFace_y]
    obs_cov = np.asarray(R)
    # Updateing the kalman filter and publishing
    filterAndPub(cov,obs_cov,counter)

    # This method is called when a message rerivecs on the topic /people_tracker_measurements
    #@classmethod
def legCallback(self,leg):
    print('person leg')
    # This bool is used to ensure that only one set of data is updated in the
    # kalman filter per time step
    msgTransmit = True
    # There can be detected mutliple persons on this topic so we need to look througe all
    # of them and store them
    for l in range(len(leg)):
        _leg_x = leg[l].people.pos.x
        _leg_y = leg[l].people.pos.y
        _name = leg[l].people.name
        _leg_cov = [[leg[l].covariance[0],leg[l].covariance[1],leg[l].covariance[2]],[leg[l].covariance[3],leg[l].covariance[4],leg[l].covariance[5]],[leg[l].covariance[6],leg[l].covariance[7],leg[l].covariance[8]]]
        unknownPerson[l] = personInfo(_name,_leg_x,_leg_y,_leg_cov,0.0,0.0,R0)
    # /people_tracker_measurements all so provides a name of the tracked objects
    # if the knownPerson has the same name as unknownPerson it is the same persons
    # so the kalman filter only needs to be updated updates
    #if unknownPerson[0].persName.eq(knownPerson.persName):
    print('same person')
    unknownPerson[0].set(knownPerson)
    R = [[knownPerson.persLeg_cov[1], 0],[0, knownPerson.persLeg_cov[4]]]
    obs = [knownPerson.persLeg_x,knownPerson.persLeg_y]
    obs_cov = np.asarray(R)
    filterAndPub(cov,obs_cov,counter)
        # If unknownPerson does not have the same name we start a new filtering of a new persons
        # and restart the kalman filter
    '''
    else:
        unknownPerson[0].set(knownPerson)
        print('new person')
        R = [[knownPerson.persLeg_cov[1], 0],
        [0, knownPerson.persLeg_cov[4]]]
        obs = [knownPerson.persLeg_x,knownPerson.persLeg_y]
        kf = KalmanFilter(transition_matrices = F,
        observation_matrices = H,
        transition_covariance = Q,
        observation_covariance = R, # the covariance will be adapted depending on Sensor_ID
        initial_state_mean = X0,
        initial_state_covariance = P0)
        oldTime = rospy.Time.now()
        rate.sleep()
        rospy.spin()
        counter +=1
    '''


print("running")
# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously
rospy.init_node('kalman_filter', anonymous=True)
oldTime =rospy.Time.now()
try:
    # Defining Publisher
    point_pub = rospy.Publisher('/sensorPosition', Point, queue_size=10)
    # When sleep() is inwoke it will in that the node is running at
    # rate of 10 Hz
    rate = rospy.Rate(10) # 10hz
    # While we have not closed the node this loop will run
    while not rospy.is_shutdown():
    #calculating the difference in time
        dt = rospy.Time.now() - oldTime
        msgTransmit = False
        leg_detection_sub = rospy.Subscriber("/people_tracker_measurements",PositionMeasurementArray,legCallback,1)
        if msgTransmit != True:
            face_detection_sub = rospy.Subscriber("/facedetector", face_detector,faceCallback,1)
        if msgTransmit != True:
            obs = None
            obs_cov = None
            filterAndPub(obs,obs_cov,counter)



except rospy.ROSInterruptException:
    pass


# This section is only for plotting and saving the values of the track person
if False :
    # extracting the Sensor update points for the plot
    Sensor_1_update_index = [i for i, x in enumerate(Sensor) if x == 1]
    Sensor_2_update_index = [i for i, x in enumerate(Sensor) if x == 2]

    Sensor_1_update_X = [ X[i] for i in Sensor_1_update_index ]
    Sensor_1_update_Y = [ Y[i] for i in Sensor_1_update_index ]

    Sensor_2_update_X = [ X[i] for i in Sensor_2_update_index ]
    Sensor_2_update_Y = [ Y[i] for i in Sensor_2_update_index ]

    # plot of the resulted trajectory
    plt.plot(RefX, RefY, "k-", label="Real Trajectory")
    plt.plot(Sensor_1_update_X, Sensor_1_update_Y, "ro", label="Sensor 1")
    plt.plot(Sensor_2_update_X, Sensor_2_update_Y, "bo", label="Sensor 2")
    plt.plot(x_est[:, 0], x_est[:, 1], "g.", label="Filtered Trajectory", markersize=1)
    plt.grid()
    plt.legend(loc="upper left")
    plt.show()
