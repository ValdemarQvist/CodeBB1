#!/usr/bin/env python
##Based upon Sebastian Thrun's code used in Robotics AI at Udacity.com
import rospy
from std_msgs.msg import String
import message_filters as mf
import people_msgs
from geometry_msgs.msg import *
from turtlesim.msg import *
from faceid.msg import face_detector
from math import *
import numpy as np
from pykalman import KalmanFilter

#time holder
dt =0.0

class matrix:

    # implements basic operations of a matrix class

    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueErro
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError
        else:
            # multiply if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                   res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
                except:
                   raise ValueError
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)
## if of no input
def filter(x, P):
    for n in range(len(measurements)):

        # prediction
        x = (F * x) + u  #State matrix (location and velocity)
        P = F * P * F.transpose() #Covariance matrix

        # measurement update
        Z = matrix([measurements[n]]) #Measurements
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P

    print ('x= ')
    x.show()
    print ('P= ')
    P.show()
    return [x, P]

def filterPredict(x, P, dt, F, u):

	# prediction
	x = (F * x) + u  #Mean vector (location and velocity)
	P = F * P * F.transpose() #Covariance matrix
	return [x, P]
## if input and
def filterUpdate(x, P, H, R, I):
	# measurement update
	Z = matrix([measurements[n]]) #Measurements ##TODO insert measurements from sensor[x1,x2],[xdot1,xdot2],[y1,y2],[ydot1,ydot2]
	y = Z.transpose() - (H * x) # measurement residual, difference between truth and expectation
	S = H * P * H.transpose() + R #residual covariance, increased by measurement noise
	K = P * H.transpose() * S.inverse() #optimal Kalman gain, gives least squared errors in updates
	x = x + (K * y) #Mean vector (location and velocity)
	P = (I - (K * H)) * P #Covariance matrix
	return [x, P]

class personInfo:

    def __init__(self,name,leg_x,leg_y,leg_cov,face_x,face_y,face_cov):
        self.persName = name
        self.persLeg_x = leg_x
        self.persLeg_y = leg_y
        self.persleg_cov = leg_cov
        self.persFace_x = face_x
        self.persFace_y = face_y
        self.persFace_cov = face_cov

    def __eq__(self, other):
        return self.persName == other.perName

    def set(self,other):
        other.persName = self.persName
        other.persLeg_x = self.persLeg_x
        other.persLeg_y = self.persLeg_y
        other.persLeg_cov = self.persLeg_cov
        other.persFace_x = self.persFace_x
        other.persFace_y = self.persFace_y
        other.persFace_cov = self.persFace_cov

class kalman:
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # spin() simply keeps python from exiting until this node is stopped
    # /people_msgs/PositionMeasurement
    # spin() simply keeps python from exiting until this node is stopped
    # /facedetection
    def faceCallback(face):
        for f in range(len(face)):
            _face_x = face[f].people.pos.x
            _face_y = face[f].people.pos.y
            _face_cov =[[face[f].covariance[0],face[f].covariance[1],face[f].covariance[2]],[face[f].covariance[6],face[f].covariance[7],face[f].covariance[8]],[face[f].covariance[12],face[f].covariance[13],face[f].covariance[14]]]
        R = [[knownPerson.persFace_cov[1], 0],[0, knownPerson.persFace_cov[4]]]
        obs = [knownPerson.persFace_x,knownPerson.persFace_y]
        obs_cov = np.asarray(R)
        filtered_state_means[counter], filtered_state_covariances[counter] = (
        kf.filter_update(
            filtered_state_means[counter-1],
            filtered_state_covariances[counter-1],
            observation = obs,
            observation_covariance = obs_cov)
        )

    def legCallback(leg):
        for l in range(len(leg)):
            _leg_x = leg[l].people.pos.x
            _leg_y = leg[l].people.pos.y
            _name = leg[l].people.name
            _leg_cov = [[leg[l].covariance[0],leg[l].covariance[1],leg[l].covariance[2]],[leg[l].covariance[3],leg[l].covariance[4],leg[l].covariance[5]],[leg[l].covariance[6],leg[l].covariance[7],leg[l].covariance[8]]]
            unknownPerson = personInfo(_name,_leg_x,_leg_y,_leg_cov,_face_x,_face_y,_face_cov)

        if unknownPerson.persName.eq(knownPerson.persName):
            print('new person')
            kf = KalmanFilter(transition_matrices = F,
                              observation_matrices = H,
                              transition_covariance = Q,
                              observation_covariance = R_1, # the covariance will be adapted depending on Sensor_ID
                              initial_state_mean = X0,
                              initial_state_covariance = P0)

            knownPerson = unknownPerson
        else:
            knownPerson = unknownPerson

            R = [[knownPerson.persFace_cov[1], 0],
                [0, knownPerson.persFace_cov[4]]]
            obs = [knownPerson.persLeg_x,knownPerson.persLeg_y]
            obs_cov = np.asarray(R)
            filtered_state_means[counter], filtered_state_covariances[counter] = (
            kf.filter_update(
                filtered_state_means[counter-1],
                filtered_state_covariances[counter-1],
                observation = obs,
                observation_covariance = obs_cov)
            )
            print('same person')


    def __init__(self):
        print("running")
        oldTime =0.0
        rospy.init_node('kalman_filter', anonymous=True)
        try:
            point_pub = rospy.Publisher('/sensorPosition', Point, queue_size=10)
            rate = rospy.Rate(10) # 10hz

            while not rospy.is_shutdown():
                dt = rospy.Time.now() - oldTime   #Time since last measureme
                #Callback loop to ensure measurements within a time period
                leg_detection_sub = rospy.Subscriber("/people_tracker_measurements",PositionMeasurementArray,legCallback)
                face_detection_sub = rospy.Subscriber("/facedetector", face_detector,faceCallback)
                oldTime = rospy.Time.now()
                rate.sleep()
                rospy.spin()

        except rospy.ROSInterruptException:
            pass

initial_xy = [-3.0, 0.0] #Initial position before finding a person

x = matrix([[initial_xy[0]], [initial_xy[1]], [0.], [0.]])#initial state(location and velocity)

u = matrix([[0.], [0.], [0.], [0.]]) # external motion ##TODO Pavelocity data from turtlebot only moving in x direction

P = matrix([[0., 0., 0., 0.],
            [0., 0., 0., 0.],
            [0., 0., 1000., 0.],
            [0., 0., 0., 1000.]]) # initial uncertainty: 0 for positions x and y, 1000 for the two velocities

F = matrix([[1., 0., dt, 0.],
            [0., 1., 0., dt],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]]) # next state function: generalize the 2d version to 4d

H = matrix([[1., 0., 0., 0.],
            [0., 1., 0., 0.]]) # measurement function: reflect the fact that we observe x and y but not the two velocities

R = matrix([[0.1, 0.], [0., 0.1]]) # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal ##TODO change uncertainty

I = matrix([[1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.]]) # 4d identity matrix

#Initiate Kalman functions
#x, P = filter(x, P)
#x, P = filterUpdate(x,P,H,R,I)
#x, P = filterPredict(x,P,dt,F,u)

k = kalman()
