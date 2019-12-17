# CodeBB1
This is where the different code that is used to control the turtlebot in project BB-1 is stored.
In this repository several algorithms, which has been used to control the turtlebot, can be found. The repository consists of 6 different algorithms:

**NB: Hover the mouse over the pictures to show the caption**

## 1: Kalman Learning Algorithm 
[Click here to read in depth about the Kalman Algorithm](http://web.eecs.utk.edu/~leparker/Courses/CS594-fall08/Lectures/Nov-20-Localization-Mapping-III.pdf?fbclid=IwAR09qqvL_XFGQxKLONP_L07A6dernhlGAMSVIidJq074ANnLy4_oJM5IV6g)

![Rudolf E. Kalman](https://github.com/ValdemarQvist/CodeBB1/blob/master/pictures/1.PNG "Rudolf E. Kalman")

*This algorithm is based on Sebastein's pykalman filter*

This algorithm is used to be implemented in the test to measure the deviations in the outputs when it is applied or when it is not. It is also used for us to get a better understanding on how the algorithm works, since it is implemented in the two major algorithms in this project - face and leg detection.


## 2: Bilka_config-master

![Gmapping](https://github.com/ValdemarQvist/CodeBB1/blob/master/pictures/Capture.PNG "Gmapping")

This algorithm is used for the configuration of the sensors setup, the launch files, AMCL and Gmapping. This is to make it easier for us to bringup the turtlebot and modulate the different aformentioned files.

## 3:Bilka_move-master
[Click here to read in depth about the Move_Base](http://wiki.ros.org/move_base)

*This algorithm is based on ROS' MOVE_BASE*

This algorithm is used to navigate through BILKA while avoiding dynamic and static objects. It calculates itself in a known map and utilises the MOVE_BASE to move to the desired item requested by the costumer. The algorithm includes the GUI used to determine which item the costumer would like to be guided to, which is based on GTKMM.

![GTKMM GUI](https://github.com/ValdemarQvist/CodeBB1/blob/master/pictures/gtkmm_gui2.png "GTKMM GUI")

## 4: face-id

![Detected face of Mark and Jonathan](https://github.com/ValdemarQvist/CodeBB1/blob/master/pictures/detecedFace.png "Detected face of Mark and Jonathan")

*This algorithm is based on the people-package from ROS, see [face_detector](https://github.com/wg-perception/people/tree/melodic/face_detector) for the original algorithm.*

This algorithm handles the detection of the face of a costumer, so that the tracking of the person can be realised. It can be used in dynamic or static scenarios to detect the features of the face, which can be read in the report. 
The project uses a RGB-D camera to capture the different features of the guided costumer.

## 5:people_legs-master

![Rviz Visualisation of the leg_detector](https://github.com/ValdemarQvist/CodeBB1/blob/master/pictures/rviz.png "Rviz Visualisation of the leg_detector")

*This algorithm is based on the people-package from ROS, see [leg_detection](https://github.com/wg-perception/people/tree/melodic/leg_detector) for the original algortihm.*

This Algorithm handles the detection of the legs of the costumer. This is based on several features from legs and the distance between them to determine if it is actually a person, which can be read in the report.
This project uses 2 SICK-TIM LIDARs to determine these features of the guided costumer.

## 6: Velocity_controller

This algorithm is produced to keep the appropriate distance to the costumer, so that the robot never enters the intimate-zone described in the HAAL-distancing, which can be read in the report. The robot should accelerate its speed the most at 1.2 meters and then decrease so the person following can keep up.

![Hall's inter-personal zones](https://github.com/ValdemarQvist/CodeBB1/blob/master/pictures/hall.png "Hall's inter-personal zones")


