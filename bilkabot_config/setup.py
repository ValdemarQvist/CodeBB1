import os
import rospkg
from distutils.dir_util import copy_tree

currentDirectory = os.getcwd()

rospack = rospkg.RosPack()
packageList = rospack.list()

for package in ['sick_tim', 'turtlebot_bringup', 'turtlebot_description', 'turtlebot_navigation']:
    if package in str(packageList):
        src = currentDirectory + "/" + package
        dst = rospack.get_path(package)
        copy_tree(src, dst)
        print(package + " configured properly")
    else:
        print(package + " is missing")
