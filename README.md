# arm_control

Robot Arm Control Library in Python

## Profile tool

python -m cProfile -o test.prof PyISP\hdr.py

snakeviz test.prof

## Changelog

* Add urdf file for arm test
* Add URDF class for parsing urdf file
* Add the Robot class for kinematics
* Add the GUI for visualizing the urdf file
* Add the GUI for visualizing the workspace of a robot //2023.2.24
* Fix the package include error //2023.2.27
* Add doc folder for documentation and video demo
* Update file structure and name
* Add symbolic kinematics
* Add jacobian transpose method to solve inverse kinematics //2023.2.28
* Add optimization method to solve inverse kinematics //2023.3.1
* Fix the bug of optimization method //2023.3.3
* Add inverse kinematics animation to show how the angle update //2023.3.3
* Better GUI for inverse kinematics animation, add gif demo //2023.3.3
* Add go though via point demo in tool/ik_gui_shape.py //2023.3.4
* Add jacobian inverse method to solve inverse kinematics //2023.3.4
* add some new robot urdf model //2023.3.6
* add collision detection (Not tested) //2023.3.29
* test collision detection using naive method, error rate 0.0002 //2023.3.29
* Add mesh visualization for the URDF, example UR10 arm //2023.4.12
  