# delta_des_state_generator

This node is a closed-loop navigation creation and queue reader designed and implemented by group Delta for EECS 376: Mobile Robotics at Case Western Reserve University. 

The node takes series of points and headings from a path sender program and plots a course between each of these points. After the course is plotted it is used by the steering algorithm to physically navigate the course.

Path points are given in the MAP FRAME.

Further code to be implemented: Lidar dictated safety stop, and obstacle avoidance; click to add path point in rviz; fix starting heading error; more robust method of tracking path adherence.


    