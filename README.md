# README #

This is my work for my Senior Design project at the FAMU-FSU College of Engineering. We had to build a robot to compete in the IEEE SoutheastCon 2015 Competition. We placed 7th out of 39 schools, beating schools such as UF and Georgia Tech. 

Our team website can be found [here](http://www.eng.fsu.edu/me/senior_design/2015/team29/index.html). 

This code is meant to be loaded onto an Arduino Mega that communicates with 3 subordinate Arduino Mini's. This setup is described in depth in this [design document](http://www.eng.fsu.edu/me/senior_design/2015/team29/docs/MS7Report.pdf). 

* One Arduino Mini was in control of the Line Following subsystem and associated sensors.
* Another was in control of the Etch A Sketch and Rubik's cube subsystems.
* Another was in control of the Simon Says/ Playing card subsystems.

The Arduino Mega itself was responsible for facilitating communication between the Minis using the [COBS Protocol](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) for redundancy. In addition, it calculated the forward kinematics of a four wheeled mecanum platform based on this [paper](http://www.omikk.bme.hu/collections/phd/Villamosmernoki_es_Informatikai_Kar/2013/Kalman_Viktor/ertekezes.pdf). This allows the user to provide a vector representing an x, y, and angular velocity for the overall robot, which gets translated into individual angular velocities for each of the mecanum wheels. These velocities are then fed into a timer interrupt based velocity PID, which maintains the velocity in the real world based on feedback from motor encoders. This repository holds the code which does the above.