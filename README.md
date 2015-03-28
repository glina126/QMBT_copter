# QMBT_copter
A simple, yet versatile quadcopter controller and code. Based on the Arduino platform using the Atmega328p and MPU6050.

<img src="http://cdn.makeagif.com/media/3-28-2015/fegofJ.gif" alt="arduino quadcopter">
<img src="http://cdn.makeagif.com/media/3-28-2015/taoqIo.gif" alt="arduino quadcopter">

![alt text](https://github.com/glina126/QMBT_copter/blob/master/IMG_0184.JPG "QMBT with its code in the background")

##Final Update:
This project is changing direction. The atmega328p microcontroller is simply insufficient for being used as a quadcopter brain. <br>
This is due to a few reasons mainly because - not enough interrupts and 8bit architecture.  <br>
The loop runs at a mere 33 miliseconds, which is not sufficient.  <br>
Also, due to the lack of interrupts, the precision at which we can sample the RC RX is very low causing noise in the system. <br>
That being said, the project is changing gears and moving on to a much better microcontroller, LPC11U35 Cortex-m0 running at 48mhz. <br> 

###This project has a few milestones that are set on its path. 
1. Create a versatile PCB which will overcome the common problems such as power distribution. 
2. Create lightweight code that will be self sufficient, aka, no external program will be needed for changing settings of the quadcopter, except for a terminal window. 
3. Ultimate goal is to have full support of peripheral devices such as a sonic distance sensor to help aid in obstacle avoidance, or a GPS unit for man-less operation. 

This is just the beginning of the project, though experimentation with the PCB have been conducted for a while now. At this point, the project is reset to a fresh start. 

