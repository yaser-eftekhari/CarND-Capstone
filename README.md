This is the project repo for the final project of Team RoadWeavers  

* Team Lead: Harish Balasubramaniam (harish.balasub@gmail.com), 
* Team Member 1: Yaser Eftekhari (yasseref@yahoo.com, yaser.eftekhari@gmail.com), 
* Team Member 2: Sean Robinson (sean.robinson36@gmail.com), 
* Team Member 3: Tharak Krishnan (tharak.krishnan@gmail.com)   

for the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Slow Computer Instructions
In case you have a slow machine, please checked out **[optimization branch](https://github.com/yaser-eftekhari/CarND-Capstone/tree/optimization)** branch. In this branch the code is optimized to be able to run on slower machines. Lots of unnecessary code has been removed and the critical parts of the code run based on frequency and not even driven.

If you are interested in more details about the code, please checkout the branch and see the readme file for more detailed information about how the code has been implemented.

### Overview of implementation
Here is a general description of the modules and overview of their implementation. For more details please consult the documentation in the code.

#### Traffic Light Detection and Classification
In order to correctly detect and classify the light, we started from the SSD-Inception model from tensorflow. The model was trained once with 500 annotated simualator data and once with real data (from Bosch dataset as well as the rosbag provided). Hence, the model was extended not only to detect the traffic light but also recognize the color of the light.

In order to increase the accuracy of classification traffic light status is reported only if 3 or more consecutive classifications agree on the status.

The classification of traffic light starts only if the car is within 50 meters of the traffic light and it has not passed it all together.


#### Waypoint Updater
This module is supposed to adjust the speed of the vehicle based on traffic light and obstacles in the road.
This module sets the speed based on a few parameters:
* If the car is too far from the light, keep on going with no speed change.
* Depending on the speed of the car, look ahead for some number of waypoints.:

    - If the light is green, proceed with no change
    - If the light is not green but the waypoint is not too close, set the speed to decay based on the distance of the waypoint to the light
    - If the car is too close to the light, stop

#### dbw node
This module is responsible for triggering the appropriate changes in the steering angle, trottle and break based on the waypoint locations and set speed.

In order to control the steering, throttle and break, we use the twist controller module. There each trigger is derived based on a PID model with appropriate parameters achieved through trial and error.

A low pass filter is also used to smooth out the jittery commands.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
