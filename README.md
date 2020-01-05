# Lidar+ROS+Raspberry Pi Robot
## 1. Building a robot.
------
This part is somewhat looser than the others. In a nutshell, find some motors, wheels, motor controllers, and some connecting materials. Throw them all at a wall and hope that the come together nicely. 

## 2. Installing ROS

ROS (Robot Operation System) is a framework that facilitates the use of a wide variety of "packages" to control a robot. Those packages range all the way from motion control, to path planning, mapping, localization, SLAM, perception, and more. ROS provides a relatively simple interface with those packages, and the ability to of course create custom packages.

Note: The Raspberry Pi 4 is more computationally capable than its predecessors. However, installing ROS on the Pi3 is currently (as of December 2019) easier, and allegedly more reliable.

### Get the disc image

I dowloaded the Ubuntu 16.04 Xenial with pre-installed ROS from [Ubiquity Robotics](https://downloads.ubiquityrobotics.com/pi.html). They have great instructions on how to install and download the image. The main points are:
* Download the image from the top of the page.
* Flash it to an SD card (at least 8GB). You can use Etcher, it works well.
* Connect to the WiFi network that starts with `ubiquityrobot`. Password is `robotseverywhere`.
* Go to Terminal, and connect to your Pi using `ssh ubuntu@10.42.0.1`. Password is `ubuntu`.
* Run `roscore` to make sure that things are working properly. If you get a warning/errors, try stopping ROS and starting it again with `killall -9 roscore`.

## 3. Remotely connecting to ROS
Something we would want to be able to do is to access the ROS communication messages from our laptop. There are a couple of steps to do here.
* Spin a Linux machine with ROS Kinetic Kame. Either a virtual machine or a real machine. You can use VMWare-Fusion with Ubuntu 16.04 or something similar. We will refer to that machine as the Observer machine. The robot is the Master.
* On the Master, find the `ROS_IP` and `ROSMASTER_URI`. These two things are the information both machines will need to communicate. Find the `ROS_IP` by running `ifconfig`. 

* I would add these lines to `.bashrc` on the Observer machine, or create a script to run them together. This is not required, but would make your life (potentially) easier in the long run (so you won't need to type those lines every time you'd want to connect to the robot :) ).

* On the Master (robot), run `roscore`. 
* On the observer, you now have access to the messages and topics that are on the Master. More on that soon.

* On the robot (the machine running `roscore`):
* * `ROS_IP` is its own IP.
* * `ROS_MASTER_URI` is HTTP://<its own IP>:11311.

* On the observer computer:
* * `ROS_IP` is its own IP.
* * `ROS_MASTER_URI` is the robot's IP

In this example (the IPs would probably be different in your network), on the robot, we set: `export ROS_IP=192.168.43.228 export ROS_MASTER_URI=http://192.168.43.228:11311`

On the observer laptop, we set: `export ROS_IP=192.168.43.123 export ROS_MASTER_URI=http://ubiquityrobot.local:11311`. This master URI looks different (but is actually the same under the alias). I believe that setting it to `192.168.42.228` would work (should be the same as the `.local`), but I did not test it.

## A couple of notes here:
* To make sure the communication works, I followed [this tutorial](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes) to publish basic shapes to rviz.
* I had to make the messages compatible with Indigo, following [an answer here](https://answers.ros.org/question/261071/rviz-client-md5sum-error/). (The solution with downloading the common msgs Indigo folder and using the `visualization_msgs` package foder in `catkin_ws/src`.)
* In rviz, make sure to set the frame to `my_frame` (if following tutorial).



### 4. Connecting to WiFi
A short step, to make sure both machines have internet connectivity. The information is taken from [this website](https://learn.ubiquityrobotics.com/connect_network).

* On the robot machine, `pifi add YOURNETWOKNAME YOURNETWORKPASSWORD`
* Restart the Pi, `sudo reboot`.
Now the Raspberry Pi will connect to your WiFi network on startup. To connect to it, connect your computer to the same network, and `ssh ubuntu@ubiquityrobot.local` with the password `ubuntu`.

Woo! Now both machines have internet, and can communicate over SSH.

## 5. Testing the lidar

This step was a bit of a doozy. It took me a while to figure out how to get the lidar to run. But I did! So hopefully you won't have to suffer too.

I am using the YDLIDAR G2 for this build. The first step is to install the necessary drivers. The driver is a ROS package.
* `cd catkin_workspace/src`.
* `git clone https://github.com/EAIROBOT/ydlidar_ros.git`.
* `catkin_make`
* Follow the directions from the repository, written below:
* * `roscd ydlidar_ros/startup`
* * `sudo chmod 777 ./*`
* * `sudo sh initenv.sh`
* Go back to your catkin workspace, and run `source devel/setup.bash`.
* `git checkout G2`. Move to the branch of your Lidar model.
* Run `catkin_make` again.

Test the lidar with `roslaunch ydlidar_ros lidar.launch`. Visualize the scans in Rviz, by adding the topic `/scan`. 

## ROS + Arduino; Getting them to talk to each other.
As we know, the Raspberry Pi is the "brain" of our robot, perceiving the environment and planning in it. The Arduino, is simply used to control the motors of the robot. It doesn't do much thinking. So our goal here, is to get commands from the Raspberry Pi to the Arduino, so it'll be able to tell the motors how to move, accordingly. In high level, what we do is install *rosserial*, a ROS module that enables Arduino communication, on both the Raspberry Pi and the Arduino.
* Following the steps from [the ROS website](http://wiki.ros.org/rosserial_arduino/Tutorials), we start with installing the package. `sudo apt-get install ros-kinetic-rosserial-arduino`, and then, `sudo apt-get install ros-kinetic-rosserial`. If you are using a ROS version different from Kinetic, change the word `kinetic` to your version.
* In the following commands, substitute `catkin_ws` with the name of your catkin workspace.
    ```cd catkin_ws/src
  git clone https://github.com/ros-drivers/rosserial.git
  cd catkin_ws
  catkin_make
  catkin_make install
  ```
* In your Arduino IDE, install the rosserial library. I found it the easiest to just do it from the IDE itself. Search for `rosserial` in the Library Manager and install it.

And that's it!

For a test run, try the [HelloWorld example](http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World), from the examples included with the rosserial library. Flash the Arduino with it, and connect to the Raspberry Pi. To run it:

* On the Raspberry Pi `roscore`
* In a second Raspberry Pi terminal, `rosrun rosserial_python serial_node.py /dev/ttyACM0`. Change `ttyACM0` with the port of your Arduino. You can check the port by navigating to `~/dev/`, and observing which files disappear and re-appear when the Arduino is disconnected and connected.
* In a third terminal, `rostopic echo chatter` to see the messages being sent.
