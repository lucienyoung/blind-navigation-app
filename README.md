# Blind Navigation App with ROS Client
The human-machine interface for the CityU's blind navigation system, version 1.0, which has realized the communication between the ROS (Robot Operating System) and Android application, with following the fundamental rosbridge protocol v2.0.  

The realization of ROS client is referred to the github repository as follows:
1. [ROSBridgeClient](https://github.com/djilk/ROSBridgeClient.git)
2. [RosClient](https://github.com/hibernate2011/RosClient.git)

# Required library:
1. [EventBus](https://github.com/greenrobot/EventBus.git)
2. [java_websocket](https://github.com/TooTallNate/Java-WebSocket.git)
3. json-simple

# How to use:
1. If you haven't installed the ROS server at your PC beforehand, please run
```
    sudo apt-get install ros-<rosdistro>-rosbridge-server
```

2. Launch a ROS server at the PC terminal. To launch the file, run:
```
    roslaunch rosbridge_server rosbridge_websocket.launch
```  

3. Make sure the IP address and port number of the ROS server at your PC in accordance with that of ROS client embedded in your Android App.

4. Input your IP address and port number at the text bars and click the connect botton to establish the connection.

!<div align=center>[image](https://github.com/lucienyoung/blind-navigation-app/assets/137718915/d7ae9478-c1c3-49d1-a24f-1193defbf0b8)

>To build a successful communication, we first launch a ROS server at the PC, which will create a WebSocket proxy under the current IP address at port 9090. The Android end should listen to the same IP address and port so as to connect to the server. Once the connection is successfully established, the user interface will jump to the navigation panel, ready to navigate. The system has two working modes, one is for recording and another for navigation. Note that these two modes cannot work together; we disabled relevant functions in one when another is working in case any mistouching happens. For convenience, we provide text input to specify the goal name when recording. The record button determines whether or not to subscribe to the odometry topic, while the upload button provides the database insertion function. We also provide the reset function for the goal database, it just needs to input the reset into the text input box, but this operation is dangerous since it will clear all recordings.  
>Initially, the system subscribe to the topic that records the 2D odometry calculated by [VINS](https://github.com/lucienyoung/map-building-ros). This step should be done along with the map building. Then, when starting to navigate, visually impaired users can input the goal name, and the system will automatically search for the corresponding pose coordinate. If the goal name is successfully retrieved, the ROS client at the Android end will advertise the goal-setting topic following the [Move-Base](http://wiki.ros.org/move_base) framework and publish the goal data in JSON format; otherwise, a no-matched-result warning will arise.  
>Unlike the robot, visually impaired people cannot follow a planned path even when given precise commands. It is inappropriate to set a timer to output navigation commands continuously. On the one hand, it remains unknown how long the interval should be to guarantee safe navigation in real-world tasks since the walking behaviors vary for each blind user. On the other hand, intensive navigation commands will pose too many cognitive loads on users and sometimes may even mislead them. In this regard, we design an interacting button for blind people so that they can decide when to receive the navigation prompts. At last, the system also allows users to inquire how far their current location is from their destination, giving them a global sense of orientation.  

![image](https://github.com/lucienyoung/blind-navigation-app/assets/137718915/615052d5-7ab9-4909-b93c-b3c712ce9d7c)<div align=center>

>In order to send goals and receive navigation commands, we need to realize the communication between the PC and Android smartphone. In addition, the navigation goals need to be stored in advance for users to retrieve through speech input. In this regard, we build a SQLite database to store the position and orientation of any traversed key points. Given that we assume that the system navigates blind users on the same floor, we only record the 2D pose of each point of interest. The frame ID is also important, which indicates a pose is seen from which coordinate system.
