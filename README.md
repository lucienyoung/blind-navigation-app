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

![image](https://github.com/lucienyoung/blind-navigation-app/assets/137718915/d7ae9478-c1c3-49d1-a24f-1193defbf0b8)

