# Blind Navigation App with ROS Client
The human-machine interface for the CityU's blind navigation system, version 1.0, which has realized the communication between the ROS (Robot Operating System) and Android application, with following the fundamental rosbridge protocol v2.0.
The realization of ROS client is referred to the github repository as follows:
1. ROSBridgeClient: https://github.com/djilk/ROSBridgeClient.git
2. RosClient: https://github.com/hibernate2011/RosClient.git

# Required library:
1. EventBus
2. java_websocket
3. json-simple

# How to use
1. Launch a ROS server at the PC terminal. To launch the file, run:

    roslaunch rosbridge_server rosbridge_websocket.launch

2. Make sure the IP address and port number of the ROS server in your PC in accordance with that of ROS client embedded in your Android App. Input your IP address and port number at the text bars and click the connect botton to establish the connection.
