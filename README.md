# How to use
  1. Read master branch's [README.md](https://github.com/hll4fork/mavros/blob/master/README.md)

# How to build
  $ mkdir mavros_ws/src
  
  $ cd mavros_ws
  
  $ catkin init
  
  $ cd src
  
  $ git clone https://github.com/huanglilong/mavros
  
  $ cd ..
  
  $ wstool init src

  $ rosinstall_generator mavlink | tee /tmp/mavlink.rosinstall

  $ wstool merge -t src /tmp/mavlink.rosinstall

  $ wstool update -t src

  $ catkin build [mavros]
  
# How to connect to pixhawk
  Read px4's [doc](http://dev.px4.io/pixhawk-companion-computer.html)
  
  $ roslaunch mavros px4.launch
  
  Press safety switch here!
  
  $ rosrun mavros mavsaftey arm
  
  $ rosrun mavros offb_node
  
# Get the topic's messages
  $ rqt_graph
  
  $ rosrun rqt_topic rqt_topic
