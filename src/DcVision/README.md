# DcVision

## Check out vision pipeline (https://hri-gitlab.honda-ri.de/sfuchs/dexco_vision)

```
  ssh -X dexcoop-01
  git clone git@hri-gitlab.honda-ri.de:sfuchs/dexco_ros_ws.git
  cd dexco_ros_ws
  git submodule update --init --recursive
```

## Build vision pipeline

```
  source /opt/ros/melodic/setup.bash
  catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Starting: See https://hri-gitlab.honda-ri.de/sfuchs/dexco_vision/README.md



## Virtual vision

The virtual vision module works with a VirtualKinect2 instance that is run in
the TestPolygonPlanner program. It renders a depth and an ir image from the
OpenGL buffer, and publishes it on the following topics:

```
  /kinect2_dexco/sd/image_depth_rect
  /kinect2_dexco/sd/image_ir_rect
  /kinect2_dexco/sd/camera_info
```

The IR image is initialized with 100 (Intensity treshold from Config File). Here are
the specs of the Kinect2 sensor:

```
  RGB: 84 x 54 deg (1920 x 1080)
  IR/D: 71 x 60deg  (512 x 424)
```

The program must be started on a localhost (not via ssh). Here is how to start it:

```
  source ~/ros-melodic.env
  export ROS_MASTER_URI=http://localhost:11311
  bin/TestPolygonPlanner -m 1 -ros -kinect -noLimits -skipTrajectoryCheck -speedUp 3

```

Here is how tos tart the vision pipeline. It automatically starts a roscore if
there's not already one running:

```
source /hri/sit/latest/External/anaconda3/envs/common/3.7/BashSrc
conda activate /hri/storage/user/sfuchs/Software/conda/dexco
cd /home/mgienger/localdisk/dexco_ros_ws
source devel/setup.bash
roslaunch dexco_vision plane_segmentation_robolab-virtual.launch
```


The "-kinect" option automatically sets the camera view to the kinect transformation
and updates the field of view. 

## Issues

  - Currently, the transforms seem a bit inclined.
  - Weird python 2 vs. 3 errors when starting plane_segmentation: Don't source
    any ros environments before activating the anaconda environment. Ideally,
    start the roslaunch (above) from a fresh shell.
