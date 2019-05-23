# Apriltags2_VO for VINS

![avatar](/support_image/apriltag.gif) 

This is a specific verision of the apriltags2_ros( based on https://github.com/AprilRobotics/apriltag_ros ) in order to correct the accumulative error of VINS pose estimator. **However, it can also be used as visual odometry (VO) alone.** I summarize my contributions as follow:  

1、a modified and tasted configuration file for the camera of Intel Realsense d435i.  

2、the topic "/tag_detections" output the pose of the body frame which is consistent with the output from VINS-Mono, instead of the transformation matrix form the tag frame to the camera frame.  
3、a topic "/tag_Odometry" is published with the type of "nav_msgs::Odometry", which can be visualized in RVIZ.

4、a topic "/path" is published with the type of "nav_msgs::Path", which can be visualized in RVIZ.


you can read my blog for more details: 

https://blog.csdn.net/qq_41839222/article/details/89606859

https://blog.csdn.net/qq_41839222/article/details/90482421

![avatar](/support_image/01.png)  

![avatar](/support_image/02.png)  



----

# apriltags2_ros

This is a Robot Operating System (ROS) wrapper of the [AprilTags 2 visual fiducial detector](https://april.eecs.umich.edu/software/apriltag.html). For details and tutorials, please see the [ROS wiki](http://wiki.ros.org/apriltags2_ros).

**Authors**: Danylo Malyuta  
**Maintainer**: Danylo Malyuta, danylo.malyuta@gmail.com  
**Affiliation**: NASA Jet Propulsion Laboratory, California Institute of Technology

# Contributing

Pull requests are welcome! Especially for the following areas:

- Publishing of the AprilTag 2 algorithm intermediate images over a ROS image topic (that AprilTag 2 [already generates](https://github.com/dmalyuta/apriltags2_ros/blob/526b9455121ae0bb6b4c1c3db813f0fbdf78393c/apriltags2/src/apriltag.c#L1167-L1395) when `tag_debug==1`)
- Conversion of the [bundle calibration script](https://github.com/dmalyuta/apriltags2_ros/blob/526b9455121ae0bb6b4c1c3db813f0fbdf78393c/apriltags2_ros/scripts/calibrate_bundle.m) from MATLAB to Python
- Extend calibration to support calibrating tags that cannot appear simultaneously with the master tag, but do appear simultaneously with other tags which themselves or via a similar relationship appear with the master tag (e.g. a bundle with the geometry of a cube - if the master is on one face, tags on the opposite face cannot currently be calibrated). This is basically "transform chaining" and potentially allows calibration of bundles with arbitrary geometry as long as a transform chain exists from any tag to the master tag
- Replacement of AprilTag 2 core algorithm's usage of custom linear algebra and image processing functions with Boost and OpenCV
- Supporting multiple tag family detection (currently all tags have to be of the same family). This means calling the detector once for each family. Because the core AprilTag 2 algorithm is the performance bottleneck, detection of `n` tag families will possibly decrease performance by `1/n`

# Copyright

The source code in `apriltags2/` is wholly the work of the [APRIL Robotics Lab](https://april.eecs.umich.edu/software/apriltag.html) at The University of Michigan. This package simply rearranges the source code to be able to compile it with catkin.

The source code in `apriltags2_ros/` is original code that is the ROS wrapper itself, see the [LICENSE](https://github.com/dmalyuta/apriltags2_ros/blob/526b9455121ae0bb6b4c1c3db813f0fbdf78393c/LICENSE). It is inspired by [apriltags_ros](https://github.com/RIVeR-Lab/apriltags_ros) and provides a superset of its functionalities.

If you use this code, please kindly cite:


- D. Malyuta, “[Navigation, Control and Mission Logic for Quadrotor Full-cycle Autonomy](https://www.research-collection.ethz.ch/handle/20.500.11850/248154),” Master thesis, Jet Propulsion Laboratory, 4800 Oak Grove Drive, Pasadena, CA 91109, USA, December 2017.
- J. Wang and E. Olson, "[AprilTag 2: Efficient and robust fiducial detection](http://ieeexplore.ieee.org/document/7759617/)," in ''Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)'', October 2016.

```
@mastersthesis{malyuta:2017mt,
  author = {Danylo Malyuta},
  title = {{Guidance, Navigation, Control and Mission Logic for Quadrotor Full-cycle Autonomy}},
  language = {english},
  type = {Master thesis},
  school = {Jet Propulsion Laboratory},
  address = {4800 Oak Grove Drive, Pasadena, CA 91109, USA},
  month = dec,
  year = {2017}
}
@inproceedings{Wang2016,
  author = {Wang, John and Olson, Edwin},
  booktitle = {2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  doi = {10.1109/IROS.2016.7759617},
  isbn = {978-1-5090-3762-9},
  month = {oct},
  pages = {4193--4198},
  publisher = {IEEE},
  title = {{AprilTag 2: Efficient and robust fiducial detection}},
  year = {2016}
}
```
