# Mahjong Object Pose Estimation

- We use the PPF to do the object recognition feature and hash table for 3D descriptor.
- By utilizing the [point cloud library](http://pointclouds.org/), we now only could run the program on Win7 visual studio 2010.
- **Author : Cheng-Hei Wu**
- **Maintainer : [Howard Chen](https://github.com/s880367), [Yueh Chuan](https://github.com/YuehChuan), [Yu-Hsien Chang](https://github.com/TacoHsien)**

## Requirment
- PCL 1.7.0
- Boost-1.50.0
- Eigen-3.0.5
- qhull-6.2.0.1385
- Qt_4.8.0
- VTK-5.8.0


## TODOS
- Run this program on Ubuntu 14.04
- Modified the **socket part** for linux compatible
- Modified the **kinect bringup** part for linux compatible
- Be careful about the **function syntax** of  the 3rd party version, like boost.
