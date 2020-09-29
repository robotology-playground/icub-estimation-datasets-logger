## Vicon Segment Trajectory Generator

This is a workaround solution to compensate for the unavailability of segment poses from the Vicon Nexus software streamed through `yarp-vicon-bridge`. The idea is to use a use the point cloud information of the marker positions attached to a rigid body segment from the CAD as a reference point cloud, and run a Iterative Closest Point algorithm to align an incoming point cloud generated from marker positions streamed by Vicon Nexus through the `yarp-vicon-bridge`. This way, we will have the pose of the segment rigid body in the Vicon world, given the Vicon markers.

#### Dependencies

- `Eigen 3.2.`92

- `PCL`
- `matio`
- `matlogger2`


#### How to run

```sh
mkdir build
cd build
cmake ..
make
```



