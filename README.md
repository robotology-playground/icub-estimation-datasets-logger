# icub-base-estimation-datasets
Repository to collect, store and parse data sets collected on iCub through YARP interface for State Estimation experiments.

__NOTE: The author promises to document this repository in a better way, in the near future! Until then, hang on with the development phase.__

## Overview

In this repository, we collect and store data sets through the YARP interface for offline processing mainly aimed for state estimation. These data sets will be mainly collected for different trials of walking experiments, so that they can be readily fed into estimation algorithms for rapid testing and validation offline.
The data collection is done using `yarpdatadumper` in a way that they can be played like `rosbag` using the `yarpdataplayer`.

This repository will also include Matlab and C++ parser, which can be readily used.

The data sets, in general would include,
- Joint state information
  - joint positions,
  - joint velocities (computed),
  - estimated joint torques,
  - currents (optional)
- Head IMU measurements and calibrated stereo camera images
- Waist IMU measurements
- Six axis FT sensor measurements (feet, legs, arms)[measured/post-WholeBodyDynamics]
- Estimated end-effector wrenches at the hands and feet

## Output ports of interest
### For joint state information
- `/icub/all_joints/stateExt:o` head, torso, right arm without hands, left arm without hands, right leg, left leg

### For visual-inertial odometry
- `/icub/inertial`
- `/icub/head/state:o`
- `/icub/camCalib/left/out`
- `/icub/camCalib/right/out`

Make sure `ENABLE_yarpcar_mjpeg` option is set to `ON` while compiling YARP.

### Waist IMU
- `/icub/xsens_inertial`

### Six Axis FT sensors
#### Measured
  - `/icub/left_arm/analog:o`
  - `/icub/right_arm/analog:o`
  - `/icub/left_leg/analog:o`
  - `/icub/right_leg/analog:o`
  - `/icub/left_foot/analog:o`
  - `/icub/right_foot/analog:o`

#### Post WholeBodyDynamics
Try to publish processed FT sensor measurements out of `wholeBodyDynamics` device.

### End Effector Wrenches
- `/wholeBodyDynamics/left_arm/cartesianEndEffectorWrench:o`
- `/wholeBodyDynamics/right_arm/cartesianEndEffectorWrench:o`
- `/wholeBodyDynamics/left_foot/cartesianEndEffectorWrench:o`
- `/wholeBodyDynamics/right_foot/cartesianEndEffectorWrench:o`
