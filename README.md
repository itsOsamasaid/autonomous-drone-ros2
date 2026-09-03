# FlyDuck: indoor autonomous drone

A 2D indoor navigation drone. No GPS: position and heading come from a 2D LiDAR running SLAM,
which ArduPilot's EKF3 fuses with the flight controller's IMU and a downward rangefinder.
ROS 2 handles perception and mission logic. ArduPilot flies the drone.

## Approach

Without GPS, working out where the drone is turns out to be the difficult part. Planning and
autonomy depend entirely on that estimate, so the estimator gets built and tested first.

The simulation runs ArduPilot SITL, which is the same ArduCopter firmware and EKF3 that run on the
real drone, loaded with the same parameters. Gazebo only provides physics and sensor data. The pose
has to come out of those sensors and through EKF3 before anything flies, same as on hardware.

Each phase has a gate it has to pass before the next one starts.

## Architecture

```
RPLIDAR C1 ──► /scan ──► slam_toolbox ──► map→odom
                                            │
                              vision_pose_bridge
                                            │
                          /mavros/vision_pose/pose
                                            │
Evo Mini ──► /mavros/rangefinder_sub ──►  EKF3  ◄── IMU
                                            │
                                  ArduPilot GUIDED ──► motors
                                            ▲
                                    Nav2 ───┘  (velocity setpoints)
```

TF frames: `map → odom → base_link → laser`

ArduPilot does the fusion, stabilisation and position/altitude hold. ROS 2 does perception, the
bridge into the EKF, and mission logic. Nothing on the ROS side re-implements low-level control.
