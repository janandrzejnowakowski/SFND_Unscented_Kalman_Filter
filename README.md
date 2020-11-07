# SFND_Unscented_Kalman_Filter

## Process noise optimization with NIS

The values of process noise standard deviations (std_a_ and std_yawdd_ have been optimized using NIS calculation.
The goal is that the ratio of cases for which NIS is within the theoretical 95% value (I will call it NIS95 below) is as close to 0.95 as possible for all three cars.

The initial values of 30, 30 give very bad results: NIS95 for both lidar and radar at the end of the simulation are below 0.05.
After trial and error, I found that a good NIS for both process noise values are 5.

After the entire simulation, the values for NIS95 for both LIDAR and RADAR for all 3 cars are between 0.92 and 0.964.
The specific results are shown below:

- NIS for LIDAR within 95% threshold: 0.963211
- NIS for RADAR within 95% threshold: 0.926667
- NIS for LIDAR within 95% threshold: 0.949833
- NIS for RADAR within 95% threshold: 0.923333
- NIS for LIDAR within 95% threshold: 0.953177
- NIS for RADAR within 95% threshold: 0.92

## Demo

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

## Dependencies

* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4
* PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`


