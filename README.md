# Unscented Kalman Filter Project Starter Code
[Udacity - Self-Driving Car NanoDegree Unscented Kalman Filter Project]
(https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project)

## Overview
---
In this project, the aim is to implement the Unscented Kalman Filter in c++. LIDAR and RADAR measurements detecting a bicycle that travels around the vehicle, were provided by Udacity.

## The Project

The changes preformed to implement the Unscented Kalman Filter are listed as follows.

* Complete CalculateRMSE function in the Tools class.
* Complete ProcessMeasurement, Prediction, UpdateLidar, and UpdateRadar functions in UKF class.

### RMSE

The RMSE values for three files and some of the experimented parameters are given in the table below. Files are new (obj_pose-laser-radar-synthetic-input.txt), old1 (sample-laser-radar-measurement-data-1.txt), old2 (sample-laser-radar-measurement-data-2.txt). The P and x rows specify the initialization value of the given matrix or vector. The velocity parameters are different for lidar initialization and radar initialization.

The Requi. column has the requirement values specified in the rubric. Best_old2 column has the parameters that worked best on the old2 data, similarly Best_old1 on the old1 data and Best_new on the new data. The Low.RMSE column has the parameters that were result in the lowest average RMSE. The selected column has the selected parameters.

|:  File    :|: Parameter   :|: Requi.:|:Best_old2:|:Best_old1:|:         :|:         :|:         :|:         :|:         :|:Best_new :|:Low.RMSE :|:         :|: SELECTED:|
|:          :|: avg RMSE    :|: REQ   :|: 0.30322 :|:   N/A   :|: 0.28465 :|: 0.26635 :|: 0.26219 :|: 0.26205 :|: 0.26064 :|:         :|: 0.25766 :|: 0.25908 :|: 0.25824 :|
|:          :|: P(0,0)      :|: REQ   :|: 1       :|:   1     :|: 1       :|: 1       :|: 1       :|: 1       :|: 1       :|: 1       :|: 0.5     :|: 2       :|: 1       :|
|:          :|: P(1,1)      :|: REQ   :|: 1       :|:   1     :|: 1       :|: 1       :|: 1       :|: 1       :|: 1       :|: 1       :|: 0.5     :|: 2       :|: 1       :|
|:          :|: P(2,2) lidar:|: REQ   :|: 1       :|:   1     :|: 1       :|: 1       :|: 20      :|: 20      :|: 20      :|: 20      :|: 20      :|: 20      :|: 20      :|
|:          :|: P(2,2) radar:|: REQ   :|: 1       :|:   1     :|: 1       :|: 1       :|: 20      :|: 1       :|: 1       :|: 1       :|: 1       :|: 1       :|: 1       :|
|:          :|: P(3,3)      :|: REQ   :|: 2*M_PI  :|:   10    :|: 10      :|: 10      :|: 10      :|: 10      :|: 13.15   :|: 13.15   :|: 13.15   :|: 13.15   :|: 13.15   :|
|:          :|: P(4,4)      :|: REQ   :|: 1       :|:   1     :|: 1       :|: 1       :|: 1       :|: 1       :|: 1       :|: 1       :|: 0.1     :|: 0.1     :|: 0.1     :|
|:          :|: x(2) lidar  :|: REQ   :|: 0       :|:   0     :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 5       :|: 0       :|: 0       :|: 0       :|
|:          :|: x(2) radar  :|: REQ   :|: rhodot  :|:   rhodot:|: rhodot  :|: rhodot  :|: rhodot  :|: rhodot  :|: rhodot  :|: rhodot  :|: rhodot  :|: rhodot  :|: rhodot  :|
|:          :|: x(3)        :|: REQ   :|: 0       :|:   0     :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|
|:          :|: x(4)        :|: REQ   :|: 0       :|:   0     :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|: 0       :|
|:          :|: std_a_      :|: REQ   :|: 1.28    :|:   1.28  :|: 1.28    :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|
|:          :|: std_yawdd_  :|: REQ   :|: 0.275   :|:   1     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|: 0.6     :|
|:----------:|:-------------:|:-------:|:---------:|:---------:|:---------:|:---------:|:---------:|:---------:|:---------:|:---------:|:---------:|:---------:|:---------:|
|:  NEW     :|: px          :|: 0.090 :|: 0.07266 :|: 0.06612 :|: 0.06688 :|: 0.06135 :|: 0.06122 :|: 0.06122 :|: 0.06118 :|: 0.06194 :|: 0.06107 :|: 0.06106 :|: 0.06107 :|
|:  NEW     :|: py          :|: 0.100 :|: 0.08405 :|: 0.08237 :|: 0.08132 :|: 0.08355 :|: 0.08397 :|: 0.08397 :|: 0.08290 :|: 0.08425 :|: 0.08272 :|: 0.08315 :|: 0.08282 :|
|:  NEW     :|: vx          :|: 0.400 :|: 0.34609 :|: 0.32535 :|: 0.32269 :|: 0.31807 :|: 0.27277 :|: 0.27277 :|: 0.27266 :|: 0.15068 :|: 0.27395 :|: 0.27215 :|: 0.27259 :|
|:  NEW     :|: vy          :|: 0.300 :|: 0.24575 :|: 0.20633 :|: 0.19953 :|: 0.19367 :|: 0.18756 :|: 0.18756 :|: 0.16066 :|: 0.16177 :|: 0.15274 :|: 0.15303 :|: 0.15289 :|
|:  NEW     :|: avg         :|: 0.223 :|: 0.18714 :|: 0.17005 :|: 0.16760 :|: 0.16416 :|: 0.15138 :|: 0.15138 :|: 0.14435 :|: 0.11466 :|: 0.14262 :|: 0.14235 :|: 0.14234 :|
|:  OLD1    :|: px          :|: 0.090 :|: 0.14161 :|: 0.05071 :|: 0.07256 :|: 0.07165 :|: 0.07148 :|: 0.07165 :|: 0.07163 :|:    -    :|: 0.07152 :|: 0.07153 :|: 0.07153 :|
|:  OLD1    :|: py          :|: 0.090 :|: 0.16615 :|: 0.05756 :|: 0.08021 :|: 0.07757 :|: 0.07835 :|: 0.07758 :|: 0.07743 :|:    -    :|: 0.07838 :|: 0.07837 :|: 0.07838 :|
|:  OLD1    :|: vx          :|: 0.650 :|: 0.68869 :|: 0.51380 :|: 0.56535 :|: 0.56874 :|: 0.56801 :|: 0.56873 :|: 0.57031 :|:    -    :|: 0.57038 :|: 0.57037 :|: 0.57037 :|
|:  OLD1    :|: vy          :|: 0.650 :|: 0.71170 :|: 0.53167 :|: 0.57231 :|: 0.56764 :|: 0.56954 :|: 0.56766 :|: 0.56905 :|:    -    :|: 0.57130 :|: 0.57129 :|: 0.57130 :|
|:  OLD1    :|: avg         :|: 0.370 :|: 0.42704 :|: 0.28843 :|: 0.32261 :|: 0.32140 :|: 0.32184 :|: 0.32140 :|: 0.32210 :|:    -    :|: 0.32290 :|: 0.32289 :|: 0.32289 :|
|:  OLD2    :|: px          :|: 0.200 :|: 0.18855 :|:    -    :|: 0.19775 :|: 0.19118 :|: 0.19114 :|: 0.19114 :|: 0.19124 :|: 0.18892 :|: 0.19050 :|: 0.19216 :|: 0.19088 :|
|:  OLD2    :|: py          :|: 0.200 :|: 0.18496 :|:    -    :|: 0.19032 :|: 0.19002 :|: 0.19001 :|: 0.19001 :|: 0.18991 :|: 0.19400 :|: 0.18918 :|: 0.19162 :|: 0.19001 :|
|:  OLD2    :|: vx          :|: 0.550 :|: 0.43024 :|:    -    :|: 0.52629 :|: 0.37492 :|: 0.37438 :|: 0.37438 :|: 0.37347 :|: 0.51077 :|: 0.36811 :|: 0.38146 :|: 0.37468 :|
|:  OLD2    :|: vy          :|: 0.550 :|: 0.37823 :|:    -    :|: 0.54060 :|: 0.49781 :|: 0.49790 :|: 0.49790 :|: 0.50727 :|: 0.49780 :|: 0.48213 :|: 0.48276 :|: 0.48244 :|
|:  OLD2    :|: avg         :|: 0.375 :|: 0.29550 :|:    -    :|: 0.36374 :|: 0.31348 :|: 0.31336 :|: 0.31336 :|: 0.31547 :|: 0.34787 :|: 0.30748 :|: 0.31200 :|: 0.30950 :|

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`
