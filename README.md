# Unscented Kalman Filter Project
[Udacity - Self-Driving Car NanoDegree Unscented Kalman Filter Project]
(https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project)

[//]: # (References)
[Report]: ./writeup_report.md "Project Report"

## Overview
---
In this project, the aim is to implement the Unscented Kalman Filter in c++. LIDAR and RADAR measurements detecting a bicycle that travels around the vehicle, were provided by Udacity.

## The Project

The changes preformed to implement the Unscented Kalman Filter are listed as follows.

* Complete CalculateRMSE function in the Tools class.
* Complete ProcessMeasurement, Prediction, UpdateLidar, and UpdateRadar functions in UKF class.

The details of the study is given in the ![alt text][Report].


### RMSE

The selected parameters and the according RMSE values for three different data are given in the table below. The detailed version of the table is given in the ![alt text][Report]. Files are new (obj_pose-laser-radar-synthetic-input.txt), old1 (sample-laser-radar-measurement-data-1.txt), old2 (sample-laser-radar-measurement-data-2.txt). The P and x rows specify the initialization value of the given matrix or vector. The velocity parameters are different for lidar initialization and radar initialization.


|:  File    :|: Parameter   :|: Requirements :|: SELECTED:|
|:          :|: avg RMSE    :|: REQ          :|: 0.25824 :|
|:          :|: P(0,0)      :|: REQ          :|: 1       :|
|:          :|: P(1,1)      :|: REQ          :|: 1       :|
|:          :|: P(2,2) lidar:|: REQ          :|: 20      :|
|:          :|: P(2,2) radar:|: REQ          :|: 1       :|
|:          :|: P(3,3)      :|: REQ          :|: 13.15   :|
|:          :|: P(4,4)      :|: REQ          :|: 0.1     :|
|:          :|: x(2) lidar  :|: REQ          :|: 0       :|
|:          :|: x(2) radar  :|: REQ          :|: rhodot  :|
|:          :|: x(3)        :|: REQ          :|: 0       :|
|:          :|: x(4)        :|: REQ          :|: 0       :|
|:          :|: std_a_      :|: REQ          :|: 0.6     :|
|:          :|: std_yawdd_  :|: REQ          :|: 0.6     :|
|:----------:|:-------------:|:--------------:|:---------:|
|:  NEW     :|: px          :|: 0.090        :|: 0.06107 :|
|:  NEW     :|: py          :|: 0.100        :|: 0.08282 :|
|:  NEW     :|: vx          :|: 0.400        :|: 0.27259 :|
|:  NEW     :|: vy          :|: 0.300        :|: 0.15289 :|
|:  OLD1    :|: px          :|: 0.090        :|: 0.07153 :|
|:  OLD1    :|: py          :|: 0.090        :|: 0.07838 :|
|:  OLD1    :|: vx          :|: 0.650        :|: 0.57037 :|
|:  OLD1    :|: vy          :|: 0.650        :|: 0.57130 :|
|:  OLD2    :|: px          :|: 0.200        :|: 0.19088 :|
|:  OLD2    :|: py          :|: 0.200        :|: 0.19001 :|
|:  OLD2    :|: vx          :|: 0.550        :|: 0.37468 :|
|:  OLD2    :|: vy          :|: 0.550        :|: 0.48244 :|

### NIS

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
