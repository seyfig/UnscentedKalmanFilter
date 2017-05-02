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

The RMSE values for RADAR only and LIDAR only processes are given in the table below. The selected parameters from the previous table were used to run the filter. For the old2 data, in some conditions values in P matrix diverged. Therefore, if NIS value was greater than 100, the update process skipped.

|:  File    :|: Parameter   :|: Requirements    :|: RADAR+LIDAR :|: RADAR ONLY  :|: LIDAR ONLY  :|
|:----------:|:-------------:|:-----------------:|:-------------:|:-------------:|:-------------:|
|:  NEW     :|: px          :|: 0.090           :|: 0.06107     :|: 0.20441     :|: 0.15989     :|
|:  NEW     :|: py          :|: 0.100           :|: 0.08282     :|: 0.23349     :|: 0.14592     :|
|:  NEW     :|: vx          :|: 0.400           :|: 0.27259     :|: 0.29249     :|: 0.46023     :|
|:  NEW     :|: vy          :|: 0.300           :|: 0.15289     :|: 0.19576     :|: 0.20690     :|
|:  OLD1    :|: px          :|: 0.090           :|: 0.07153     :|: 0.12338     :|: 0.11348     :|
|:  OLD1    :|: py          :|: 0.090           :|: 0.07838     :|: 0.16317     :|: 0.12891     :|
|:  OLD1    :|: vx          :|: 0.650           :|: 0.57037     :|: 0.56044     :|: 0.67635     :|
|:  OLD1    :|: vy          :|: 0.650           :|: 0.57130     :|: 0.57606     :|: 0.61939     :|
|:  OLD2    :|: px          :|: 0.200           :|: 0.19088     :|: 1.82587     :|: 0.28737     :|
|:  OLD2    :|: py          :|: 0.200           :|: 0.19001     :|: 3.73854     :|: 1.94057     :|
|:  OLD2    :|: vx          :|: 0.550           :|: 0.37468     :|: 2.19923     :|: 0.95943     :|
|:  OLD2    :|: vy          :|: 0.550           :|: 0.48244     :|: 2.08024     :|: 1.00235     :|

### NIS

The NIS values summary for the three dataset are given in the following table. The NIS graphs are given in the ![alt text][Report].

|:  File  :|: Value                        :|: Radar Only :|: Lidar Only :|: Fusion Radar :|: Fusion Lidar :|
|:--------:|:------------------------------:|:------------:|:------------:|:--------------:|:--------------:|
|:  new   :|: mean                         :|: 2.71033    :|: 1.73162    :|: 2.83573      :|: 1.77478      :|
|:  new   :|: std                          :|: 2.26216    :|: 1.80275    :|: 2.22844      :|: 1.78846      :|
|:  new   :|: max                          :|: 12.08430   :|: 13.48040   :|: 11.07110     :|: 15.24100     :|
|:  new   :|: min                          :|: 0.04757    :|: 0.00326    :|: 0.10111      :|: 0.01118      :|
|:  new   :|: X<sup>2</sup><sub>.050</sub> :|: 4.80%      :|: 3.21%      :|: 4.00%        :|: 2.01%        :|
|:  old1  :|: mean                         :|: 4.01252    :|: 0.95504    :|: 4.36468      :|: 0.62636      :|
|:  old1  :|: std                          :|: 5.49846    :|: 0.81375    :|: 6.09372      :|: 0.61166      :|
|:  old1  :|: max                          :|: 55.44820   :|: 4.70880    :|: 58.16000     :|: 3.25361      :|
|:  old1  :|: min                          :|: 0.01391    :|: 0.00284    :|: 0.01568      :|: 0.00200      :|
|:  old1  :|: X<sup>2</sup><sub>.050</sub> :|: 15.06%     :|: 0.00%      :|: 16.86%       :|: 0.00%        :|
|:  old2  :|: mean                         :|: 7.33908    :|: 2.51252    :|: 1.17496      :|: 0.82762      :|
|:  old2  :|: std                          :|: 20.33697   :|: 13.58639   :|: 1.38871      :|: 0.81314      :|
|:  old2  :|: max                          :|: 124.94000  :|: 134.86600  :|: 9.12698      :|: 4.63514      :|
|:  old2  :|: min                          :|: -7.65543   :|: -1.12689   :|: 0.02756      :|: 0.00329      :|
|:  old2  :|: X<sup>2</sup><sub>.050</sub> :|: 11.00%     :|: 2.02%      :|: 1.00%        :|: 0.00%        :|

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
