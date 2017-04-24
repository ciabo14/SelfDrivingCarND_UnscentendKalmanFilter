# SelfDrivingCarND_UnscetedKalmanFilter

Implementation of the Unscented Kalman Filter (UKF) starting from the Udacity lessons. The Kalman filter is used for a sensor fusion application where data from lidar and radar are fused together for the estimation of the current position of an object (of a bike in this case). With the UKF non linear process/measurement functions are linearized with the use of sigma points. Furthermore a consistency analisys for the noise parameter is performed with the support of the NIS (Normalized Innovation Squared) indicator.

## Repository structure

The repository is made of 3 different folders:

1.  src: includes all the src files required for executable build and to execute the kalman filter;
2.  data: includes all the original data provided by udacity and the results output files; are also included the NIS output files for all the datasets
3.  executable: includes the executable of the program;
4.  utilities: includes the python script to create the graphs starting from the data in the data folder. The path for the input/output file in the python are hard coded.
5.  images: includes the images shown in the README

## USAGE instruction:

In order to execute the executable in the executable file you should follow the following usage instruction:

**Usage instructions:**

```c++
./ExtendedKF.exe
path/to/input.txt
path/to/output.txt
path/to/NISFile.txt
[r|l|b] for [radar only|laser only|both]
```
The executables is compiled in order to be executed with the single usage for lidar or radar or with the usage of both. The usage of a single sensor will select data from the input file in order to use only L|R or data from all the sensors.

## NIS index analysis and noise optimization:

Process noise parameters were optimized using the NIS indicator. The consistency check was done in the dataset 'obj_pose-laser-radar-synthetic-input.txt'. Since process noise is define by two parameters \sigma_{a} and \sigma_{\ddot{\phi}}, we have 2 degree of freedom and the NIS reference value is 5.991.

Whit the noise values optimized, the percentage of the vaules over the reference values are 6.6%.

##Results

The computed RMSE, also output of the executable, is stricty dependant on the sensor used. The results for the provided files are:

1. INPUT filename: **obj_pose-laser-radar-synthetic-input.txt**

Sensor  |     px     |     py     |     vx     |     vy     |
------- | ---------- | ---------- | ---------- | ---------- |
BOTH    |  0.0644369 |  0.0816021 |  0.32561   |  0.221245  |

![alt tag](https://github.com/ciabo14/SelfDrivingCarND_KalmanFilter/blob/master/images/dataset_new_image.png)

2. INPUT filename: **sample-laser-radar-measurement-data-1.txt**

Sensor  |     px     |     py     |     vx     |     vy     |
------- | ---------- | ---------- | ---------- | ---------- |
BOTH    |  0.0723371 |  0.0795866 |  0.589185  |  0.574702  |

![alt tag](https://github.com/ciabo14/SelfDrivingCarND_KalmanFilter/blob/master/images/dataset_old_1_image.png)

3. INPUT filename: **sample-laser-radar-measurement-data-2.txt**

Sensor  |     px     |     py     |     vx     |     vy     |
------- | ---------- | ---------- | ---------- | ---------- |
BOTH    |  0.193473  |  0.189554  |  0.419756  |  0.528889  |

![alt tag](https://github.com/ciabo14/SelfDrivingCarND_KalmanFilter/blob/master/images/dataset_old_2_image.png)
