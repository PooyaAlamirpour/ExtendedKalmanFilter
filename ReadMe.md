# SELF-DRIVING CAR: Extended Kalman Filter

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

The EKF(Extended Kalman Filter) is nonlinear of the main Kalman Filter which is used for estimating the current mean and covariance of the system. If the measurement and the state transition model are both linear, as in that case, the extended Kalman filter is identical to the regular one. Having stated this, the extended Kalman filter can give a reasonable performance and is arguably the de facto standard in navigation systems and GPS. In this project, I have shared my work on the Extended Kalman Filter. Thank [Udacity](https://www.udacity.com/) for providing an amazing platform for simulating the project.

### Files in the Github src Folder
You can find my implementation under src folder which is explained as follow:
- `main.cp`p communicates with the [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
- `FusionEKF.cpp` initializes the filter, calls the predict function, calls the update function
- `kalman_filter.cpp` defines the predict function, the update function for lidar, and the update function for radar
- `tools.cpp` function to calculate RMSE and the Jacobian matrix

### How the Files Relate to Each Other
Here is a brief overview of what happens when you run the code files:
- `Main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`
- `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. `FusionEKF.cpp` has a variable called `ekf_`, which is an instance of a `KalmanFilter` class. The `ekf_` will hold the matrix and vector values. You will also use the ekf_ instance to call the predict and update equations.
- The `KalmanFilter` class is defined in `kalman_filter.cpp` and `kalman_filter.h`. You will only need to modify 'kalman_filter.cpp', which contains functions for the prediction and update steps.

### Running the project
Feel free to clone [this](https://github.com/PooyaAlamirpour/ExtendedKalmanFilter) repository and make a change. Before cloning this project you should consider that there some dependencies. Not all steps will be necessary, for example, installing git and cloning the project repository, if this has already been done.
```bash
$ sudo apt-get update
$ sudo apt-get install git
$ sudo apt-get install cmake
$ sudo apt-get install openssl
$ sudo apt-get install libssl-dev
$ sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
```
This project involve using an open source package called uWebSocketIO. This package facilitates the same connection between the simulator and code. The package does this by setting up a web socket server connection from the C++ program to the simulator, which acts as the host.
```bash
sudo apt-get update
sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
```
Now it is time to clone and build the project.
```bash
$ git clone https://github.com/PooyaAlamirpour/ExtendedKalmanFilter.git
$ mkdir build && cd build
$ ./ExtendedKF
```
Once the project builds and runs successfully, run the simulator. You should see the below picture. A small car that generates green dots which is indicated the output of the Extended Kalman Filter.
![Output](https://github.com/PooyaAlamirpour/BehavioralCloning/blob/master/Pictures/welcome-simulation.jpg)


## References
- [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter)

