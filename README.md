# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Overview ##

This project presents an implementation of a model based predictive control algorithm
to steer the car in the Udacity Self-Driving Car Simulator around a racetrack.

## Build Instructions ##

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Videos ##
Two videos of the car completing the course at roughly 40 mph and 80 mph can be found here https://youtu.be/eZcKhXgED60 and here https://youtu.be/j9wrlbY2iwU

## Vehicle model ##
The bicycle model discussed in the lecture videos is used as motion model for the car.
This motion model is based on the following six state values:

* x-coordinate of the car (y)
* y-coordinate of the car (x)
* orientation, i.e, yaw anlge of the car (psi)
* velocity of the car (v)
* cross-track error of the car (cte)
* orientation error of the car (epsi)

The vehicle can be actuated by adjusting the:

* steering angle (delta)
* throttle (a)

Based on these parameters, the motion of the car is described by the following
equations which relate the state of the car at time `t` to the state of the car
at time `t+1`:

```
x_t+1 = x_t + v_t * cos(psi_t) * dt
y_t+1 = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t / Lf * delta * dt
v_t+1 = v_t + a_t * dt
```
Here, `dt` denotes the time step size and `Lf` is distance between the front
of the car and its center of gravity.

## Model Based Predictive Control ##
In contrast to PID control, Model based predictive control (MPC) exploits
physical knowledge about the system of interest and aims at choosing optimal
values for the actuation parameters `delta`and `a` such that the error to the
desired car trajectory is minimal.

### Way Points Pre-Processing and Polynomial Fit ###
The desired trajectory of the car is provided in the form of way points given in
a map coordinate system. To simplify the math, it is convenient to first transform
these way points in the coordinate system of the car.

In order to be able to evaluate deviations from the trajectory also between
said way points, a third order polynomial is fitted to the six way points
closest to the vehicle.  

Given the desired trajectory of the car and a model describing its motion under
actuation, it is now possible to set up an optimization problem to find a set
of optimal values for the actuation parameters.

One of the main task of this project, was the proper definition and set-up of this
optimization problem, or, more specifically, the cost function of the optimization
problem.

### Timestep Length and Elapsed Duration ###
First, the time-interval over which the motion of the car is predicted using the model, needs to be specified.
An additional parameter is the number of time-steps into which this interval is subdivided.
I started out using 30 time-steps with a time-step size of `dt`= 50ms.
This choice led to trajectory predictions that seemed unnecessary long, so I decreased
the number of time-steps while also increasing the time step size to reduce computational
cost. Through iterations I found that `N`=10 time-steps and a time-step of `dt`= 100ms yielded
good results if the target speed of the car was set to 40 mph.

### Cost Function with Penalty Factors ###
The choice which terms to include in the cost function and how to weight the different
terms presented the second major task in designing the controller.

I started out with a very simple cost function that only included only the cross-track error,
the velocity error, and the direction error. This choice turned out to deliver
relatively poor performance. The car quickly went off track or started to oscillate wildly.

I then designed the following cost function:

```
Cost = cte_weight* cte^2 + epsi_weight* e_psi^2 + ref_v_weight* e_vel^2
        + delta_weight * delta^2 + a_weight * a^2 + deltadot_weight * d_delta^2 + adot_weight d_a^2 ,
```

which penalizes any differences from the desired state, large values for actuators, a
and large changes in consecutive actuations. The above cost function has several tunable
parameters. Namely, the factors with which the individual contributions can be weighted.
Largely through try and error, I arrived at the following weights which resulted in a
well performing controller at desired speed of 40 mph:

* cte_weight = 1.0
* epsi_weight = 10.0
* ref_v_weight = 1.0
* delta_weight = 150.0
* a_weight = 10.0
* deltadot_weight = 30.0
* adot_weight = 1.0

For a speed of 80 mph, the following parameter setting provided satisfactory results:

* cte_weight = 1.0
* epsi_weight = 20.0
* ref_v_weight = 1.0
* delta_weight = 15000.0
* a_weight = 10.0
* deltadot_weight = 300.0
* adot_weight = 2.0

### Dealing with Latency ###
One of the benefits of MPC is that one can account for latency rather easily.
Since we have a sequence of predictions where the car will be and also corresponding
optimal actuator settings for these points in time, we can simply anticipate the
latency in the system and choose to pass actuator settings corresponding
not to the present but to a future point in time.

In this project I implemented the MPC such that it returns the actuator settings
that are optimal roughly 100 ms in the future, thereby taking into account the
latency in the system.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.
