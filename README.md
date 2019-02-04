# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

# Autonomous driving with PID controllers

## Objective
The objective of this project is to use PID controllers to control the steering angle and the speed for driving a car in a game simulator. The simulator provides cross-track error (CTE) via websocket. The PID controllers give steering and speed commands to drive the car reliably around the simulator track.

## Reflection

### Describe the effect each of the P, I, D components had in your implementation.

**Description of PID values in PID control**

* **P** (proportional) accounts for present values of the error. For example, if the error is large and positive, the control output will also be large and positive. This controller tries to take the car to the center line from both sides. Due to the overshoot effect, this controller gives the zig-zag pattern around the center line if not tuned properly.

* **I** (integral) accounts for all past values of the error. For example, if the current output is not sufficiently strong, the integral of the error will accumulate over time, and the controller will respond by applying a stronger action. This controller eliminates the bias from CTE and helps the car to be on center line.

* **D** (differential) accounts for possible future trends of the error, based on its current rate of change. This controller reduces the zig-zag effect of P Controller which was mentioned earlier, and help car to reach the center line smoothly.

### Finally parameters

* PID parameters used for **steering angles**: 

    * P value: 0.2
    * I value: 0.002
    * D value: 7.8

* PID parameters used for **speed**: 

    * P value: 0.45 
    * I value: 0.0
    * D value: 1.5

### Describe how the final hyperparameters were chosen.

After the basic code implementation, I started tuning my PID controllers value manually. In order to automatically fine tune the parameters, an optimization algorithm **twiddle** can be used.

The parameters are tuned manually with the order of: P, D, I. The D and I are first set to zeros, and P value is set to 0.2. P value is adjusted up and down till the car could drive around the first corner and hard to improve more. Then keep the P value as it is, and increase the D value. Use the same approach for D value and I values.

### Video Output 

[Video](https://github.com/viralzaveri12/CarND-PID-Control-Project/blob/master/video/pid_controller.mp4)

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Code Style

* [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

