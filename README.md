# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

#Implementation
##General
Each timestep we get the previous path that has not been followed from the simulator. A difficult concept for
me was determiniing what to do with this path. I had troubles trying to create a new path mainly because the car would
travel on the previous path during calculations of the new path. Ultimatley I decided it was easiest to just continue 
using the previous path and start the new calculations from the end of this path. This obviously has issues, for example
if you sense something in the new timestep that interferes with the previous path then you would not react to it,
because you are starting from the end of that path. To mitigate this risk I tried to use a minimum amount of points ahead.
With the current implementation of 25 points the vehicle will be able to react in at least 500 ms. Which is roughly 11 m
and too far for any real world implementation.

## Vehicle class
Created a vehicle class to store the vehicle state. The vehicle class is created with some parameters and then updated
every time step with the previous path and the current position of the car. The vehicle class also has functions to 
determine what action to take, find vehicles ahead and behind of the car in a lane, and create paths for staying in the
current lane or merging lanes.

## Going straight
For driving straight, we find the speed of the car in front of us, and predict given our cars speed and the speed of
the car in front of us weather we will get to an unsafe traveling distance. If we will, then we slow down to the speed
of the car in front of us. What I found when implementing this is that the siumulator does not give an accurate speed
for the car in front, so the safety distance needs to be a certain length or we will collide trying to match the velocity
 of the car in front. I added in logic to go 90% of the speed of the car in front of us if we are in the unsafe zone to 
prevent collisions There are currently no safety procedures implemented if we would need to exceed the comfortable
 acceleration to prevent a collision.

### merging

#### Jerk Minimizing Functions
I tried very hard to implement jerk minimizing functions into this path planner. But I was not able to find an elegant
way to do it. The main issue is that jerk minimizing functions must control the velocity in order to minimize the jerk.
In the simulator the only way to control velocity is through the spacing of the points and the conversion from s d to x y
coordinates had discontinuities. So the simulator inherently made it extremely difficult to use the functions. The most
elegant way I found was to create a jerk minimized function and create a spline with the function sampled at larger 
spacings to attempt to remove the discontinuities and then try to resample the spline given the known velocities.
This however still had issues as resembling either introduced back the discontinuities or removed the velocity control 
from the function. I didn't feel it was worth getting more complicated cause the main issue of having to space the points
 to get desired velocity doesn't not seem very usefull to spend a ton of time finding a solution for.
  
#### merging path
Currently merging is implemented using a minimum and maximum merge time. If there is a car in the other lane the car will
determine if it can merge into that lane in the minimum merge time and if it will and the plannet tells it to merge it will
merge if there is not a car then it just uses the max merge time. The path is created by creating a spline from the previous
path and the point the car reaches the center of the new lane.

### Path Planner
The path planner is a simple cost function taking into account a goal lane, speed and the distance to cars in ahead and behind

# TODO's
* Clean up the code
* better lane changing that allows the vehicle to exit a lane change if another car merges into the lane and also does
a better job accounting for the distance of the vehicle in the current lane when perfomring the lane change
* Make the vehicle parameters less speed dependent currently I think a lot of the parameters that can be tuned such as the
min and max merge time are tuned for highway speeds and may not fairwell if the speed was lowered.

### I think the cost funciton could be improved on

---

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

