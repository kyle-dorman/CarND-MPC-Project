# Writeup

In this project, I created a model predictive controller (MPC) which actuates a simulator car's steering angle and throttle (acceleration & brake) given the car's current global position, speed, global orientation, current actuation values, and a best path. This solution proved more robust to handle actuation latency than a simple PID controller. In the image below, the yellow path is the desired path and the green path is the suggested path my the MPC.

[//]: # (Image References)
[image1]: ./writeup_images/turn.png "Car turning"

![alt image][image1]

### The Model
The model I used has six state variables:
* `x` : x position in car's coordinate space (meters)
* `y` : y position in car's coordinate space (meters)
* `psi` : car orientation (radians)
* `v` : car velocity (m/s)
* `cte` : the cross track error (meters)
* `epsi` : orientation error (radians)

The model has two actuators:
* `delta` : the car's steering wheel angle (radians)
* `a` : the car's acceleration (m/s^2)

The model has two polynomial functions:
* `f(x)` : the car's desired path
* `f'(x)` : the derivative of the car's desired path

The model has one constant:
* `Lf` : length from front of the car to the car's center of gravity

The model has one additional variable:
* `d_psi` : the desired car orientation. The arctangent to the car's desired path.

The model's update equations are:
* `x1 = x0 + v0 * cos(psi0) * dt`
* `y1 = y0 + v0 * sin(psi0) * dt`
* `psi1 = psi0 + v0 / Lf * delta * dt`
* `v1 = v0 + a0 * dt`
* `cte1 = cte0 + v0 * sin(epsi0) * dt`
* `cte0 = f(x0) - y0`
* `epsi1 = epsi0 + v0 / Lf * delta * dt`
* `espi0 = psi0 - d_psi`
* `d_psi = arctan(f'(x0))`

### Time step Length & Elapsed Duration (n & dt)
I've chosen an elapsed duration(N) of 25 because this allows the path to "see" far enough into the future to account for turns ahead while not being so large that it slows down the program. I've chosen a time step length(dt) of 0.05 seconds. This time step was short enough that the car didn't over correct by assuming an overly linear start and was long enough to actuate strong enough to account for the 100 ms latency. If the lag parameter changed of the top speed used (I am using 60 mph right now) changed these two hyper-parameters would have to be tuned again. 

### Model Predictive Control with Latency
The project assumes a latency of 100 ms. Because MPC uses a path of future points to calculate the next actuation commands, accounting for latency is much easier than when using a PID controller directly. In this project my steps to deal with latency were as follows:

1. project path coordinates into vehicle coordinate space.
2. fit a 3rd order polynomial equation to this path
3. use the above model update equations to predict the car's x, y, psi, and v 100 ms into the future given the current actuation commands. 
4. Use this updated x, y, psi, and v state to calculate the cte and epsi.

The other method I tried was to add additional constraints to the Ipopt solver to state the the actuations would be fixed for the first X states. This wasn't possible because I ended up having more constraints than free parameters. 

### Future
More work could be done to increase the speed of the car and tune the error weights more. 

