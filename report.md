# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Project Goal
The goal of this project is to make a car go around the simulated track using Model Predictive Control

## Introduction
Model Predictive Control is a technique for controlling a robot using a model of the robot, its actuators and its contraints. The controller is given a desired path and it solves a minimization problem planning the current and future use of actuators that minimizes the difference between the desired path and the real path.

## The model
MPC needs a model of the car to operate. The model consists mainly of constraints on how the vehicle operates, and a cost function that models what characteristics of the trajectory we value most.

### State vector
We feed our controller a state vector that describes the state of the car using the following variables:
 *  x: x position of the car
 *  y: y position of the car
 *  psi: yaw
 *  v: velocity
 *  cte: cross track error (distance from the car to the center of the track)
 *  epsi: yaw error, difference from desired to actual yaw
 
 The controller then returns values for two actuators
 * steering
 * acceleration

### Constraints
Constraints are expressed as an equation that the solver must keep at zero. We will review all constraints now

#### Constraints of linear movement

fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

These constraints state that the position at t+1 must be equal to the position at t, extrapolated in a straight line to the present time.
#### Constraint of yaw change

fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);

This constraint limits how fast the car can change its angle. It is limited by the steering input divided by the factor Lf. The factor Lf is a measure of how far the steering is from the center of mass of the vehicle.
This constraint is a physical limitation of how the car can turn

#### Constraint of acceleration

fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

Again, a constraint for linearly accelerated movement. Velocity at t+1 equals velocity at T plus acceleration times the amount of time between timesteps.

#### Constraint of CTE

fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

This equation calculates the cross track error at timestep t+1. It is equal to the CTE at t plus the velocity component perpendicular to the track times delta t.

#### Constraint of yaw error

fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

This constraint refers to the error between the desired and actual yaw. It must update in the same way as the constraint of yaw change explained before

### Cost function

The cost function refers to the characteristics we value most of our designed trajectory. This implementation consists of 3 components.

#### Difference to the reference state

fg[0] += 1000*CppAD::pow(vars[cte_start + t], 2);
fg[0] += 1000*CppAD::pow(vars[epsi_start + t], 2);
fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);

The distance between the car and the track center, the difference between the desired and actual yaw and the difference between desired and actual velocity.
The first two are multiplied by a factor of 1000 to make the solver more aggresive in reducing these errors. 

#### Minimize use of actuators

fg[0] += 40*CppAD::pow(vars[delta_start + t], 2);
fg[0] += 40*CppAD::pow(vars[a_start + t], 2);

This just puts the amount of actuator input into the cost function. The car should try to steer as little as possible

#### Minimize value gap between sequential actuations
fg[0] += 170000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); 
fg[0] += 5000*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);

These equations make the solver try to keep a more constant path. This would make for a more comfortable drive in a self driving car.

### Choice of N and dt values
Our solver must use a discrete path prediction and we can choose 2 parameters. How many future steps we want to predict, and what is the time difference between the two.
I tried N=20 and dt=0.05 for most of the project, but I was having trouble with latency, so I decided to change it to N=10 and dt = 0.1
Planning much longer results in the estimated path going further than the reference path given by the simulator and gives strange results, as reported by some users on the forums.

### Coordinate transform, polynomial fitting and latency estimation

The coordinates given from the simulator are absolute, but it is easier to work with coordinates relative to the car. Also, the display of reference and predicted path uses relative coordinates, so these have to be transformed in order to show the path.
The transform happens uses the following code:

          //Transform
          for (unsigned int i=0; i< ptsx.size();i++){
            double x = ptsx[i] - px;
            double y = ptsy[i] - py; 
            ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
          }
          

A third order polynomial is then fitted using the polyfit function. If the track was more complex, or the prediction longer, a higher order polynomial would be required.

          auto coeffs = polyfit(xvals, yvals, 3);
          
Finally, because there is a delay between the solution and the actuation in the simulator, we need to actually get the solution for a car 100ms further. To do this, the position of the car is extrapolated by 100ms before creating the state vector for the solver. This happens using the following code

 //Take delay into account
          double delay = 0.1;
          double Lf = 2.67;
          double x_delay = v * delay;
          double y_delay = 0;
          double psi_delay = -v*delay*delta/Lf;
          double v_delay = v + acceleration * delay;
          double cte_delay = cte + (v * sin(epsi0) * delay);
          double epsi_delay = epsi0 - (v * atan(coeffs[1] * delay / Lf));


Thank you for reading
