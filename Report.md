# Control and Trajectory Tracking for Autonomous Vehicles

## PID controller

Running the CARLA simulator as described in the [README](README.md)
(without the controllers implemented, or with the control coefficients set to zero)
causes the vehicle to remain in its start position.

![Alt text](project/pid_controller/screenshot/screenshot1.jpg?raw=true)

## PID controller for throttle & steer - Evaluate PID efficiency

After starting with initial control coefficients `Kp`, `Kd` and `Ki` close to zero for each controller, 
I ended up with the coefficients for throttle being around an order of 10 smaller than those for steering.

At the same time, I decreased `Kp` and increased `Kd` for throttle, 
trying to achieve a less jerky acceleration/deceleration of the vehicle.

Seeing that the steering control was overshooting a lot, I also decreased `Kp` here.
Since the dampening effect of `Kd` slowed down the reaction of the steering controller quite a bit, I kept that value very low.

Overall, it could be observed that both controllers were reacting slowly, probably due to the update cycle time.
Hence, errors in both controllers had to be balanced out carefully
(e.g. if the target speed was reached too fast, the steer controller would not react in time to avoid a car in front, 
or overshoot ans steer off the road).

Also, after overtaking the second vehicle on the left side of the road, 
the path planner seems to produce a trajectory towards the wall: even though the vehicle has reached the right lane already, 
it starts to *progressively* steer towards the wall, almost as if initiating a right turn. 
This behaviour persisted for a range of different controller settings.

### Throttle Control

![Alt text](project/pid_controller/screenshot/throttle_04.jpg?raw=true)

The above plot shows the error between the current and the target velocity (blue) and the respective throttle (green) and brake (orange) outputs of the controller.
Applying a decent amount of throttle, the controller tries to bring the vehicle up to its target speed.
Over >30 iterations, the error could not be fully eliminated, 
probably due to the conservative parametrization of the controller explained above. 

The velocity error initially starts at -1.0, indicating that a step change in velocity is issued to the controller.

### Steer Control

The steering control plot shows the error between the current and target yaw angle, i.e. the yaw angle of the vehicle in comparison to
its desired orientation on the target trajectory (blue), as well as the respective control output (orange).

Other than the throttle controller, the steering controller seems to produce some oscillations/overshooting.
One can clearly see the influence of the `Kp` coefficient (the controller is acting when the error gets nominally bigger).
Increasing `Kd` to achieve a dampening effect here did not yield the desired results though - instead the steering became "slower", 
causing the vehicle to deviate from its trajectory even further and most often crash into obstacles.

![Alt text](project/pid_controller/screenshot/steer_04.jpg?raw=true)

### What is the effect of the PID according to the plots, how each part of the PID affects the control command?

Each element of the PID controller has a distinct effect:
* The proportional (P) element applies a control input based on an error `err(t)`, i.e. based on the difference between the current and the desired value (setpoint) of the controlled process variable. The larger this difference, the higher the control input. Increasing the respective control coefficient `Kp` leads to a fast decrease of `err(t)`, but the system will likely overshoot and oscillate around the setpoint.
* The derivative (D) part takes into account the change between the current and previous error `err(t) - err(t-1)`. Increasing `Kd` has a dampening effect on the overall output of the controller, reducing overshoot of the controlled process variable, but also increasing the time until which the setpoint is reached.
* lastly, the integral (I) part applies a control input based on the integral below the error over time. This part of the controller is especially useful to tackle biases and drifts in our control variable, because the integral continues to grow (and the control input is increased) if an offset to the final setpoint persists.

### How would you design a way to automatically tune the PID parameters?

As outlined in the course, I would resort to optimization techniques such as "Twiddle" in order to find the coefficients which best minimize the error `err` for each controller.

### PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?

Advantages of PID control include its simplicity in implementation and tuning - by defining a suitable control variable and a set of control coefficients, it is possible to yield decent results for a variety of systems.

A model-based controller on the other hand needs more effort in its implementation and more computational power in each time step than a simple PID controller. For complex, multi-variable systems though, it might be crucial to rely on a model and take into account how different variables influence each other in order to achieve the desired system behaviour.

Furthermore, a model-based controller can incorporate limits in the system or control variables, meaning that it can apply the optimal amount of control input to reach the setpoint in a given time. A PID controller can't do this - it might run into those limits and/or ultimately not produce the desired system behaviour.

### Improvements of the PID controller

One might consider incorporating other values into the steering control, 
e.g. instead of minimizing the orientation/angle error, minimize the perpendicular deviation from the desired trajectory. 

Depending on the length of the given trajectory, it might not be the best solution to choose the 
*final* position/orientation/velocity on the trajectory as a setpoint for our controller.
As can be seen in the simulation, this approach can lead to high control inputs and overshooting, 
or might not even yield the necessary control input at all: 
if the given trajectory is an S-curve, the target/setpoint yaw might be very similar to the current one, 
thus producing only a small error. Yet, the vehicle *should* steer in order to actually reach the target position on the trajectory.

Implementing a realistic trajectory *towards the target trajectory* instead of a single set point
might lead to a better behaviour under those circumstances.