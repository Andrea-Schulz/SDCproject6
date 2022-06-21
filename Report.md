# Control and Trajectory Tracking for Autonomous Vehicles

## PID controller

run the simulator and see in the desktop mode the car in the CARLA simulator. Take a screenshot and add it to your report. The car should not move in the simulation.

## PID controller for throttle

Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

## PID controller for steer

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

## Evaluate PID efficiency

### Add the plots to your report and explain them (describe what you see)

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
