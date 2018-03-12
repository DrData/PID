# PID Controls

---
## Rubic
### Compilation
1. Code should compile.

### Implementation
1. The PID procedure follows what was taught in the lessons.
It's encouraged to be creative, particularly around hyperparameter tuning/optimization. However, the base algorithm should follow what's presented in the lessons.

The PID procedure is straight forward. The steering angle prediction is given by three terms, the proportional ( Kp \* p\_error), integral (Ki \* i\_error) and derivative terms ( Kd \* d\_error):

steering = - Kp \* p\_error - Ki \* i\_error - Kd \* d\_error

p_error is the current error,
i_error is the accumulated error (sum of all errors)
d_error is the difference of the current error from the previous error.

For each measurement from the simulator the crosstrack error, cte, is provided that gives in meters the deviation from the ideal track, the center of the road, from the currect position.

This cte is sent to the PID::UPdate methode where it is processed. As stated before 
```
d_error = cte - p_error // Note that the previous value of p_error is the previous cte
p_error = cte
i_error = i_ierror + cte
```
This is implemented in the PID::TotalError(). The essential stte

```
PID::TotalError() {
    return  - Kp * p_error - Ki * i_error - Kd * d_error;
    }
```
The steering angle is sent back to the simulator.

3. Optimization
The initial optimization was done manually so that the car could make the lap at the default throttle value of 0.3;

After that, Twiddle optimization procedure was implemented in a class named Twiddle.

Twiddle was implemented as a state machine based on the current estimated error and best error. 

There here 3 states, intialization, step 1, that implemented the first part of Twiddle where the parameter delta is expanded if it was found to lower the error and step 2, where the parameter delta value is contracted.

The error is computed by accumulating the errors for some fixed number of steps.  After the required number of steps were reached, the Twiddle state would be updated accordingly.



### Reflection
1. Describe the effect each of the P, I, D components had in your implementation.

    * Effect of P was to send a response directly proportional to the current crosstrack error. This has the effect of reducing the error, but also, if the proportion is large enough to cause the car to overshoot the centerline target. Too small a term would also reduce the error partial and if the track were straight to eveutaully reach the centerline.
    * The derivative term add a contribution that is proportional to the difference between the current cte and the prvious cte. So if the cte is decreasing, the derivative, D, term is opposite in effect to the P term. This helps to reduce the overshoot of a proportional only term. A large overshoot will cause a bigger counter effect from the D term. If the cte is only slight reduced, then the D term has a smaller effect. If the error continues to increase, then the D term reinforces the proportional term.
    * The integral term has the effecto correcting for a constant bias or drift in the car. If the errors are random about the center then the sum of the errors should fluctuate about zero. But if there is a bias, then the sum of the cte will have a non-zero mean. So, the I term makes a correction of for non-zero bias by adding a correction to the bias accumulated in the sum.

In this actual implementation:
* Here is a vidoe with a proportional constant of 3.0 and zero for the other 2 errors. As you can see in the straight away it can adjust, but at some point it startes to oscillate and blows up.

![Video P-term](./images/Pterm-1.mp4)

* Here is a video with only P and D terms. As there is not expected to be any drift noise in the car, this should be sufficient and we are able to navigate a lap with onlyh PD terms. There are some variations due to the track running counterclock wise which may introduce a bias.

![Video PD-only](./images/pdterm_only.mp4)


Below is a video with a large coefficent for the integral term. It overcompensates for a zero bias and pulls the car off track immediately.

![Video Big Integral constant term](./images/ibig.mp4)

Below is a video with a small coefficent for the integral term and it seems to help with reduce the oscillations as it tends to help with the counter clockwise bias. Althougth we have show it not needed, but seems to help.

![Video small Integral constant term](./images/pid_sight_i.mp4)



Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected?

Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

2. Describe how the final hyperparameters were chosen.

We started with manual tuning to try to get the car around the track once.

    * We start primarily with PD terms as show from above. The derivative term was crucial in keeping the oscillations down. I switched between the P and D terms to get the car around one lap. While P caused oscillations, it was needed to get the car to make a big enough adjustment to get to centerline. The D term was then adjusted to reduce oscillations. The selected manual parameters were: 0.1 .00135 2.1 at a throttle of 0.3

    * Automatic adjustment with Twiddle was attempted, but it didn't really improve the manual parameters by much.
    Below is video of the Twiddle optimization. It runs for some predetermined number of steps to estimate the error and then it is reset by issuing a 'reset' message command to the simulator.
    
    ```
    	 std::string msg = "42[\"reset\",{}]";
    ```
![Video Twiddle Optimization](./images/optr_hi.mp4)
  
  I ran it for 130 iteration and there were slight improvements. It may require additional iterations.
  
  I tried to speed up the iterations by stop iterations that either had near zero velcitiy or had crosstrack error greater than 6.5 m, which is off track.
  
To run pid with optimization:
Linux:
./pid 0.1 .00135 2.1 1.0 1.0 .001 1.0
Windows:
pid.exe 0.1 .00135 2.1 1.0 1.0 .001 1.0

The arguments are:
Kp Ki Kd Throttle dKp dKi dKd
where the latter 3 are the initial deltas used in Twiddle.

### Simulation
1. The vehicle must successfully drive a lap around the track.

Here is a video with a manually tuned PID:



![One Lap - Manually tuned PID](./images/run_final.mp4)

To run:
pid.exe 
or
pid.exe 0.1 0.00135 2.1 0.3


