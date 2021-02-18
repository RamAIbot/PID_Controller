# PID Controller

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<p> The project focuses on designing a PID controller for the autonomous vehicle to drive through the simulated environment which is done by Unity. The car travles with a constant speed of 30mph and the PID controller controls the steering angles based on the manoeuvre. An additional PID controller is used to keep the speed at 30mph to prevent any accidents.</p>

<img src="" alt="title"/>

<h2> PID Controller </h2>

<h3> Proportional Controller </h3>

<p> The proportional component is useful for finding out the suddent change in error and act accordingly. The error function we use here is the Cross track error which is the difference between the vehcile location from the center of the lane. The proportional controller just adjusts the steering value with the constant Kp multiplied with the cross track error. This enables the vehcile to track error fast and act faster.</p>

<p> The main drawback of this P controller is that the vehicle overshoots the desired position and then comes back to it by adjusting the steering angle in opposite direction based on the error. But this overshoot drives our vehicle to oscillations as the car crosses the center lane each time and while coming back to center it again overshoots and causes oscillation.</p>

<h3> Proportional Derivative Controller </h3>

<p> To solve this oscillations around the steady state, we add an derivative component. The derivate component takes the difference between the previous cross track error and present cross track error divided by time difference and multiplies by constant Kd. This causes the error to steady state without any overshoot thus eliminating the oscillations around the steady state. Based on the difference between the previous and current CTE value it reduces the steering angle given by the P controller. When current error is less than previous error it counter steers to avoid the oscillations around the steady state.</p>

<p> The main drawback with the PD controller is that it is highly vulnerable to noise. Previously we assumed that the wheels are perfeclty aligned with the 0 degress, but whenever there is a systematic bias such that the wheel may not be 100% aligned with 0 degress but may have some offset as below.</p>

<img src="" alt="bias"/>

<p> As PD controller is vulnerable to noise it cannot solve this problem and the error does not converge to 0 and stays at some higher value.</p>

<h3> Proportional Integral Controller </h3>

<p> An Integral controller can be used to solve the problem of systematic bias because integral controller always brings the stady state to 0 irrespective of any noise. The integral controller computes the sum of previous CTE error and multiplies by a constant Ki. Whenerver it find that the error remains higher (when PD didn't converge) based on the Ki value it applies a steering action to eliminate the error and bring the car closer to the center of lane eventhough there exist a systematic bias.</p>

<p> The only drawback of having the Integral element is that,it always tries to bring the error to 0 and practically it is not possible and the error can only be brought down to a lower value. The Integral element still steers even when the error is lower and that causes overshoot which leads to oscillations. When the speed is higher, these oscillations are amplified and car is thrown out of track. Thus to solve this we always keep a fairly lower value for Ki.</p>

<p> The Integral element is also modified to take the previous 50 CTE error alone detect the current steering angle.This allows faster response when the PD didn't converge and we have sharp corners.</p>


<h2>PID Equation </h2>

<img src="" alt="pid"/>

<h2> System architecture </h2>

<p> We use separate controller to control the steering angle and a separate controller for controlling the throttle. The steering angle controller uses CTE as the error metric while the throttle controller uses Current Speed -  desired speed (30mph) as the error metric.</p>

<img src="" alt="control"/>

<h2> Tuned Values </h2>

<p> The Tuning of the Kp,Ki and Kd values is done manually based on the above definition. The values are given in the below table.</p>

<h3> Steering angle Controller </h3>

| Kp | Ki | Kd |
|  :---: |     :---:      |    :---:      |
| 0.15   | 0.00000005     | 1.1           |

<h3> Throttle Controller </h3>

| Kp | Ki | Kd |
|  :---: |     :---:      |    :---:      |
| 0.3   | 0.0003     | 0.003          |


<h2> Implementation Steps </h2>

```
Download the simulator from the repo
https://github.com/udacity/self-driving-car-sim/releases
Make a build directory: mkdir build && cd build
Compile: cmake .. && make
Run it: ./pid

```
<h2> Output Recorded Videos </h2>

```

```

<h2>Higher Speed Control action </h2>

<h3>Architecture of high speed control</h3>

<img src="" alt="highspeed"/>

<p> The above controller works well for lower speeds of 30mph.Sometimes in straight lanes we must increase the speed to reach the goal faster.In the same way we need to decrease the speed which was in the stright lane to make a turn. To facilitate that we have added additional controller for speed reduction. Here we just use PD controller. The key here is that we only reduce the speed when the absolute value CTE is higher and only when the speed is greater than 30mph. Since at 30mph the car don't go to oscillations in sharp turns.</p>

<p> The drawback of this method is that the car oscillates around the steady state due to the sudden reduction speed. If we just reduce the throttle value in steps of 0.1 when the bsolute value CTE is higher and  when the speed is greater than 30mph, this may fail at sharp turns since after travelling through the straight road before reaching the turn the car will be at high speed and gradual decrease may not be sufficient to make a turn. Thus the PD controller solves that but we may have oscillation around the steady state.</p>

<h3> Output of high speed control </h3>

```

```

