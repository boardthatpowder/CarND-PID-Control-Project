# CarND-Controls-PID Project Writeup

The following is a writeup for the [PID COntroller](https://github.com/udacity/CarND-PID-Control-Project) project of the [Udacity Self-Driving Car Engineer Nanodegree Program](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) based on the provided [Rubik](https://review.udacity.com/#!/rubrics/1972/view).

Refer to the projects original [README.md](https://github.com/udacity/CarND-PID-Control-Project/blob/master/README.md) for details of how to set up this project, as well as how to install and run the required simulator.

# Rubik Points

## Compilation

### Criteria: Your code should compile

To compile the project:

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## Implementation

### Criteria: The PID procedure follows what was taught in the lessons

[src/PID.cpp](src/PID.cpp) contains the PID implementation as taught in the lesson.

I used a PID controller for both controlling the steering and throttle.

The PID controller was used for steering as follows:

```c++
main.cpp
106          pid_steering.UpdateError(cte);
107          steer_value = pid_steering.TotalError();
108          if (steer_value<-1.0) {
109            steer_value = -1.0;
110          } else if (steer_value>1.0) {
111            steer_value = 1.0;
112          }
113
114          double brake = 0;
115          double throttle;
```

The PID controller was used for throttle control as follows:

```c++
main.cpp
118          pid_throttle.UpdateError(steer_value);
119          throttle = 1.0 - pid_throttle.TotalError();
120          if (throttle<0) {
121            brake = fabs(throttle);
122            throttle = 0;
123          } else if (throttle < 0.3) {
124            throttle=0.3;
125          } else if (throttle>1.0) {
126            throttle = 1.0;
127          }
```

## Reflection

### Criteria: Describe the effect each of the P, I, D components had in your implementation

- `P (Proportional)` : the steering / throttle values are set proportional to the error.  Increasing `P` has the effect of oscillating wildly, overshooting its target and often resulting in the car driving off the track, as the following chart of my experimentations with `P` for steering shows:

![charts/ps/results-ps.jpg](charts/ps/results-ps.jpg)

- `I (Inetegral)` : increases the action in relation to not only the error, but also the time for which it has persisted.  This has the effect of increasing the steering / throttle being applied.

- `D (Derivative)` : considers the rate of change of the error.  A higher `D` results in smoothing out oscillations as the following chart of my experimentations with `D` shows:

![charts/ds/results-ds.jpg](charts/ds/results-ds.jpg)


### Criteria: Describe how the final hyperparameters were chosen

I used manual tuning to determine my final parameters.  I opted not to choose other techniques such as twiddle or SGD as the simulator has no headless mode, therefore cannot be scripted in order to gather the required data for those methods.

I started by experimenting with `P` for the steering control.  The following chart shows the effect of starting at `1` then halving until `0.03125` where the car was able to stay on the track the longest:

![charts/ps/results-ps.jpg](charts/ps/results-ps.jpg)

Even though `0.03125` stayed on the track the longest, it struggled with applying sharp enough corrections to make the turns.  Therefore I started with `0.0625` as `P`, and started increasing `D` by `1`.  When the steering smoothed out I then increased `P` along with `D` until I had sharp enough steering to safely turn around the sharpest corners, but still smooth enough to prevent oscillations.  The following chart shows my results:

![charts/ps/results-ds.jpg](charts/ds/results-ds.jpg)

Once I was happy with a `PID` for steering, I experimented with a `PID` for throtttle control.

The following chart shows my results with applying a `P` for throttle.  Lower `P` for throttle resulted in the car overshooting the track, whereas higher `P` throttle values allowed the car to travel the furthest distant, but suffered with speed due to the larger oscillations in throttle control:

![charts/pt/results-pt-0.0625.jpg](charts/pt/results-pt-0.0625.jpg)

![charts/pt/results-pt-0.125.jpg](charts/pt/results-pt-0.125.jpg)

![charts/pt/results-pt-0.25.jpg](charts/pt/results-pt-0.25.jpg)

![charts/pt/results-pt-0.5.jpg](charts/pt/results-pt-0.5.jpg)

![charts/pt/results-pt-1.jpg](charts/pt/results-pt-1.jpg)

![charts/pt/results-pt-2.jpg](charts/pt/results-pt-2.jpg)

![charts/pt/results-pt-3.jpg](charts/pt/results-pt-3.jpg)

![charts/pt/results-pt-4.jpg](charts/pt/results-pt-4.jpg)

![charts/pt/results-pt-5.jpg](charts/pt/results-pt-5.jpg)

![charts/pt/results-pt-cte.jpg](charts/pt/results-pt-cte.jpg)

![charts/pt/results-pt-speed.jpg](charts/pt/results-pt-speed.jpg)

Now that I could see the effect of `P` on throttle controler, I started to apply `D`:

![charts/dt/results-dt-3_0_1.jpg](charts/dt/results-dt-3_0_1.jpg)

![charts/dt/results-dt-3_0_2.jpg](charts/dt/results-dt-3_0_2.jpg)

![charts/dt/results-dt-3_0_3.jpg](charts/dt/results-dt-3_0_3.jpg)

![charts/dt/results-dt-3_speed.jpg](charts/dt/results-dt-3_speed.jpg)

After applying throttle, my steering could no longer take the tougher corners at speed, therefore I experimented with both the steering and throttle controls:

![charts/tweak_all/results-cte.png](charts/tweak_all/results-cte.png)

The final parameters I chose for the `PID` coeffecients were:

- Steering:
  - `P`: 0.1
  - `I`: 0
  - `D`: 4
- Throttle:
  - `P`: 3
  - `I`: 0
  - `D`: 2


## Simulation
 
### Criteria: The vehicle must successfully drive a lap around the track

The following charts represent the CTE, angle, steer, speed and throttle of an entire journey around the track for my chosen coefficients:

![charts/tweak_all/results-final-cte.png](charts/tweak_all/results-final-cte.png)

![charts/tweak_all/results-final-angle.png](charts/tweak_all/results-final-angle.png)

![charts/tweak_all/results-final-steer.png](charts/tweak_all/results-final-steer.png)

![charts/tweak_all/results-final-speed.png](charts/tweak_all/results-final-speed.png)

![charts/tweak_all/results-final-throttle.png](charts/tweak_all/results-final-throttle.png)
