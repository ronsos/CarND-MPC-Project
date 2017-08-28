## Model Predictive Control Project

### The Model 
A MPC controller is used for commanding the steering and throttle of the vehicle. The simulator provides a set of waypoints that represent a desired path. This is shown in yellow in the video. A curvefit is applied to these waypoints. The curvefit is then provided to the optimizer, IPOPT. 

The output state from the simulator is given in map coordinates. I converted the position state from map (px, py) to car coordinates in main.cpp, according to the following equations:

`x = ptsx[i] - px`

`y = ptsy[i] - py`

`ptsx_car[i] = x * cos(-psi) - y * sin(-psi)`

`ptsy_car[i] = x * sin(-psi) + y * cos(-psi)`
 
The polynomial curvefit is a 3rd order fit applied to the position state in car coordinates. This fit is then used to calculate the crosstrack error (cte) and the error in vehicle orientation.
  
`coeffs = polyfit(ptsx_car, ptsy_car, 3)`

`cte = polyeval(coeffs, 0) - 0` 

`epsi = -atan(coeffs[1])` 
 
The state gets passed into the MPC solver, where it is used to set the initial condition. The `coeffs` curvefit additionally is passed into the evaluation of the cost function. This curvefit is used to provide the constraint target for crosstrack error and vehicle orientation error. 

`f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0`

`psides0 = CppAD::atan(3*coeffs[3]*x0*x0 + 2*coeffs[2]*x0 + coeffs[1])`

These are used in the main set of dynamic equations. The dynamics equations are treated as constraints that must be driven to zero. The dynamics/constraints model is as follows:

`fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt)`

`fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt)`

`fg[1 + psi_start + t] = psi1 - (psi0 + v0/Lf * delta0 * dt)`

`fg[1 + v_start + t] = v1 - (v0 + a0 * dt)`

`fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt)`

`fg[1 + epsi_start + t] = epsi1 - ((psi0-psides0) - v0/Lf * delta0 * dt)`

The solution that is returned from the solver consists of two main parts: commands, and the predicted path. The commands are applied in terms of throttle value and steering command. The projected state from the MPC solver solution is put into the `mpc_x_vals` and `mpc_y_vals` arrays and plotted in the simulator in green. 
 

### Timestep Length and Elapsed Duration (N & dt)
The timestep, dt, represents the length of time between each point used in the discretization of the state propagation in the optimzer. 

The number of points, N, is the total amount of points used in the optimizer. 

Thus, the product of these parameters, `N*dt`, represents the total length of the propagation in time, or time horizon. The larger this number, the greater the length of time forward the optimizer is considering. 

The total distance down the track the optimizer will consider is `N*dt*velocity`. The key in tuning the N and dt parameters is to consider far enough forward that the controller has time to response to sharp turns but not so far in the future that the curvefit will have to content with very complex geometry and potentially ugly overfitted solutions. Also, the computational limits need to be considered. For a given `N*dt`, a combination of increasing N / lowering dt will tend to require more computational effort. 

The final values that I chose were N=8 and dt=0.1. These values worked for a velocity of 90 mph. For 60 mph, I used N=10 and dt=0.1. These values seemed to provide enough down track information that the car could achieve the turn rates necessary for sharp turns, but without excessive computations. 

The cost function required tuning for each velocity target as well. 


### Polynomial Fitting and MPC Preprocessing
The polynominal fit used is a 3rd order fit. This allows some curvature to the fit and also an inflection point. The `polyfit` function was used to develop the curvefit. 

`coeffs = polyfit(ptsx_car, ptsy_car, 3)`

The fit is applied to the vehicle state in transformed coordinates relative to the car. The vehicle orientation is used as the x-axis for the new states. The cross track error and orientation error are calculated in the car frame. 

`cte = polyeval(coeffs, 0) - 0` 

`epsi = -atan(coeffs[1])` 
          

### Model Predictive Control with Latency 
An assumed latency value of 100 ms was specified in the project requirements. This was dealt with in the following manner. First, the velocity units were converted from mph to m/s, (due to the position states, px and py, being in meters):

`v_ms = v * 5280 * .3048 / 3600; // convert velocity to m/s`

The velocity is then used to propagate the current state forward 0.1 s. 

`px = px + v_ms*cos(psi)*latency`

`py = py + v_ms*sin(psi)*latency`

`psi = psi + v_ms*(-delta)/Lf*latency`

`v = v + accel*latency`
 
The model was initially tuned with zero latency. Then the latency was changed to 100 ms, and the controller was retuned. The only parameters that needed to be adjusted  were the cost function weights. 

### Simulation Screenshot
The screen capture below shows the MPC controller output in green, compared to the desired path in the yellow. The controller successfully completes the course, keeping the car on the road at high speed (up to 90 mph on straight portions). 

![img1](img1.png "MPC Control Project")


### Simulation
To run the simulation:
1. Clone this repo.
2. Make a build directory: mkdir build && cd build
3. Compile: cmake .. && make
4. Run it: ./mpc.
