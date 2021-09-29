The following code can be used to fit the parameters for our optimization model. They have been introduced in [Optimal Behavior Planning for Autonomous Driving: A Generic Mixed-Integer Formulation](https://ieeexplore.ieee.org/document/9304743).

To approximate the front axle collision shape and the curvature constraint using quadratic polynomials (based on the velocities vx and vy), run the following scripts:
- Fitting the Front axle Position (Section III.B2 and Section IV.A): run_approximation_front_axis_veldep.m
- Fitting the Curvature (Section III.C Section IV.B): run_approximation_curvature.m

To approximate the front axle collision shape using a constant approximation, run the following script:
- Fitting the front axle position (Section III.B1): run_approximation_front_axis_const.m

The script visualize_approximated_car_polygon.m can be used to visualize the vehicle's collision shape at a given operation point.

Besides Matlab (we used version 2020a), the following toolboxes are required to run the parameter fitting:

- Curve Fitting Toolbox: [prepareSurfaceData](https://de.mathworks.com/help/curvefit/preparesurfacedata.html)
- Optimization Toolbox: [lsqlin](https://de.mathworks.com/help/optim/ug/lsqlin.html)