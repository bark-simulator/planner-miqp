% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

classdef Simulation < handle
    properties (Constant, GetAccess = public)
        
        N = 20;
        ts = 0.2;
        
        wheelbase = 2.8;
        radius = 1;
        
        min_vel_x_y = -20;
        max_vel_x_y = 20;
        
        % parameters for the bicycle model
        acc_max = 2;
        acc_min = -4;
        delta_max = 0.25;
        delta_min = -parameters.Simulation.delta_max;
        
        straight_acc_long_min = [parameters.Simulation.acc_min; 0];
        straight_acc_long_max = [parameters.Simulation.acc_max; 0];
        % setting to zero, as lateral acc is already != 0, as we always shift the vectors (our regions are defined that way)
        straight_acc_lat_min = [0; -1.6]; % MIQP paper: -1 --> 1-sin(2*pi/16/2)*2 ~= 0.6
        straight_acc_lat_max = [0; 1.6]; % MIQP paper: 1
        
        jerk_max = 3;
        straight_jerk_long_min = [-parameters.Simulation.jerk_max; 0]; % MIQP paper: -3
        straight_jerk_long_max = [parameters.Simulation.jerk_max; 0]; % MIQP paper: 3
        straight_jerk_lat_min = [0; -1.4]; % MIQP paper: -2 --> 2-sin(2*pi/16/2)*3 ~= 1.4
        straight_jerk_lat_max = [0; 1.4]; % MIQP paper: 2
        
        minimum_region_change_speed = 2; % can we set it to 1?
        
        debug_plots_enabled = false;
        max_tries = 3; % number of tries to counter communication issues to cplex
        
        orientation_approx_type = 1; % 1... velocity dependent; 2... constant approximation
        
        collision_check_epsilon = 0.02;
        
        % choose tighter bounds for SQP
        sqp_eps_kappa = 0.01;
        sqp_eps_jerk = 0.2;
        sqp_eps_acc = 0.2;
        
        % weights and slack (position weight is a parameter of the function call)
        weight_jerk = 0.5;
        weight_slack = 30;
        weight_slack_obstacle = 2000;
        safety_distance_slack = 3; % for all time steps constant, could also vary
        
        cplex_max_solution_time = 300;
        cplex_max_num_timing_fail = 10;
        cplex_relative_mip_gap_tolerance = 0.1;
        
        mipdisplay = 2;
        mipemphasis = 0;
        relobjdif = 0;
        cutpass = 0;
        probe = 0;
        repairtries = 0;
        rinsheur = 0;
        varsel = 0;
        mircuts = 0;
        parallelmode = 0;
        
        warmstart_cplex = true;
        
    end
end