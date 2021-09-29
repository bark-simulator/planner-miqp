% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

close all
clear all
clc

%% Parameters, change here to your specific needs

fraction_parameters = parameters.Fitting.fraction_parameters;

vx_min = parameters.Fitting.vxy_min;
vx_inc = parameters.Fitting.vxy_inc;
vx_max = parameters.Fitting.vxy_max;

%% Calculate Orientations

for i=1:size(fraction_parameters,1)
    theta_lb = atan2(fraction_parameters(i,2), fraction_parameters(i,1));
    theta_ub = atan2(fraction_parameters(i,4), fraction_parameters(i,3));
    disp(['theta = [', num2str(rad2deg(theta_lb)), ', ', num2str(rad2deg(theta_ub)), ']'])
    
    if sin(theta_ub) > sin(theta_lb)
        poly_sin_ub{i} = [sin(theta_ub), 0, 0]'; % we null out the dependencies for vx and vy!
        poly_sin_lb{i} = [sin(theta_lb), 0, 0]';
    else
        poly_sin_ub{i} = [sin(theta_lb), 0, 0]'; % we null out the dependencies for vx and vy!
        poly_sin_lb{i} = [sin(theta_ub), 0, 0]';
    end
    
    if cos(theta_ub) > cos(theta_lb)
        poly_cos_ub{i} = [cos(theta_ub), 0, 0]';
        poly_cos_lb{i} = [cos(theta_lb), 0, 0]';
    else
        poly_cos_ub{i} = [cos(theta_lb), 0, 0]';
        poly_cos_lb{i} = [cos(theta_ub), 0, 0]';
    end
end

%% SAVING

lin_result.poly_sin_ub = poly_sin_ub;
lin_result.poly_sin_lb = poly_sin_lb;
lin_result.poly_cos_ub = poly_cos_ub;
lin_result.poly_cos_lb = poly_cos_lb;

name_str = ['../data/', datestr(now,'yyyymmdd'), '_polynoms_from_fitting_', num2str(parameters.Fitting.num_regions), '_using_theta.mat'];
save(name_str,'lin_result', 'vx_min', 'vx_max', 'vx_inc', 'fraction_parameters')