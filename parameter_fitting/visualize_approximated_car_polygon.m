% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

clear all
close all
clc

car = @(l,w) [-w, -w; l+w, -w; l+w, w; -w, w];
car_circle = @(refPoint, r) [refPoint(1)+r*cos(0:0.1:2*pi+0.1); refPoint(2)+r*sin(0:0.1:2*pi+0.1)]';

filepath = fullfile('..', 'data', 'polynoms_from_fitting_32.mat');
load(filepath);

%% Parameters, change here to your specific needs
r_circle = 1.0;
vx = 5;
vy = 2.5;
l = parameters.Simulation.wheelbase;
refpt = [0,0];

%%
theta_exact = atan2(vy,vx);
idx_region = approximation.calculate_region(fraction_parameters, vy, vx);

carpoly = polyshape(car(l,0.85));
carpoly = rotate(carpoly, theta_exact/pi*180, refpt);

cos_interp_qudratic_ub = common.eval_linear_polynom(lin_result.poly_cos_ub{idx_region}, vx, vy);
x_front_ub_calc  = refpt(1) + parameters.Simulation.wheelbase * cos_interp_qudratic_ub;

cos_interp_qudratic_lb = common.eval_linear_polynom(lin_result.poly_cos_lb{idx_region}, vx, vy);
x_front_lb_calc  = refpt(1) + parameters.Simulation.wheelbase * cos_interp_qudratic_lb;

sin_interp_qudratic_ub = common.eval_linear_polynom(lin_result.poly_sin_ub{idx_region}, vx, vy);
y_front_ub_calc  = refpt(2) + parameters.Simulation.wheelbase * sin_interp_qudratic_ub;

sin_interp_qudratic_lb = common.eval_linear_polynom(lin_result.poly_sin_lb{idx_region}, vx, vy);
y_front_lb_calc  = refpt(2) + parameters.Simulation.wheelbase * sin_interp_qudratic_lb;

circle_rear = car_circle(refpt, r_circle);
circle_front = car_circle([refpt(1)+parameters.Simulation.wheelbase * cos(theta_exact), refpt(2)+parameters.Simulation.wheelbase * sin(theta_exact)], r_circle);
circle_front_ub_ub = car_circle([x_front_ub_calc, y_front_ub_calc], r_circle);
circle_front_ub_lb = car_circle([x_front_ub_calc, y_front_lb_calc], r_circle);
circle_front_lb_ub = car_circle([x_front_lb_calc, y_front_ub_calc], r_circle);
circle_front_lb_lb = car_circle([x_front_lb_calc, y_front_lb_calc], r_circle);

X_front = [circle_front_ub_ub(:,1), circle_front_ub_lb(:,1), circle_front_lb_ub(:,1), circle_front_lb_lb(:,1)];
Y_front = [circle_front_ub_ub(:,2), circle_front_ub_lb(:,2), circle_front_lb_ub(:,2), circle_front_lb_lb(:,2)];
convhull_front = convhull(X_front,Y_front);

X_front_all = [X_front, circle_rear(:,1)];
Y_front_all = [Y_front, circle_rear(:,2)];
convhull_all = convhull(X_front_all,Y_front_all);

figure(100);
cla;
hold on;
axis equal;
plot(circle_rear(:,1), circle_rear(:,2), 'b-','LineWidth', 2);
plot(circle_front(:,1), circle_front(:,2), 'b-','LineWidth', 2);
plot(refpt(1), refpt(2), 'kx')
plot(x_front_ub_calc, y_front_ub_calc, 'kx')
plot(x_front_ub_calc, y_front_lb_calc, 'kx')
plot(x_front_lb_calc, y_front_ub_calc, 'kx')
plot(x_front_lb_calc, y_front_lb_calc, 'kx')

plot(X_front(convhull_front),Y_front(convhull_front),'r-','LineWidth', 2);
plot(X_front_all(convhull_all),Y_front_all(convhull_all),'k:','LineWidth', 2);

plot(carpoly)