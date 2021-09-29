% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

close all
clear all
clc

warning('off', 'curvefit:prepareFittingData:removingNaNAndInf')

%% GENERATE RELEVANT PARAMTER RANGE OF THETA=ATAN2(VY,VX)
params_ = {};

num_polynomial_coeff = parameters.Fitting.num_polynomial_coeff;
wheelbase = parameters.Simulation.wheelbase;

vx_min = parameters.Fitting.vxy_min;
vx_inc = parameters.Fitting.vxy_inc;
vx_max = parameters.Fitting.vxy_max;
vx = vx_min:vx_inc:vx_max;

vy_min = parameters.Fitting.vxy_min;
vy_inc = parameters.Fitting.vxy_inc;
vy_max = parameters.Fitting.vxy_max;

delta_sin = parameters.Fitting.delta_sin;
delta_cos = parameters.Fitting.delta_cos;

vy = vy_min:vy_inc:vy_max;
[grid_x, grid_y] = meshgrid(vx,vy);

fraction_parameters = parameters.Fitting.fraction_parameters;

% rotating regions ... this is necessary, as we wan't to make sure that
% sins and coss contain common points
rotMatrix = @(alpha) [cos(alpha), -sin(alpha); sin(alpha), cos(alpha)];
alpha = diff(parameters.Fitting.alpha_vec(1:2))*0.1; % should be a small fraction of what each region covers
fraction_parameters_col_max = parameters.Fitting.fraction_parameters_col*rotMatrix(alpha);
fraction_parameters_max = [fraction_parameters_col_max, [fraction_parameters_col_max(2:end, :); fraction_parameters_col_max(1,:)]];
fraction_parameters_col_min = parameters.Fitting.fraction_parameters_col*rotMatrix(-alpha);
fraction_parameters_min = [fraction_parameters_col_min, [fraction_parameters_col_min(2:end, :); fraction_parameters_col_min(1,:)]];

for idx_r = 1:size(fraction_parameters,1)
    atans{idx_r} = nan*ones(size(grid_x)); % aka theta
end

region_nbr = nan*ones(size(grid_x)); % aka theta
for idx_x=1:length(vx)
    for idx_y=1:length(vy)
        idx_region = approximation.calculate_region(fraction_parameters, vy(idx_y), vx(idx_x));
        idx_region = idx_region(1);
        region_nbr(idx_y, idx_x) = idx_region;
        
        atans{idx_region}(idx_y, idx_x) = atan2(vy(idx_y),vx(idx_x));
        
        idx_region_min = approximation.calculate_region(fraction_parameters_min, vy(idx_y), vx(idx_x));
        if idx_region_min ~= idx_region
            atans{idx_region_min}(idx_y, idx_x) = atan2(vy(idx_y),vx(idx_x));
        end
        %
        idx_region_max = approximation.calculate_region(fraction_parameters_max, vy(idx_y), vx(idx_x));
        if idx_region_max ~= idx_region
            atans{idx_region_max}(idx_y, idx_x) = atan2(vy(idx_y),vx(idx_x));
        end
        
    end
end

assert(any(any(isnan(region_nbr)))==0, "regions are not well defined")

figure; hold on
legendstr = {};
surf(grid_x, grid_y, region_nbr)
xlabel('v_x')
ylabel('v_y')
%%

for idx_p = 1:size(fraction_parameters,1)
    sins{idx_p} = sin(atans{idx_p});
    coss{idx_p} = cos(atans{idx_p});
end

figure; hold on
for i=1:length(atans)
    surf(grid_x, grid_y, atans{i})
end
title('theta=atan(vy/vx)')
zlabel('theta=atan(vy/vx)')
xlabel('v_x')
ylabel('v_y')
view(3)

figure; hold on
for i=1:length(sins)
    surf(grid_x, grid_y, sins{i})
end
title('sin(theta)')
zlabel('sin(theta)')
xlabel('v_x')
ylabel('v_y')
view(3)

figure; hold on
for i=1:length(coss)
    surf(grid_x, grid_y, coss{i})
end
title('cos(theta)')
zlabel('cos(theta)')
xlabel('v_x')
ylabel('v_y')
view(3)

%% BYCICLE regionL FOR REAL VALUES
for i=1:length(sins)
    x_front_real{i} = 0 + wheelbase.*coss{i};
    y_front_real{i} = 0 + wheelbase.*sins{i};
end

%% Solve constrained linear least-squares problem for sin(theta)
% use lsq for fitting: x = lsqlin(C,d,A,b) solves the linear system C*x = d in the least-squares sense, subject to A*x <= b.

[A__, b__] = approximation.prepare_matrices_fitting(sins, vx, vy, fraction_parameters);
poly_sin_ub__ = lsqlin(A__, b__, -A__, -b__, [], [], [], []); % minimizes C*x = zData, subject to C*x >= zData
poly_sin_lb__ = lsqlin(A__, b__, A__, b__, [], [], [], []); % minimizes C*x = zData, subject to C*x <= zData

for i=1:size(fraction_parameters,1)
    poly_sin_ub{i} = poly_sin_ub__(num_polynomial_coeff*(i-1)+1:num_polynomial_coeff*i);
    poly_sin_lb{i} = poly_sin_lb__(num_polynomial_coeff*(i-1)+1:num_polynomial_coeff*i);
    
    [y_front_interp_UB{i}, sintheta_UB{i}] = calc_y_front_interp(wheelbase,vx,vy,poly_sin_ub{i});
    [y_front_interp_LB{i}, sintheta_LB{i}] = calc_y_front_interp(wheelbase,vx,vy,poly_sin_lb{i});
end

%% Solve constrained linear least-squares problem for cos(theta)
% use lsq for fitting: x = lsqlin(C,d,A,b) solves the linear system C*x = d in the least-squares sense, subject to A*x ? b.

[A__, b__] = approximation.prepare_matrices_fitting(coss, vx, vy, fraction_parameters);
poly_cos_ub__ = lsqlin(A__, b__, -A__, -b__, [], [], [], []); % minimizes C*x = zData, subject to C*x >= zData
poly_cos_lb__ = lsqlin(A__, b__, A__, b__, [], [], [], []); % minimizes C*x = zData, subject to C*x <= zData

for i=1:size(fraction_parameters,1)
    poly_cos_ub{i} = poly_cos_ub__(num_polynomial_coeff*(i-1)+1:num_polynomial_coeff*i);
    poly_cos_lb{i} = poly_cos_lb__(num_polynomial_coeff*(i-1)+1:num_polynomial_coeff*i);
    [x_front_interp_UB{i}, costheta_UB{i}] = calc_x_front_interp(wheelbase, vx, vy, poly_cos_ub{i});
    [x_front_interp_LB{i}, costheta_LB{i}] = calc_x_front_interp(wheelbase, vx, vy, poly_cos_lb{i});
end

%% PLOTTING OF INTERPOLATION ERRORS

% Y-Axis UPPER BOUND
helper_plot_surf_pos_neg(grid_x, grid_y, y_front_real, y_front_interp_UB, 'position error (real-interp) y UPPER BOUND')

% Y-Axis LOWER BOUND
helper_plot_surf_pos_neg(grid_x, grid_y, y_front_real, y_front_interp_LB, 'position error (real-interp) y LOWER BOUND')

% X-Axis UPPER BOUND
helper_plot_surf_pos_neg(grid_x, grid_y, x_front_real, x_front_interp_UB, 'position error (real-interp) x UPPER BOUND')

% X-Axis LOWER BOUND
helper_plot_surf_pos_neg(grid_x, grid_y, x_front_real, x_front_interp_LB, 'position error (real-interp) x LOWER BOUND')

%% SAVING

lin_result.x_front_interp_UB = x_front_interp_UB;
lin_result.x_front_interp_LB = x_front_interp_LB;
lin_result.y_front_interp_UB = y_front_interp_UB;
lin_result.y_front_interp_LB = y_front_interp_LB;
lin_result.y_front_real = y_front_real;
lin_result.x_front_real = x_front_real;

lin_result.poly_sin_ub = poly_sin_ub;
lin_result.poly_sin_lb = poly_sin_lb;
lin_result.poly_cos_ub = poly_cos_ub;
lin_result.poly_cos_lb = poly_cos_lb;

lin_result.costheta_UB = costheta_UB;
lin_result.costheta_LB = costheta_LB;
lin_result.sintheta_UB = sintheta_UB;
lin_result.sintheta_LB = sintheta_LB;

name_str = ['../data/', datestr(now,'yyyymmdd'), '_polynoms_from_fitting_', num2str(parameters.Fitting.num_regions), '.mat'];
save(name_str,'lin_result', 'vx_min', 'vx_max', 'vx_inc', 'fraction_parameters', 'region_nbr', 'grid_x', 'grid_y', 'atans')

%%
cub = reshape(cell2mat(lin_result.poly_cos_ub)',3*parameters.Fitting.num_regions,1);
fprintf('%1.20f, ', cub);
fprintf('\n');
clb = reshape(cell2mat(lin_result.poly_cos_lb)',3*parameters.Fitting.num_regions,1);
fprintf('%1.20f, ', clb);
fprintf('\n');
sub = reshape(cell2mat(lin_result.poly_sin_ub)',3*parameters.Fitting.num_regions,1);
fprintf('%1.20f, ', sub);
fprintf('\n');
slb = reshape(cell2mat(lin_result.poly_sin_lb)',3*parameters.Fitting.num_regions,1);
fprintf('%1.20f, ', slb);
fprintf('\n');

%% DEBUGGING INFORMATION

x_front_interp_UB = lin_result.x_front_interp_UB;
x_front_interp_LB = lin_result.x_front_interp_LB;
y_front_interp_UB = lin_result.y_front_interp_UB;
y_front_interp_LB = lin_result.y_front_interp_LB;
y_front_real = lin_result.y_front_real;
x_front_real = lin_result.x_front_real;

disp('Linear cos:')
for i = 1:length(x_front_interp_UB)
    disp(['x_UB: ', num2str(max(max(abs(x_front_real{i}-x_front_interp_UB{i}))))])
    disp(['x_LB: ', num2str(max(max(abs(x_front_real{i}-x_front_interp_LB{i}))))])
    
    disp(['y_UB: ', num2str(max(max(abs(y_front_real{i}-y_front_interp_UB{i}))))])
    disp(['y_LB: ', num2str(max(max(abs(y_front_real{i}-y_front_interp_LB{i}))))])
end

vx_ = -vx_max:vx_inc:vx_max;
vy_ = vx_;
[grid_x_, grid_y_] = meshgrid(vx_, vy_);

figure; hold on
% y... sin
for idx_p = 1:size(fraction_parameters,1)
    lin_result.sintheta_UB{idx_p}(~(region_nbr==idx_p))=nan; % invaliate function evaluation at other regions
    lin_result.sintheta_LB{idx_p}(~(region_nbr==idx_p))=nan; % invaliate function evaluation at other regions
    
    surf(grid_x, grid_y, sin(atans{idx_p}))
    
    surf(grid_x, grid_y, lin_result.sintheta_UB{idx_p}, 'FaceColor', [1,0,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
    surf(grid_x, grid_y, lin_result.sintheta_LB{idx_p}, 'FaceColor', [0,1,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
    
end
title('sintheta')
zlabel('sintheta')
xlabel('v_x')
ylabel('v_y')
view(3)
colorbar

%%
figure; hold on
% y... sin
for idx_p = 1
    lin_result.sintheta_UB{idx_p}(~(region_nbr==idx_p))=nan; % invaliate function evaluation at other regions
    lin_result.sintheta_LB{idx_p}(~(region_nbr==idx_p))=nan; % invaliate function evaluation at other regions
    
    surf(grid_x, grid_y, sin(atans{idx_p}))
    
    surf(grid_x, grid_y, lin_result.sintheta_UB{idx_p}, 'FaceColor', [1,0,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
    surf(grid_x, grid_y, lin_result.sintheta_LB{idx_p}, 'FaceColor', [0,1,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
    
end
legend({'sin(\theta)', 'upper bound', 'lower bound'})
xlabel('v_x[m/s]')
ylabel('v_y[m/s]')
view(3)

%%

figure; hold on
% x... cos
for idx_p = 1:size(fraction_parameters,1)
    lin_result.costheta_UB{idx_p}(~(region_nbr==idx_p))=nan;
    lin_result.costheta_LB{idx_p}(~(region_nbr==idx_p))=nan;
    
    surf(grid_x, grid_y, cos(atans{idx_p}))
    
    surf(grid_x, grid_y, lin_result.costheta_UB{idx_p}, 'FaceColor', [1,0,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
    surf(grid_x, grid_y, lin_result.costheta_LB{idx_p}, 'FaceColor', [0,1,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
    
end

title('costheta_{UB}')
zlabel('costheta_{UB}')
xlabel('v_x')
ylabel('v_y')
view(3)
colorbar


%% errors front axis x and y

figure;
ax1=subplot(2,2,1); hold on
for idx_p = 1:size(fraction_parameters,1)
    lin_result.costheta_UB{idx_p}(~(region_nbr==idx_p))=nan;
    surf(grid_x, grid_y, abs(lin_result.x_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.costheta_UB{idx_p}))
    surf(grid_x, -grid_y, abs(lin_result.x_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.costheta_UB{idx_p}))
    surf(-grid_x, grid_y, abs(lin_result.x_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.costheta_UB{idx_p}))
    surf(-grid_x, -grid_y, abs(lin_result.x_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.costheta_UB{idx_p}))
end
title('error x_{UB} [m]')
xlabel('v_x')
ylabel('v_y')
colorbar

ax2=subplot(2,2,2); hold on
for idx_p = 1:size(fraction_parameters,1)
    lin_result.sintheta_UB{idx_p}(~(region_nbr==idx_p))=nan;
    surf(grid_x, grid_y, abs(lin_result.y_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.sintheta_UB{idx_p}))
    surf(grid_x, -grid_y, abs(lin_result.y_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.sintheta_UB{idx_p}))
    surf(-grid_x, grid_y, abs(lin_result.y_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.sintheta_UB{idx_p}))
    surf(-grid_x, -grid_y, abs(lin_result.y_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.sintheta_UB{idx_p}))
end
title('error y_{UB} [m]')
xlabel('v_x')
ylabel('v_y')
colorbar

ax3=subplot(2,2,3); hold on
for idx_p = 1:size(fraction_parameters,1)
    lin_result.costheta_LB{idx_p}(~(region_nbr==idx_p))=nan;
    surf(grid_x, grid_y, abs(lin_result.x_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.costheta_LB{idx_p}))
    surf(grid_x, -grid_y, abs(lin_result.x_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.costheta_LB{idx_p}))
    surf(-grid_x, grid_y, abs(lin_result.x_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.costheta_LB{idx_p}))
    surf(-grid_x, -grid_y, abs(lin_result.x_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.costheta_LB{idx_p}))
end
title('error x_{LB} [m]')
xlabel('v_x')
ylabel('v_y')
colorbar

ax4=subplot(2,2,4); hold on
for idx_p = 1:size(fraction_parameters,1)
    lin_result.sintheta_LB{idx_p}(~(region_nbr==idx_p))=nan;
    surf(grid_x, grid_y, abs(lin_result.y_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.sintheta_LB{idx_p}))
    surf(grid_x, -grid_y, abs(lin_result.y_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.sintheta_LB{idx_p}))
    surf(-grid_x, grid_y, abs(lin_result.y_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.sintheta_LB{idx_p}))
    surf(-grid_x, -grid_y, abs(lin_result.y_front_real{idx_p} - parameters.Simulation.wheelbase*lin_result.sintheta_LB{idx_p}))
end
title('error y_{LB} [m]')
xlabel('v_x')
ylabel('v_y')
colorbar
linkaxes([ax1,ax2,ax3, ax4],'xy')

%% HELPER FUNCTIONS

function [y_front_interp, sin_interp_qudratic] = calc_y_front_interp(l, vx, vy, poly)
y_front_interp = zeros(length(vy), length(vx));
sin_interp_qudratic = zeros(length(vy), length(vx));
for x=1:length(vx)
    for y=1:length(vy)
        sin_interp_qudratic(y,x) = common.eval_linear_polynom(poly, vx(x), vy(y));
        y_front_interp(y,x) = 0 + l * (sin_interp_qudratic(y,x));
    end
end
end

function [x_front_interp, cos_interp_qudratic] = calc_x_front_interp(l, vx, vy, poly)
x_front_interp = zeros(length(vy), length(vx));
cos_interp_qudratic = zeros(length(vy), length(vx));
for x=1:length(vx)
    for y=1:length(vy)
        cos_interp_qudratic(y,x) = common.eval_linear_polynom(poly, vx(x), vy(y));
        x_front_interp(y,x) = 0 + l * (cos_interp_qudratic(y,x));
    end
end
end

function helper_plot_surf_pos_neg(grid_x, grid_y, y_front_real, y_front_interp, title_string)

figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,1,1)
hold on
for i = 1:length(y_front_interp)
    error_neg = y_front_real{i} - y_front_interp{i};
    error_neg(error_neg>0)=nan;
    
    surf(grid_x, grid_y, error_neg);
end
title([title_string, '; negative error'])
xlabel('v_x')
ylabel('v_y')
view(2)
colorbar;

subplot(2,1,2)
hold on
for i = 1:length(y_front_interp)
    error_pos = y_front_real{i} - y_front_interp{i};
    error_pos(error_pos<0)=nan;
    
    surf(grid_x, grid_y, error_pos);
end
title([title_string, '; positive error'])
xlabel('v_x')
ylabel('v_y')
view(2)
colorbar;
end