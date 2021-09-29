% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

clc
close all
clear all

warning('off','MATLAB:rankDeficientMatrix')
warning('off','curvefit:prepareFittingData:removingNaNAndInf')

%% Parameters, change here to your specific needs

vx_min = parameters.Fitting.vxy_min;
vx_inc = parameters.Fitting.vxy_inc; % you may change this to 1 or 0.1 depending on your resources
vx_max = parameters.Fitting.vxy_max;
vx = vx_min:vx_inc:vx_max;

vy_min = parameters.Fitting.vxy_min;
vy_inc = parameters.Fitting.vxy_inc; % you may change this to 1 or 0.1 depending on your resources
vy_max = parameters.Fitting.vxy_max;

vy = vy_min:vy_inc:vy_max;
[grid_vx, grid_vy] = meshgrid(vx,vy);

% bounds are not fitted for velocities below v_min_no_region_change
v_min_no_region_change = parameters.Simulation.minimum_region_change_speed;

ax = -4:0.5:4;

kappa_max = tan(parameters.Simulation.delta_max)/parameters.Simulation.wheelbase;
kappa_min = tan(parameters.Simulation.delta_min)/parameters.Simulation.wheelbase;

fraction_parameters = parameters.Fitting.fraction_parameters;
num_polynomial_coeff = parameters.Fitting.num_polynomial_coeff;

%% upper bound of kappa
% (vx.*ay - ax .* vy)./(vx.^2+vy.^2).^(3/2) <= kappa_max
% can be transformed to
% ay <= kappa_max/vx .* (vx.^2+vy.^2).^(3/2) + ax.*vy./vx
figure(12); clf;
figure(13); clf;
num_ai = length(ax);
for ii = 1:num_ai
    z = calc_z_bounded(kappa_max, grid_vx, grid_vy, ax(ii));
    z_prime = z - ax(ii)*grid_vy./grid_vx; % we are fitting on z_prime!
    
    figure(12);
    num_subplot = ceil(sqrt(length(ax)));
    subplot(num_subplot,num_subplot,ii)
    surf(grid_vx, grid_vy, z)
    title(['a_y <= ... a.k.a z for a_x=', num2str(ax(ii))])
    zlabel('a_y_{max}')
    xlabel('v_x')
    ylabel('v_y')
    view(3)
    
    figure(13);
    subplot(num_subplot,num_subplot,ii)
    surf(grid_vx, grid_vy, z_prime)
    title(['a_y -a_x *vy/vx <= ... a.k.a. z* for a_x=', num2str(ax(ii))])
    zlabel('a_y_{max}')
    xlabel('v_x')
    ylabel('v_y')
    view(3)
end

%% lower bound of kappa
% kappa_min <= (vx.*ay - ax .* vy)./(vx.^2+vy.^2).^(3/2)
% can be transformed to
% ay >= kappa_min/vx .* (vx.^2+vy.^2).^(3/2) + ax.*vy./vx

figure(21); clf;
figure(22); clf;
num_ai = length(ax);
for ii = 1:num_ai
    z = calc_z_bounded(kappa_min, grid_vx, grid_vy, ax(ii));
    z_prime = z - ax(ii)*grid_vy./grid_vx; % we are fitting on z_prime!
    
    figure(21);
    num_subplot = ceil(sqrt(length(ax)));
    subplot(num_subplot,num_subplot,ii)
    surf(grid_vx, grid_vy, z)
    title(['a_y >= ... a.k.a z for a_x=', num2str(ax(ii))])
    zlabel('a_y_{min}')
    xlabel('v_x')
    ylabel('v_y')
    view(3)
    
    figure(22);
    subplot(num_subplot,num_subplot,ii)
    surf(grid_vx, grid_vy, z_prime)
    title(['a_y -a_x *vy/vx >= ... a.k.a. z* for a_x=', num2str(ax(ii))])
    zlabel('a_y_{min}')
    xlabel('v_x')
    ylabel('v_y')
    view(3)
end

%% Fitting

quadrants = approximation.calculate_quadrant_per_region(fraction_parameters);

ax_ii = 0;
disp(['We are fitting for a_x = ', num2str(ax_ii)])

% initializing cell arrays
for idx_r = 1:size(fraction_parameters,1)
    z_prime_for_fitting_ub{idx_r} = nan*ones(size(grid_vx));
    z_prime_for_fitting_lb{idx_r} = nan*ones(size(grid_vx));
end

region_nbr = nan*ones(size(grid_vx));
fraction_matrix = nan*ones(size(grid_vx));
for idx_x=1:length(vx)
    for idx_y=1:length(vy)
        idx_region = approximation.calculate_region(fraction_parameters, vy(idx_y), vx(idx_x));
        idx_region = idx_region(1);
        region_nbr(idx_y, idx_x) = idx_region;
        fraction_matrix(idx_y, idx_x) = mean([fraction_parameters(idx_region,[2,4])])/mean(fraction_parameters(idx_region,[1,3]));
        
        if quadrants(idx_region) == 1 || quadrants(idx_region) == 4
            % upper bound of kappa
            z_ub_scalar = calc_z_bounded(kappa_max, vx(idx_x), vy(idx_y), ax_ii);
            % lower bound of kappa
            z_lb_scalar = calc_z_bounded(kappa_min, vx(idx_x), vy(idx_y), ax_ii);
        elseif quadrants(idx_region) == 2 || quadrants(idx_region) == 3
            % flipping fits cause v_x is <0 there
            % upper bound of kappa
            z_ub_scalar = calc_z_bounded(kappa_min, vx(idx_x), vy(idx_y), ax_ii);
            % lower bound of kappa
            z_lb_scalar = calc_z_bounded(kappa_max, vx(idx_x), vy(idx_y), ax_ii);
        else
            error('check quadrants!')
        end
        
        % removing data for small velocities
        if (abs(vx(idx_x))<=v_min_no_region_change && abs(vy(idx_y))<=v_min_no_region_change)
            z_ub_scalar = nan;
            z_lb_scalar = nan;
        end
        
        z_prime_for_fitting_ub{idx_region}(idx_y, idx_x) = z_ub_scalar - ax_ii*vy(idx_y)./vx(idx_x);
        z_prime_for_fitting_lb{idx_region}(idx_y, idx_x) = z_lb_scalar - ax_ii*vy(idx_y)./vx(idx_x);
    end
end

[A__ub, b__ub] = approximation.prepare_matrices_fitting(z_prime_for_fitting_ub, vx, vy, fraction_parameters);
[A__lb, b__lb] = approximation.prepare_matrices_fitting(z_prime_for_fitting_lb, vx, vy, fraction_parameters);
poly_z_prime_ub__ = lsqlin(A__ub, b__ub, [], [], [], [], [], []); % minimizes C*x = zData
poly_z_prime_lb__ = lsqlin(A__lb, b__lb, [], [], [], [], [], []); % minimizes C*x = zData

%% Post-Processing of fit

for idx_reg=1:size(fraction_parameters,1)
    poly_z_prime_ub{idx_reg} = poly_z_prime_ub__(num_polynomial_coeff*(idx_reg-1)+1:num_polynomial_coeff*idx_reg);
    poly_z_prime_lb{idx_reg} = poly_z_prime_lb__(num_polynomial_coeff*(idx_reg-1)+1:num_polynomial_coeff*idx_reg);
    
    z_prime_ub_interp{idx_reg} = calc_z_prime_interp(vx, vy, poly_z_prime_ub{idx_reg});
    z_prime_ub_interp{idx_reg}(~(region_nbr==idx_reg))=nan;
    
    z_prime_lb_interp{idx_reg} = calc_z_prime_interp(vx, vy, poly_z_prime_lb{idx_reg});
    z_prime_lb_interp{idx_reg}(~(region_nbr==idx_reg))=nan;
    
    for idx_x=1:length(vx)
        for idx_y=1:length(vy)
            % removing data for small velocities
            if (abs(vx(idx_x))<=v_min_no_region_change && abs(vy(idx_y))<=v_min_no_region_change)
                z_prime_ub_interp{idx_reg}(idx_y, idx_x) = nan;
                z_prime_lb_interp{idx_reg}(idx_y, idx_x) = nan;
            end
        end
    end
end

lin_curvature_result.poly_z_prime_ub = poly_z_prime_ub;
lin_curvature_result.poly_z_prime_lb = poly_z_prime_lb;

savestr = ['../data/', datestr(now,'yyyymmdd'), '_poly_curvature_', num2str(parameters.Fitting.num_regions), '_', num2str(parameters.Fitting.vxy_max),  '_', num2str(v_min_no_region_change), '.mat'];
save(savestr, 'lin_curvature_result', 'fraction_parameters')
disp('Saved the data')

%%
ubvec = reshape(cell2mat(lin_curvature_result.poly_z_prime_ub)',3*parameters.Fitting.num_regions,1);
disp('poly_z_prime_ub = ');
fprintf('%1.20f, ', ubvec);
fprintf('\n');

lbvec = reshape(cell2mat(lin_curvature_result.poly_z_prime_lb)',3*parameters.Fitting.num_regions,1);
disp('poly_z_prime_lb = ');
fprintf('%1.20f, ', lbvec);
fprintf('\n');


%% Plotting

for idx_ax = 1:length(ax)
    for idx_reg=1:size(fraction_parameters,1)
        % upper bound of kappa
        z_ub1_ = calc_bounded_z_from_z_prime(z_prime_for_fitting_ub{idx_reg}, ax(idx_ax), grid_vy./grid_vx);
        z_ub2_ = calc_z_bounded(kappa_max, grid_vx, grid_vy, ax(idx_ax));
        if ~(all(all(z_ub1_ == z_ub2_)))
            warning('by only using ax=0 in fit ub and then cutting of max and min, we loose some data points')
        end
        
        z_interp_ub{idx_reg, idx_ax} = calc_bounded_z_from_z_prime(z_prime_ub_interp{idx_reg}, ax(idx_ax), grid_vy./grid_vx);
        % calculate z with fraction term instead of vy/vx
        z_interp_frac_ub{idx_reg, idx_ax} = calc_bounded_z_from_z_prime(z_prime_ub_interp{idx_reg}, ax(idx_ax), fraction_matrix);
        
        z_ub{idx_reg, idx_ax} = z_ub1_;
        
        % lower bound of kappa
        z_lb1_ = calc_bounded_z_from_z_prime(z_prime_for_fitting_lb{idx_reg}, ax(idx_ax), grid_vy./grid_vx);
        z_lb2_ = calc_z_bounded(kappa_min, grid_vx, grid_vy, ax(idx_ax));
        if ~(all(all(z_lb1_ == z_lb2_)))
            warning('by only using ax=0 in fit lb and then cutting of max and min, we loose some data points')
        end
        
        z_interp_lb{idx_reg, idx_ax} = calc_bounded_z_from_z_prime(z_prime_lb_interp{idx_reg}, ax(idx_ax), grid_vy./grid_vx);
        % calculate z with fraction term instead of vy/vx
        z_interp_frac_lb{idx_reg, idx_ax} = calc_bounded_z_from_z_prime(z_prime_lb_interp{idx_reg}, ax(idx_ax), fraction_matrix);
        
        z_lb{idx_reg, idx_ax} = z_lb1_;
    end
end

figure(112); clf;
figure(113); clf;
figure(114); clf;
figure(432); clf;
% figure(641); clf;
% figure(642); clf;
figure(851); clf;
figure(852); clf;
figure(920); clf;

figure(112)
hold on
for idx_reg = 1:size(fraction_parameters,1)
    surf(grid_vx, grid_vy, z_prime_for_fitting_ub{idx_reg})
    surf(grid_vx, grid_vy, z_prime_ub_interp{idx_reg}, 'FaceColor', [1,0,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
end
title(['z prime ub correct vs interp not dependent on a_x'])
zlabel('z prime')
xlabel('v_x')
ylabel('v_y')
view(3)
colorbar

figure(113)
hold on
for idx_reg = 1:size(fraction_parameters,1)
    surf(grid_vx, grid_vy, z_prime_for_fitting_lb{idx_reg})
    surf(grid_vx, grid_vy, z_prime_lb_interp{idx_reg}, 'FaceColor', [1,0,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
end
title('z prime lb correct vs interp not dependent on a_x')
zlabel('z prime')
xlabel('v_x')
ylabel('v_y')
view(3)
colorbar

for idx_ax = 1:length(ax)
    figure(114)
    num_subplot = ceil(sqrt(length(ax)));
    subplot(num_subplot,num_subplot,idx_ax)
    hold on
    for idx_reg = 1:size(fraction_parameters,1)
        surf(grid_vx, grid_vy, z_ub{idx_reg, idx_ax})
        surf(grid_vx, grid_vy, z_interp_ub{idx_reg, idx_ax}, 'FaceColor', [1,0,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
        surf(grid_vx, grid_vy, z_interp_frac_ub{idx_reg, idx_ax}, 'FaceColor', [0,1,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
    end
    title(['a_y <= ... ub, ideal (vy/vx) (red) and with frac (green) at a_x=',num2str(ax(idx_ax))])
    zlabel('z')
    xlabel('v_x')
    ylabel('v_y')
    view(3)
    colorbar
    
    figure(432)
    subplot(num_subplot,num_subplot,idx_ax)
    hold on
    for idx_reg = 1:size(fraction_parameters,1)
        surf(grid_vx, grid_vy, z_lb{idx_reg, idx_ax})
        surf(grid_vx, grid_vy, z_interp_lb{idx_reg, idx_ax}, 'FaceColor', [1,0,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
        surf(grid_vx, grid_vy, z_interp_frac_lb{idx_reg, idx_ax}, 'FaceColor', [0,1,0], 'FaceAlpha', 0.5, 'EdgeColor', 'none')
    end
    title(['a_y >= ... lb, ideal (vy/vx) (red) and with frac (green) at a_x=',num2str(ax(idx_ax))])
    zlabel('z')
    xlabel('v_x')
    ylabel('v_y')
    view(3)
    colorbar 
    
    figure(851);
    subplot(num_subplot,num_subplot,idx_ax)
    hold on
    for idx_reg = 1:size(fraction_parameters,1)
        surf(grid_vx, grid_vy, z_lb{idx_reg, idx_ax}-z_interp_frac_lb{idx_reg, idx_ax})
    end
    title(['z-z_{interp}>0 ... constraining not enough for a_{y,lb} at a_x=',num2str(ax(idx_ax))])
    zlabel('error')
    xlabel('v_x')
    ylabel('v_y')
    colorbar
    
    figure(852)
    subplot(num_subplot,num_subplot,idx_ax)
    hold on
    for idx_reg = 1:size(fraction_parameters,1)
        surf(grid_vx, grid_vy, z_ub{idx_reg, idx_ax}-z_interp_frac_ub{idx_reg, idx_ax})
    end
    title(['z-z_{interp}<0 ... constraining not enough for a_{y,ub} at a_x=',num2str(ax(idx_ax))])
    zlabel('error')
    xlabel('v_x')
    ylabel('v_y')
    colorbar
    
    figure(920)
    subplot(num_subplot,num_subplot,idx_ax)
    hold on
    for idx_reg = 1:size(fraction_parameters,1)
        invalid_diff = z_interp_frac_ub{idx_reg, idx_ax}-z_interp_frac_lb{idx_reg, idx_ax};
        invalid_diff(invalid_diff>0) = nan; % removing valid data
        surf(grid_vx, grid_vy, invalid_diff)
    end
    title(['invalid if (a_{y,ub}-a_{y,lb})<0 at a_x=',num2str(ax(idx_ax))])
    xlabel('v_x')
    ylabel('v_y')
    colorbar
end

%% HELPER FUNCTIONS

function z = calc_bounded_z_from_z_prime(z_prime, ax_i, frac_vy_vx)
z = z_prime + ax_i*frac_vy_vx;
z(z >= parameters.Simulation.acc_max) = nan;
z(z <= parameters.Simulation.acc_min) = nan;
end

function z = calc_z_bounded(kappa, vx, vy, ax_i)
z = kappa./vx .* (vx.^2+vy.^2).^(3/2) + ax_i.*vy./vx;
z(z >= parameters.Simulation.acc_max) = nan;
z(z <= parameters.Simulation.acc_min) = nan;
end
function [z_prime_interp] = calc_z_prime_interp(vx, vy, poly)
z_prime_interp = zeros(length(vy), length(vx));
for x=1:length(vx)
    for y=1:length(vy)
        z_prime_interp(y,x) = common.eval_linear_polynom(poly, vx(x), vy(y));
    end
end
end