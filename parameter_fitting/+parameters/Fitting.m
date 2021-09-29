% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

classdef (Abstract) Fitting < handle
    properties (Constant, GetAccess = public)
        
        % parameters apply to both dimensions
        vxy_min = -20
        vxy_inc = 1
        vxy_max = 20
        
        delta_sin = 0.005;
        delta_cos = parameters.Fitting.delta_sin;
        
        %vy_vx_frac_max = 0.4
        
        num_regions = 32
        alpha_vec = linspace(0,2*pi,parameters.Fitting.num_regions+1)'
        fraction_parameters_col = parameters.Fitting.vxy_max * [cos(parameters.Fitting.alpha_vec(1:end-1)), sin(parameters.Fitting.alpha_vec(1:end-1))];
        
        fraction_parameters = [parameters.Fitting.fraction_parameters_col, [parameters.Fitting.fraction_parameters_col(2:end, :); parameters.Fitting.fraction_parameters_col(1,:)]];
        
        
        num_polynomial_coeff = 3;
    end
end