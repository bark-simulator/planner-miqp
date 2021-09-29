% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

function idx_region_vec = calculate_region(fraction_parameters, vy_i, vx_i)
idx_region_vec = [];
for idx = 1:size(fraction_parameters,1)
    upper_bound = fraction_parameters(idx,3) * vy_i <= fraction_parameters(idx,4) * vx_i + 1e-3;
    lower_bound = fraction_parameters(idx,1) * vy_i >= fraction_parameters(idx,2) * vx_i - 1e-3;
    if upper_bound && lower_bound
        idx_region_vec(end+1) = idx;
    end
end

if isempty(idx_region_vec)
    error("invalid region")
end