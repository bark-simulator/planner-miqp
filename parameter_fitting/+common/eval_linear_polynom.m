% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

function [y] = eval_linear_polynom(poly, v_long, v_lat)
y = poly(1) + poly(2)*v_long + poly(3)*v_lat;
end
