% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

function quadrants = calculate_quadrant_per_region(fraction_parameters)

for idxreg = 1:parameters.Fitting.num_regions
    angle_line1(idxreg) = wrapTo2Pi(atan2(fraction_parameters(idxreg,2),fraction_parameters(idxreg,1)));
    angle_line2(idxreg) = wrapTo2Pi(atan2(fraction_parameters(idxreg,4),fraction_parameters(idxreg,3)));
    if(idxreg==parameters.Fitting.num_regions)
        % dirty hack
        angle_line2(idxreg) = angle_line2(idxreg) + 2*pi;
    end
    mean_angle(idxreg) = (angle_line1(idxreg) + angle_line2(idxreg)) / 2;
    quadrants(idxreg) = floor(mean_angle(idxreg)/(pi/2)) + 1;
end

end

