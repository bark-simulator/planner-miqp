% Copyright (c) 2021 fortiss GmbH
%
% Authors: Klemens Esterle and Tobias Kessler
%
% This work is licensed under the terms of the MIT license.
% For a copy, see <https://opensource.org/licenses/MIT>.

function [A__, b__] = prepare_matrices_fitting(trigonometric, vx, vy, fraction_parameters)
C__ = [];
d__ = [];

num_beta = 2*size(fraction_parameters,1); % TODO: parameter auslagern, generischer machen!!

for i = 1:length(trigonometric)
    [xData, yData, zData] = prepareSurfaceData( vx, vy, trigonometric{i} );
    
    C = [ones(size(xData)), xData, yData]; % linear fit
    
    C__ = [[C__, zeros(size(C__,1), size(C,2))];
        [zeros(size(C,1), size(C__,2)), C]];
    d__ = [d__; zData];
    
    block_size(i,:) = size(C__);
end

% fill rest of A and b to zero, as the additional equality variables do not impact the cost function
A__ = [[C__, zeros(size(C__,1), num_beta*2)];
    [zeros(num_beta*2, size(C__,2)), zeros(num_beta*2, num_beta*2)]];
b__ = [d__; zeros(num_beta*2, 1)];

end