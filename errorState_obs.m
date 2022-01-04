function [yk] = errorState_obs(delta_x, r, theta, mag_bias)
    sensor_locs = [[r/2; r; 0] [-r/2; r; 0] [r/2; 0; 0] [-r/2; 0; 0] [r/2; -r; 0] [-r/2; -r; 0]];
    persistent H
    persistent errorStateMasks;
    persistent stateMasks;

    % global invAB;
    if isempty(H)
            
        H = [calcAB(sensor_locs(:, 1)); ...
         calcAB(sensor_locs(:, 2)); ...
         calcAB(sensor_locs(:, 3)); ...
         calcAB(sensor_locs(:, 4)); ...
         calcAB(sensor_locs(:, 5)); ...
         calcAB(sensor_locs(:, 6))];

        [~, errorStateMasks] = getErrorStateMask();
        [~, stateMasks]      = getStateMask();

    end
    
    theta = delta_x(end - 14: end) + theta;
    mag_bias = delta_x(errorStateMasks.mag_bias) + mag_bias;

    yk = H * theta + [mag_bias; zeros(3, 1)];
end