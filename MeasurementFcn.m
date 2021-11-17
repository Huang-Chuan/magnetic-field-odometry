function [yk] = MeasurementFcn(x, sensor_disp)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    r = sensor_disp;
    yk = 1e-6 * [r^2 -r 1; 0 0 1; r^2 r 1] * [x(7);x(8);x(9)] + [x(4); x(5); x(6)];
end

