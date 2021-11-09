function [true_value, measure_value] = magnetometer(model, p, noise_var)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
true_value = model(p);
measure_value = true_value + noise_var * randn(size(true_value));
end

