function [m] = magnetic_model(x)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    c0 = [0.1; -0.5; 0];
    c1 = [-0.2; -0.5; 0];
    %m = c0(1) * x^2 + c0(2) * x + c0(3);
    if  x > 0
        m = c0(1) * x^2 + c0(2) * x + c0(3);
    else
        m = c1(1) * x^2 + c1(2) * x + c1(3);
    end
end

