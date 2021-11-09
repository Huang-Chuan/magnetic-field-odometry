function [xk] = StateTransitionFcn(xk_1, u, dT)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    dp = xk_1(2) * dT;
    F = [1 dT  0  0  0; ...
         0  1  0  0  0; ...
         0  0  1  0  0; ...
         0  0  2*dp  1  0; ...
         0  0  dp^2 dp  1];
    G = [1/2*dT^2; dT; 0; 0; 0];
    
    xk = F * xk_1 + G * u;
end

