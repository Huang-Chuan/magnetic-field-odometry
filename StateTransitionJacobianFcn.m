function [Jx, Jw] = StateTransitionJacobianFcn(xk, vk, u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    dT = 0.01;
    mk = vk(1);
    % position process noise
    wkp = vk(2);
    
    dp = xk(2) * dT + 1/2 * (u - mk) * dT^2 + wkp ;

    c2  = xk(3);
    c1  = xk(4);

    
    Jx = [1 dT 0 0 0;...
          0  1 0 0 0;...
          0  0 1 0 0;...
          0  2*c2*dT 2*dp 1 0;...
          0  2*c2*dT*dp + c1*dT dp^2 dp 1];
    
    Jw = [-1/2 * dT^2; -dT; 0; -dT^2 * c2; -dp * dT^2 * c2 - 1/2 * dT^2 * c1];
    Jw = [Jw eye(5)];
end

