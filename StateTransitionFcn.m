function [x] = StateTransitionFcn(xk, vk, u)
%UNTITLED2 Summary of this function goes here
%   vk = [m_k, w_k]
    % acceleromter reading noise
    mk = vk(1);
    % position process noise
    wkp = vk(2);
    
    dT = 0.01;
    dp = xk(2) * dT + 1/2 * (u - mk) * dT^2 + wkp ;

    c2  = xk(3);
    c1  = xk(4);
    
    F = [1 dT  0  0  0; ...
         0  1  0  0  0; ...
         0  0  1  0  0; ...
         0  0  2*dp  1  0; ...
         0  0  dp^2 dp  1];
    G = [1/2*dT^2; dT; 0; 0; 0];
    
    GG = [-1/2*dT^2; -dT ; 0 ; -1/2*dT^2; -c2 * dp * dT^2 - 1/2 * c1 * dT^2];
    
   
    x = F * xk + G * u + GG * vk(1) + vk(2:6);
end

