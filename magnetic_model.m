function [m] = magnetic_model(x, type)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    c0 = [0.2; -0.8; 20];
    c1 = [-0.2; -0.1; 20];
    c  = [1; 0; -50];
    
    switch type
        case 0
            m = 1e-6 * (c(1) * x^2 + c(2) * x + c(3));
        case 1
            if  x > 0
                m = 1e-6 * (c0(1) * x^2 + c0(2) * x + c0(3));
            else
                m = 1e-6 * (c1(1) * x^2 + c1(2) * x + c1(3));
            end
        case 2
            m =  1e-6 * (-0.5 * x + 30);
    end
    
    

end

