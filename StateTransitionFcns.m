classdef StateTransitionFcns
    properties 
        opt
    end
    methods(Static)
        function [x] = StateTransitionFcn0(xk, vk, u)
            % accelerometer noise
            delta_a = vk(1);
            % position process noise
            wkp = vk(2);
            
            dT = 0.01;
            dp = xk(2) * dT + 1/2 * (u - delta_a) * dT^2 + wkp ;

            c2  = xk(end-2);
            c1  = xk(end-1);
            
            F = [1 dT    0  0  0; ...
                0  1     0  0  0; ...
                0  0     1  0  0; ...
                0  0    2*dp  1  0; ...
                0  0    dp^2  dp  1];
        
            G = [1/2*dT^2; dT; 0; 0; 0];
            
            GG = [-1/2*dT^2; -dT;  0; -1/2*dT^2; -c2 * dp * dT^2 - 1/2 * c1 * dT^2];
            
        
            x = F * xk + G * u + GG * vk(1) + vk(2:end);
        end

        function [x] = StateTransitionFcn1(xk, vk, u)
            % accelerometer noise
            delta_a = vk(1);
            % position process noise
            wkp = vk(2);
            
            dT = 0.01;
            dp = xk(2) * dT + 1/2 * (u - delta_a - xk(3)) * dT^2 + wkp ;

            c2  = xk(end-2);
            c1  = xk(end-1);
            
            F = [1 dT  -1/2 * dT^2       0  0  0; ...
                0  1  -dT                0  0  0; ...
                0  0   1                 0  0  0; ...
                0  0  0                 1  0  0; ...
                0  0  0              2*dp  1  0; ...
                0  0  0              dp^2  dp  1];
        
            G = [1/2*dT^2; dT; 0; 0; 0; 0];
            
            GG = [-1/2*dT^2; -dT; 0; 0; -1/2*dT^2; -c2 * dp * dT^2 - 1/2 * c1 * dT^2];
            
        
            x = F * xk + G * u + GG * vk(1) + vk(2:end);
        end

        function [x] = StateTransitionFcn2(xk, vk, u)
            % accelerometer noise
            delta_a = vk(1);
            % position process noise
            wk_p = vk(2);
            
            dT = 0.01;
            dp = xk(2) * dT + 1/2 * (u - delta_a) * dT^2 + wk_p ;

            c2  = xk(end-2);
            c1  = xk(end-1);
            
            F = [1 dT 0  0  0     0  0  0; ...
                0  1  0  0  0     0  0  0; ...
                0  0  1  0  0     0  0  0; ...
                0  0  0  1  0     0  0  0; ...
                0  0  0  0  1     0  0  0; ...
                0  0  0  0  0     1  0  0; ...
                0  0  0  0  0  2*dp  1  0; ...
                0  0  0  0  0  dp^2  dp  1];
        
            G = [1/2*dT^2; dT; 0; 0; 0; 0; 0; 0];
            
            GG = [-1/2*dT^2;  -dT; ...
                          0;    0;  0; ...
                          0; -1/2*dT^2; -c2 * dp * dT^2 - 1/2 * c1 * dT^2];
            
        
            x = F * xk + G * u + GG * vk(1) + vk(2:end);
        end

        function [x] = StateTransitionFcn3(xk, vk, u)
            % accelerometer noise
            delta_a = vk(1);
            % position process noise
            wk_p = vk(2);
            
            dT = 0.01;
            dp = xk(2) * dT + 1/2 * (u - delta_a - xk(3)) * dT^2 + wk_p ;

            c2  = xk(end-2);
            c1  = xk(end-1);
            
            F = [1 dT  -1/2 * dT^2   0   0  0     0  0  0; ...
                0  1  -dT            0   0  0     0  0  0; ...
                0  0   1             0   0  0     0  0  0; ...
                
                0  0  0              1   0  0     0  0  0; ...
                0  0  0              0   1  0     0  0  0; ...
                0  0  0              0   0  1     0  0  0; ...
                
                0  0  0              0   0  0     1  0  0; ...
                0  0  0              0   0  0  2*dp  1  0; ...
                0  0  0              0   0  0  dp^2  dp  1];
        
            G = [1/2*dT^2; dT; 0; 0; 0; 0; 0; 0; 0];
            
            GG = [-1/2*dT^2;       -dT; 0; ...
                          0;         0; 0; ...
                          0; -1/2*dT^2; -c2 * dp * dT^2 - 1/2 * c1 * dT^2];
            
        
            x = F * xk + G * u + GG * vk(1) + vk(2:end);
        end

    end

    methods
        function [StateTransitionFcn] = getStateTransitionFcn(obj)
            if (~obj.opt.hasMagBias) && (~obj.opt.hasAccBias) 
                StateTransitionFcn = @obj.StateTransitionFcn0;
            elseif (~obj.opt.hasMagBias) && (obj.opt.hasAccBias) 
                StateTransitionFcn = @obj.StateTransitionFcn1;
            elseif (obj.opt.hasMagBias) && (~obj.opt.hasAccBias) 
                StateTransitionFcn = @obj.StateTransitionFcn2;
            elseif (obj.opt.hasMagBias) && (obj.opt.hasAccBias) 
                StateTransitionFcn = @obj.StateTransitionFcn3;
            else
               StateTransitionFcn = 0;
            end
         end
    
    
    end


end