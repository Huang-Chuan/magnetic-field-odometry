function [x0, P] = init_filter(settings)
% x0: nominal state
%  P: error state covariance matrix 
    x0 = [settings.init_pos settings.init_vel settings.init_orientation ...
          settings.init_acc_bias settings.init_gyro_bias settings.init_mag_bias setting.init_coeff    
          ];
    P  = settings.P;
end