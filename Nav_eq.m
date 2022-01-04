function [x, RAB_C1, AB, pos_sel, dp_body] = Nav_eq(xk, u, dt)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    persistent numStates;
    persistent masks;
    persistent invAB
    % global invAB;
    if isempty(invAB)
        invAB = inv([calcAB([0,0,1]); calcAB([0, 1, 0]); calcAB([1, 0, 0]); calcAB([1, 1, 1]); calcAB([0, 0, 0])]);
    end


    if isempty(numStates)
        [numStates, masks] = getStateMask();
    end
    
    % position, velocity,  accelerometer bias, magnetometer bias, theta
    pk = xk(masks.pos);
    vk = xk(masks.vel);
    q_nb = xk(masks.q_nb);
    acc_bias = xk(masks.acc_bias);
    gyro_bias = xk(masks.gyro_bias);
    mag_bias = xk(masks.mag_bias);
    theta = xk(masks.theta);

    %  parse u
    acc_m = u(1:3);
    omega_m = u(4:end);


    R_nb = q2r(q_nb);
    acc_nav = R_nb * (acc_m - acc_bias) + [0; 0; 9.81];
    dp = vk * dt + 1/2 * acc_nav * dt^2;
    
    % 
    x = zeros(size(xk));
    x(masks.pos) = pk + dp;
    x(masks.vel) = vk + acc_nav * dt;
    x(masks.acc_bias) = acc_bias;
    
    
    rotangle = (omega_m - gyro_bias) * dt;
    rot12m = axang2rotm([rotangle / norm(rotangle); norm(rotangle)].');
    
    x(masks.gyro_bias) = gyro_bias;
    x(masks.q_nb) = (quatmultiply(q_nb.', rotvec2quat(rotangle.'))).';

    % coordinates of selected points in R1
    dp_body = R_nb.' * dp;
    pos_sel = rot12m * [0 0 1 1 0; 0 1 0 1 0; 1 0 0 1 0] + dp_body;
    
    
    AB = [calcAB(pos_sel(:, 1));
          calcAB(pos_sel(:, 2));
          calcAB(pos_sel(:, 3));
          calcAB(pos_sel(:, 4));
          calcAB(pos_sel(:, 5))];

    RAB_C1 = [rot12m.' * AB(1:3, :); 
              rot12m.' * AB(4:6, :);
              rot12m.' * AB(7:9, :);
              rot12m.' * AB(10:12, :);
              rot12m.' * AB(13:15, :)];
   
    x(masks.mag_bias) = mag_bias;
    x(masks.theta) = invAB * RAB_C1 * theta;
end

