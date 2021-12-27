function [x] = StateTransitionFcns(xk, vk, u)
    global numStates;
    global stateMasks;
    persistent invAB
    % global invAB;
    if isempty(invAB)
        invAB = inv([calcAB([0,0,1]); calcAB([0, 1, 0]); calcAB([1, 0, 0]); calcAB([1, 1, 1]); calcAB([0, 0, 0])]);
    end


    % accelerometer noise
    acc_w = vk(1 : 3);
    % velocity noise
    vel_w = vk(4 : 6);
    % acc bias drift
    acc_bias_n = vk(7 : 9);
    % mag bias drift (5 x 3)
    mag_bias_n = vk(10 : 24);
    % coeff drift
    coeff_w = vk(25 : end);
    
    % position, velocity,  accelerometer bias, magnetometer bias, theta
    pk = xk(1:3);
    vk = xk(4:6);
    acc_bias = xk(7:9);
    mag_b = xk(10:24);
    theta = xk(25:end);
    % sample rate
    dT = 0.01;

    %  parse u
    acc_m = u(1:3);
    q_nb  = u(4:7);
    omega_m = u(8:end);


    R_nb = q2r(q_nb);
    acc_nav = R_nb * (acc_m - acc_bias - acc_w) + [0; 0; 9.81];

    dp = vk * dT + 1/2 * acc_nav * dT^2;
    
    % 
    x = zeros(size(xk));
    x(1:3) = pk + dp;
    x(4:6) = vk + acc_nav * dT + vel_w;
    x(7:9) = acc_bias + acc_bias_n;
    x(10:24) = mag_b + mag_bias_n;
   
    rotangle = omega_m * dT;
    rot12m = axang2rotm([rotangle / norm(rotangle); norm(rotangle)].');
    %rot12m = eye(3);

    % coordinates of selected points in R1
    dp_body = R_nb.' * dp;
    pos_sel = rot12m * [0 0 1 1 0; 0 1 0 1 0; 1 0 0 1 0] + dp_body;
    RAB_C1 = [rot12m.' * calcAB(pos_sel(:, 1)); 
              rot12m.' * calcAB(pos_sel(:, 2));
              rot12m.' * calcAB(pos_sel(:, 3));
              rot12m.' * calcAB(pos_sel(:, 4));
              rot12m.' * calcAB(pos_sel(:, 5))];
   
   
    x(25:end) = invAB * RAB_C1 * theta + coeff_w;

end
