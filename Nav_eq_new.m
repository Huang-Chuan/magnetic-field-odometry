function [x, F, Q] = Nav_eq_new(xk, u, dt, processNoiseCov, settings)
%   INPUT:
%                 xk:  current state
%                  u:  accelerometer reading and gyroscope reading
%    processNoiseCov:  covariance matrix for process noise
%   
%   OUTPUT:
%                  x:  predict state               
%                  F:  transition matrix for error state
%                  Q:  processNoise for error state transition model
    persistent numStates;
    persistent masks;
    persistent numErrorStates;
    persistent errorMasks;
    persistent invA;

%   same A as in paper
    if isempty(invA)
        invA = inv([calcAB([0, 0, 1]); calcAB([0, 1, 0]); calcAB([1, 0, 0]); calcAB([1, 1, 1]); calcAB([0, 0, 0])]);
    end

    if isempty(numStates)
        numStates = settings.numStates;
        masks = settings.stateMask;
        numErrorStates = settings.numErrorStates;
        errorMasks = settings.errorStateMask;           
    end
    


    % position, velocity,  accelerometer bias, magnetometer bias, theta
    pk = xk(masks.pos);
    vk = xk(masks.vel);
    q_nb = xk(masks.q_nb);
    R_nb = q2r(q_nb);
    acc_bias = xk(masks.acc_bias);
    gyro_bias = xk(masks.gyro_bias);
    mag_bias = xk(masks.mag_bias);
    theta = xk(masks.theta);

    %  parse u
    acc_m = u(1:3);
    omega_m = u(4:end);


    acc_nav = R_nb * (acc_m - acc_bias) + [0; 0; 9.81];
    dp = vk * dt + 1/2 * acc_nav * dt^2;
    
    % 
    x = zeros(size(xk));
    x(masks.pos) = pk + dp;
    x(masks.vel) = vk + acc_nav * dt;
    x(masks.acc_bias) = acc_bias;
    
    omega_h = omega_m - gyro_bias;
    rotangle = omega_h * dt;
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

    B = [rot12m.' * AB(1:3, :); 
              rot12m.' * AB(4:6, :);
              rot12m.' * AB(7:9, :);
              rot12m.' * AB(10:12, :);
              rot12m.' * AB(13:15, :)];
   
    x(masks.mag_bias) = mag_bias;
    x(masks.theta) = invA * B * theta;

    % construct F
    F = zeros(numErrorStates);
    F(errorMasks.pos, errorMasks.pos) = eye(3);
    F(errorMasks.pos, errorMasks.vel) = eye(3) * dt;
    F(errorMasks.vel, errorMasks.vel) = eye(3);
    F(errorMasks.vel, errorMasks.epsilon) = -R_nb * vect2skew(acc_m - acc_bias) * dt;
    F(errorMasks.vel, errorMasks.acc_bias) = -R_nb * dt;
    F(errorMasks.epsilon, errorMasks.epsilon) = rot12m';
    F(errorMasks.epsilon, errorMasks.gyro_bias) = -eye(3)*dt;
    F(errorMasks.acc_bias, errorMasks.acc_bias) = eye(3);
    F(errorMasks.gyro_bias, errorMasks.gyro_bias) = eye(3);
    F(errorMasks.mag_bias, errorMasks.mag_bias) = eye(sum(errorMasks.mag_bias));


    J1J2 = [dB_dpsi(dp_body, [0, 0, 1]', dt, omega_h, theta); ...
            dB_dpsi(dp_body, [0, 1, 0]', dt, omega_h, theta); ...
            dB_dpsi(dp_body, [1, 0, 0]', dt, omega_h, theta); ...
            dB_dpsi(dp_body, [1, 1, 1]', dt, omega_h, theta); ...
            dB_dpsi(dp_body, [0, 0, 0]', dt, omega_h, theta)];

    M = zeros(21, numErrorStates);
    M(1:15, errorMasks.theta) = eye(15);
    M(16:18, errorMasks.vel)   = R_nb.' * dt;
    M(16:18, errorMasks.epsilon)  = vect2skew(R_nb.' *  dt * (vk + [0; 0; 9.81] * dt / 2));
    M(16:18, errorMasks.acc_bias)   = -dt^2 / 2 * eye(3);
    M(19:21, errorMasks.gyro_bias)  = - eye(3);

    F(errorMasks.theta, :) = invA * [B J1J2] * M;

    % construct Q
    Fi = zeros(numErrorStates, size(processNoiseCov, 1));
    Fi(errorMasks.vel, 1:3) = eye(3) * dt;
    Fi(errorMasks.epsilon, 4:6) = eye(3) * dt;
    Fi(errorMasks.acc_bias, 7:9) = eye(3);
    Fi(errorMasks.gyro_bias, 10:12) = eye(3);
    Fi(errorMasks.mag_bias, 13: 13 + (settings.numSensors - 1) * 3 - 1)  = eye((settings.numSensors - 1) * 3);
    Fi(errorMasks.theta, 1 : 3) = -invA * J1J2(:, 1:3) * (dt^2/2);
    Fi(errorMasks.theta, 4 : 6) = -invA * J1J2(:, end-2:end);
    Fi(errorMasks.theta, 13 + (settings.numSensors - 1) * 3 : 13 + (settings.numSensors - 1) * 3 + 14) = eye(15);
    Q = Fi * processNoiseCov * Fi';



end

