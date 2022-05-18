function [x, F, Q] = Nav_eq(xk, u, dt, processNoiseCov, settings)
%   INPUT:
%                 xk:  current state
%                  u:  accelerometer reading and gyroscope reading
%                 dt:  sampling interval
%    processNoiseCov:  covariance matrix for process noise
%           settings:  struct setting 
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

    %   invA as in eq. 13
    if isempty(invA)
        invA = inv([calcPhi([0, 0, 1]); calcPhi([0, 1, 0]); calcPhi([1, 0, 0]); calcPhi([1, 1, 1]); calcPhi([0, 0, 0])]);
    end

    if isempty(numStates)
        numStates = settings.numStates;
        masks = settings.stateMask;
        numErrorStates = settings.numErrorStates;
        errorMasks = settings.errorStateMask;           
    end
    
    % parse xk
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
    
    % nominal state \hat{x}_k as in eq. 16
    x = zeros(size(xk));
    x(masks.pos) = pk + dp;
    x(masks.vel) = vk + acc_nav * dt;
    x(masks.acc_bias) = acc_bias;
    
    omega_h = omega_m - gyro_bias;
    rotangle = omega_h * dt;
    rot12m = axang2rotm([rotangle / norm(rotangle); norm(rotangle)].');
    
    x(masks.gyro_bias) = gyro_bias;
    x(masks.q_nb) = (quatmultiply(q_nb.', rotvec2quat(rotangle.'))).';
 
    % coordinates of r^{b_k} as in eq. 11
    dp_body = R_nb.' * dp;
    pos_sel = rot12m * [0 0 1 1 0; 0 1 0 1 0; 1 0 0 1 0] + dp_body;
    

    % B as in eq. 11
    B = [rot12m.' * calcPhi(pos_sel(:, 1)); 
         rot12m.' * calcPhi(pos_sel(:, 2));
         rot12m.' * calcPhi(pos_sel(:, 3));
         rot12m.' * calcPhi(pos_sel(:, 4));
         rot12m.' * calcPhi(pos_sel(:, 5))];
   
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

    % J1J2 as in eq. 23b and 23c
    J1J2 = [dB_dpsi(dp_body, [0, 0, 1]', dt, omega_h, theta); ...
            dB_dpsi(dp_body, [0, 1, 0]', dt, omega_h, theta); ...
            dB_dpsi(dp_body, [1, 0, 0]', dt, omega_h, theta); ...
            dB_dpsi(dp_body, [1, 1, 1]', dt, omega_h, theta); ...
            dB_dpsi(dp_body, [0, 0, 0]', dt, omega_h, theta)];
    
    % M as in eq. 23d
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

