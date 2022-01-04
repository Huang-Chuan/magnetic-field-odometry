function [next] = errorState_eq(errorState, noise, dt, acc_m, omega_m, x, RAB_C1, AB, pos_sel, dp)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    persistent errorMasks;
    persistent stateMasks;
    persistent invA;
    persistent numStates;
    
    if isempty(invA)
        invA = inv([calcAB([0,0,1]); calcAB([0, 1, 0]); calcAB([1, 0, 0]); calcAB([1, 1, 1]); calcAB([0, 0, 0])]);
    end
    
    if isempty(numStates)
        [numStates, errorMasks] = getErrorStateMask();
        [~, stateMasks] = getStateMask();
    end

    acc_noise = noise(1:3);
    acc_rw_noise = noise(4:6);
    gyro_noise = noise(7:9);
    gyro_rw_noise = noise(10:12);
    mag_rw_noise = noise(13:27);
    theta_noise = noise(28:end);

    % state
    vel = x(stateMasks.vel);
    q_nb = x(stateMasks.q_nb);
    R_nb = q2r(q_nb);
    acc_bias = x(stateMasks.acc_bias);
    gyro_bias = x(stateMasks.gyro_bias);
    theta = x(stateMasks.theta);

    delta_pk = errorState(errorMasks.pos);
    delta_vk = errorState(errorMasks.vel);
    epsilon = errorState(errorMasks.epsilon);
    delta_acc_bias = errorState(errorMasks.acc_bias);
    delta_gyro_bias = errorState(errorMasks.gyro_bias);
    delta_mag_bias = errorState(errorMasks.mag_bias);
    delta_theta = errorState(errorMasks.theta);
    

    rotvec_12 = (omega_m - gyro_bias) * dt;
    R12 = axang2rotm([rotvec_12 / norm(rotvec_12); norm(rotvec_12)].');
    
    next = zeros(size(errorState));
    next(errorMasks.pos) = delta_pk + delta_vk * dt;
    next(errorMasks.vel) = delta_vk + (-R_nb * vect2skew(acc_m - acc_bias) * epsilon - R_nb * delta_acc_bias) * dt + acc_noise * dt;
    next(errorMasks.epsilon) = R12' * epsilon - delta_gyro_bias * dt + gyro_noise * dt;
    next(errorMasks.acc_bias) =  delta_acc_bias + acc_rw_noise;
    next(errorMasks.gyro_bias) = delta_gyro_bias + gyro_rw_noise;
    next(errorMasks.mag_bias) =  delta_mag_bias + mag_rw_noise;
    

    J1J2 = [dB_dpsi(dp, [0, 0, 1]', dt, (omega_m - gyro_bias), theta); ...
            dB_dpsi(dp, [0, 0, 1]', dt, (omega_m - gyro_bias), theta); ...
            dB_dpsi(dp, [1, 0, 0]', dt, (omega_m - gyro_bias), theta); ...
            dB_dpsi(dp, [1, 1, 1]', dt, (omega_m - gyro_bias), theta); ...
            dB_dpsi(dp, [0, 0, 0]', dt, (omega_m - gyro_bias), theta)];

    M = zeros(21, numStates);
    M(1:15, errorMasks.theta) = eye(15);
    M(16:18, errorMasks.vel)   = R_nb.' * dt;
    M(16:18, errorMasks.epsilon)  = vect2skew(R_nb.' *  dt * (vel + [0; 0; 9.81] * dt / 2));
    M(16:18, errorMasks.acc_bias)   = -dt^2 / 2 * eye(3);

    M(19:21, errorMasks.gyro_bias)  = - eye(3);
    
    M = M(:, ~errorMasks.mag_bias);
    next(errorMasks.theta) = invA * [RAB_C1 J1J2] * (M * [delta_pk;delta_vk;epsilon;delta_acc_bias;delta_gyro_bias;delta_theta] - [zeros(15, 1); dt^2/2 * acc_noise; zeros(3, 1)]) ... 
                             - J1J2(:, end-2:end) * gyro_noise  + theta_noise;


    
    
    
end
