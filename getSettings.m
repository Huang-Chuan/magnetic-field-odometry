function settings = getSettings()
    % settings.radius = 1;   % meters
    % settings.speed = 0.5;      % meters per second
    % settings.climbRate = 0.0001;  % meters per second
    % settings.initialYaw = 90; % degrees
    % settings.pitch = 1;      % degrees
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%             SENSOR PARAMETERS           %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % sensor locations
    settings.sensor_locs = PosMagArray();
    % sampling frequnecy
    settings.fs = 100;
    % sampling interval
    settings.dT = 1 / settings.fs;
    % sampling time 
    settings.duration = 60;
    % data samples 
    settings.numSamples = settings.duration * settings.fs;
    % number of sensors in array
    settings.numSensors = size(PosMagArray, 2);





    fprintf('Using %d magnetometers\n', settings.numSensors);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%         MAGNETIC FIELD SETTINGS        %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    settings.generate_from_model = false;
    fprintf('Generate magnetic field from model: %d\n', settings.generate_from_model);
    settings.model = 'model.mat';
    if ~settings.generate_from_model
        fprintf('load from file: %s\n', settings.model);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%             INIT PARAMETERS             %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % settings.init_pos = [settings.radius, 0, -1];
    % settings.init_vel = [0, settings.speed, settings.climbRate];
    % settings.init_orientation = quaternion([settings.initialYaw,settings.pitch,0],'eulerd','zyx','frame');
    settings.init_acc_bias = [0 0 0];
    settings.init_gyro_bias = [0 0 0];
    settings.init_mag_bias = zeros(1, (settings.numSensors - 1) * 3);
    settings.init_coeff    = [25 25 25  0.001 * ones(1, 12)];
    
    
    % init uncertainty
    settings.init_sigma_acc_const_bias = 0.01;
    settings.init_sigma_gyro_const_bias = 0.05*pi/180;
    settings.init_sigma_mag_const_bias = 0.01;
    settings.init_sigma_coeff    = sqrt(0.5);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%             FILTER PARAMETERS           %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Process noise covariance (Q)
    % IMU measurement noise
    settings.sigma_acc_w = 0.05;
    settings.sigma_gyro_w =  0.1*pi/180;


    % IMU bias random walk noise 
    settings.sigma_acc_bias_rw = 10^-6;
    settings.sigma_gyro_bias_rw = 10^-7;
    settings.sigma_mag_bias_rw = 10^-7;


    % coefficient process noise
    settings.sigma_coeff_w =1;


    % Q matrix
    settings.Q = blkdiag(settings.sigma_acc_w^2 * eye(3), ...
        settings.sigma_gyro_w^2 * eye(3), ...
        settings.sigma_acc_bias_rw^2 * eye(3), ...
        settings.sigma_gyro_bias_rw^2* eye(3), ...
        settings.sigma_mag_bias_rw^2* eye(((settings.numSensors - 1) * 3)), ...
        0.1*settings.sigma_coeff_w^2* eye(8), ...
        settings.sigma_coeff_w^2* eye(7));



    % magnetometer measurement (R)
    % Standard deviations, need to be squared
    settings.sigma_mag_w = 1e-2;
    settings.R = eye(settings.numSensors * 3) * settings.sigma_mag_w^2;

    % Initial Kalman filter uncertainties (position, velocity, orientation, sensor bias, coefficient) 
    settings.P = diag([zeros(9, 1); 
                        settings.init_sigma_acc_const_bias^2   * ones(3, 1); 
                        settings.init_sigma_gyro_const_bias^2  * ones(3, 1);
                        settings.init_sigma_mag_const_bias^2   * ones((settings.numSensors - 1) * 3, 1);
                        settings.init_sigma_coeff^2 * ones(15, 1)           ;]);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%             STATE MASKS                 %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [settings.numErrorStates, settings.errorStateMask] = makeErrorStateMask(settings);
    [settings.numStates, settings.stateMask] = makeStateMask(settings);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%             STATE MASKS                 %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



    % measurement matrix
    settings.A = [];
    for i = 1 : size(settings.sensor_locs, 2)
        settings.A =  [settings.A; calcAB(settings.sensor_locs(:, i))];
    end

    settings.H = zeros(settings.numSensors * 3, settings.numErrorStates);
    settings.H(1:sum(settings.errorStateMask.mag_bias), settings.errorStateMask.mag_bias) = eye(sum(settings.errorStateMask.mag_bias));
    settings.H(:, settings.errorStateMask.theta) = settings.A;



end


function [numErrorStates, masks] = makeErrorStateMask(settings)
    numErrorStates = 16 + (settings.numSensors - 1) * 3 + 14;
    pos = 1 : 3;
    vel = 4 : 6;
    epsilon = 7 : 9;
    acc_bias = 10 : 12;
    gyro_bias = 13 : 15;
    mag_bias = 16 : 16 + (settings.numSensors - 1) * 3 - 1; 
    theta = 16 + (settings.numSensors - 1) * 3 : 16 + (settings.numSensors - 1) * 3 + 14;
    
    
    masks.pos = false(numErrorStates, 1);
    masks.vel = false(numErrorStates, 1);
    masks.epsilon = false(numErrorStates, 1);
    masks.acc_bias = false(numErrorStates, 1);
    masks.gyro_bias = false(numErrorStates, 1);
    masks.mag_bias = false(numErrorStates, 1);
    masks.theta = false(numErrorStates, 1);


    masks.pos(pos) = true;
    masks.vel(vel) = true;
    masks.epsilon(epsilon) = true;
    masks.acc_bias(acc_bias) = true;
    masks.gyro_bias(gyro_bias) = true;
    masks.mag_bias(mag_bias) = true;
    masks.theta(theta) = true;

end



function [numStates, masks] = makeStateMask(settings)
    numStates = 17 + (settings.numSensors - 1) * 3 + 14;

    pos = 1 : 3;
    vel = 4 : 6;
    q_nb = 7 : 10;
    acc_bias = 11 : 13;
    gyro_bias = 14 : 16;
    mag_bias = 17 : 17 + (settings.numSensors - 1) * 3 - 1;
    theta = 17 + (settings.numSensors - 1) * 3  : 17 + (settings.numSensors - 1) * 3 + 14;
    
    
    masks.pos = false(numStates, 1);
    masks.vel = false(numStates, 1);
    masks.q_nb = false(numStates, 1);
    masks.acc_bias = false(numStates, 1);
    masks.gyro_bias = false(numStates, 1);
    masks.mag_bias = false(numStates, 1);
    masks.theta = false(numStates, 1);


    masks.pos(pos) = true;
    masks.vel(vel) = true;
    masks.q_nb(q_nb) = true;
    masks.acc_bias(acc_bias) = true;
    masks.gyro_bias(gyro_bias) = true;
    masks.mag_bias(mag_bias) = true;
    masks.theta(theta) = true;

end