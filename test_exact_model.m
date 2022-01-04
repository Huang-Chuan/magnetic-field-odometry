    close all;
    clear all;
    addpath('common/');
    
    global numStates;
    global stateMasks;
    global invAB;
    invAB = inv([calcAB([0,0,1]); calcAB([0, 1, 0]); calcAB([1, 0, 0]); calcAB([1, 1, 1]); calcAB([0, 0, 0])]);
    [numStates, stateMasks] = getStateMask();
    % spacing of sensors
    sensor_spacing = 0.05;
    % sampling interval
    fs = 100;
    dT = 1 / fs;
    % monte carlo simulation
    N = 4;

    % simulate IMU noise
    sigma_acc_w = 0.05;
    sigma_coeff_w = 1e-3;
    % simulate IMU bias random walk
    sigma_acc_bias_rw = 10^-5;
    % simulate magnetomer bias random walk(mu T)
    sigma_mag_bias_rw = 10^-5;
    % simulate magnetometer noise (mu T)
    sigma_mag_w = 1e-2;
    % simulate vel noise
    sigma_vel_w = 1e-10;
    % 
    accel_const_bias = 0.01;
    mag_const_bias = 0.01;

    % rng(1);
    duration = 40;
    numSamples = duration*fs;


    timeVector = 0:1/fs:(duration-1/fs);

    % designated accBody
    acc = 1 ./ (1 + exp(-((0:numSamples-1)/numSamples - 1/2))) - 0.5;
    %acc = sin(2*pi*0.5*(0:numSamples-1) * dT);
    %accBody = [acc.', flip(acc.'), -9.8*ones(numSamples, 1)];
    accBody = [acc.', flip(acc.'), acc.'];
    % designated omega
    omega = [0.03 * ones(numSamples, 1) 0.01 * ones(numSamples, 1) 0.03 * ones(numSamples, 1)];
    % designated qBody
    qBody = [ones(numSamples, 1) zeros(numSamples, 3)];
    for i = 2 : numSamples
        qBody(i, :) = quatmultiply(qBody(i - 1, :), rotvec2quat(omega(i - 1, :) * dT)); 
    end

    trajectory = kinematicTrajectory;
    [position,orientation,velocity,acceleration,angularVelocity] = trajectory(accBody, omega);
    %plot(position(:,1),position(:,2),'r.');




    % [accelerometer white noise, other white noise]
    acc_w = sigma_acc_w^2 * ones(1, 3);
    vel_w = sigma_vel_w^2 * ones(1, 3);
    acc_bias_rw = sigma_acc_bias_rw^2 * ones(1, 3);
    mag_bias_rw = sigma_mag_bias_rw^2 * ones(1, 15);
    coeff_b_w = sigma_coeff_w^2 * ones(1, 15);

    processNoise =  diag([acc_w,  vel_w, acc_bias_rw, mag_bias_rw, coeff_b_w]);
    assert(all(size(processNoise) == [39, 39]))


    stateCovariance =  diag([zeros(3, 1); zeros(3, 1); accel_const_bias^2 * ones(3, 1); mag_const_bias^2 * ones(15, 1); 0.001 * ones(15, 1)]);
    init_x0_m = [zeros(24, 1); 25; 25; 25; 0.001 * randn(12, 1)];




    % ImuMag_data = repmat(struct('IMU', [], 'MAG', []), N, 1);
    
    % for iter = 1 : N
    %    Xs{iter} = zeros(numSamples, numStates);
    %    Xs{iter}(:, stateMasks.pos) = position;
    %    Xs{iter}(:, stateMasks.vel) = velocity;
    % end


    % for iter = 1 : N
    %     bias = accel_const_bias * randn(1, 3);
    %     accBias(:, iter) = -bias.';
    %     Xs{iter}(:, stateMasks.acc_bias) = repmat(-bias, numSamples, 1);
    %     accparams = accelparams('RandomWalk', [sigma_acc_bias_rw * sqrt(fs/2), 0, 0], 'NoiseDensity', sigma_acc_w/sqrt(fs/2), 'ConstantBias', bias);
    %     IMU = imuSensor('SampleRate', fs, 'Accelerometer', accparams);
    %     [accelerometerReadings,gyroscopeReadings] = IMU(accBody , omega);
    %     ImuMag_data(iter).IMU = -accelerometerReadings;
        
    %     % Xs{iter}(:, stateMasks.acc_bias) = repmat(bias, numSamples, 1);
        
    %     % accelerometerReadings = accBody + bias + sigma_acc_w * randn(size(accBody)) + [0 0 -9.8];
    %     % ImuMag_data(iter).IMU = accelerometerReadings;
    % end

    % magBias = zeros(18, N);
    % for iter = 1 : N
    %     const_bias = mag_const_bias * rand(1, 15);
    %     Xs{iter}(:, stateMasks.mag_bias) = repmat(const_bias, numSamples, 1);
    % end


    % sensor_locs = [[sensor_spacing/2; sensor_spacing; 0] [-sensor_spacing/2; sensor_spacing; 0] [sensor_spacing/2; 0; 0] [-sensor_spacing/2; 0; 0] [sensor_spacing/2; -sensor_spacing; 0] [-sensor_spacing/2; -sensor_spacing; 0]];
    % for iter = 1 : N
    %     theta_0 =  init_x0_m(end-14:end) + sqrt(0.001) * randn(15, 1);
        
    %     for i = 1 : numSamples
    %         pos_nav = Xs{iter}(i, stateMasks.pos);
    %         % coordinates of all sensors in nagvigation 
    %         X = q2r(qBody(i, :)) * sensor_locs + pos_nav.';
    %         %Xs{iter}(i, 1)
    %         measurementMatrix =  [calcAB(X(:, 1)); calcAB(X(:, 2)); calcAB(X(:, 3)); calcAB(X(:, 4)); calcAB(X(:, 5)); calcAB(X(:, 6))];
    %         magnetometerReadings(i, :) = reshape(q2r(qBody(i, :)).' *  reshape(measurementMatrix * theta_0, 3, []), 1, 18) ; 
                    
    %     %     pos = q2r(qBody(i, :)) * [0 0 1 1 0; 0 1 0 1 0; 1 0 0 1 0] + pos_nav.';
    %     %     RAB_c1 = [q2r(qBody(i, :)).' * calcAB(pos(:, 1)); 
    %     %                q2r(qBody(i, :)).' * calcAB(pos(:, 2)); 
    %     %                q2r(qBody(i, :)).' * calcAB(pos(:, 3)); 
    %     %                q2r(qBody(i, :)).' * calcAB(pos(:, 4)); 
    %     %                q2r(qBody(i, :)).' * calcAB(pos(:, 5)); ];    

    %     %     theta_k =  invAB * RAB_c1 * theta_0;

    %     %     uu = [calcAB(sensor_locs(:, 1)); ...
    %     %  calcAB(sensor_locs(:, 2)); ...
    %     %  calcAB(sensor_locs(:, 3)); ...
    %     %  calcAB(sensor_locs(:, 4)); ...
    %     %  calcAB(sensor_locs(:, 5)); ...
    %     %  calcAB(sensor_locs(:, 6))] * theta_k;
    %     %     vv = magnetometerReadings(i, :).';
    %     %     sum((uu-vv).^2)
            
        
    %     end
    %     magnetometerReadings = magnetometerReadings + sigma_mag_w * randn(size(magnetometerReadings));
    %     magnetometerReadings = magnetometerReadings + [Xs{iter}(:, stateMasks.mag_bias) zeros(numSamples, 3)]; 
        
    %     ImuMag_data(iter).MAG = magnetometerReadings;
    % end



    % x0 = init_x0_m;
    [Xs, ImuMag_data, x0] = data_gen(init_x0_m, stateCovariance, sensor_spacing, processNoise, sigma_mag_w, accBody, ...
                                    qBody, omega, N);


    %% EKF Setup

    XData = cell(N, 1);
    PData = cell(N, 1);


    t = now;
    d = datetime(t,'ConvertFrom','datenum');


    tic
    %% run with iteration
    parfor iter = 1 : N
        magnetometerReadings = ImuMag_data(iter).MAG;
        accelerometerReadings = ImuMag_data(iter).IMU;
        qBodys = qBody;
        omegas = omega;
        
        initialStateGuess = x0;
        %initialStateGuess = Xs{iter}(1,:).';
        obj = extendedKalmanFilter(@StateTransitionFcns, @MeasurementFcns, initialStateGuess, 'HasAdditiveProcessNoise', false);
        obj.StateCovariance = stateCovariance;
        
        obj.ProcessNoise = processNoise;
        obj.MeasurementNoise = sigma_mag_w^2 * eye(18);
        
        X = zeros(numSamples, length(initialStateGuess));
        P = zeros(numSamples, length(initialStateGuess));
        X(1, :) = initialStateGuess;
        P(1, :) = diag(stateCovariance);
        
        
        % start from the second
        for i = 1 : numSamples - 1
            predict(obj, [accelerometerReadings(i, :).'; qBodys(i, :).'; omegas(i, :).']);
            correct(obj, magnetometerReadings(i + 1, :).', sensor_spacing);
            X(i + 1, :) = obj.State.';
            P(i + 1, :) = diag(obj.StateCovariance);
        end
        
        XData{iter} = X;
        PData{iter} = P;
    end
    toc


    %% Statistics
    error_struct = calc_error_2(XData, PData, Xs, numSamples, N);


    % % position error plot
    figure;
    subplot(1,3,1)
    hold on;
    curve1 =  error_struct.stdPosErr(:, 1);
    curve2 =  zeros(size(error_struct.stdPosErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.msePosErr(:, 1), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('position error x');
    %legend('Location','northwest');
    %saveas(gcf,sprintf('exp%d\\PosError.jpg', 0));

    subplot(1,3,2)
    hold on;
    curve1 =  error_struct.stdPosErr(:, 2);
    curve2 =  zeros(size(error_struct.stdPosErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.msePosErr(:, 2), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('position error y');
    %legend('Location','northwest');


    subplot(1,3,3)
    hold on;
    curve1 =  error_struct.stdPosErr(:, 3);
    curve2 =  zeros(size(error_struct.stdPosErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.msePosErr(:, 3), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('position error z');
    %legend('Location','northwest');



    % % velocity error plot
    figure;
    subplot(1,3,1)
    hold on;
    curve1 =  error_struct.stdVelErr(:, 1);
    curve2 =  zeros(size(error_struct.stdVelErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error x');
    legend('Location','northwest');
    %saveas(gcf,sprintf('exp%d\\VelErrorX.jpg', 0));

    subplot(1,3,2)
    hold on;
    curve1 =  error_struct.stdVelErr(:, 2);
    curve2 =  zeros(size(error_struct.stdVelErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error y');
    legend('Location','northwest');
    %saveas(gcf,sprintf('exp%d\\VelErrorY.jpg', 0));


    subplot(1,3,3)
    hold on;
    curve1 =  error_struct.stdVelErr(:, 3);
    curve2 =  zeros(size(error_struct.stdVelErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error z');
    legend('Location','northwest');
   %saveas(gcf,sprintf('exp%d\\VelErrorZ.jpg', 0));



    % % acc bias error plot
    % if(opt.hasAccBias)
    figure;
    subplot(1,3,1)
    hold on;
    curve1 =  error_struct.stdAccBiasErr(:, 1);
    curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseAccBiasErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('acc bias error x');
    legend('Location','northwest');
    %saveas(gcf, sprintf('exp%d\\accBiasError.jpg', 0));
    subplot(1,3,2)
    hold on;
    curve1 =  error_struct.stdAccBiasErr(:, 2);
    curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseAccBiasErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('acc bias error y');
    legend('Location','northwest');

    subplot(1,3,3)
    hold on;
    curve1 =  error_struct.stdAccBiasErr(:, 3);
    curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseAccBiasErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('acc bias error z');
    legend('Location','northwest');


    figure;
    subplot(1,3,1)
    hold on;
    curve1 =  error_struct.stdMagBiasErr(:, 1);
    curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseMagBiasErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('magnetometer bias error x'); 
    legend('Location','northwest');
    %saveas(gcf, sprintf('exp%d\\magBiasError.jpg', 0));
    subplot(1,3,2)
    hold on;
    curve1 =  error_struct.stdMagBiasErr(:, 2);
    curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseMagBiasErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('magnetometer bias error y'); 
    legend('Location','northwest');
    
    subplot(1,3,3)
    hold on;
    curve1 =  error_struct.stdMagBiasErr(:, 3);
    curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseMagBiasErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('magnetometer bias error z'); 
    legend('Location','northwest');