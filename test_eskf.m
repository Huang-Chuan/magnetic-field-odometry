    close all;
    clear all;
    addpath('common/');
    rng(1);
    %global numStates;
    %global stateMasks;
    %global invAB;
    invAB = inv([calcAB([0,0,1]); calcAB([0, 1, 0]); calcAB([1, 0, 0]); calcAB([1, 1, 1]); calcAB([0, 0, 0])]);
    [numStates, stateMasks] = getStateMask();
    [numErrorStates, errorStateMasks] = getErrorStateMask();
    
    settings = getSettings();
    [position,orientation,velocity,acceleration,angularVelocity] = trajectory_gen(settings);
    figure;
    plot3(position(:, 1), position(:, 2), position(:, 3));
    title("trajectory");

    % spacing of sensors
    sensor_spacing = 0.05;
    % sampling interval
    fs = 100;
    dT = 1 / fs;
    % monte carlo simulation
    N = 4;

    % simulate IMU noise
    sigma_acc_w = 0.05;
    sigma_gyro_w =  0.1*pi/180;
    sigma_coeff_w = 1e-3;
    % simulate IMU bias random walk
    sigma_acc_bias_rw = 0 * 10^-6;
    % simulate magnetomer bias random walk(mu T)
    sigma_mag_bias_rw = 0;
    % simulate gyro bias random walk
    sigma_gyro_bias_rw = 0* 10^-7;
    % simulate magnetometer noise (mu T)
    sigma_mag_w = 1e-2;
    % 
    accel_const_bias = 0.01;
    gyro_const_bias = 0.05*pi/180;
    mag_const_bias = 0.01;
    init_sigma_coeff = 0.01;

    % rng(1);
    duration = 60;
    numSamples = duration*fs;


    timeVector = 0:1/fs:(duration-1/fs);

    % designated accBody
    radius = 5;   % meters
    speed = 1;      % meters per second
    climbRate = 0.01;  % meters per second
    initialYaw = 90; % degrees
    pitch = 15;      % degrees
    initPos = [radius, 0, 0];
    initVel = [0, speed, climbRate];
    initOrientation = quaternion([initialYaw,pitch,0],'eulerd','zyx','frame');

    trajectory = kinematicTrajectory('SampleRate',fs, ...
        'Velocity',initVel, ...
        'Position',initPos, ...
        'Orientation',initOrientation);

    accBody = zeros(numSamples,3);
    accBody(:,2) = speed^2/radius;
    accBody(:,3) = 0.001;

    omega = zeros(numSamples,3);
    omega(:,3) = speed/radius;

    pitchRotation = quaternion([0,pitch,0],'eulerd','zyx','frame');
    omega = rotateframe(pitchRotation,omega);
    accBody = rotateframe(pitchRotation,accBody);

    [position,orientation,velocity,acceleration,angularVelocity] = trajectory(accBody, omega);
    %plot(position(:,1),position(:,2),'r.');

    figure;
    plot3(position(:, 1), position(:, 2), position(:, 3));
    title("trajectory");

    figure;
    eulerAngles = eulerd(orientation,'ZYX','frame');
    plot(timeVector,eulerAngles(:,1),'bo',...
     timeVector,eulerAngles(:,2),'r.',...
     timeVector,eulerAngles(:,3),'g.')
    axis([0,duration,-180,180])
    legend('Yaw','Pitch','Roll')
    xlabel('Time (s)')
    ylabel('Rotation (degrees)')
    title('Orientation')
    
    
    
    
    % [accelerometer white noise, other white noise]
    acc_w = sigma_acc_w^2 * ones(1, 3);
    acc_bias_rw = sigma_acc_bias_rw^2 * ones(1, 3);
    gyro_w = sigma_gyro_w^2 * ones(1, 3);
    gyro_bias_rw = sigma_gyro_bias_rw^2 * ones(1, 3);
    mag_bias_rw = sigma_mag_bias_rw^2 * ones(1, 15);
    coeff_b_w = sigma_coeff_w^2 * ones(1, 15);


    init_x0_m = [initPos.'; initVel.'; compact(initOrientation).'; zeros(21, 1); 25; 25; 25; 0.001 * ones(12, 1)];


    ImuMag_data = repmat(struct('IMU', [], 'MAG', []), N, 1);

    for iter = 1 : N
       Xs{iter} = zeros(numSamples, numStates);
       Xs{iter}(:, stateMasks.pos) = position;
       Xs{iter}(:, stateMasks.vel) = velocity;
       Xs{iter}(:, stateMasks.q_nb) = compact(orientation);
    end

    sensor_locs = [[sensor_spacing/2; sensor_spacing; 0] [-sensor_spacing/2; sensor_spacing; 0] [sensor_spacing/2; 0; 0] [-sensor_spacing/2; 0; 0] [sensor_spacing/2; -sensor_spacing; 0] [-sensor_spacing/2; -sensor_spacing; 0]];
    
    theta_0 = init_x0_m(end-14:end);
    theta_0_b = theta_0;
    
    A =  [calcAB(sensor_locs(:, 1)); calcAB(sensor_locs(:, 2)); calcAB(sensor_locs(:, 3)); ...
                            calcAB(sensor_locs(:, 4)); calcAB(sensor_locs(:, 5)); calcAB(sensor_locs(:, 6))];
    for iter = 1 : N
        acc_bias = cumsum([accel_const_bias * randn(1, 3); sigma_acc_bias_rw * randn(numSamples - 1, 3)]);
        Xs{iter}(:, stateMasks.acc_bias) = acc_bias;
        accelerometerReadings = accBody + rotateframe(orientation, [0 0 -9.81]) + acc_bias + sigma_acc_w * randn(size(accBody));

        gyro_bias = cumsum([gyro_const_bias * randn(1, 3); sigma_gyro_bias_rw * randn(numSamples - 1, 3)]);
        Xs{iter}(:, stateMasks.gyro_bias) = gyro_bias;
        gyroReadings = omega + gyro_bias + sigma_gyro_w * randn(size(omega));

        %plot(timeVector, gyro_bias/pi*180);

        ImuMag_data(iter).IMU = [accelerometerReadings gyroReadings];

        
        mag_bias = cumsum([mag_const_bias * randn(1, 15); sigma_mag_bias_rw * randn(numSamples - 1, 15)]);
        Xs{iter}(:, stateMasks.mag_bias) = mag_bias;


        magnetometerReadings = zeros(numSamples, 18);
        theta = theta_0_b;
        for i = 1 : numSamples
            
            magnetometerReadings(i, :) = reshape(A * theta, 1, 18);
            rotvec = omega(i, :) * dT;
            q_12 = rotvec2quat(rotvec);
            R_12 = q2r(q_12);
            %theta = theta_0  
            if(i == numSamples) break;
            end
            dp_nav = position(i + 1, :) - position(i, :);
            pos_sel = R_12 * [0 0 1 1 0; 0 1 0 1 0; 1 0 0 1 0] + rotateframe(orientation(i, :), dp_nav).';
            RAB_C1 = [R_12.' * calcAB(pos_sel(:, 1)); 
              R_12.' * calcAB(pos_sel(:, 2));
              R_12.' * calcAB(pos_sel(:, 3));
              R_12.' * calcAB(pos_sel(:, 4));
              R_12.' * calcAB(pos_sel(:, 5))];
            theta = invAB * RAB_C1 * theta + sigma_coeff_w * randn(size(theta));
        end
        %randn
        magnetometerReadings = magnetometerReadings + sigma_mag_w * randn(size(magnetometerReadings));
        magnetometerReadings = magnetometerReadings + [Xs{iter}(:, stateMasks.mag_bias) zeros(numSamples, 3)]; 
        ImuMag_data(iter).MAG = magnetometerReadings;


    end
    

        %const_bias = mag_const_bias * rand(1, 15);




    % for iter = 1 : N

    % end


    init_x0_m(end-14:end) = theta_0_b;

    x0 = init_x0_m;


    XData = cell(N, 1);
    PData = cell(N, 1);


    t = now;
    d = datetime(t,'ConvertFrom','datenum');


    processNoiseCov =  diag([acc_w, gyro_w, acc_bias_rw, gyro_bias_rw, mag_bias_rw, coeff_b_w]);
    MeasurementNoiseCov = sigma_mag_w^2 * eye(18);

    xx = getSettings();
    disp(all(xx.Q==processNoiseCov, 'all'));
    disp(all(xx.R==MeasurementNoiseCov, 'all'));
    tic
    %% run with iteration
    parfor iter = 1 : N
        magnetometerReadings = ImuMag_data(iter).MAG;
        accelerometerReadings = ImuMag_data(iter).IMU(:, 1:3);
        gyroReadings = ImuMag_data(iter).IMU(:, 4:6);

        P = diag([zeros(9, 1); 
                accel_const_bias^2 * ones(3, 1); 
                gyro_const_bias^2  * ones(3, 1);
                mag_const_bias^2   * ones(15, 1);
                init_sigma_coeff^2 * ones(15, 1)]);


        X = zeros(numSamples, numStates);
        Ps = zeros(numSamples, numStates - 1);
        X(1, :) = x0;
        Ps(1, :) = diag(P);

        x = x0;
        for i = 1 : numSamples - 1
            %fprintf("%d step\n", i);
            acc_m = accelerometerReadings(i, :)';
            omega_m = gyroReadings(i, :)';
            u = [acc_m; omega_m];
            [xh, F, Q] =  Nav_eq_(x, u, dT, processNoiseCov);
            % cov propagation
            P = F * P * F' + Q;

            H = zeros(18, numErrorStates);
            H(1:15, errorStateMasks.mag_bias) = eye(sum(errorStateMasks.mag_bias));
            H(:, errorStateMasks.theta) = A;

            K = P * H' / (H * P * H' + MeasurementNoiseCov);
            
            delta_x = K * (magnetometerReadings(i + 1, :).' - (A * xh(stateMasks.theta) + [xh(stateMasks.mag_bias); zeros(3, 1)]));
            
            P = (eye(size(P)) - K * H) * P * (eye(size(P)) - K * H)' + K * MeasurementNoiseCov * K';
            %P = (eye(size(P)) - K * H) * P;
            xh(stateMasks.pos) = xh(stateMasks.pos) +  delta_x(errorStateMasks.pos);
            xh(stateMasks.vel) = xh(stateMasks.vel) +  delta_x(errorStateMasks.vel);
            xh(stateMasks.q_nb) = quatmultiply(xh(stateMasks.q_nb).',  [1 1/2*delta_x(7:9).']);
            xh(stateMasks.q_nb) = xh(stateMasks.q_nb).' / norm(xh(stateMasks.q_nb)) 
            xh(stateMasks.acc_bias) = xh(stateMasks.acc_bias) +  delta_x(errorStateMasks.acc_bias);
            xh(stateMasks.gyro_bias) = xh(stateMasks.gyro_bias) +  delta_x(errorStateMasks.gyro_bias);
            xh(stateMasks.mag_bias) = xh(stateMasks.mag_bias) +  delta_x(errorStateMasks.mag_bias);
            xh(stateMasks.theta) = xh(stateMasks.theta) +  delta_x(errorStateMasks.theta);
            x = xh;


            X(i + 1, :) = x;
            Ps(i + 1, :) = diag(P);
            G = blkdiag(eye(6), eye(3) - 1 / 2 * vect2skew(delta_x(7:9)) ,eye(36));
            P = G * P * G.';
        end
        
        XData{iter} = X;
        PData{iter} = Ps;
    end
    toc

    %% Statistics
    error_struct = calc_error_2(XData, PData, Xs, numSamples, N);

    figure;
    subplot(3,1,1)
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
   
    %saveas(gcf,sprintf('exp%d\\PosError.jpg', 0));

    subplot(3,1,2)
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
    %


    subplot(3,1,3)
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
    %



    % % velocity error plot
    figure;
    subplot(3,1,1)
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
    
    %saveas(gcf,sprintf('exp%d\\VelErrorX.jpg', 0));

    subplot(3,1,2)
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
    
    %saveas(gcf,sprintf('exp%d\\VelErrorY.jpg', 0));


    subplot(3,1,3)
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
    
   %saveas(gcf,sprintf('exp%d\\VelErrorZ.jpg', 0));


    % orientation error
    figure;
    subplot(3,1,1)
    hold on;
    curve1 =  error_struct.stdQErr(:, 1);
    curve2 =  zeros(size(error_struct.stdQErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, 2 * error_struct.mseQErr(:, 1), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('orientation error x');
    %
    %saveas(gcf,sprintf('exp%d\\PosError.jpg', 0));

    subplot(3,1,2)
    hold on;
    curve1 =  error_struct.stdQErr(:, 2);
    curve2 =  zeros(size(error_struct.stdQErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, 2 * error_struct.mseQErr(:, 2), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('orientation error y');
    %


    subplot(3,1,3)
    hold on;
    curve1 =  error_struct.stdQErr(:, 3);
    curve2 =  zeros(size(error_struct.stdQErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, 2 * error_struct.mseQErr(:, 3), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('orientation error z');
   

   
    % % acc bias error plot
    % if(opt.hasAccBias)
    figure;
    subplot(3,1,1)
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
    
    %saveas(gcf, sprintf('exp%d\\accBiasError.jpg', 0));
    subplot(3,1,2)
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
    

    subplot(3,1,3)
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
    


    figure;
    subplot(3,1,1)
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
    
    %saveas(gcf, sprintf('exp%d\\magBiasError.jpg', 0));
    subplot(3,1,2)
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
    
    
    subplot(3,1,3)
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
    


    % gyro bias
    figure;
    subplot(3,1,1)
    hold on;
    curve1 =  error_struct.stdGyroErr(:, 1);
    curve2 =  zeros(size(error_struct.stdGyroErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseGyroErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('gyro bias error x');
    
    %saveas(gcf,sprintf('exp%d\\VelErrorX.jpg', 0));

    subplot(3,1,2)
    hold on;
    curve1 =  error_struct.stdGyroErr(:, 2);
    curve2 =  zeros(size(error_struct.stdGyroErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseGyroErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('gyro bias error y');
    
    %saveas(gcf,sprintf('exp%d\\VelErrorY.jpg', 0));


    subplot(3,1,3)
    hold on;
    curve1 =  error_struct.stdGyroErr(:, 3);
    curve2 =  zeros(size(error_struct.stdGyroErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseGyroErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('gyro bias error z');
    