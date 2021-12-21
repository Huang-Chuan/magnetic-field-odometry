    close all;
    addpath('common/');
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


    % rng(1);
    duration = 40;
    numSamples = duration*fs;


    timeVector = 0:1/fs:(duration-1/fs);

    % designated accBody
    acc = 1 ./ (1 + exp(-((0:numSamples-1)/numSamples - 1/2))) - 0.52;
    accBody = [acc.', flip(acc.'), -9.8*ones(numSamples, 1)];
    % designated omega
    omega = [2*pi*ones(numSamples, 1) zeros(numSamples, 2)];
    % designated qBody
    qBody = [ones(numSamples, 1) zeros(numSamples, 3)];




    % [accelerometer white noise, other white noise]
    acc_w = sigma_acc_w^2 * ones(1, 3);
    vel_w = sigma_vel_w^2 * ones(1, 3);
    acc_bias_rw = sigma_acc_bias_rw^2 * ones(1, 3);
    mag_bias_rw = sigma_mag_bias_rw^2 * ones(1, 15);
    coeff_b_w = sigma_coeff_w^2 * ones(1, 15);

    processNoise =  diag([acc_w,  vel_w, acc_bias_rw, mag_bias_rw, coeff_b_w]);
    assert(all(size(processNoise) == [39, 39]))


    stateCovariance =  diag([zeros(3, 1); zeros(3, 1); 0.001 * ones(3, 1); 0.001 * ones(15, 1); 0.001 * ones(15, 1)]);
    init_x0_m = [zeros(24, 1); 25; 25; 25; 0.5 * randn(12, 1)];





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
            predict(obj, [accelerometerReadings(i, :).'; qBody(i, :).'; omega(i, :).']);
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

    % % Plot error in position/velocity estimation

    % figure;
    % plot(timeVector,ImuMag_data(1).MAG(:,1),'b.',...
    %     timeVector,ImuMag_data(1).MAG(:,2),'r.',...
    %     timeVector,ImuMag_data(1).MAG(:,3),'g.');
    % xlabel('time/s');
    % ylabel('mag/T');
    % title('magnetometer reading');
    % saveas(gcf, sprintf('exp%d\\magnetometer_reading.jpg', order));

    % figure;
    % hold on;

    % % position error plot
    figure;
    hold on;
    curve1 =  error_struct.stdPosErr_x;
    curve2 =  zeros(size(error_struct.stdPosErr_x));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.msePosErr_x, 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('position error');
    legend('Location','northwest');
    saveas(gcf,sprintf('exp%d\\PosError.jpg', 0));

    % % velocity error plot
    figure;
    hold on;
    curve1 =  error_struct.stdVelErr_x;
    curve2 =  zeros(size(error_struct.stdVelErr_x));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr_x, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error x');
    legend('Location','northwest');
    saveas(gcf,sprintf('exp%d\\VelErrorX.jpg', 0));

    figure;
    hold on;
    curve1 =  error_struct.stdVelErr_y;
    curve2 =  zeros(size(error_struct.stdVelErr_y));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr_y, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error y');
    legend('Location','northwest');
    saveas(gcf,sprintf('exp%d\\VelErrorY.jpg', 0));


    figure;
    hold on;
    curve1 =  error_struct.stdVelErr_z;
    curve2 =  zeros(size(error_struct.stdVelErr_z));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr_z, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error z');
    legend('Location','northwest');
    saveas(gcf,sprintf('exp%d\\VelErrorZ.jpg', 0));



    % % acc bias error plot
    % if(opt.hasAccBias)
    figure;
    hold on;
    curve1 =  error_struct.stdAccBiasErr_x;
    curve2 =  zeros(size(error_struct.stdAccBiasErr_x));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseAccBiasErr_x, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('acc bias error');
    legend('Location','northwest');
    saveas(gcf, sprintf('exp%d\\accBiasError.jpg', 0));
    % end

    % % mag bias error plot
    % if(opt.hasMagBias)
    %     figure;
    %     hold on;
    %     curve1 =  error_struct.stdMagBiasErr;
    %     curve2 =  zeros(size(error_struct.stdMagBiasErr));
    %     inBetween = [curve1; flipud(curve2)];
    %     x = [timeVector fliplr(timeVector)];
    %     h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    %     h.FaceAlpha = 0.8;
    %     plot(timeVector, error_struct.mseMagBiasErr, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    %     xlabel('time/s');
    %     ylabel('error/\mu T');
    %     title('mag bias error');
    %     legend('Location','northwest');
    %     saveas(gcf, sprintf('exp%d\\magBiasError.jpg', order));
    % end

    % % coeff plot
    % figure;
    % hold on;
    % curve1 =  error_struct.stdC2Err;
    % curve2 =  zeros(size(error_struct.stdC2Err));
    % inBetween = [curve1; flipud(curve2)];
    % x = [timeVector fliplr(timeVector)];
    % h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    % h.FaceAlpha = 0.8;
    % plot(timeVector, error_struct.mseC2Err, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    % xlabel('time/s');
    % title('C2 error');
    % legend('Location','northwest');
    % saveas(gcf, sprintf('exp%d\\C2Error.jpg', order));

    % figure;
    % hold on;
    % curve1 =  error_struct.stdC1Err;
    % curve2 =  zeros(size(error_struct.stdC2Err));
    % inBetween = [curve1; flipud(curve2)];
    % x = [timeVector fliplr(timeVector)];
    % h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    % h.FaceAlpha = 0.8;
    % plot(timeVector, error_struct.mseC1Err, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    % xlabel('time/s');
    % title('C1 error');
    % legend('Location','northwest');
    % saveas(gcf, sprintf('exp%d\\C1Error .jpg', order));

    % figure;
    % hold on;
    % curve1 =  error_struct.stdC0Err;
    % curve2 =  zeros(size(error_struct.stdC2Err));
    % inBetween = [curve1; flipud(curve2)];
    % x = [timeVector fliplr(timeVector)];
    % h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    % h.FaceAlpha = 0.8;
    % plot(timeVector, error_struct.mseC0Err, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    % xlabel('time/s');
    % title('C0 error');
    % legend('Location','northwest');
    % saveas(gcf, sprintf('exp%d\\C0Error.jpg', order));
