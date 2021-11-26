clear all;
close all;

%% System configuration
opt.hasAccBias = true;
opt.hasMagBias = false;

% type of magnetic field
type = 1;
% simulate IMU noise
sigma_acceleration = 0.05;
% simulate IMU bias random walk
sigma_acc_bias = 10^-4;
% simulate magnetometer noise (mu T)
sigma_magnetometer = 1e-2;
% simulate magnetomer bias (mu T)
sigma_const_bias_m = 0.1;
% spacing of sensors
sensor_spacing = 0.1;
% initial position
init_pos = [0.3, 0, 0];
% initial velocity 
init_vel = [1, 0, 0];
% sampling interval
fs = 100;
dT = 1 / fs;
% monte carlo simulation
N = 4;
% rng(1);
%% illustrate magnetic field
x = -10 : 0.1 : 5;
field1 = arrayfun(@(x) magnetic_model(x, type), x);
figure;
plot(x, field1)
xlabel('x/m');
ylabel('mag/\mu T');
title('magnetic field');
%% trajectory
traj = kinematicTrajectory('Position', init_pos, ...
                           'Velocity', init_vel,...
                           'Orientation', quaternion([0 0 0], 'eulerd', 'ZYX', 'frame'));

duration = 40;
numSamples = duration*fs;

% designated acceleration 
acc = 1 ./ (1 + exp(-((0:numSamples-1)/numSamples - 1/2))) - 0.52;
accBody = [acc.', zeros(numSamples, 2)];

angVelBody = zeros(numSamples,3);


q = ones(numSamples,1,'quaternion');
[pos, orientationNED, vel, accNED, angVelNED] = traj(accBody, angVelBody);

timeVector = 0:1/fs:(duration-1/fs);
figure(3)
plot(timeVector,pos(:,1),'b.',...
     timeVector,pos(:,2),'r.',...
     timeVector,pos(:,3),'g.')
xlabel('Time (s)')
ylabel('Position (m)')
title('NED Position Over Time')
legend('North','East','Down')
%% Generate IMU data
ImuMag_data = repmat(struct('IMU', [], 'MAG', []), N, 1);

%(random walk is turned off, only constant bias)
accparams = accelparams('RandomWalk', [sigma_acc_bias * sqrt(fs/2) * 0, 0, 0], 'NoiseDensity', sigma_acceleration/sqrt(fs/2), 'ConstantBias', 0.1 * randn(1, 3));
IMU = imuSensor('SampleRate', fs, 'Accelerometer', accparams);
[accelerometerReadings,gyroscopeReadings] = IMU(accBody, angVelBody);
figure;
plot(timeVector, -accelerometerReadings(:, 1));

accBias = zeros(3, N);
for iter = 1 : N
    if(opt.hasAccBias)
        bias = 0.1 * randn(1, 3);
    else
        bias = [0 0 0];
    end
    accBias(:, iter) = -bias.';
    accparams = accelparams('RandomWalk', [sigma_acc_bias * sqrt(fs/2) * 0, 0, 0], 'NoiseDensity', sigma_acceleration/sqrt(fs/2), 'ConstantBias', bias);
    IMU = imuSensor('SampleRate', fs, 'Accelerometer', accparams);
    [accelerometerReadings,gyroscopeReadings] = IMU(accBody, angVelBody);
    ImuMag_data(iter).IMU = accelerometerReadings;
end

%% Generate IMU and Magnetometer data

model = @magnetic_model;
magBias = zeros(3, N);

for i = 1 : size(pos, 1)
    mag_gt(i, 1) = model(pos(i, 1) - sensor_spacing, type) ;
    mag_gt(i, 2) = model(pos(i, 1), type) ; 
    mag_gt(i, 3) = model(pos(i, 1) + sensor_spacing, type) ;
end

r = sensor_spacing;
H = [r^2 -r 1;0 0 1; r^2 r 1]; 
c = calc_coeff(mag_gt, H);


for iter = 1 : N
    magnetometerReadings = zeros(numSamples, 3);
    if(opt.hasMagBias)
        const_bias = sigma_const_bias_m * rand(1, 3);
    else
        const_bias = [0 0 0];
    end
    magBias(:, iter) = const_bias.';
    for i = 1 : size(pos, 1)
        magnetometerReadings(i, 1) = model(pos(i, 1) - sensor_spacing, type) + sigma_magnetometer * randn() + const_bias(1);
        magnetometerReadings(i, 2) = model(pos(i, 1), type) + sigma_magnetometer * randn() + const_bias(2);
        magnetometerReadings(i, 3) = model(pos(i, 1) + sensor_spacing, type) +  sigma_magnetometer * randn() + const_bias(3);
    end
    ImuMag_data(iter).MAG = magnetometerReadings;
end


%% EKF Setup

XData = cell(N, 1);
PData = cell(N, 1);

% select stateTransitionFunc accordingly
stateTransitionCls = StateTransitionFcns();
stateTransitionCls.opt = opt;

% select measurementFunc accordingly
measurementCls = MeasurementFcns();
measurementCls.opt = opt;

t = now;
d = datetime(t,'ConvertFrom','datenum');


% [accelerometer white noise, other white noise]
processNoise = [sigma_acceleration^2, 1e-10, 1e-10, sigma_acc_bias^2, 1e-6, 1e-6, 1e-6, 1e-1, 1e-1, 1e-1];
% [pk, vk, delta_a_k, delta_m_k_m1, delta_m_k_0, delta_m_k_1, c0, c1, c2]
stateCovariance =  [0, 0, 0.01,  sigma_const_bias_m^2, sigma_const_bias_m^2, sigma_const_bias_m^2, 20, 20, 20];

% select processNoise, stateCovariance accordingly
if (~opt.hasMagBias) && (~opt.hasAccBias) 
    processNoise = diag([processNoise(1 : 3),  processNoise(end - 2 : end)]);
    stateCovariance = diag([stateCovariance(1 : 2),  stateCovariance(end - 2 : end)]);
elseif(~opt.hasMagBias) && (opt.hasAccBias) 
    processNoise = diag([processNoise(1 : 4),  processNoise(end - 2 : end)]);
    stateCovariance = diag([stateCovariance(1 : 3),  stateCovariance(end - 2 : end)]);
elseif(opt.hasMagBias) && (~opt.hasAccBias) 
    processNoise = diag([processNoise(1 : 3), processNoise(5 : 7), processNoise(end - 2 : end)]);
    stateCovariance = diag([stateCovariance(1 : 2),  stateCovariance(5 : 7), stateCovariance(end - 2 : end)]);
elseif(opt.hasMagBias) && (opt.hasAccBias) 
    processNoise = diag(processNoise);
    stateCovariance = diag(stateCovariance);
else
    processNoise    = 0;
    stateCovariance = 0;
end


r = sensor_spacing;
H = [r^2 -r 1;0 0 1; r^2 r 1]; 


stateTransitionFcn = stateTransitionCls.getStateTransitionFcn();
measurementFcn = measurementCls.getMeasurementFcn();
tic
% run with iteration
parfor iter = 1 : N
    magnetometerReadings = ImuMag_data(iter).MAG;
    accelerometerReadings = ImuMag_data(iter).IMU;
    
    coeff = H \ (magnetometerReadings(1, :).');
    if(opt.hasAccBias) &&(opt.hasMagBias)
        initialStateGuess = [init_pos(1); init_vel(1); 0; 0; 0; 0; coeff];
    elseif (~opt.hasAccBias) &&(opt.hasMagBias)
        initialStateGuess = [init_pos(1); init_vel(1); 0; 0; 0; coeff];
    elseif (opt.hasAccBias) &&(~opt.hasMagBias)
        initialStateGuess = [init_pos(1); init_vel(1); 0; coeff];
    else
        initialStateGuess = [init_pos(1); init_vel(1); coeff];
    end
    
    obj = extendedKalmanFilter(stateTransitionFcn, measurementFcn, initialStateGuess, 'HasAdditiveProcessNoise', false);
    obj.StateCovariance = stateCovariance;
    
    obj.ProcessNoise = processNoise;
    obj.MeasurementNoise = sigma_magnetometer^2 * eye(3);
    
    X = zeros(numSamples, length(initialStateGuess));
    P = zeros(numSamples, length(initialStateGuess), length(initialStateGuess));
    X(1, :) = initialStateGuess;
    P(1, :, :) = stateCovariance;
    
    
    % start from the second
    for i = 1 : numSamples - 1
        predict(obj, -accelerometerReadings(i, 1));
        correct(obj, magnetometerReadings(i + 1, :).', sensor_spacing);
        X(i + 1, :) = obj.State.';
        P(i + 1, :, :) = obj.StateCovariance;
    end
    
    XData{iter} = X;
    PData{iter} = P;
end
toc


%% Statistics
error_struct = calc_error(XData, PData, pos, vel, accBias, magBias, c, numSamples, N, opt);

%% Plot error in position/velocity estimation

figure;
plot(timeVector,ImuMag_data(1).MAG(:,1),'b.',...
     timeVector,ImuMag_data(1).MAG(:,2),'r.',...
     timeVector,ImuMag_data(1).MAG(:,3),'g.');
xlabel('time/s');
ylabel('mag/T');
title('magnetometer reading');
saveas(gcf, 'exp\magnetometer_reading.jpg')

figure;
hold on;

% position error plot
curve1 =  error_struct.stdPosErr;
curve2 =  zeros(size(error_struct.stdPosErr));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.msePosErr, 'r','LineWidth', 2, 'DisplayName','Upd');
xlabel('time/s');
ylabel('error/m');
title('position error');
legend('Location','northwest');
saveas(gcf,'exp\PosError.jpg');

% velocity error plot
figure;
hold on;
curve1 =  error_struct.stdVelErr;
curve2 =  zeros(size(error_struct.stdVelErr));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseVelErr, 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time/s');
ylabel('error/m.s^{-1}');
title('velocity error');
legend('Location','northwest');
saveas(gcf,'exp\VelError.jpg');

% acc bias error plot
if(opt.hasAccBias)
    figure;
    hold on;
    curve1 =  error_struct.stdAccBiasErr;
    curve2 =  zeros(size(error_struct.stdAccBiasErr));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseAccBiasErr, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('acc bias error');
    legend('Location','northwest');
    saveas(gcf, 'exp\accBiasError.jpg')
end

% mag bias error plot
if(opt.hasMagBias)
    figure;
    hold on;
    curve1 =  error_struct.stdMagBiasErr;
    curve2 =  zeros(size(error_struct.stdMagBiasErr));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseMagBiasErr, 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/\mu T');
    title('mag bias error');
    legend('Location','northwest');
    saveas(gcf, 'exp\magBiasError.jpg')
end

% coeff plot
figure;
hold on;
curve1 =  error_struct.stdC2Err;
curve2 =  zeros(size(error_struct.stdC2Err));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseC2Err, 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time/s');
title('C2 error');
legend('Location','northwest');
saveas(gcf, 'exp\C2Error.jpg')

figure;
hold on;
curve1 =  error_struct.stdC1Err;
curve2 =  zeros(size(error_struct.stdC2Err));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseC1Err, 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time/s');
title('C1 error');
legend('Location','northwest');
saveas(gcf, 'exp\C1Error.jpg')

figure;
hold on;
curve1 =  error_struct.stdC0Err;
curve2 =  zeros(size(error_struct.stdC2Err));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseC0Err, 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time/s');
title('C0 error');
legend('Location','northwest');
saveas(gcf, 'exp\C0Error.jpg')
