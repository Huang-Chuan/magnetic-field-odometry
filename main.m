close all;
clear all;
addpath('common/');
rng(1);
%%
N = 4;
settings = getSettings();
save('settings.mat', 'settings');
numStates = settings.numStates;
stateMasks = settings.stateMask;
numErrorStates = settings.numErrorStates;
errorStateMasks = settings.errorStateMask;


numSamples = settings.numSamples;
dT = settings.dT;
P0 = settings.P; 

timeVector = 0:dT:(settings.duration-dT);

[position,orientation,velocity,acceleration,angularVelocity] = trajectory_gen(settings);
[ImuMag_data, ImuMag_bias, theta_cell, aux] = sensor_data_gen(settings, position, orientation, acceleration, angularVelocity, N);

% create groundtruth 
xs = cell(N, 1);
for i = 1 : N
    xs{i} = [position velocity compact(orientation) ImuMag_bias(i).IMU ImuMag_bias(i).MAG theta_cell{i}];
end


XData = cell(N, 1);
PData = cell(N, 1);

t = now;
d = datetime(t,'ConvertFrom','datenum');

processNoiseCov =  settings.Q ;
measurementNoiseCov = settings.R;

NEES_metric = zeros(N, numSamples - 1);
Phi = settings.Phi;
tic
for iter = 1 : N
    if(mod(iter, 10) == 1)
        fprintf('%d iter\n', iter);
    end
    
    magnetometerReadings = ImuMag_data(iter).MAG;
    accelerometerReadings = ImuMag_data(iter).IMU(:, 1:3);
    gyroReadings = ImuMag_data(iter).IMU(:, 4:6);
    
    % get initial value for polynomial coefficient
    coeff = inv(Phi.'*Phi) * Phi.'*magnetometerReadings(1, :).';

    x0 = [position(1,:) velocity(1,:) compact(orientation(1,:)) settings.init_acc_bias settings.init_gyro_bias settings.init_mag_bias coeff.'].'; 
    X = zeros(numSamples, numStates);
    Ps = zeros(numSamples, numStates - 1);
    
    X(1, :) = x0;
    Ps(1, :) = diag(P0);

    x = x0;
    P = P0;
    for i = 1 : numSamples - 1
            acc_m = accelerometerReadings(i, :)';
            omega_m = gyroReadings(i, :)';
            u = [acc_m; omega_m];
            [xh, F, Q] =  Nav_eq(x, u, dT, processNoiseCov, settings);
            % cov propagation
            P = F * P * F' + Q;
            H = settings.H;
            
            % first 20s: both position and magnetic field measurements are available
            % next  40s: only magnetic field measurements are available
            if i < 2000
                H = [settings.H; [eye(3) zeros(3, numErrorStates - 3)]];
                measurementNoiseCov_ =  blkdiag(measurementNoiseCov, 0.01^2*eye(3));
                K = P * H' / (H * P * H' + measurementNoiseCov_);
                delta_z = [magnetometerReadings(i + 1, :).' - (Phi * xh(stateMasks.theta) + [xh(stateMasks.mag_bias); zeros(3, 1)]);...
                           (position(i+1, :).'+ 0.01*randn(3,1))-xh(stateMasks.pos)];
                P = (eye(size(P)) - K * H) * P * (eye(size(P)) - K * H)' + K * measurementNoiseCov_ * K';
                delta_x = K * delta_z;
            else
                K = P * H' / (H * P * H' + measurementNoiseCov);   
                delta_z = magnetometerReadings(i + 1, :).' - (Phi * xh(stateMasks.theta) + [xh(stateMasks.mag_bias); zeros(3, 1)]);
                P = (eye(size(P)) - K * H) * P * (eye(size(P)) - K * H)' + K * measurementNoiseCov * K';
                delta_x = K * delta_z;
            end

            xh(stateMasks.pos) = xh(stateMasks.pos) +  delta_x(errorStateMasks.pos);
            xh(stateMasks.vel) = xh(stateMasks.vel) +  delta_x(errorStateMasks.vel);
            xh(stateMasks.q_nb) = quatmultiply(xh(stateMasks.q_nb).',  [1 1/2*delta_x(7:9).']);
            xh(stateMasks.q_nb) = xh(stateMasks.q_nb).' / norm(xh(stateMasks.q_nb)); 
            xh(stateMasks.acc_bias) = xh(stateMasks.acc_bias) +  delta_x(errorStateMasks.acc_bias);
            xh(stateMasks.gyro_bias) = xh(stateMasks.gyro_bias) +  delta_x(errorStateMasks.gyro_bias);
            xh(stateMasks.mag_bias) = xh(stateMasks.mag_bias) +  delta_x(errorStateMasks.mag_bias);
            xh(stateMasks.theta) = xh(stateMasks.theta) +  delta_x(errorStateMasks.theta);
             
            x = xh;
            X(i + 1, :) = x;
            Ps(i + 1, :) = diag(P);
            G = blkdiag(eye(6), eye(3) - 1 / 2 * vect2skew(delta_x(7:9)) ,eye(3 + 3 + (settings.numSensors - 1) * 3 + 15));
            P = G * P * G.';
            
            est_error = xh- xs{iter}(i + 1, :).';
            est_error = est_error(~stateMasks.mag_bias);
            est_error = est_error([1:6, 8:length(est_error)]);
            NEES_metric(iter, i) = est_error.' * (P(~errorStateMasks.mag_bias, ~errorStateMasks.mag_bias) \ est_error);
    end
    
    XData{iter} = X;
    PData{iter} = Ps;
end
toc

NEES_metric = mean(NEES_metric);
save('NEES_metric.mat', 'NEES_metric');

%
error_struct = calc_error(XData, PData, xs, numSamples, N, settings);
save('error_struct.mat', 'error_struct');


% run filter without magnetometer aiding
parfor iter = 1 : N
    if(mod(iter, 10) == 1)
        fprintf('%d iter\n', iter);
    end
    
    magnetometerReadings = ImuMag_data(iter).MAG;
    accelerometerReadings = ImuMag_data(iter).IMU(:, 1:3);
    gyroReadings = ImuMag_data(iter).IMU(:, 4:6);
      

    x = XData{iter}(2000, :).';
    for i = 2000 : numSamples - 1
            acc_m = accelerometerReadings(i, :)';
            omega_m = gyroReadings(i, :)';
            u = [acc_m; omega_m];
            [xh, F, Q] =  Nav_eq(x, u, dT, processNoiseCov, settings);
            x = xh;
            XData{iter}(i + 1, :) = x.';
    end
end

error_struct_1 = calc_error(XData, PData, xs, numSamples, N, settings);
save('error_struct_1.mat', 'error_struct_1');


plot_figure;
