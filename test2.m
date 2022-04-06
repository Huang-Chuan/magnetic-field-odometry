close all;
clear all;
addpath('common/');
rng(1);
%%
N = 1e3;
settings = getSettings();
save('settings.mat', 'settings');
numStates = settings.numStates;
stateMasks = settings.stateMask;
numErrorStates = settings.numErrorStates;
errorStateMasks = settings.errorStateMask;


numSamples = settings.numSamples;
dT = settings.dT;
P0 = settings.P; 
fs = 100;
dT = 1 / fs;
timeVector = 0:1/fs:(settings.duration-1/fs);

[position,orientation,velocity,acceleration,angularVelocity] = trajectory_gen_new(settings);


figure;
plot3(position(:, 1), position(:, 2), position(:, 3), 'k');
hold on;
plot3(position(end, 1), position(end, 2), position(end, 3), '.', 'markerSize', 15);
title("Trajectory",'FontSize',12,'FontName','Times New Roman');
for n=1:100:size(position, 1)
    u = rotatepoint(orientation(n), [0.1 0 0]);
    quiver3(position(n,1),position(n,2),position(n,3),u(1),u(2),u(3),'r');
    u = rotatepoint(orientation(n), [0 0.1 0]);
    quiver3(position(n,1),position(n,2),position(n,3),u(1),u(2),u(3),'b')
    u = rotatepoint(orientation(n), [0 0 0.1]);
    quiver3(position(n,1),position(n,2),position(n,3),u(1),u(2),u(3),'g')
end
axis equal;
grid minor;
xlabel('x [m]','FontSize',12,'FontName','Times New Roman')
ylabel('y [m]','FontSize',12,'FontName','Times New Roman')
zlabel('z [m]','FontSize',12,'FontName','Times New Roman')
saveas(gcf, 'groundtruth', 'epsc');


A = settings.A;
%%
[ImuMag_data, ImuMag_bias, theta_cell, aux] = sensor_data_gen(settings, position, orientation, acceleration, angularVelocity, N);
% figure;
% for i = 1 : 15
%     subplot(3, 5, i)
%     plot(aux(:, i));
% end
% saveas(gcf, 'processNoise', 'jpg');

%%
% create gt
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

residual = zeros(3, numSamples);
tic
parfor iter = 1 : N
    if(mod(iter, 10) == 1)
        fprintf('%d iter\n', iter);
    end
    
    magnetometerReadings = ImuMag_data(iter).MAG;
    accelerometerReadings = ImuMag_data(iter).IMU(:, 1:3);
    gyroReadings = ImuMag_data(iter).IMU(:, 4:6);
    
    % get initial value for polynomial coefficient
    coeff = inv(A.'*A) * A.'*magnetometerReadings(1, :).';
 

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
            [xh, F, Q] =  Nav_eq_new(x, u, dT, processNoiseCov, settings);
            % if (iter == 1) && (mod(i, 500) == 1)
            %     r = PosMagArray();
            %     s = rotatepoint(orientation(i + 1), r.') + position(i + 1, :);
            %     XX = reshape(s(:, 1), 6, 5);
            %     YY = reshape(s(:, 2), 6, 5);
            %     ZZ = reshape(s(:, 3), 6, 5);
            %     [MPredX, MPredY, MPredZ] = predict_field(xh(stateMasks.theta), position(i + 1, :), orientation(i + 1), XX, YY, ZZ);
            %     [MX, MY, MZ] = get_actual_field('model.mat',  XX, YY, ZZ);
            %     diffMX = MPredX - MX;
            %     diffMY = MPredY - MY;
            %     diffMZ = MPredZ - MZ;
            %     figure('visible','off');
            %     title(sprintf('time %d', i));
            %     subplot(3,3,1);
            %     surf(MPredX);
            %     subplot(3,3,2);
            %     surf(MPredY);
            %     subplot(3,3,3);
            %     surf(MPredZ);
            %     subplot(3,3,4);
            %     surf(MX);
            %     subplot(3,3,5);
            %     surf(MY);
            %     subplot(3,3,6);
            %     surf(MZ);
            %     subplot(3,3,7);
            %     surf(diffMX);
            %     subplot(3,3,8);
            %     surf(diffMY);
            %     subplot(3,3,9);
            %     surf(diffMZ);
            %     saveas(gcf,sprintf('res/Pred_time_%d', i),'jpg');
            % end
            % cov propagation
            P = F * P * F' + Q;
            H = settings.H;
            
            if i < 2000
                H = [settings.H; [eye(3) zeros(3, numErrorStates - 3)]];
                measurementNoiseCov_ =  blkdiag(measurementNoiseCov, 0.01^2*eye(3));
                K = P * H' / (H * P * H' + measurementNoiseCov_);
                delta_z = [magnetometerReadings(i + 1, :).' - (A * xh(stateMasks.theta) + [xh(stateMasks.mag_bias); zeros(3, 1)]);...
                           (position(i+1, :).'+ 0.01*randn(3,1))-xh(stateMasks.pos)];
                P = (eye(size(P)) - K * H) * P * (eye(size(P)) - K * H)' + K * measurementNoiseCov_ * K';
                delta_x = K * delta_z;

            else
                K = P * H' / (H * P * H' + measurementNoiseCov);   
                delta_z = magnetometerReadings(i + 1, :).' - (A * xh(stateMasks.theta) + [xh(stateMasks.mag_bias); zeros(3, 1)]);
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
    end
    
    XData{iter} = X;
    PData{iter} = Ps;
end
toc


%
error_struct = calc_error_2_new(XData, PData, xs, numSamples, N, settings);
save('error_struct.mat', 'error_struct');


%% run filter without magnetometer aiding
parfor iter = 1 : N
    if(mod(iter, 10) == 1)
        fprintf('%d iter\n', iter);
    end
    
    magnetometerReadings = ImuMag_data(iter).MAG;
    accelerometerReadings = ImuMag_data(iter).IMU(:, 1:3);
    gyroReadings = ImuMag_data(iter).IMU(:, 4:6);
      

    x = XData{iter}(2000, :).';
    %P = diag(PData{iter}(2000, :));
    for i = 2000 : numSamples - 1
            acc_m = accelerometerReadings(i, :)';
            omega_m = gyroReadings(i, :)';
            u = [acc_m; omega_m];
            [xh, F, Q] =  Nav_eq_new(x, u, dT, processNoiseCov, settings);
            x = xh;
            XData{iter}(i + 1, :) = x.';
            %P = F * P * F' + Q;
    end
end

error_struct = calc_error_2_new(XData, PData, xs, numSamples, N, settings);
save('error_struct1.mat', 'error_struct');

%%
%plot_figure
%%
figure;
plot3(position(:, 1), position(:, 2), position(:, 3), 'b');
hold on;
plot3(position(end, 1), position(end, 2), position(end, 3), '.', 'markerSize', 15);

for n=1:100:size(position, 1)
    u = rotatepoint(orientation(n), [0.1 0 0]);
    quiver3(position(n,1),position(n,2),position(n,3),u(1),u(2),u(3),'r');
    u = rotatepoint(orientation(n), [0 0.1 0]);
    quiver3(position(n,1),position(n,2),position(n,3),u(1),u(2),u(3),'b')
    u = rotatepoint(orientation(n), [0 0 0.1]);
    quiver3(position(n,1),position(n,2),position(n,3),u(1),u(2),u(3),'g')
end
axis equal;
grid minor;
xlabel('x [m]','FontSize',12,'FontName','Times New Roman')
ylabel('y [m]','FontSize',12,'FontName','Times New Roman')
zlabel('z [m]','FontSize',12,'FontName','Times New Roman')
title('The pose of body frame and trajectory','FontSize',12,'FontName','Times New Roman')
print('pose', '-depsc', '-r600', '-painters' );

figure;
tmp = load(settings.model);

m.moments = tmp.mm(:, 1:end-2);
m.pos_dipoles = tmp.MM;
m.f_earth = tmp.mm(:, end-1);
clear tmp
[Xq,Yq,Zq]=meshgrid(-1:0.2:1,-1:0.2:1,0:0.1:1);
m.pos_grid=[reshape(Xq,1,numel(Xq)); reshape(Yq,1,numel(Yq)); reshape(Zq,1,numel(Zq))];

% Allocate memory
m.f_grid=m.f_earth*ones(1,size(m.pos_grid,2));

% Location loop
for ii=1:size(m.pos_grid,2)
    % Dipole loop
    for kk=1:size(m.moments,2)
        m.f_grid(:,ii)= m.f_grid(:,ii)+dipole(m.pos_grid(:,ii),m.pos_dipoles(:,kk),m.moments(:,kk));
    end
end

quiverC3D(m.pos_grid(1,:)',m.pos_grid(2,:)',m.pos_grid(3,:)',m.f_grid(1,:)',m.f_grid(2,:)',m.f_grid(3,:)','Colorbar',true,'LineWidth',0.5);

title('Generated magnetic-field','FontSize',12,'FontName','Times New Roman')
xlabel('x [m]','FontSize',12,'FontName','Times New Roman')
ylabel('y [m]','FontSize',12,'FontName','Times New Roman')
zlabel('z [m]','FontSize',12,'FontName','Times New Roman')
axis tight;

print('magnetic-field', '-depsc', '-r600', '-painters' );