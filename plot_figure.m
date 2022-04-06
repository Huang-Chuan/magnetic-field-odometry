% dir = 'result';
% load(fullfile(dir, 'error_struct.mat'));
% load(fullfile(dir, 'settings.mat'));
settings = getSettings();


timeVector = 0:1/settings.fs:(settings.duration-1/settings.fs);

figure;
subplot(3,1,1)
hold on;
curve1 =  error_struct.stdPosErr(:, 1);
curve2 =  zeros(size(error_struct.stdPosErr(:, 1)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma Upd');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.msePosErr(:, 1), 'r','LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m]','FontSize',12,'FontName','Times New Roman')
title('Position error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,2)
hold on;
curve1 =  error_struct.stdPosErr(:, 2);
curve2 =  zeros(size(error_struct.stdPosErr(:, 2)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma Upd');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.msePosErr(:, 2), 'r','LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m]','FontSize',12,'FontName','Times New Roman')
title('Position error y-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,3)
hold on;
curve1 =  error_struct.stdPosErr(:, 3);
curve2 =  zeros(size(error_struct.stdPosErr(:, 3)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma Upd');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.msePosErr(:, 3), 'r','LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m]','FontSize',12,'FontName','Times New Roman')
title('Position error z-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

print('figures/position', '-depsc', '-r600', '-painters' );


% % velocity error plot
figure;
subplot(3,1,1)
hold on;
curve1 =  error_struct.stdVelErr(:, 1);
curve2 =  zeros(size(error_struct.stdVelErr(:, 1)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseVelErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m/s]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Velocity error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,2)
hold on;
curve1 =  error_struct.stdVelErr(:, 2);
curve2 =  zeros(size(error_struct.stdVelErr(:, 2)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseVelErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m/s]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Velocity error y-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,3)
hold on;
curve1 =  error_struct.stdVelErr(:, 3);
curve2 =  zeros(size(error_struct.stdVelErr(:, 3)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseVelErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m/s]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Velocity error z-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on
print('figures/velocity', '-depsc', '-r600', '-painters' );

% orientation error
figure;
subplot(3,1,1)
hold on;
curve1 =  error_struct.stdQErr(:, 1) * 180 / pi;
curve2 =  zeros(size(error_struct.stdQErr(:, 1)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma Upd');
h.FaceAlpha = 0.8;
plot(timeVector, 2 * error_struct.mseQErr(:, 1)  * 180 / pi, 'r','LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$^{\circ}$]','FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Orientation error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on
%

subplot(3,1,2)
hold on;
curve1 =  error_struct.stdQErr(:, 2) * 180 / pi;
curve2 =  zeros(size(error_struct.stdQErr(:, 2)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma Upd');
h.FaceAlpha = 0.8;
plot(timeVector, 2 * error_struct.mseQErr(:, 2)  * 180 / pi, 'r','LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$^{\circ}$]','FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Orientation error y-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on



subplot(3,1,3)
hold on;
curve1 =  error_struct.stdQErr(:, 3)* 180 / pi;
curve2 =  zeros(size(error_struct.stdQErr(:, 3)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma Upd');
h.FaceAlpha = 0.8;
plot(timeVector, 2 * error_struct.mseQErr(:, 3)* 180 / pi, 'r','LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$^{\circ}$]','FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Orientation error z-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

print('figures/orientation', '-depsc', '-r600', '-painters' );

% % acc bias error plot
figure;
subplot(3,1,1)
hold on;
curve1 =  error_struct.stdAccBiasErr(:, 1);
curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 1)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseAccBiasErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m $s^{-2}$]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Acceleration bias error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,2)
hold on;
curve1 =  error_struct.stdAccBiasErr(:, 2);
curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 2)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseAccBiasErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m $s^{-2}$]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Acceleration bias error y-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,3)
hold on;
curve1 =  error_struct.stdAccBiasErr(:, 3);
curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 3)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseAccBiasErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m $s^{-2}$]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Acceleration bias error z-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

print('figures/accbias', '-depsc', '-r600', '-painters' );

figure;
subplot(3,1,1)
hold on;
curve1 =  error_struct.stdMagBiasErr(:, 1);
curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 1)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseMagBiasErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$\mu T$]','FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('magnetometer bias error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,2)
hold on;
curve1 =  error_struct.stdMagBiasErr(:, 2);
curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 2)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseMagBiasErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$\mu T$]','FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('magnetometer bias error y-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,3)
hold on;
curve1 =  error_struct.stdMagBiasErr(:, 3);
curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 3)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseMagBiasErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$\mu T$]','FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('magnetometer bias error z-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

print('figures/magbias', '-depsc', '-r600', '-painters' );

% gyro bias
figure;
subplot(3,1,1)
hold on;
curve1 =  error_struct.stdGyroErr(:, 1) * 180 / pi;
curve2 =  zeros(size(error_struct.stdGyroErr(:, 1)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseGyroErr(:, 1)* 180 / pi, 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$^{\circ}/s$]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Gyroscope bias error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

subplot(3,1,2)
hold on;
curve1 =  error_struct.stdGyroErr(:, 2)* 180 / pi;
curve2 =  zeros(size(error_struct.stdGyroErr(:, 2)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseGyroErr(:, 2)* 180 / pi, 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$^{\circ}/s$]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Gyroscope bias error y-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on


subplot(3,1,3)
hold on;
curve1 =  error_struct.stdGyroErr(:, 3)* 180 / pi;
curve2 =  zeros(size(error_struct.stdGyroErr(:, 3)));
inBetween = [curve1; flipud(curve2)];
x = [timeVector fliplr(timeVector)];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','\sigma');
h.FaceAlpha = 0.8;
plot(timeVector, error_struct.mseGyroErr(:, 3)* 180 / pi, 'r', 'LineWidth', 2, 'DisplayName','Upd');
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [$^{\circ}/s$]', 'FontSize',12,'FontName','Times New Roman', 'interpreter','latex');
title('Gyroscope bias error z-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

print('figures/gyrobias', '-depsc', '-r600', '-painters' );



% coefficient plot
f=figure;
t = tiledlayout(3, 5);
for i = 1 : 15
    nexttile,
    hold on;
    a = area(timeVector, error_struct.stdThetaErr(:, i));
    a.FaceColor=[.5 .5 .5];
    a.EdgeColor='none';
    a.FaceAlpha = 0.3;
    
    plot(timeVector, error_struct.mseThetaErr(:, i), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    %
    set(gca, 'YScale', 'log')
    ylim([1e-3, 1])
    
   
    
    title(strcat('$\theta_{', num2str(i), '}$'),'FontSize',12,'FontName','Times New Roman','interpreter','latex');
    grid minor;
    
    box on
end
h  = axes(f, 'visible', 'off');
title(t, 'Coefficient error');
xlabel(t,'time [s]','FontSize',12,'FontName','Times New Roman')
ylabel(t, 'error','FontSize',12,'FontName','Times New Roman')
print('figures/coefferr', '-depsc', '-r600', '-painters' );





