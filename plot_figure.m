settings = getSettings();
timeVector = 0:1/settings.fs:(settings.duration-1/settings.fs);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%             Magnetic field              %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
tmp = load(settings.model);

m.moments = tmp.mm(:, 1:end-2);
m.pos_dipoles = tmp.MM;
m.f_earth = tmp.mm(:, end-1);
clear tmp
[Xq,Yq,Zq]=meshgrid(-1:0.4:1,-1:0.4:1,0:0.2:1);
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




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%             trajectory plot             %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
saveas(gcf, 'figures/groundtruth', 'epsc');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               position plot             %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
subplot(3,1,1)
hold on;
a = area(timeVector, error_struct.stdPosErr(:, 1));
a.FaceColor=[.5 .5 .5];
a.FaceAlpha = 0.3;
set(gca, 'YScale', 'log')
plot(timeVector, error_struct.msePosErr(:, 1), 'r','LineWidth', 2);
plot(timeVector, error_struct_1.msePosErr(:, 1), 'k--','LineWidth', 2);
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m]','FontSize',12,'FontName','Times New Roman')
title('position error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on
subplot(3,1,2)
hold on;
a = area(timeVector, error_struct.stdPosErr(:, 2));
a.FaceColor=[.5 .5 .5];
a.FaceAlpha = 0.3;
set(gca, 'YScale', 'log')
plot(timeVector, error_struct.msePosErr(:, 2), 'r','LineWidth', 2);
plot(timeVector, error_struct_1.msePosErr(:, 2), 'k--','LineWidth', 2);
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m]','FontSize',12,'FontName','Times New Roman')
title('position error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on
subplot(3,1,3)
hold on;
a = area(timeVector, error_struct.stdPosErr(:, 3));
a.FaceColor=[.5 .5 .5];
a.FaceAlpha = 0.3;
set(gca, 'YScale', 'log')
plot(timeVector, error_struct.msePosErr(:, 3), 'r','LineWidth', 2);
plot(timeVector, error_struct_1.msePosErr(:, 3), 'k--','LineWidth', 2);
xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('error [m]','FontSize',12,'FontName','Times New Roman')
title('position error x-axis','FontSize',12,'FontName','Times New Roman')
grid minor;
box on

print('figures/position', '-depsc', '-r600', '-painters' );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%             velocity plot               %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%             orientation plot            %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               acc bias plot             %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               mag bias plot             %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%               gyro bias plot            %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        coefficient      plot            %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%          ANEES      plot                %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lowerBound = (1 - 2 / (9 * 3e4) - 2.576 * sqrt(2 / (9 * 3e4)))^3;
upperBound = (1 - 2 / (9 * 3e4) + 2.576 * sqrt(2 / (9 * 3e4)))^3;
figure;
plot(timeVector(2:end), NEES_metric/30, 'r', 'LineWidth', 2, 'DisplayName','ANEES');
hold on;
curve1 =  lowerBound * ones(size(timeVector(2:end))).';
curve2 =  upperBound * ones(size(timeVector(2:end))).';
inBetween = [curve1; flipud(curve2)];
x = [timeVector(2:end) fliplr(timeVector(2:end))];
h = fill(x, inBetween, [.5 .5 .5], 'DisplayName','0.99 confidence interval');
h.FaceAlpha = 0.5;

xlabel('time [s]','FontSize',12,'FontName','Times New Roman')
ylabel('value','FontSize',12,'FontName','Times New Roman');
title('Average normalized estimation error squared (ANEES) ','FontSize',12,'FontName','Times New Roman')
grid minor;
box on
legend;
print('figures/ANEES', '-depsc', '-r600', '-painters' );