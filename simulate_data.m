% generate data format used by MAINS
close all;
clear all;
addpath('common/');
rng(1);
%%
N = 4;
settings = getSettings();
numSamples = settings.numSamples;
dT = settings.dT;
timeVector = 0:dT:(settings.duration-dT);

[position,orientation,velocity,acceleration,angularVelocity] = trajectory_gen(settings);
[ImuMag_data, ImuMag_bias, theta_cell, aux] = sensor_data_gen(settings, position, orientation, acceleration, angularVelocity, N);

for i = 1 : N
    data.name = ['exp', num2str(i)];
    
    data.gt.pos = position';
    data.gt.ori = quat2rotm(orientation);
    data.gt.rpys = rad2deg(quat2eul(orientation))';
    data.gt.numFrames = numSamples;
    data.gt.t = timeVector;

    data.u = ImuMag_data.IMU;
    data.mag_array.field = ImuMag_data.MAG;
    data.t = timeVector;
    save(fullfile("data", data.name), "data");
end
