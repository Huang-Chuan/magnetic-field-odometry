function [numStates, masks] = getErrorStateMask()
    numStates = 45;

    pos = 1 : 3;
    vel = 4 : 6;
    epsilon = 7 : 9;
    acc_bias = 10 : 12;
    gyro_bias = 13 : 15;
    mag_bias = 16 : 30;
    theta = 31 : 45;
    
    
    masks.pos = false(45, 1);
    masks.vel = false(45, 1);
    masks.epsilon = false(45, 1);
    masks.acc_bias = false(45, 1);
    masks.gyro_bias = false(45, 1);
    masks.mag_bias = false(45, 1);
    masks.theta = false(45, 1);


    masks.pos(pos) = true;
    masks.vel(vel) = true;
    masks.epsilon(epsilon) = true;
    masks.acc_bias(acc_bias) = true;
    masks.gyro_bias(gyro_bias) = true;
    masks.mag_bias(mag_bias) = true;
    masks.theta(theta) = true;

end