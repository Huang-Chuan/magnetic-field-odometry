function [numStates, masks] = getStateMask()
    numStates = 46;

    pos = 1 : 3;
    vel = 4 : 6;
    q_nb = 7 : 10;
    acc_bias = 11 : 13;
    gyro_bias = 14 : 16;
    mag_bias = 17 : 31;
    theta = 32 : 46;
    
    
    masks.pos = false(46, 1);
    masks.vel = false(46, 1);
    masks.q_nb = false(46, 1);
    masks.acc_bias = false(46, 1);
    masks.gyro_bias = false(46, 1);
    masks.mag_bias = false(46, 1);
    masks.theta = false(46, 1);


    masks.pos(pos) = true;
    masks.vel(vel) = true;
    masks.q_nb(q_nb) = true;
    masks.acc_bias(acc_bias) = true;
    masks.gyro_bias(gyro_bias) = true;
    masks.mag_bias(mag_bias) = true;
    masks.theta(theta) = true;

end