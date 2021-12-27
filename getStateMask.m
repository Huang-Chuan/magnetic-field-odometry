function [numStates, masks] = getStateMask()
    
    numStates = 39;

    pos = 1 : 3;
    vel = 4 : 6;
    acc_bias = 7 : 9;
    mag_bias = 10 : 24;
    theta = 25 : 39;
    
    
    masks.pos = false(39, 1);
    masks.vel = false(39, 1);
    masks.acc_bias = false(39, 1);
    masks.mag_bias = false(39, 1);
    masks.theta = false(39, 1);


    masks.pos(pos) = true;
    masks.vel(vel) = true;
    masks.acc_bias(acc_bias) = true;
    masks.mag_bias(mag_bias) = true;
    masks.theta(theta) = true;

end