function [position,orientation,velocity,acceleration,angularVelocity] = trajectory_gen(settings)
% generate groundtruth of the trajectory and IMU measurement

    numSamples = settings.numSamples;
    fs = settings.fs;
    % the first point will be discard
    t = (0 : numSamples - 1).' * 1/fs;
    
    % create the trajectory in paper
    ascend_acc = 5e-4;
    omega = 0.3; 
    roll_rate = 0.2/pi*180;
    pitch_rate = 0.2/pi*180;
    yaw_rate = 0.2/pi*180;
    
    x = sin(omega * t) ;
    vx = omega * cos(omega * t);
    y = cos(omega * t) ;
    vy = -omega * sin(omega * t);
    z = 1/2 * ascend_acc * t.^2 ;
    vz = ascend_acc * t;

    position = [x y z];
    velocity = [vx vy vz];

    orientation = quaternion([roll_rate * t pitch_rate * t yaw_rate * t], ...
                             'eulerd','ZYX','frame');
    acceleration = [-omega^2 * sin(omega * t)   -omega^2 * cos(omega * t)  ascend_acc * ones(numSamples, 1)];
    
    for i = 1 : numSamples - 1
        dq = quatmultiply(quatconj(compact(orientation(i))), compact(orientation(i + 1)));
        angularVelocity(i, :) = rotatepoint(orientation(i), 2 * dq(end-2:end) * fs);
    end
    angularVelocity(numSamples, :) = rotatepoint(orientation(i), 2 * dq(end-2:end) * fs);


    % overwrite position velocity and orientation using navigation equation
    x0 = [position(1,:) velocity(1,:) compact(orientation(1,:)) settings.init_acc_bias settings.init_gyro_bias settings.init_mag_bias settings.init_coeff].'; 
    x = x0;
    for i = 1 : numSamples - 1
        accelerometerReadings = rotateframe(orientation(i), acceleration(i, :) - settings.g');
        gyroReadings = rotateframe(orientation(i), angularVelocity(i, :));
        [x, ~, ~] = Nav_eq(x, [accelerometerReadings.'; gyroReadings.'], settings.dT, settings.Q, settings);
        position(i + 1, :) = x(1:3);
        velocity(i + 1, :) = x(4:6);
        orientation(i + 1, :) = quaternion(x(7:10).');
    end
end

