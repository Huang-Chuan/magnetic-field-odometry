function [position,orientation,velocity,acceleration,angularVelocity] = trajectory_gen(settings)

    numSamples = settings.numSamples;
    % designated accBody
    % radius = settings.radius;   % meters
    % speed = settings.speed;      % meters per second
    % %climbRate = settings.climbRate;  % meters per second
    % %initialYaw = settings.initialYaw; % degrees
    % pitch = settings.pitch;      % degrees
    % initPos = settings.init_pos;
    % initVel = settings.init_vel;
    % initOrientation = settings.init_orientation;

    % trajectory = kinematicTrajectory('SampleRate',settings.fs, ...
    %     'Velocity',initVel, ...
    %     'Position',initPos, ...
    %     'Orientation',initOrientation);


    omega = 0.3; 
    numSamples = 6000;

    ascend_acc = 5e-4;
    fs = 100;
    % first time will be discard
    t = (0 : numSamples).' * 1/fs;

    x = sin(omega * t) ;
    y = cos(omega * t) ;
    z = 1/2 * ascend_acc * t.^2 ;

    waypoints = [x y z];

    %plot3(x, y, z)

    roll_rate = 0.2/pi*180;
    yaw_rate = 0.2/pi*180;
    orientation_ = quaternion([roll_rate * t zeros(size(t)) yaw_rate * t], ...
                            'eulerd','ZYX','frame');

    trajectory = waypointTrajectory(waypoints, ...
        'TimeOfArrival', t, ...
        'Orientation',orientation_, ...
        'SampleRate',fs);

    count = 1;
    while ~isDone(trajectory)
        [position(count,:), orientation(count,:), velocity(count,:),acceleration(count,:),angularVelocity(count,:)] = trajectory();
        count = count + 1;
    end






    % N_ = 1000;
    % accBody = zeros(settings.numSamples,3);
    % accBody(:,1) = [-0.1 * ones(N_, 1); zeros(numSamples - N_, 1)];
    % accBody(:,2) = [zeros(N_, 1); speed^2/radius * ones(numSamples - N_, 1)];
    % accBody(:,3) = [zeros(N_, 1); 0.001* ones(numSamples - N_, 1)];

    % omega = zeros(settings.numSamples,3);
    % omega(:, 1) = [2*pi / 10 * ones(N_, 1); zeros(numSamples - N_, 1)];
    % omega(:,3) =  [zeros(N_, 1); -speed/radius * ones(numSamples - N_, 1)];


    % pitchRotation = quaternion([0,pitch,0],'eulerd','zyx','frame');
    % omega = rotateframe(pitchRotation,omega);

    
    % accBody = rotateframe(pitchRotation,accBody);
    
    % [position,orientation,velocity,acceleration,angularVelocity] = trajectory(accBody, omega);

end