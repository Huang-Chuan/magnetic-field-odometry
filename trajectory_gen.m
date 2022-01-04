function [position,orientation,velocity,acceleration,angularVelocity] = trajectory_gen(settings)

    numSamples = settings.numSamples;
    % designated accBody
    radius = settings.radius;   % meters
    speed = settings.speed;      % meters per second
    %climbRate = settings.climbRate;  % meters per second
    %initialYaw = settings.initialYaw; % degrees
    pitch = settings.pitch;      % degrees
    initPos = settings.init_pos;
    initVel = settings.init_vel;
    initOrientation = settings.init_orientation;

    trajectory = kinematicTrajectory('SampleRate',settings.fs, ...
        'Velocity',initVel, ...
        'Position',initPos, ...
        'Orientation',initOrientation);

    % original code    
    % accBody = zeros(settings.numSamples,3);
    % accBody(:,2) = speed^2/radius;
    % accBody(:,3) = 0.001;

    % omega = zeros(settings.numSamples,3);
    % omega(:, 1) = 0.05;
    % omega(:,3) = speed/radius;

    % pitchRotation = quaternion([0,pitch,0],'eulerd','zyx','frame');
    % omega = rotateframe(pitchRotation,omega);
    

    % dt = settings.dT;
    % q = repmat(quaternion([0 0.05 0] .* dt, 'rotvec'), settings.numSamples, 1);
    
    
    % for i = 2:numel(q)
    %     q(i) = q(i-1) .* q(i);
    % end

    % accBody = rotateframe(q,accBody);
    % accBody = rotateframe(pitchRotation,accBody);
    N_ = 1000;
    accBody = zeros(settings.numSamples,3);
    accBody(:,1) = [-0.1 * ones(N_, 1); zeros(numSamples - N_, 1)];
    accBody(:,2) = [zeros(N_, 1); speed^2/radius * ones(numSamples - N_, 1)];
    accBody(:,3) = [zeros(N_, 1); 0.001* ones(numSamples - N_, 1)];

    omega = zeros(settings.numSamples,3);
    omega(:, 1) = [2*pi / 10 * ones(N_, 1); zeros(numSamples - N_, 1)];
    omega(:,3) =  [zeros(N_, 1); -speed/radius * ones(numSamples - N_, 1)];


    pitchRotation = quaternion([0,pitch,0],'eulerd','zyx','frame');
    omega = rotateframe(pitchRotation,omega);

    
    accBody = rotateframe(pitchRotation,accBody);
    
    [position,orientation,velocity,acceleration,angularVelocity] = trajectory(accBody, omega);

end