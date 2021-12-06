function [Xs, ImuMag_data, x0] = data_gen(opt, stateCovariance, sensor_spacing, processNoise, sigma_magnetometer, accBody, stateTransitionFcn, measurementFcn, N)
    init_pos = 0.3;
    init_vel = 1;
    init_accBias = 0;
    init_magBias = [0; 0; 0];
    init_coeff   = [1; 0; -50];

    x0 = [init_pos; init_vel; init_accBias; init_magBias; init_coeff];
    if (~opt.hasMagBias) && (~opt.hasAccBias) 
        %processNoise = diag([processNoise(1 : 3),  processNoise(end - 2 : end)]);
        %stateCovariance = diag([stateCovariance(1 : 2),  stateCovariance(end - 2 : end)]);
        %x0 = mvnrnd([x0(1:2); x0(end - 2 : end)].', stateCovariance, 1);
        x0 = [x0(1:2); x0(end - 2 : end)];
        %x0 = x0.';
    elseif(~opt.hasMagBias) && (opt.hasAccBias) 
        %processNoise = diag([processNoise(1 : 4),  processNoise(end - 2 : end)]);
        %stateCovariance = diag([stateCovariance(1 : 3),  stateCovariance(end - 2 : end)]);
        %x0 = mvnrnd([x0(1 : 3); x0(end - 2 : end)].', stateCovariance, 1);
        x0 = [x0(1 : 3); x0(end - 2 : end)];
        %x0 = x0.';
    elseif(opt.hasMagBias) && (~opt.hasAccBias) 
        %processNoise = diag([processNoise(1 : 3), processNoise(5 : 7), processNoise(end - 2 : end)]);
        %stateCovariance = diag([stateCovariance(1 : 2),  stateCovariance(5 : 7), stateCovariance(end - 2 : end)]);
        %x0 = mvnrnd([x0(1 : 2); x0(5 : 7); x0(end - 2 : end)].', stateCovariance, 1);
        x0 = [x0(1 : 2); x0(5 : 7); x0(end - 2 : end)];
        %x0 = x0.';
    else
        %processNoise = diag(processNoise);
        %stateCovariance = diag(stateCovariance);
        x0 = x0;
        %x0 = x0.';
    end

    Xs = cell(N, 1);
    for iter = 1 : N
        Xs{iter} = zeros(size(accBody, 1), length(x0));
        accelerometerReadings = zeros(size(accBody));
        magnetometerReadings = zeros(size(accBody));

        x = mvnrnd(x0.', stateCovariance, 1).';
        Xs{iter}(1, :) = x.';
        
        y = measurementFcn(x, sensor_spacing) + sigma_magnetometer * randn(3, 1);
        magnetometerReadings(1, :) = y.';

        for i = 1 : size(accBody, 1) - 1
            % generate random noise according to processNoise
            vk = mvnrnd(zeros(1, size(processNoise, 1)), processNoise, 1);
            vk = vk.';
            if(opt.hasAccBias)
                accelerometerReadings(i, 1) = accBody(i, 1) + vk(1) + x(3);
            else
                accelerometerReadings(i, 1) = accBody(i, 1) + vk(1);
            end  
            x = stateTransitionFcn(x, vk, accelerometerReadings(i, 1));
            Xs{iter}(i + 1, :) = x.';
            y = measurementFcn(x, sensor_spacing) + sigma_magnetometer * randn(3, 1);
            magnetometerReadings(i + 1, :) = y.';
        end

        ImuMag_data(iter).IMU = accelerometerReadings;
        ImuMag_data(iter).MAG = magnetometerReadings;
    end
end