function [Xs, ImuMag_data, x0] = data_gen(init_x0_m, stateCovariance, sensor_spacing, processNoise, ...
                                          sigma_magnetometer, accBody, qBody, omega,  N)


    x0 = init_x0_m;
    Xs = cell(N, 1);
    for iter = 1 : N
        Xs{iter} = zeros(size(accBody, 1), length(x0));
        accelerometerReadings = zeros(size(accBody));
        magnetometerReadings = zeros(size(accBody, 1), 18);

        x = mvnrnd(x0.', stateCovariance, 1).';
        Xs{iter}(1, :) = x.';
        
        y = MeasurementFcns(x, sensor_spacing) + sigma_magnetometer * randn(18, 1);
        magnetometerReadings(1, :) = y.';

        for i = 1 : size(accBody, 1) - 1
            % generate random noise according to processNoise
            vk = mvnrnd(zeros(1, size(processNoise, 1)), processNoise, 1);
            vk = vk.';
            R_nb = q2r(qBody(i, :));
            accelerometerReadings(i, :) = accBody(i, :) + (R_nb.' * [0; 0; -9.81]).' + vk(1 : 3).' + x(7:9).';
            x = StateTransitionFcns(x, vk, [accelerometerReadings(i, :).'; qBody(i, :).'; omega(i, :).']);
            Xs{iter}(i + 1, :) = x.';
            y = MeasurementFcns(x, sensor_spacing) + sigma_magnetometer * randn(18, 1);
            magnetometerReadings(i + 1, :) = y.';
        end

        ImuMag_data(iter).IMU = accelerometerReadings ;
        ImuMag_data(iter).MAG = magnetometerReadings;
    end
end