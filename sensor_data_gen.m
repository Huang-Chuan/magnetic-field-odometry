function [ImuMag_data, ImuMag_bias, theta_cell, aux] = sensor_data_gen(settings, position, orientation, acceleration, angularVelocity, N)
    % OUTPUT:
    % theta_cell: N x 1 cell, each cell contatins theta matrix of dimension
    % numSamples * dim(coeff)  
    numSamples = settings.numSamples;
    numSensors = settings.numSensors;
    invA = inv([calcAB([0,0,1]); calcAB([0, 1, 0]); calcAB([1, 0, 0]); calcAB([1, 1, 1]); calcAB([0, 0, 0])]);
    sensor_locs = settings.sensor_locs;
    A = settings.A;
    %theta_data = zeros(15, settings.numSamples, N);

    ImuMag_data = repmat(struct('IMU', [], 'MAG', []), N, 1);
    ImuMag_bias = repmat(struct('IMU', [], 'MAG', []), N, 1);


    theta_cell = cell(N, 1);

    m_field = 0;

    if settings.generate_from_model 
        for iter = 1 : N
            theta = zeros(15, numSamples);
            theta(:, 1) = (settings.init_coeff + settings.init_sigma_coeff * randn(size(settings.init_coeff))).';
            for i = 2 : numSamples 
                % propagate theta to next time step
                theta(:, i) = propogate_theta(settings, theta(:, i - 1), invA, ...
                                                position(i, :) - position(i - 1, :), orientation(i - 1, :), angularVelocity(i - 1, :));
            end
            theta_cell{iter} = theta;
        end
    else
        tmp = load(settings.model);
        %N_poles = 50;
        m.moments = tmp.mm(:, 1:end-2);
        m.pos_dipoles = tmp.MM;
        m.f_earth = tmp.mm(:, end-1);
        clear tmp
        
        m_field = repmat(m.f_earth.', numSamples, numSensors);
        parfor ii = 1 : size(position, 1)
            R = position(ii, :) + rotatepoint(orientation(ii), sensor_locs.');
            mag_measurements = m_field(ii, :);
            % sensor loop
            for jj = 1 : numSensors
                for kk = 1 : size(m.pos_dipoles, 2)
                    mag_measurements((jj - 1) * 3 + 1: jj * 3) = mag_measurements((jj - 1) * 3 + 1: jj * 3) + dipole(R(jj, :).', m.pos_dipoles(:,kk),m.moments(:,kk)).';
                end
                mag_measurements((jj - 1) * 3 + 1: jj * 3) = rotateframe(orientation(ii), mag_measurements((jj - 1) * 3 + 1: jj * 3));
            end
            m_field(ii, :) = mag_measurements;
        end
        
        
        
        theta = inv(A.'* A) *A.' * m_field.';
        for iter = 1 : N
            theta_cell{iter} = theta;
        end

        for i = 2 : numSamples 
            % propagate theta to next time step
            theta_n = propogate_theta_(settings, theta(:, i - 1), invA, ...
                                            position(i, :) - position(i - 1, :), orientation(i - 1, :), angularVelocity(i - 1, :));
            aux(i - 1, :) = (theta_n - theta(:, i)).';
        end
        



    end



    parfor iter = 1 : N
        acc_bias = cumsum([settings.init_sigma_acc_const_bias * randn(1, 3); settings.sigma_acc_bias_rw * randn(numSamples - 1, 3)]);
        accelerometerReadings = rotateframe(orientation, acceleration + [0 0 -9.81]) + acc_bias + ...
                                settings.sigma_acc_w * randn(size(acceleration));
        gyro_bias = cumsum([settings.init_sigma_gyro_const_bias * randn(1, 3); settings.sigma_gyro_bias_rw * randn(numSamples - 1, 3)]);
        gyroReadings = rotateframe(orientation, angularVelocity) + gyro_bias + settings.sigma_gyro_w * randn(size(angularVelocity));
        ImuMag_bias(iter).IMU = [acc_bias gyro_bias];
        
        mag_bias = zeros(numSamples, 3 * (numSensors - 1));
        ImuMag_bias(iter).MAG = mag_bias;

        if settings.generate_from_model
            magnetometerReadings = (A * theta_cell{iter}).' + [mag_bias zeros(numSamples, 3)] + settings.sigma_mag_w * randn(size((A * theta_cell{iter}).')); 
        else
            magnetometerReadings = m_field + [mag_bias zeros(numSamples, 3)] + settings.sigma_mag_w * randn(size(m_field)); 
        end
        ImuMag_data(iter).IMU = [accelerometerReadings gyroReadings];
        ImuMag_data(iter).MAG = magnetometerReadings;
    end


    theta_cell = cellfun(@(x)x.', theta_cell, 'UniformOutput', false);

    fprintf('sensor data generating complete......\n');
end


function theta = propogate_theta(settings, theta, invA, dp_nav, orientation, angularVelocity)
        
        rotvec = rotateframe(orientation, angularVelocity)  * settings.dT;
        R_12  = q2r(rotvec2quat(rotvec));
        pos_sel = R_12 * [0 0 1 1 0; 0 1 0 1 0; 1 0 0 1 0] + rotateframe(orientation, dp_nav).';
        B = [R_12.' * calcAB(pos_sel(:, 1)); 
            R_12.' * calcAB(pos_sel(:, 2));
            R_12.' * calcAB(pos_sel(:, 3));
            R_12.' * calcAB(pos_sel(:, 4));
            R_12.' * calcAB(pos_sel(:, 5))];
        theta = invA * B * theta + settings.sigma_coeff_w * randn(size(theta));
end

function theta = propogate_theta_(settings, theta, invA, dp_nav, orientation, angularVelocity)
        
        rotvec = rotateframe(orientation, angularVelocity)  * settings.dT;
        R_12  = q2r(rotvec2quat(rotvec));
        pos_sel = R_12 * [0 0 1 1 0; 0 1 0 1 0; 1 0 0 1 0] + rotateframe(orientation, dp_nav).';
        B = [R_12.' * calcAB(pos_sel(:, 1)); 
            R_12.' * calcAB(pos_sel(:, 2));
            R_12.' * calcAB(pos_sel(:, 3));
            R_12.' * calcAB(pos_sel(:, 4));
            R_12.' * calcAB(pos_sel(:, 5))];
        theta = invA * B * theta;
end