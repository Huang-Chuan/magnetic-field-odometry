function error_struct = calc_error(XData, PData, Xs, numSamples, N, settings)
    numStates = settings.numErrorStates;
    errorMasks = settings.errorStateMask;

    mseError = zeros(numSamples, numStates, N);
    stdError = zeros(numSamples, numStates, N);

    for iter = 1 : N
        stdError(:, :, iter) = PData{iter};
    end

    for iter = 1 : N
        delta_q = quatmultiply(quatconj(XData{iter}(:, 7:10)) ,Xs{iter}(:, 7:10));
        mseError(:, :, iter) = [(XData{iter}(:, 1:6) - Xs{iter}(:, 1:6)).^2  ...
        delta_q(:,2:4).^2 ...
        (XData{iter}(:, 11:end) - Xs{iter}(:, 11:end)).^2];
    end
    
    
    
    mseError = sqrt(mean(mseError, 3));
    stdError = sqrt(mean(stdError, 3));

    error_struct.msePosErr = mseError(:, errorMasks.pos);
    error_struct.mseVelErr = mseError(:, errorMasks.vel);
    error_struct.mseQErr  =  mseError(:, errorMasks.epsilon);
    error_struct.mseAccBiasErr = mseError(:, errorMasks.acc_bias);
    error_struct.mseMagBiasErr = mseError(:, errorMasks.mag_bias);
    error_struct.mseThetaErr = mseError(:, errorMasks.theta);
    error_struct.mseGyroErr = mseError(:, errorMasks.gyro_bias);


    error_struct.stdPosErr = stdError(:, errorMasks.pos);
    error_struct.stdVelErr = stdError(:, errorMasks.vel);
    error_struct.stdQErr  =  stdError(:, errorMasks.epsilon);
    error_struct.stdAccBiasErr = stdError(:, errorMasks.acc_bias);
    error_struct.stdMagBiasErr = stdError(:, errorMasks.mag_bias);
    error_struct.stdThetaErr = stdError(:, errorMasks.theta);
    error_struct.stdGyroErr = stdError(:, errorMasks.gyro_bias);



end
