function [msePosError,stdPositionError, mseVelError, stdVelError, mseBiasError, stdBiasError] = calc_error(XData, PData, pos,...
                                                                                                 vel, accbias, ...
                                                                                                 numSamples, N)

    msePosError = zeros(numSamples, N);
    stdPositionError = zeros(numSamples, N);
    mseVelError      = zeros(numSamples, N);
    stdVelError      = zeros(numSamples, N);
    mseBiasError     = zeros(numSamples, N);
    stdBiasError     = zeros(numSamples, N);
    
    for iter = 1 : N
        
        msePosError(:, iter) = (XData{iter}(:, 1) - pos(:, 1)).^2;
        stdPositionError(:, iter) = PData{iter}(:, 1, 1);
        
        mseVelError(:, iter) = (XData{iter}(:, 2) - vel(:, 1)).^2;
        stdVelError(:, iter) = PData{iter}(:, 2, 2);

        mseBiasError(:, iter) = (XData{iter}(:, 3) - accbias(1, iter)).^2;
        stdBiasError(:, iter)= PData{iter}(:, 3, 3);

    end

    msePosError = sqrt(mean(msePosError, 2));
    stdPositionError = sqrt(mean(stdPositionError, 2));
    
    mseVelError = sqrt(mean(mseVelError, 2));
    stdVelError = sqrt(mean(stdVelError, 2));
    
    mseBiasError = sqrt(mean(mseBiasError, 2));
    stdBiasError = sqrt(mean(stdBiasError, 2));
end

