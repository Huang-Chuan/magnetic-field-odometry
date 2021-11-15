function [absPositionError,stdPositionError, absVelError, stdVelError] = calc_error(XData, PData,pos, vel, numSamples, N)

    absPositionError = zeros(numSamples, N);
    stdPositionError = zeros(numSamples, N);
    absVelError      = zeros(numSamples, N);
    stdVelError      = zeros(numSamples, N);
    for iter = 1 : N
        absPositionError(:, iter) = (XData{iter}(:, 1) - pos(:, 1)).^2;
        stdPositionError(:, iter) = PData{iter}(:, 1, 1);
        absVelError(:, iter) = (XData{iter}(:, 2) - vel(:, 1)).^2;
        stdVelError(:, iter) = PData{iter}(:, 2, 2);
    end
    absPositionError = sqrt(mean(absPositionError, 2));
    stdPositionError = sqrt(mean(stdPositionError, 2));
    absVelError = sqrt(mean(absVelError, 2));
    stdVelError = sqrt(mean(stdVelError, 2));
end

