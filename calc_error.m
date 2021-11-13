function [absPositionError,stdPositionError, absVelError, stdVelError] = calc_error(XData, PData,pos, vel, numSamples, N)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
    absPositionError = zeros(numSamples, N);
    stdPositionError = zeros(numSamples, N);
    absVelError      = zeros(numSamples, N);
    stdVelError      = zeros(numSamples, N);
    for iter = 1 : N
        absPositionError(:, iter) = abs(XData{iter}(:, 1) - pos(:, 1));
        stdPositionError(:, iter) = sqrt(PData{iter}(:, 1, 1));
        absVelError(:, iter) = abs(XData{iter}(:, 2) - vel(:, 1));
        stdVelError(:, iter) = sqrt(PData{iter}(:, 2, 2));
    end
    absPositionError = mean(absPositionError, 2);
    stdPositionError = mean(stdPositionError, 2);
    absVelError = mean(absVelError, 2);
    stdVelError = mean(stdVelError, 2);
end

