function [q] = rotvec2quat(rotvec)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
assert(all(size(rotvec) == [1, 3]));

phi = norm(rotvec);
u = rotvec / phi;
q = [cos(phi/2) u * sin(phi / 2)];
end

