function [MPredX, MPredY, MPredZ]= predict_field(theta, pos, orientation, X, Y, Z)
% Input: 
% xh: state vector
% position: current sensor position
% orientation: current sensor orientation
% X      : x-coordinate in navigation frame, M x N 
% Y      : y-coordinate in navigation frame, M x N 
% Z      : z-coordinate in navigation frame, M x N 

XYZ = [X(:) Y(:) Z(:)];

cord_local = rotateframe(orientation, XYZ - pos);
MPred = [];
for i = 1 : size(cord_local, 1)
    MPred = [MPred; rotatepoint(orientation, (calcAB(cord_local(i, :)) * theta).')];
end


MPredX = reshape(MPred(:, 1), size(X, 1), size(X, 2));
MPredY = reshape(MPred(:, 2), size(X, 1), size(X, 2));
MPredZ = reshape(MPred(:, 3), size(X, 1), size(X, 2));

end