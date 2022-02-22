function [MPredX, MPredY, MPredZ]= get_actual_field(model, X, Y, Z)
% Input: 
% xh: state vector
% position: current sensor position
% orientation: current sensor orientation
% X      : x-coordinate in navigation frame, M x N 
% Y      : y-coordinate in navigation frame, M x N 
% Z      : z-coordinate in navigation frame, M x N 


tmp = load(model);
%N_poles = 50;
m.moments = tmp.mm(:, 1:end-2);
m.pos_dipoles = tmp.MM;
m.f_earth = tmp.mm(:, end-1);
clear tmp

cord_nav = [X(:) Y(:) Z(:)];

MPred = [];
for i = 1 : size(cord_nav, 1)
    mag_field = m.f_earth.';
    for kk = 1 : size(m.pos_dipoles, 2)
        mag_field = mag_field + dipole(cord_nav(i, :).', m.pos_dipoles(:,kk),m.moments(:,kk)).';
    end

    MPred = [MPred; mag_field];
end


MPredX = reshape(MPred(:, 1), size(X, 1), size(X, 2));
MPredY = reshape(MPred(:, 2), size(X, 1), size(X, 2));
MPredZ = reshape(MPred(:, 3), size(X, 1), size(X, 2));

end