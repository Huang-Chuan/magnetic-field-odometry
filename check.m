eq = @(rx, ry, rz)[0, 0, 1,  0,     0, rz, ry,  2*rx,           0,               0, ry*rz, ry^2 - rz^2,     2*rx*rz,     2*rx*ry, 3*rx^2 - 3*rz^2; ...
                  0, 1, 0, rz,  2*ry,  0, rx,     0,     2*ry*rz, 3*ry^2 - 3*rz^2, rx*rz,     2*rx*ry,           0, rx^2 - rz^2,               0;
                  1, 0, 0, ry, -2*rz, rx,  0, -2*rz, ry^2 - rz^2,        -6*ry*rz, rx*ry,    -2*rx*rz, rx^2 - rz^2,    -2*ry*rz,        -6*rx*rz];
%y = [eq(0,0,1); eq(0, 1, 0); eq(1, 0, 0); eq(1, 1, 0); eq(1, 1, 1)];
AB_c2 = [eq(0,0,1); eq(0, 1, 0); eq(1, 0, 0); eq(1, 1, 1); eq(0, 0, 0)];

dx = [0.5; 12; 14];
theta_km1 = 0.01*randn(15, 1);
rotationVector = 2*pi * [1, 0, 0];
R12 = rotationVectorToMatrix(rotationVector);

pos = R12 * [0 0 1 1 0; 0 1 0 1 0; 1 0 0 1 0] + dx;

RAB_c1 = [];
for i = 1 : 5
    RAB_c1 = [RAB_c1; R12.' * eq(pos(1, i),pos(2, i),pos(3, i))];    
end

RAB_c1_ = [eq(pos(1, 1),pos(2, 1),pos(3, 1)) ]

%AB_c1 = [eq(0,0,1); eq(0, 1, 0); eq(1, 0, 0); eq(1, 1, 1); eq(0, 0, 0)];

theta_k =  inv(AB_c2) * RAB_c1 * theta_km1

% test arbitray postion 
p = randn(3, 1);
%p = [0;0;1];
mag_c2 = eq(p(1), p(2), p(3)) * theta_k;
p1 = R12 * p + dx;
mag_c1 = R12.' * eq(p1(1), p1(2), p1(3)) * theta_km1;