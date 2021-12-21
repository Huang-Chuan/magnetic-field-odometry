function AB = calcAB(X)
    rx = X(1);
    ry = X(2);
    rz = X(3);
    AB =[0, 0, 1,  0,     0, rz, ry,  2*rx,           0,               0, ry*rz, ry^2 - rz^2,     2*rx*rz,     2*rx*ry, 3*rx^2 - 3*rz^2; ...
        0, 1, 0, rz,  2*ry,  0, rx,     0,     2*ry*rz, 3*ry^2 - 3*rz^2, rx*rz,     2*rx*ry,           0, rx^2 - rz^2,               0;
        1, 0, 0, ry, -2*rz, rx,  0, -2*rz, ry^2 - rz^2,        -6*ry*rz, rx*ry,    -2*rx*rz, rx^2 - rz^2,    -2*ry*rz,        -6*rx*rz];

end