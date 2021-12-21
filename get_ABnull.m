function ABnull=get_ABnull(r,order)
rx=r(1);
ry=r(2);
rz=r(3);


switch order
    
    case 1
        ABnull=[ 0, 0, 1,  0,     0, rz, ry,  2*rx;...
            0, 1, 0, rz,  2*ry,  0, rx,     0;...
            1, 0, 0, ry, -2*rz, rx,  0, -2*rz];
        
    case 2
        ABnull=[ 0, 0, 1,  0,     0, rz, ry,  2*rx,           0,               0, ry*rz, ry^2 - rz^2,     2*rx*rz,     2*rx*ry, 3*rx^2 - 3*rz^2;...
            0, 1, 0, rz,  2*ry,  0, rx,     0,     2*ry*rz, 3*ry^2 - 3*rz^2, rx*rz,     2*rx*ry,           0, rx^2 - rz^2,               0;...
            1, 0, 0, ry, -2*rz, rx,  0, -2*rz, ry^2 - rz^2,        -6*ry*rz, rx*ry,    -2*rx*rz, rx^2 - rz^2,    -2*ry*rz,        -6*rx*rz];
        
    case 3
        ABnull=[ 0, 0, 1,  0,     0, rz, ry,  2*rx,           0,               0, ry*rz, ry^2 - rz^2,     2*rx*rz,     2*rx*ry, 3*rx^2 - 3*rz^2,                0,                   0,  ry^2*rz - rz^3/3,      ry^3 - 3*ry*rz^2,        2*rx*ry*rz,                2*rx*ry^2 - 2*rx*rz^2, 3*rx^2*rz - rz^3, 3*ry*rx^2 - 3*ry*rz^2, 4*rx^3 - 12*rx*rz^2;...
            0, 1, 0, rz,  2*ry,  0, rx,     0,     2*ry*rz, 3*ry^2 - 3*rz^2, rx*rz,     2*rx*ry,           0, rx^2 - rz^2,               0, 3*ry^2*rz - rz^3, 4*ry^3 - 12*ry*rz^2,        2*rx*ry*rz, 3*rx*ry^2 - 3*rx*rz^2,  rx^2*rz - rz^3/3,                2*ry*rx^2 - 2*ry*rz^2,                0,      rx^3 - 3*rx*rz^2,                   0;...
            1, 0, 0, ry, -2*rz, rx,  0, -2*rz, ry^2 - rz^2,        -6*ry*rz, rx*ry,    -2*rx*rz, rx^2 - rz^2,    -2*ry*rz,        -6*rx*rz, ry^3 - 3*ry*rz^2, 4*rz^3 - 12*ry^2*rz, rx*ry^2 - rx*rz^2,           -6*rx*ry*rz, ry*rx^2 - ry*rz^2, - 2*rx^2*rz - 2*ry^2*rz + (4*rz^3)/3, rx^3 - 3*rx*rz^2,           -6*rx*ry*rz, 4*rz^3 - 12*rx^2*rz];
        
        
    otherwise
        error('Only model orders 1-3 supported');
end
