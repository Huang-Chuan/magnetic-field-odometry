function r=PosMagArray()
% spatial locations of magnetometer sensors
  r = NaN(3, 30);
  dx = 0.064;
  dy = 0.055;
  kk = 0;

  for jj=1:5
    for ii=1:6
      kk=kk+1;

      r(1, kk) = (ii-3.5)*dx;
      r(2, kk) = -(jj-3)*dy;
      r(3, kk) = 0;
    end
  end
end
