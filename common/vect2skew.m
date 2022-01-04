function y = vect2skew(v)
%This is a function to generate the skew symmectric form [v] of a vector v
%Usage:
%   vect2skew(v)
%       v: a 3x1 or 1x3 vector

    y=[0 -v(3) v(2) ; v(3) 0 -v(1) ; -v(2) v(1) 0 ];

end