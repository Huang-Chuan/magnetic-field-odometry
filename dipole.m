function h=dipole(p,pref,m)
r=p-pref;
h=(3.*(r*r')./norm(r)^5-eye(3)./norm(r)^3)*m;
end

