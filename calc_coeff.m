function c = calc_coeff(mag_data, H)
% calculate local coefficients along the trajectory
    c = H \ mag_data.';
    c = c.';
end

