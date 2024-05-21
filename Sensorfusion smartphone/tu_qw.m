function[x, P] = tu_qw(x, P, omega, T, Rw)
% tu_qw: Time update function for gyroscope measurements

% Write out the expression for F and G
F = eye(size(x,1)) + (T/2) * Somega(omega);
G = (T/2) * Sq(x);

% Apply the EKF prediction step
x = F * x;
P = F * P * F' + G * Rw * G';

end