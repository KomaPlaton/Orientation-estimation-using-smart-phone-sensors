function [x, P] = mu_g(x, P, yacc, Ra, g0)
% mu_g: Time update function for accelerometer measurements

% Write out the expression for hx and Hx
hx = Qq(x)' * g0;
[Q0, Q1, Q2, Q3] = dQqdq(x);
Hx = [Q0' * g0, Q1' * g0, Q2' * g0, Q3' * g0];
                       
% Apply the EKF update step
S = Hx * P * Hx' + Ra;
K = P * Hx' * inv(S);

x = x + K * (yacc - hx);
P = P - K * S * K';

end