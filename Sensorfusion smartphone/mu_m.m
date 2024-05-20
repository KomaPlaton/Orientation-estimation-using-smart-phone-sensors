function [x, P] = mu_m(x, P, mag, Rm, m0)
hx = Qq(x)'*m0;
[Q0, Q1, Q2, Q3] = dQqdq(x);
Hx = [Q0'*m0, Q1'*m0, Q2'*m0, Q3'*m0];

S = Hx * P * Hx' + Rm;

K = P * Hx' * inv(S);

x = x + K * (mag - hx);

P = P - K * S * K';

end