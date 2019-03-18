function u = getLQRTrackingControl(y0,p)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[A,B] = getLinearizedSystem(y0,p);
s = size(B);
Q = 18000 * eye(s(1));
R = eye(s(2));
[P, L, G] = care(A, B, Q, R);
x_d = deg2rad( [10 0 10 0]' );

% b = (A-B*1\R*B'*P)^(-T)(Q*x_d)
b = inv(A-B*inv(R)*B'*P)' * (Q*x_d);
u = @(t_in, y_in) -G * y_in - B' * b;
end