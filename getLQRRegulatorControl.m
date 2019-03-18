function u = getLQRRegulatorControl(y0,p)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[A,B] = getLinearizedSystem(y0,p);
s = size(B);

[P, L, G] = care(A, B, 2*eye(s(1)))
u = @(t_in, y_in) -G * y_in;
end

