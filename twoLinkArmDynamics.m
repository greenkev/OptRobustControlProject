function [outputArg1,outputArg2] = twoLinkArmDynamics(t,y,p,arg_2)
%TWOLINKARMDYNAMICS This function contains the forward dynamics of the two
%link arm operating in a plane with no gravity. 
% The inputs are:
%   t: current time (seconds)
%   y: state vector [q_1, dq_1, q_2, dq_2]
%   p: parameter vector, this must contain all the fields in req_fields
%   arg2: this is either a vector of 2 motor torques, a handle to a
%   controller function that takes (t,y,p)
req_fields = {'m1','me','l1','lc1','lce','I1','Ie','de'};

end

