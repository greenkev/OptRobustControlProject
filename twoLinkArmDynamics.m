function dy = twoLinkArmDynamics(t,y,p,arg_2)
%TWOLINKARMDYNAMICS This function contains the forward dynamics of the two
%link arm operating in a plane with no gravity. 
% The inputs are:
%   t: current time (seconds)
%   y: state vector [q_1, dq_1, q_2, dq_2]
%   p: parameter vector, this must contain all the fields in req_fields
%   arg2: this is either a vector of 2 motor torques, a handle to a
%   controller function that takes (t,y,p)

    checkInputs(t,y,p);
    
    dy = zeros(size(y));
    
    if ~exist('arg_2','var')
        u = [0, 0]; %No torque if not controller is supplied
    elseif isa(arg_2,'function_handle')
        u = arg_2(t,y); %If it is a controller function use it
    elseif sum(isa(arg_2,'double'))
        u = arg_2; %if it is a 
    else
        error('Controller function or value is unexpected');
    end
    
    q1 = y(1);
    dq1 = y(2);
    q2 = y(3);
    dq2 = y(4);
    
    %Intermediate Parameters
    a1 = p.I1 + p.m1*p.lc1^2 + p.Ie + p.me*p.lce^2 + p.me*p.l1^2;
    a2 = p.Ie + p.me*p.lce^2;
    a3 = p.me*p.l1*p.lce*cos(p.de);
    a4 = p.me*p.l1*p.lce*sin(p.de);

    %Mass Matrix 
    H_11 = a1 + 2*a3*cos(q2) + 2*a4*sin(q2);
    H_12 = a2 + a3*cos(q2) + a4*sin(q2);
    H_22 = a2;
    M = [H_11, H_12; H_12, H_22];
    
    %Velocity Product Terms
    h = a3*sin(q2) - a4*cos(q2);
    C = [ -h*dq2, -h*(dq1 + dq2); h*dq1, 0];
    
    %Forward Dynamics
    accel = M\( -C*[dq1;dq2;] + [u(1);u(2)]);
    
    %Assembly
    dy(1) = dq1;
    dy(3) = dq2;
    dy(2) = accel(1);
    dy(4) = accel(2);
     
end

function checkInputs(t,y,p)
    req_fields = {'m1','me','l1','lc1','lce','I1','Ie','de'};
    
    for i = 1:length(req_fields)
        if ~isfield(p,req_fields{i})
           error(['Missing field "',req_fields{i},...
                  '" in parameter structure sent to twoLinkArmDynamics']);
        end        
    end
    
    if sum(size(t) ~= [1,1])
       error(['t is the wrong size, expected scalar size was [',...
             num2str(size(t)),']']);
    end
    
    if sum(size(y) ~= [1,4])
       error(['y is the wrong size, expected [1 4] size was [',...
             num2str(size(y)),']']);
    end
end
