function dy = twoLinkLinearArmDynamics(t,y,y0,p,arg_2)
%TWOLINKARMDYNAMICS This function contains the lienarized dynamics of the two
%link arm operating in a plane with no gravity. 
% The inputs are:
%   t: current time (seconds)
%   y: state vector [q_1, dq_1, q_2, dq_2]
%   y0: linearization state vector [q_1, dq_1, q_2, dq_2]
%   p: parameter vector, this must contain all the fields in req_fields
%   arg2: this is either a vector of 2 motor torques, a handle to a
%   controller function that takes (t,y,p)

    checkInputs(t,y,p);
    
    dy = zeros(size(y));
    
    if ~exist('arg_2','var')
        u = [0; 0]; %No torque if not controller is supplied
    elseif isa(arg_2,'function_handle')
        u = arg_2(t,y); %If it is a controller function use it
    elseif sum(isa(arg_2,'double'))
        u = arg_2; %if it is a 
    else
        error('Controller function or value is unexpected');
    end
    
    [A,B] = getLinearizedSystem( y0,p );
    
    dy = A*(y - y0) + B*u;
     
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
    
    if sum(size(y) ~= [4,1])
       error(['y is the wrong size, expected [4 1] size was [',...
             num2str(size(y)),']']);
    end
end
