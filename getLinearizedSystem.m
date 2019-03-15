function [ A, B ] = getLinearizedSystem( y,p )
%GETLINEARIZEDSYSTEM Summary of this function goes here
%   Detailed explanation goes here

    checkInputs(y,p);
        
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
    
    %Forward Dynamics matricies
    B_acc = M\eye(2);
    C_acc = -M\C;
    
    A = zeros(4);
    
    A(1,:) = [0, 1, 0, 0];
    A(2,:) = [0, C_acc(1,1), 0, C_acc(1,2)];
    A(3,:) = [0, 0, 0, 1];
    A(4,:) = [0, C_acc(2,1), 0, C_acc(2,2)];
    
    B = zeros(4,2);
    
    B(2,:) = [B_acc(1,1), B_acc(1,2)];
    B(4,:) = [B_acc(2,1), B_acc(2,2)];
end

function checkInputs(y,p)
    req_fields = {'m1','me','l1','lc1','lce','I1','Ie','de'};
    
    for i = 1:length(req_fields)
        if ~isfield(p,req_fields{i})
           error(['Missing field "',req_fields{i},...
                  '" in parameter structure sent to twoLinkArmDynamics']);
        end        
    end
    
    if sum(size(y) ~= [4,1])
       error(['y is the wrong size, expected [4 1] size was [',...
             num2str(size(y)),']']);
    end
end

