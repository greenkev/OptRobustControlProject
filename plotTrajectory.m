function [fig,u] = plotTrajectory(t,y,n,ctrl)

if ~exist('n','var')
   n = 3; 
end

if ~exist('ctrl','var')
   u = zeros(2,length(t)); 
elseif isa(ctrl,'function_handle')
    for i = 1:length(t)
       u(:,i) = ctrl(t(i),y(i,:)');  
    end
else
   u = ctrl; 
end

fig = figure

subplot(n,1,1)
plot(t,y(:,1))
hold on
plot(t,y(:,3))
ylabel('Angle (rad)')
xlabel('time (sec)')

if n >= 2
    subplot(n,1,2)
    plot(t,y(:,2))
    hold on
    plot(t,y(:,4))
    ylabel('Angular Velocity (rad/sec)')
    xlabel('time (sec)')
    
end

if n >= 3
    subplot(n,1,3)
    plot(t,u)    
    ylabel('Motor Torque (Nm)')
    xlabel('time (sec)')
end



end