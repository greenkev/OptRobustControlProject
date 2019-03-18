p = getParamStruct();

y0 = deg2rad( [20;2;10;1]);
% y0 = deg2rad( [0;0;0;0]);

controller = getLQRRegulatorControl([0 0 0 0]', p);
% controller = getLQRTrackingControl([0 0 0 0]', p);
dyn = @(t_in, y_in) twoLinkArmDynamics(t_in,y_in,p,controller);
dyn_lin = @(t_in, y_in) twoLinkLinearArmDynamics(t_in,y_in,[0,0,0,0]',p,controller);

tspan = linspace(0,8,1000);
% dyn_lin(3, [1, 1, 1, 1]')
[t,y] = ode45(dyn,tspan, y0);
[t_lin,y_lin] = ode45(dyn_lin,tspan, y0);


for i = 1:length(t)
       u(:,i) = controller(t(i),y(i,:)');  
end
for i = 1:length(t_lin)
       u_lin(:,i) = controller(t_lin(i),y_lin(i,:)');  
end
y = rad2deg(y);
y_lin = rad2deg(y_lin);
% plotTrajectory(t_lin,y_lin,3,u)

%animObj = twoLinkArmAnimation(p);
%animObj.animateTraj(y,'No Control')

fig = figure

subplot(3,1,1)
plot(t,y(:,1),'-','linewidth',1)
hold on
plot(t,y(:,3),'-','linewidth',2)
plot(t_lin,y(:,1),'--','linewidth',1)
plot(t_lin,y(:,3),'--','linewidth',2)
ylabel('Angle (deg)')
legend('q_1 Nonlinear','q_2 Nonlinear','q_1 Linear','q_2 Linear')
xlabel('time (sec)')


subplot(3,1,2)
plot(t,y(:,2),'-','linewidth',1)
hold on
plot(t,y(:,4),'-','linewidth',2)
plot(t_lin,y(:,2),'--','linewidth',1)
plot(t_lin,y(:,4),'--','linewidth',2)
ylabel('Angular Velocity (deg/sec)')
legend('q_1 Nonlinear','q_2 Nonlinear','q_1 Linear','q_2 Linear')
xlabel('time (sec)')

subplot(3,1,3)
plot(t,u(1,:),'-','linewidth',1) 
hold on
plot(t,u(2,:),'-','linewidth',2)   
plot(t,u_lin(1,:),'--','linewidth',1)  
plot(t,u_lin(2,:),'--','linewidth',2)   
ylabel('Motor Torque (Nm)')
legend('\tau_1 Nonlinear','\tau_2 Nonlinear','\tau_1 Linear','\tau_2 Linear')
xlabel('time (sec)')