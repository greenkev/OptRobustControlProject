%setup necessary data structures
p = getParamStruct(); %Sys Parameters
y0 = deg2rad( [20;2;10;1]); %Initial Condition
dyn = @(t_in, y_in) twoLinkArmDynamics(t_in,y_in,p); %Nonlinear Dynamics Handle
dyn_lin = @(t_in, y_in) twoLinkLinearArmDynamics(t_in,y_in,[0,0,0,0]',p); %Linear Dynamics Handle
tspan = linspace(0,5,1000); %Time span

%Run simulations
[t,y] = ode45(dyn,tspan, y0);
[t_lin,y_lin] = ode45(dyn_lin,tspan, y0);

%Convert to degrees for plotting
y = rad2deg(y);
y_lin = rad2deg(y_lin);

%Plot results
fig = figure

subplot(2,1,1)
plot(t,y(:,1),'-','linewidth',1)
hold on
plot(t,y(:,3),'-','linewidth',2)
plot(t_lin,y_lin(:,1),'--','linewidth',1)
plot(t_lin,y_lin(:,3),'--','linewidth',2)
ylabel('Angle (deg)')
legend('q_1 Nonlinear','q_2 Nonlinear','q_1 Linear','q_2 Linear')
xlabel('time (sec)')

subplot(2,1,2)
plot(t,y(:,2),'-','linewidth',1)
hold on
plot(t,y(:,4),'-','linewidth',2)
plot(t_lin,y_lin(:,2),'--','linewidth',1)
plot(t_lin,y_lin(:,4),'--','linewidth',2)
ylabel('Angular Velocity (deg/sec)')
legend('q_1 Nonlinear','q_2 Nonlinear','q_1 Linear','q_2 Linear')
xlabel('time (sec)')


%Animate the resulting Trajectories
decimation = 5; %This controls the playback speed
animObj = twoLinkArmAnimation(p);
animObj.animateTraj(deg2rad(y(1:decimation:end,:)),'Passive Simulation Nonlinear System')
pause(1);
animObj.animateTraj(deg2rad(y_lin(1:decimation:end,:)),'Passive Simulation Linear System')