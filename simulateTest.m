p = getParamStruct();

% y0 = deg2rad( [20;2;10;1]);
y0 = deg2rad( [0;0;0;0]);

u = getLQRTrackingControl([0 0 0 0]', p);
dyn = @(t_in, y_in) twoLinkArmDynamics(t_in,y_in,p,u);
dyn_lin = @(t_in, y_in) twoLinkLinearArmDynamics(t_in,y_in,[0,0,0,0]',p,u);

tspan = linspace(0,8,1000);
% dyn_lin(3, [1, 1, 1, 1]')
[t,y] = ode45(dyn,tspan, y0);
[t_lin,y_lin] = ode45(dyn_lin,tspan, y0);

plotTrajectory(t,y,3,u)
plotTrajectory(t_lin,y_lin,3,u)

%animObj = twoLinkArmAnimation(p);
%animObj.animateTraj(y,'No Control')