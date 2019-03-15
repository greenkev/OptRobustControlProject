p = getParamStruct();

y0 = deg2rad( [20;2;10;1]);

dyn = @(t_in, y_in) twoLinkArmDynamics(t_in,y_in,p);
dyn_lin = @(t_in, y_in) twoLinkLinearArmDynamics(t_in,y_in,[0,0,0,0]',p);

tspan = linspace(0,200,200);

[t,y] = ode45(dyn,tspan, y0);
[t_lin,y_lin] = ode45(dyn_lin,tspan, y0);

plotTrajectory(t,y,2)
plotTrajectory(t_lin,y_lin,2)

animObj = twoLinkArmAnimation(p);

animObj.animateTraj(y,'No Control')

