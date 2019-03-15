p = getParamStruct();

y0 = deg2rad( [20;2;10;1]);

dyn = @(t_in, y_in) twoLinkArmDynamics(t_in,y_in,p);

tspan = linspace(0,10,200);

[t,y] = ode45(dyn,tspan, y0);

plotTrajectory(t,y,1)
plotTrajectory(t,y,2)
plotTrajectory(t,y,3)

animObj = twoLinkArmAnimation(p);

animObj.animateTraj(y,'No Control')

