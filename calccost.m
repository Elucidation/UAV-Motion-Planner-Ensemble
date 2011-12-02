function val = calccost(pos,obs,goal,goalfunc)
if (isempty(goalfunc))
    goalfunc = 'linear';
end
val = 0;
x = pos(1);
y = pos(2);
goalx = goal(1);
goaly = goal(2);
for i = 1:length(obs)
    ob = obs{i};
    obval = ob.A * exp(-((x-ob.x)^2+(y-ob.y)^2)/(2*ob.sig^2));
    val = val + obval;
end
switch (goalfunc)
    case 'linear'
        goalval = sqrt((x - goalx)^2 + (y - goaly)^2);
    case 'square'
        goalval = (x - goalx)^2 + (y - goaly)^2;
    case 'exp'
        goalval = expsqrt((x - goalx)^2 + (y - goaly)^2);
    otherwise
        goalval = 0;
end
val = val + goalval;