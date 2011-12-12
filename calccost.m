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
    if (isfield(ob, 'sig'))
        ob.sigx = ob.sig;
        ob.sigy = ob.sig;
    end
    switch (goalfunc)
        case {'linear' 'square'}
            obval = 20*exp(-1/2*((x-ob.x)^2/ob.sigx^2+(y-ob.y)^2/ob.sigy^2));%1/(2*pi*ob.sigx*ob.sigy) * 
        case {'linear3' 'square3'}
            obval = 1/20*exp(-1/2*((x-ob.x)^2/ob.sigx^2+(y-ob.y)^2/ob.sigy^2));%1/(2*pi*ob.sigx*ob.sigy) * 
        otherwise
            obval = exp(-1/2*((x-ob.x)^2/ob.sigx^2+(y-ob.y)^2/ob.sigy^2));%1/(2*pi*ob.sigx*ob.sigy) * 
    end
    val = val + obval;
end
switch (goalfunc)
    case 'linear'
        goalval = sqrt((x - goalx)^2 + (y - goaly)^2);
    case 'linear2'
        goalval = 1/20 * sqrt((x - goalx)^2 + (y - goaly)^2);
    case 'linear3'
        goalval = 1/400 * sqrt((x - goalx)^2 + (y - goaly)^2);
    case 'square'
        goalval = (x - goalx)^2 + (y - goaly)^2;
    case 'square2'
        goalval = 1/200 * (x - goalx)^2 + (y - goaly)^2;
    case 'square3'
        goalval = 1/20000 * (x - goalx)^2 + (y - goaly)^2;
    case 'exp'
        goalval = expsqrt((x - goalx)^2 + (y - goaly)^2);
    otherwise
        goalval = 0;
end
val = val + goalval;