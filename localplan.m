function [path slope] = localplan(pos, goal, obs)
% history.x = [];
% history.fval = [];
costfun = @(xy)calccost(xy, obs, goal, 'linear2');
% opts = optimset('Outputfcn', @grabpts);%, 'DiffMaxChange', 1e-15, 'DiffMinChange', 1e-25);
% [min cost flag output grad] = fminunc(costfun, pos, opts);
% path = history.x;
[path slope] = graddesc(costfun, pos, goal);

function stop = grabpts(x, optimValues, state)
    stop = false;
 
     switch state
%          case 'init'
%              hold on
         case 'iter'
           history.fval = [history.fval; optimValues.fval];
           history.x = [history.x; x];
%            plot(x(1),x(2),'o');
%            text(x(1)+.15,x(2),num2str(optimValues.iteration));
%            title('Sequence of Points Computed by fmincon');
%          case 'done'
%              hold off
         otherwise
     end
end

end


