function [x delf] = graddesc(func, x0, goal)
step = 20; % total number of steps to plan out
maxstep = 1.5; % Step size of each step
delftol = 1e-3; % Value of gradient change below which algorithm is converged
dxtol = 1e-3; % Value of x change below which algorithm is converged
goaltol = 0.1; % Distance from goal for which algorithm is converged
maxit = 40; % maximum iterations
it = 0; % iteration count
x(1,:) = x0;
delf(1,:) = grad2(func, x0);
done = false;
while (~done)
    if (norm(delf(end,:)) <= delftol || (it > 1 && norm(x(end,:)-x(end-1,:)) <= dxtol) || norm(x(end,:)-goal) <= goaltol || it >= maxit)
        done = true;
        continue
    end
    unit = -delf(end,:) / norm(-delf(end,:));
    xnew = x(end,:) + min(step * norm(delf(end,:)), maxstep) * unit;
    delfnew = grad2(func, xnew);
    x = [x; xnew];
    delf = [delf; delfnew];
    it = it + 1;
end
disp([int2str(it) ' iterations']);