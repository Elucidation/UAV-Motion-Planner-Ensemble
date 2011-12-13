function [x delf] = graddesc(func, x0, goal)
step = 55;
maxstep = 1;
delftol = 1e-3;
dxtol = 1e-3;
goaltol = 1;
maxit = 40;
it = 0;
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