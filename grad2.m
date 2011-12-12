function delf = grad2(f, x)
d = 0.1;
dx = [-d 0 d];
val = zeros(3);
for i = 1:3
    for j = 1:3
        val(i,j) = f([x(1)+dx(i),x(2)+dx(j)]);
    end
end
[fx fy] = gradient(val,d);
delf = [fy(2,2) fx(2,2)];