function drawCircle(cx,cy,r,lineType)
t = linspace(0,2*pi,100);
plot( cx+r*cos(t), cy+r*sin(t), lineType,'MarkerSize',1,...
                                         'Color',[1 0.8 0.8]);
end