close all;
%% blah
ax = [25.0000  124.0000    8.0000  118.0000         0    4.9486];
for i = 1:1
    open(sprintf('localMinima%i.fig',i));
    axis(ax);
    saveas(gcf,sprintf('localMinima3d%i',i),'fig');
end

