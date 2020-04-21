clc; clear;

en_dev1 = 1;

fileName0 = 'C:\Users\Nastase\Desktop\dev0.log';
fileName1 = 'C:\Users\Nastase\Desktop\dev1.log';

[ts0, data0, seconds0] = CheckTS(fileName0);
if (en_dev1)
    [ts1, data1, seconds1] = CheckTS(fileName1);
end

plot(ts0, data0);

if (en_dev1)
    hold on;
    plot(ts1, data1, 'color', 'red');
end

grid on;

for i=1:length(seconds0)
    line([seconds0(i), seconds0(i)], [0,15000], 'color', 'green', ...
        'LineWidth', 4);
end

%%
% [x,y] = ginput(2); % click on each point
% dx = diff(x)

% set(gca,'XTick',unique(ts_s));
% set(gca,'XTickLabel',[4 8 15 21] );
% set(gca,'YGrid','off','XGrid','on');
