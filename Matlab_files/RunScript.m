
clc; clear;

fileName0 = 'C:\Users\Nastase\Desktop\log_ADC_0.log';
fileName1 = 'C:\Users\Nastase\Desktop\log_ADC_1.log';

[ts0, data0, seconds0] = CheckTS(fileName0);
[ts1, data1, seconds1] = CheckTS(fileName1);

% ts1 = ts1 - (ts1(1) - ts0(1));
data1 = data1 + 1000;

plot(ts0, data0);
hold on;
plot(ts1, data1, 'color', 'red');
grid on;

for i=1:length(seconds0)
    line([seconds0(i), seconds0(i)], [0,15000], 'color', 'green', ...
        'LineWidth', 4);
end

% [x,y] = ginput(2); % click on each point
% dx = diff(x)

% set(gca,'XTick',unique(ts_s));
% set(gca,'XTickLabel',[4 8 15 21] );
% set(gca,'YGrid','off','XGrid','on');
