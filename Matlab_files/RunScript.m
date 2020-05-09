clc; clear;

TPSN_SYNC_WORD = hex2dec('ABABABAB');
ALIGNMENT_WORD = hex2dec('A5A5A5A5');

en_dev1 = 1;

fileName0 = 'C:\Users\Nastase\Desktop\dev0.log';
fileName1 = 'C:\Users\Nastase\Desktop\dev1.log';

[sync0, ts0, data0, seconds0] = CheckTS(fileName0);
if (en_dev1)
    [sync1, ts1, data1, seconds1] = CheckTS(fileName1);
end

plot_0 = plot(ts0, data0);

if (en_dev1)
    hold on;
    plot_1 = plot(ts1, data1, 'color', 'red');
end

grid on;

% for i=1:length(seconds0)
%     line([seconds0(i), seconds0(i)], [0,15000], 'color', 'green', ...
%         'LineWidth', 3);
% end

sync0_ts = ts0(sync0 == TPSN_SYNC_WORD);
sync1_ts = ts1(sync1 == TPSN_SYNC_WORD);

for i=1:length(sync0_ts)
    sync_line_0 = line([sync0_ts(i), sync0_ts(i)], [0,15000], 'color', 'green', ...
        'LineWidth', 3);
end

for i=1:length(sync1_ts)
    sync_line_1 = line([sync1_ts(i), sync1_ts(i)], [0,15000], 'color', 'magenta', ...
        'LineWidth', 3);
end

xlabel('Timestamp [uS]');
ylabel('Voltage [mV]');

legend([plot_0, plot_1, sync_line_0, sync_line_1], 'Device 0', 'Device 1', 'Sync Dev 0', 'Sync Dev 1');

%%
% [x,y] = ginput(2); % click on each point
% dx = diff(x)

% set(gca,'XTick',unique(ts_s));
% set(gca,'XTickLabel',[4 8 15 21] );
% set(gca,'YGrid','off','XGrid','on');
