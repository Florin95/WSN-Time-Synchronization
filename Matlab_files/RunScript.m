clc; clear;

TPSN_SYNC_WORD = hex2dec('ABABABAB');
ALIGNMENT_WORD = hex2dec('A5A5A5A5');

en_dev1 = 1;
path1 = 'D:\Facultate\MASTER\Anul_2\Disertatie\PSoC6\Workspace\Connectivity_Secure_TCP_Client\Matlab_files\SNTP\Attempt4_SNTP_RESULT_1';
path2 = 'D:\Facultate\MASTER\Anul_2\Disertatie\PSoC6\Workspace\Connectivity_Secure_TCP_Client\Matlab_files\SNTP\Attempt5_SNTP_25_minutes';
path3 = 'D:\Facultate\MASTER\Anul_2\Disertatie\PSoC6\Workspace\Connectivity_Secure_TCP_Client\Matlab_files\SNTP\Attempt3_data';

path4 = 'D:\Facultate\MASTER\Anul_2\Disertatie\PSoC6\Workspace\Connectivity_Secure_TCP_Client\Matlab_files\TPSN\Attempt3_TPSN_20_minutes';

path = path4;
fileName0 = strcat(path, '\dev0.log');
fileName1 = strcat(path, '\dev1.log');

[sync0, ts0, data0, seconds0] = CheckTS(fileName0);
if (en_dev1)
    [sync1, ts1, data1, seconds1] = CheckTS(fileName1);
end

min_ts0 = min(ts0);
min_ts1 = min(ts1);
min_ts = min([min_ts0, min_ts1]);

ts0 = ts0 - min_ts;
ts1 = ts1 - min_ts;

plot_0 = plot(ts0, data0);

if (en_dev1)
    hold on;
    plot_1 = plot(ts1, data1, 'color', 'red');
end

grid on;
enable_TPSN_sync_points_mark = 0;

sync0_ts = ts0(sync0 == TPSN_SYNC_WORD);
sync1_ts = ts1(sync1 == TPSN_SYNC_WORD);
sync_line_0 = [];
sync_line_1 = [];
    
if (enable_TPSN_sync_points_mark)
    for i=1:length(sync0_ts)
        sync_line_0 = line([sync0_ts(i), sync0_ts(i)], [0,15000], 'color', 'green', ...
            'LineWidth', 3);
    end

    for i=1:length(sync1_ts)
        sync_line_1 = line([sync1_ts(i), sync1_ts(i)], [0,15000], 'color', 'magenta', ...
            'LineWidth', 3);
    end
end

xlabel('Timestamp [uS]');
ylabel('Voltage [mV]');

legend([plot_0, plot_1, sync_line_0, sync_line_1], 'Device 0', 'Device 1', 'Sync Dev 0', 'Sync Dev 1');

%%
[x,y] = ginput(2); % click on each point
dx = diff(x)



