clc; clear;

fileID = fopen('C:\Users\Nastase\Desktop\log.log', 'r');

A = fread(fileID, Inf, 'int32');
fclose(fileID);

% Extract each field
ts_s = A(1:3:end);
ts_f = A(2:3:end); 
data = A(3:3:end);
clear A;

% Get the device id for each packet
device_id = zeros(length(data), 1);

for i = 1:length(data)
    device_id(i) = bitand(data(i), hex2dec('FF'), 'int32');
end

ts_dev_0 = ts_s(device_id == 0);
ts_f_dev_0 = ts_f(device_id == 0);
data_dev_0 = data(device_id == 0);

ts_dev_1 = ts_s(device_id == 1);
ts_f_dev_1 = ts_f(device_id == 1);
data_dev_1 = data(device_id == 1);

% Get the number of packets prior to synchronizing
items_to_remove_0 = length(ts_dev_0(ts_dev_0 <= 1000)) + 1;
items_to_remove_1 = length(ts_dev_1(ts_dev_1 <= 1000)) + 1;

% Remove the packets received prior to synchronizing
ts_dev_0 = ts_dev_0(items_to_remove_0:end);
ts_f_dev_0 = ts_f_dev_0(items_to_remove_0:end);
data_dev_0 = data_dev_0(items_to_remove_0:end);

ts_dev_1 = ts_dev_1(items_to_remove_1:end);
ts_f_dev_1 = ts_f_dev_1(items_to_remove_1:end);
data_dev_1 = data_dev_1(items_to_remove_1:end);

% Make the vectors equal in length
len = min([length(ts_dev_0), length(ts_f_dev_0), length(data_dev_0)]);
ts_dev_0 = ts_dev_0(1:len);
ts_f_dev_0 = ts_f_dev_0(1:len);
data_dev_0 = data_dev_0(1:len);

len = min([length(ts_dev_1), length(ts_f_dev_1), length(data_dev_1)]);
ts_dev_1 = ts_dev_1(1:len);
ts_f_dev_1 = ts_f_dev_1(1:len);
data_dev_1 = data_dev_1(1:len);

% Remove time offset
ts_dev_0 = ts_dev_0 - min(ts_dev_0);
ts_dev_1 = ts_dev_1 - min(ts_dev_1);

data_dev_0 = bitsra(data_dev_0,8);
data_dev_1 = bitsra(data_dev_1,8);

% Convert to microvolts
data_dev_0 = data_dev_0*0.044;
data_dev_1 = data_dev_1*0.044;

% Convert to us
ts_dev_0 = ts_dev_0 .* 1000000;
% Get the whole seconds
seconds = unique(ts_dev_0);
% Add the fractional part (in microseconds)
ts_dev_0 = ts_dev_0 + ts_f_dev_0;
% Remove the offset
ts_dev_0 = ts_dev_0 - min(ts_dev_0);

% Convert to us
ts_dev_1 = ts_dev_1 .* 1000000;
% Get the whole seconds
seconds1 = unique(ts_dev_1);
% Add the fractional part (in microseconds)
ts_dev_1 = ts_dev_1 + ts_f_dev_1;
% Remove the offset
ts_dev_1 = ts_dev_1 - min(ts_dev_1);

plot(ts_dev_0, data_dev_0);
plot(ts_dev_1, data_dev_1, 'color', 'red');

% for i=1:length(seconds)
%     line([seconds(i), seconds(i)], [0,150000], 'color', 'red');
% end

% [x,y] = ginput(2); % click on each point
% dx = diff(x)

% set(gca,'XTick',unique(ts_s));
% set(gca,'XTickLabel',[4 8 15 21] );
% set(gca,'YGrid','off','XGrid','on');







