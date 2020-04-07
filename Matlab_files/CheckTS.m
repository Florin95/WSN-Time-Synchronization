clc; clear;

fileID = fopen('log.log', 'r');

A = fread(fileID, Inf, 'int32');
fclose(fileID);

% Extract each field
ts_s = A(1:3:end);
ts_f = A(2:3:end);
data = A(3:3:end);
clear A;

% Get the number of packets prior to synchronizing
items_to_remove = length(ts_s(ts_s <= 1000)) + 1;

% Remove the packets received prior to synchronizing
ts_s = ts_s(items_to_remove:end);
ts_f = ts_f(items_to_remove:end);
data = data(items_to_remove:end);

% Remove time offset
ts_s = ts_s - ts_s(1);

% Get the device id for each packet
device_id = zeros(length(data), 1);

for i = 1:length(data)
    device_id(i) = bitand(data(i), hex2dec('FF'), 'int32');
end

data = bitsra(data,8);
% Convert to microvolts
data = data*0.022;

% Convert to us
ts_s = ts_s .* 1000000;
% Get the whole seconds
seconds = unique(ts_s);
% Add the fractional part (in microseconds)
ts_s = ts_s + ts_f;
% Remove the offset
ts_s = ts_s - ts_s(1);

plot(ts_s, data);

for i=1:length(seconds)
    line([seconds(i), seconds(i)], [-300000,300000], 'color', 'red');
end

% set(gca,'XTick',unique(ts_s));
% set(gca,'XTickLabel',[4 8 15 21] );
% set(gca,'YGrid','off','XGrid','on');







