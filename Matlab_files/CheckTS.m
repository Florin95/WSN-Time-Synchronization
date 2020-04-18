function [ts, data, seconds] = CheckTS(fileName)

fileID = fopen(fileName, 'r');

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

% Make the vectors equal in length
len = min([length(ts_s), length(ts_f), length(data)]);
ts_s = ts_s(1:len);
ts_f = ts_f(1:len);
data = data(1:len);

% Remove time offset
ts_s = ts_s - min(ts_s);

% Get the device id for each packet
device_id = zeros(length(data), 1);

for i = 1:length(data)
    device_id(i) = bitand(data(i), hex2dec('FF'), 'int32');
end

data = bitsra(data,8);
% Convert to microvolts
data = data*0.044;

% Convert to us
ts_s = ts_s .* 1000000;
% Get the whole seconds
seconds = unique(ts_s);
% Add the fractional part (in microseconds)
ts_s = ts_s + ts_f;
% Remove the offset
ts_s = ts_s - min(ts_s);

ts = ts_s;

end




