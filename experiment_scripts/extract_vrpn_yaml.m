function vrpn_position_data = extract_vrpn_yaml(filename)
%EXTRACT_UWBRANGE_YAML 此处显示有关此函数的摘要
%   此处显示详细说明
fid = fopen(filename, 'r');
data = textscan(fid, '%s', 'Delimiter', '\n');
fclose(fid);

% 初始化变量
vrpn_position_data = [];
last_row = 1;

% 解析数据
vrpn_position_start = false;
for i = 1:length(data{1})
    flag_valid = true;
    line = data{1}{i};
    if strncmp(line, 'position: ', 10)
        vrpn_position_start = true;
    elseif vrpn_position_start && strncmp(line, 'x:', 2)
        value_str = strsplit(line, ': ');
        value = str2double(value_str{2});
        vrpn_position_data(last_row,1) = value;
    elseif vrpn_position_start && strncmp(line, 'y:', 2)
        value_str = strsplit(line, ': ');
        value = str2double(value_str{2});
        vrpn_position_data(last_row,2) = value;
    elseif vrpn_position_start && strncmp(line, 'z:', 2)
        value_str = strsplit(line, ': ');
        value = str2double(value_str{2});
        vrpn_position_data(last_row,3) = value;
        vrpn_position_start = false;
        last_row=last_row+1;
    end
end
end

