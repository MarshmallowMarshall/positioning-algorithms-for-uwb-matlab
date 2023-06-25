function vrpn_position_data = extract_vrpn_ts_yaml(filename)
    % 打开文件并读取数据
    fid = fopen(filename, 'r');
    data = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);

    % 初始化变量
    vrpn_position_data = {};
    last_row = 1;
    timestamp_sec = 0;

    % 解析数据
    vrpn_position_start = false;
    for i = 1:length(data{1})
        line = data{1}{i};
        if strncmp(line, '# timestamp (sec):', 18)
            value_str = strsplit(line, ': ');
            timestamp_sec = str2double(value_str{2});
        elseif strncmp(line, 'position:', 9)
            vrpn_position_start = true;
        elseif vrpn_position_start && strncmp(line, 'x:', 2)
            value_str = strsplit(line, ': ');
            value = str2double(value_str{2});
            vrpn_position_data{end+1} = struct('vrpn_position', value, 'timestamp_sec', timestamp_sec);
        elseif vrpn_position_start && strncmp(line, 'y:', 2)
            value_str = strsplit(line, ': ');
            value = str2double(value_str{2});
            vrpn_position_data{end}.vrpn_position =[vrpn_position_data{end}.vrpn_position, value];
        elseif vrpn_position_start && strncmp(line, 'z:', 2)
            value_str = strsplit(line, ': ');
            value = str2double(value_str{2});
            vrpn_position_data{end}.vrpn_position =[vrpn_position_data{end}.vrpn_position, value];
            vrpn_position_start = false;
        end
    end
end
