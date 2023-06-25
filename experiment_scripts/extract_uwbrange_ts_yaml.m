function range_raw_data = extract_uwbrange_ts_yaml(filename)
    fid = fopen(filename, 'r');
    data = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);

    % 初始化变量
    range_raw_data = {};

    % 解析数据
    range_raw_start = false;
    timestamp_start = false;
    timestamp_sec = 0;
    for i = 1:length(data{1})
        line = data{1}{i};
        
        if strncmp(line, 'range_raw:', 10)
            range_raw_start = true;
            timestamp_start = false;
        elseif strncmp(line, '# timestamp (sec):', 18)
            timestamp_sec = str2double(line(strfind(line, ':')+1:end));
        elseif strncmp(line, 'timestamp_resp_local:', 21)
            range_raw_start = false;
            timestamp_start = true;
        elseif range_raw_start && strncmp(line, 'data:', 5)
            values_str = strsplit(line(strfind(line, '[')+1:strfind(line, ']')-1), ',');
            values = str2double(values_str(~cellfun(@isempty, values_str)));
            range_raw_data{end+1} = struct('range_raw', values, 'timestamp_resp_local', [], 'timestamp_sec', timestamp_sec);
        elseif timestamp_start && strncmp(line, 'data:', 5)
            values_str = strsplit(line(strfind(line, '[')+1:strfind(line, ']')-1), ',');
            values = str2double(values_str(~cellfun(@isempty, values_str)));
            if numel(range_raw_data) > 0
                range_raw_data{end}.timestamp_resp_local = values;
            end
        end
    end
end
