function range_raw_data = extract_uwbrange_yaml(filename)
%EXTRACT_UWBRANGE_YAML 此处显示有关此函数的摘要
%   此处显示详细说明
fid = fopen(filename, 'r');
data = textscan(fid, '%s', 'Delimiter', '\n');
fclose(fid);

% 初始化变量
range_raw_data = {};

% 解析数据
range_raw_start = false;
timestamp_start = false;
for i = 1:length(data{1})
    flag_valid = true;
    line = data{1}{i};
    if strncmp(line, 'range_raw:', 10)
        range_raw_start = true;
    elseif range_raw_start && strncmp(line, 'data:', 5)
        values_str = strsplit(line(strfind(line, '[')+1:strfind(line, ']')-1), ',');
        values = str2double(values_str(~cellfun(@isempty, values_str)));
        for j = 1:length(values)-4
            if values(j) < 0
                flag_valid = false;
            end
        end
        if flag_valid == true
            range_raw_data{end+1} = values;
        end
        range_raw_start = false;
    elseif timestamp_start && strncmp(line, 'data:', 5)
        
    end
end


end

