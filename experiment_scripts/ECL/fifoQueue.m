classdef fifoQueue < handle
    properties
        buffer
        maxLength
    end
    
    methods
        % 构造函数
        function obj = fifoQueue(maxLength)
            obj.maxLength = maxLength;
            obj.buffer = zeros(maxLength, 1);
        end
        
        % 新数据入队
        function enqueue(obj, data)
            % 将新数据添加到队列头部
            obj.buffer = [data; obj.buffer];
            
            % 如果队列超过了最大长度，删除队尾的数据
            if length(obj.buffer) > obj.maxLength
                obj.buffer = obj.buffer(1:obj.maxLength);
            end
        end
        
        % 输出当前的队列
        function disp(obj)
            disp(obj.buffer);
        end
    end
end

