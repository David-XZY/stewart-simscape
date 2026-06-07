function value = getStatSafe(stats, fieldPath, defaultValue)
% getStatSafe - 鲁棒读取 CasADi/IPOPT/FATROP stats 字段
if nargin < 3
    defaultValue = NaN;
end
value = defaultValue;
if isempty(stats) || ~isstruct(stats)
    return;
end

parts = strsplit(char(string(fieldPath)), '.');
cursor = stats;
for i = 1:numel(parts)
    if isstruct(cursor) && isfield(cursor, parts{i})
        cursor = cursor.(parts{i});
    else
        return;
    end
end

if isnumeric(cursor) || islogical(cursor)
    value = double(cursor);
elseif ischar(cursor) || isstring(cursor)
    numericValue = str2double(char(string(cursor)));
    if ~isnan(numericValue)
        value = numericValue;
    else
        value = string(cursor);
    end
else
    value = cursor;
end
end
