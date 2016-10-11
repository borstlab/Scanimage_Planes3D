function list = delimitedList(string, varargin)
% DELIMITELIST(string) - returns a cell array containing the char values in string,
%     separated by whitespace.
%
% DELIMITEDLIST(string, delimiter) - returns a cell array containing the char values in string,
%     separated by delimiter. Any leading/trailing whitespace is stripped.
%
% Note: Empty strings in the list will be pruned.
% 
% Example:
%     DELIMITEDLIST('a, b, c')
%     ans = 
%            {'a,', 'b,', 'c'}
%     DELIMITEDLIST('a, b, c', ',')
%     ans = 
%            {'a', 'b', 'c'}
%
% Created: Timothy O'Connor 4/22/04
% Copyright: Cold Spring Harbor Laboratories/Howard Hughes Medical Institute

%Make sure it always returns something, even if it's empty. -- Tim O'Connor 9/20/04 TO092004b
list = {};

if isempty(string)
    return;
end

if length(varargin) > 1
    error('Matlab:badopt', 'Too many input arguments to `delimitedString`.');
end

delimiter = [];
if length(varargin) == 1
    if ~strcmpi(class(varargin{1}), 'char')
        error('Matlab:badopt', 'Invalid delimiter class, must be a char: %s', class(varargin{1}));
    end
    
    delimiter = varargin{1};
    
    if length(delimiter) > 1
        error('Matlab:badopt', 'The delimiter may only be a single character: %s', delimiter);
    end
end

if ~isempty(delimiter)
    list = cell(length(find(string == delimiter)) + 1, 1);
else
    list = {};
end

i = 1;
while ~isempty(string)
    if isempty(delimiter)
        [list{i} string] = strtok(string);
    else
        [element string] = strtok(string, delimiter);
        if ~isempty(element)
            nonspaces = find(~isspace(element));
            if ~isempty(nonspaces)
                list{i} = element(nonspaces(1) : nonspaces(end));
            end
        end
    end
    
    i = i + 1;
end

i = length(list);
while isempty(list{i}) & i > 0
    i = i - 1;
end

if i ~= length(list)
    list = list(1 : i);
end

return;