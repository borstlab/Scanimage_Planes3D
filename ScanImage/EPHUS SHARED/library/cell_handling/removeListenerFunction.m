% removeListenerFunction - Takes a cell array of listener functions, and removes
% any that match the supplied profile.
%
% SYNTAX
%  trimmedArray = removeListenerFunction(arrayOfCallbacks, profile)
%   arrayOfCallbacks - A cell array, containing executable callbacks (strings, function_handles, 
%                      cell arrays beginning with a function_handle).
%   profile - A string, function_handle, or cell array beginning with a function_handle. Matches occur under these 3 conditions:
%                case-insensitive string equality
%                function handle equality
%                cell array length and first element function_handle equality (subsequent elements are ignored)
%
% USAGE
%
% NOTES:
%
% CHANGES:
%
% Created 2/3/05 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2005
function trimmed = removeListenerFunction(callbacks, profile)

pc = class(profile);
if ~ismember(lower(pc), 'cell', 'char', 'function_handle')
    error('Callbacks must be a cell array, string, or function_handle.');
end

trimmed = {};

cellArray = strcmpi(pc, 'cell');
string = strcmpi(pc, 'char');
if ~(string | cellArray)
    funcstr = funct2str(profile);
else
    funcstr = '';
end

for i = 1 : length(callbacks)
    c = class(callbacks{i});
    if strcmp(c, pc)
        if cellArray
            arr = callbacks{i};
            if profile{1} ~= arr{1}
                trimmed{length(trimmed) + 1} = callbacks{i};
            end
        else
            if string
                if strcmpi(callbacks{i}, profile)
                    trimmed{length(trimmed) + 1} = callbacks{i};
                end
            else
                if strcmpi(funcstr, func2str(callbacks{i}))
                    trimmed{length(trimmed) + 1} = callbacks{i};
                end
            end
        end
    end
end

return;