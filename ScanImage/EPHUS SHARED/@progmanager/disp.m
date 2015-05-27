%DISP - Command-line display function for a progmanager object.
%  DISP displays some of the information which defines this program manager instance.
%  Specifically, it displays the information in the 'internal' field as well as a
%  list of running variables.
%
%  Created - Tim O'Connor 3/4/04
%
%  Changed:
%  
%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2004
function disp(obj)

fnames = fieldnames(evalin('base', [obj.name '.internal']));

fprintf(1, 'Program Manager (@progmanager)\n');

val = {};
for i = 1 : length(fnames)
    output{i} = evalin('base', [obj.name, '.internal.', fnames{i}]);
    
    if isnumeric(output{i})
        output{i} = ndArray2Str(output{i});
    else
        c = class(output{i});
    
        if strcmpi(c, 'cell')
            if isempty(output{i})
                output{i} = '{}';
            else
                output{i} = sprintf('cell array: %s', mat2str(size(output{i})));
            end
        elseif strcmpi(c, 'struct')
            output{i} = sprintf('struct: %s', mat2str(size(output{i})));
        elseif strcmpi(c, 'char')
            output{i} = ['''' output{i} ''''];
        else
            output{i} = sprintf('%s object: %s', c, mat2str(size(output{i})));
        end
    end

    lengths(i) = length(fnames{i});
end
 
m = max([lengths 16]);%16 comes from the 'Programs Running' string.
padding = ones(m + 4, 1) * ' ';

for i = 1 : length(fnames)
    fprintf(1, '   %s:%s%s\n', fnames{i}, padding(1 : m - lengths(i) + 3), output{i});
end
fprintf(1, '\n');

progs = showPrograms(obj);

fprintf(1, '   Programs Running:');
if isempty(progs)
    fprintf(1, '%sNone\n', padding(1 : m - 16 + 3));
else
    if length(progs) >= 1
        fprintf(1, '   %s%s\n', padding(1 : m - 16), progs{1});
    end
    for i = 2 : length(progs)
        fprintf(1, '   %s%s\n', padding, progs{i});
    end
end

return;