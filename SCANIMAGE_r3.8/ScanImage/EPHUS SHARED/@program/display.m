function display(obj)
%DISPLAY - Command-line display function for a program object.
%  DISPLAY displays some of the information which defines this program instance.
%  Specifically, it displays the information in the 'top' field as well as a
%  list of program names and mfilenames.
%
%  This is also the default way of displaying a program object, say
%  from a call to program
%
%  See also EDITPROGRAM 

%  Created - Tom Pologruto 3/8/04
%
%  Changed:
%
%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor
%  Laboratories 2004

obj_struct=struct(obj);

fnames = fieldnames(obj_struct);
maxstringsize=max(cellfun('size',fnames,2));

fprintf(1, 'Program (@program)\n');

val = {};
output={};
for i = 1 : length(fnames)
    output{i} = obj_struct.(fnames{i});
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

guititle='   GUIs Included:';
m=max(maxstringsize,length(guititle));
padding = repmat(' ',1,m+4);

for i = 1 : length(fnames)
    fprintf(1, '   %s:%s%s\n', fnames{i}, padding(1 : m - lengths(i) + 3), output{i});
end
fprintf(1, '\n');

progs=[];
if isstruct(obj_struct.aliases)
    progs = fieldnames(obj_struct.aliases);
end
fprintf(1, guititle);
if isempty(progs)
    fprintf(1, '%sNone\n', padding);
else
    if length(progs) >= 1
        fprintf(1, '   %s%s:   %s\n', padding(1:m-length(guititle)+4), progs{1},obj_struct.aliases.(progs{1}).m_filename);
    end 
    for i = 2 : length(progs)
        fprintf(1, '   %s%s:   %s\n',   padding, progs{i},obj_struct.aliases.(progs{i}).m_filename);
    end
end
