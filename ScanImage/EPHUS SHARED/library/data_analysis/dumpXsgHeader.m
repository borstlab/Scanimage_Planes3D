% dumpXsgHeader - Dump the entire header of an xsg file to the screen for review.
%
%  SYNTAX
%   dumpXsgHeader
%
%  NOTES
%   Relies on getStackTraceString and The Mathworks's (new as of version 7) lasterror function.
%
%  CHANGES
%
% Created 11/30/07 - Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2007
function dumpXsgHeader

[xsgData, fname] = selectAndLoadXsgFile;
if isempty(xsgData)
    return;
end

banner = '----------------------';
fprintf(1, '\n%s\n%s\ndumpXsgHeader - Dumping ''%s''...\n', banner, banner, fname);
dumpStruct(' header', xsgData.header);
fprintf(1, '\n%s\n%s\ndumpXsgHeader - Finished dumping ''%s''.\n', banner, banner, fname);

return;

%------------------------------
function dumpStruct(prefix, s)

fields = fieldnames(s);
if ischar(fields)
    fields = {fields};
end
for i = 1 : length(fields)
    recurse = 0;
    if length(s.(fields{i})) > 1
        if isstruct(s.(fields{i})(1))
            recurse = 1;
        end
    else
         if isstruct(s.(fields{i}))
             recurse = 1;
         end
    end
    if recurse
        for j = 1 : length(s.(fields{i}))
            dumpStruct([prefix '.' fields{i} '(' num2str(j) ')'], s.(fields{i})(j));
        end
    else
        fprintf(1, '%s.%s\n', prefix, fields{i});
        disp(s.(fields{i}));
    end
end

return;