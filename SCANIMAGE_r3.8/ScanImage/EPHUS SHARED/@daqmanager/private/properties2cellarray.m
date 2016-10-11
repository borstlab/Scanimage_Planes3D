%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Convert an object's properties into an N * 2 cell array (a table).
%%
%%  array = properties2cellarray(OBJ)
%%
%%  Created - Tim O'Connor 11/13/03
%%
%%  Changed:
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function properties = properties2cellarray(obj, ignorelist)

properties = {};

propStruct = set(obj);
names = fieldnames(propStruct);

%Pick up the current properties.
n = 1;
for i = 1 : length(names)

    name = names(i);
    if strcmpi(class(name), 'cell')
        name = name{1};
    end
    
    if ~ismember(name, ignorelist)
        properties{n, 1} = names{i};
        v = get(obj, names(i));
        properties{n, 2} = v{:};
        n = n + 1;
    else
%         fprintf(1, 'Ignoring %s\n', name);
    end
end

return;