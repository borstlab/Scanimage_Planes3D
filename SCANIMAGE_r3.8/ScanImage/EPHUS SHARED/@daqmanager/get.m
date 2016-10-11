%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Get object properties.
%%
%%  value = get(OBJ, PropertyName, ...)
%%
%%  Returns the value of the named property/properties.
%%
%%  Created - Tim O'Connor 4/1/03
%%
%%  Changed:
%%    Tim O'Connor 1/24/05 TO012405a: Print out all fields with no args.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function value = get(dm, varargin)
global gdm;

names = fieldnames(gdm(dm.ptr));

%TO012405a
if isempty(varargin)
    for i = 1 : length(names)
        value.(names{i}) = gdm(dm.ptr).(names{i});
    end
end

for i = 1 : 2 : length(varargin)
    matches = find(strcmpi(varargin{i}, names) == 1);
    if matches
        if length(matches) > 1
            %Choose the first hit, there should only be one though.
            matches = matches(1);
        end
        value{i} = gdm(dm.ptr).(names{matches});
    else
        error('Invalid property name: ''%s''', varargin{i});
    end
end

if length(varargin) == 1 & strcmpi(class(value), 'cell') & length(value) == 1
    value = value{1};
end 

return;