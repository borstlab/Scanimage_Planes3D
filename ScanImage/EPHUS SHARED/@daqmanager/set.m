%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Set object properties.
%%
%%  set(OBJ, PropertyName, PropertyValue, ...);
%%
%%  Sets the value of the named property/properties.
%%
%%  Created - Tim O'Connor 4/1/03
%%
%%  Changed:
%%    Tim O'Connor 1/24/05 TO012405a: Print out all fields with no args.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function value = set(dm, varargin)
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
    if any(matches)
        if length(matches) > 1
            %Choose the first hit, there should only be one though.
            matches = matches(1);
        end
        gdm(dm.ptr).(names{matches}) = varargin{i + 1};
    else
        error('Invalid property name: ''%s''', varargin{i});
    end
end

return;