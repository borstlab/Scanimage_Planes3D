%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Show detailed information about an analogoutput object.
%%
%%  displayAODetails(OBJ, channelName, ...)
%%
%%  channelName - Which channel's board to display.
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function displayAODetails(dm, varargin)

for i = 1 : length(varargin)
    identifier = varargin{i};

    if strcmpi(class(identifier), 'cell')
        identifier = identifier{1};
    end

    ao = getAO(dm, identifier);
    
    if isempty(ao)
        continue;
    end
    
    names = fieldnames(set(ao));
    
    fprintf(1, 'AnalogOutput Details for ''%s'':\n', identifier);
    
    for i = 1 : length(names)
        name = names(i);
        if strcmpi(class(name), 'cell')
            name = name{1};
        end

        val = get(ao, name);
        
        if isempty(val)
            fprintf(1, '   %s: EMPTY\n', name);
        elseif strcmpi(class(val), 'cell')
            celldisp(val, name);
        elseif isnumeric(val)
            fprintf(1, '   %s: %s\n', name, num2str(val));
        elseif strcmpi(class(val), 'char')
            fprintf(1, '   %s: %s\n', name, val);
        else
            fprintf(1, '   %s: %s of size [%s]\n', name, class(val), num2str(size(val)));
        end
    end
    
end