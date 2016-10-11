%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Add AO properties to the table of AO properties for this channel.
%%
%%  PROPERTIES = putAOProperty(OBJ, 'channelName')
%%  PROPERTIES = putAOProperty(OBJ, 'channelName', 'PROPERTY_NAME', 'PROPERTY_VALUE', ...)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%   TO062705I: Include the variable name with the stack traces. -- Tim O'Connor 6/27/05
%%   TO010606E: Optimization(s). Debugging messages are optional. -- Tim O'Connor 1/6/06
%%   TO022706D: Optimization(s). Use flags to determine if values really need to be set at "runtime". -- Tim O'Connor 2/27/06
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function val = putAOProperty(dm, name, varargin)
global gdm;

%Check the args.
if length(varargin) == 1 & strcmpi(class(varargin), 'cell')
    v = varargin{1};
end
if mod(length(v), 2) ~= 0
    error('Wrong number of arguments.');
end

chIndex = getChannelIndex(dm, name);
if ~chIndex
    errmsg = sprintf('No channel found with name: %s.', name);
    error(errmsg);
end

aoProps = gdm(dm.ptr).channels(chIndex).aoProps;
% lastSetEventString = sprintf( '''%s'' last set event - %s', name, getStackTraceString);%TO062705I
cr = sprintf('\n');
% lastSetEventString = strrep(lastSetEventString, cr, [cr '   ']);
for i = 1 : 2 : length(v) - 1
    rowIndex = getRowIndex(aoProps, v{i});

    %Replace it, if it exists.
    if rowIndex > -1
        aoProps{rowIndex, 2} = v{i + 1};
        %TO010606E
        if gdm(dm.ptr).debugMessages
            aoProps{rowIndex, 3} = strrep(sprintf( '''%s'':''%s'' last set event - %s', name, v{i}, getStackTraceString), cr, [cr '   ']);%TO062705I
        else
            aoProps{rowIndex, 3} = 'DEBUGGING_DISABLED';
        end
    else
        %Create a new entry in the table.
        x = size(aoProps, 1) + 1;
        aoProps(x, 1) = v{i};
        aoProps(x, 2) = v{i + 1};
        %TO010606E
        if gdm(dm.ptr).debugMessages
            aoProps{x, 3} = strrep(sprintf( '''%s'':''%s'' last set event - %s', name, v{i}, getStackTraceString), cr, [cr '   ']);%TO062705I
        else
            aoProps{x, 3} = 'DEBUGGING_DISABLED';
        end
    end
    
    gdm(dm.ptr).channels(chIndex).aoPropsModificationFlags(rowIndex) = 1;%TO022706D
end

gdm(dm.ptr).channels(chIndex).aoProps = aoProps;
val = aoProps;

%Make sure this property doesn't conflict with other channels on this board.
warnings = generateConflictWarnings(dm, name);
if ~isempty(warnings)
    warningMsg = 'Possible data acquisition board conflicts found:\n';
    
    for i = 1 : length(warnings)
        warningMsg = strcat(warningMsg, '  ', warnings{i}, '\n');
    end
    
    warning(errMsg);
end

return;