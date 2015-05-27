%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Add AO properties to the table of AO properties for this channel.
%%
%%  PROPERTIES = putAIProperty(OBJ, 'channelName')
%%  PROPERTIES = putAIProperty(OBJ, 'channelName', 'PROPERTY_NAME', 'PROPERTY_VALUE', ...)
%%
%%  Created - Tim O'Connor 11/129/04
%%
%%  Changed:
%%   TO062705I: Include the variable name with the stack traces. -- Tim O'Connor 6/27/05
%%   TO010606E: Optimization(s). Debugging messages are optional. -- Tim O'Connor 1/6/06
%%   TO022706D: Optimization(s). Use flags to determine if values really need to be set at "runtime". -- Tim O'Connor 2/27/06
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function val = putAIProperty(dm, name, varargin)
global gdm;

%Check the args.
if length(varargin) == 1
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

% lastSetEventString = sprintf( '''%s'' last set event - %s', name, getStackTraceString);%TO062705I
cr = sprintf('\n');
% lastSetEventString = strrep(lastSetEventString, cr, [cr '   ']);

aiProps = gdm(dm.ptr).channels(chIndex).aiProps;
for i = 1 : 2 : length(v) - 1
    rowIndex = getRowIndex(aiProps, v{i});

    %Replace it, if it exists.
    if rowIndex > -1
        gdm(dm.ptr).channels(chIndex).aiProps{rowIndex, 2} = v{i + 1};
        %TO010606E
        if gdm(dm.ptr).debugMessages
            gdm(dm.ptr).channels(chIndex).aiProps{rowIndex, 3} = strrep(sprintf( '''%s'':''%s'' last set event - %s', name, v{i}, getStackTraceString(1)), cr, [cr '   ']);%TO062705I
        else
            gdm(dm.ptr).channels(chIndex).aiProps{rowIndex, 3} = 'DEBUGGING_DISABLED';
        end
    else
        %Create a new entry in the table.
        x = size(gdm(dm.ptr).channels(chIndex).aiProps, 1) + 1;
        gdm(dm.ptr).channels(chIndex).aiProps(x, 1) = v{i};
        gdm(dm.ptr).channels(chIndex).aiProps(x, 2) = v{i + 1};
        %TO010606E
        if  gdm(dm.ptr).debugMessages
            gdm(dm.ptr).channels(chIndex).aiProps(x, 3) = strrep(sprintf( '''%s'':''%s'' last set event - %s', name, v{i}, getStackTraceString(1)), cr, [cr '   ']);%TO062705I
        else
            gdm(dm.ptr).channels(chIndex).aiProps(x, 3) = 'DEBUGGING_DISABLED';
        end
    end
    
    gdm(dm.ptr).channels(chIndex).aiPropsModificationFlags(rowIndex) = 1;%TO022706D
end

val = gdm(dm.ptr).channels(chIndex).aiProps;

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