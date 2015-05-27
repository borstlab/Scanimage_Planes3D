function updateExportedClockSettings(handle)
%% function updateExportedClockSettings
%   Handles updates to many of the exported clock settings set in the EXPORTED CLOCKS... GUI
%% SYNTAX
%   updateExportedClockSettings()
%   updateExportedClockSettings(handle)
%       handle: A GUI handle
%
%% NOTES
%       This function is an INI-named callback, so that it is invoked both on GUI changes and CFG/USR file loading -- Vijay Iyer 10/7/10
%
%% CREDITS
%   Created 10/7/10, by Vijay Iyer.
%% *******************************************************************************************************

global state

import dabs.ni.daqmx.*

persistent allClocks

clockTypes = {'frame' 'line' 'pixel'};

%Fill in active clock list
if isempty(allClocks) || ~all(arrayfun(@isvalid,allClocks))
    allClocks = [state.init.hFrameClkCtr state.init.hLineClkCtr state.init.hPixelClkCtr];
end

state.acq.hClockTasks = Task.empty();
for i=1:length(clockTypes)
    if ~isempty(state.init.([clockTypes{i} 'ClockBoardID'])) && state.acq.clockExport.([clockTypes{i} 'ClockEnable'])    
        state.acq.hClockTasks(end+1) = allClocks(i);
    end
end

return;

