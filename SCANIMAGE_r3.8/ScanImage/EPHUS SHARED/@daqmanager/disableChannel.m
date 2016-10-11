%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Deactivate a channel for sending/recieving data.
%%
%%  A warning will be generated if the channel is already disabled.
%%
%%  Returns the number of channels that have been disabled.
%%
%%  count = disableChannel(OBJ, 'name', ...)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function count = disableChannel(dm, varargin)
global gdm;

count = 0;

if nargin == 2
    name = varargin{1};
else
    for i = i : length(varargin)
        count  = count + disableChannel(dm, varargin{i});
    end
    
    return;
end

index = getChannelIndex(dm, name);
if index < 1
    errMsg = sprintf('Could not find channel ''%s''.', name);
    error(errMsg);
end

if gdm(dm.ptr).channels(index).state == 0
    warnmsg = sprintf('Channel %s is already disabled.', name);
    warning(warnmsg);
elseif gdm(dm.ptr).channels(index).state == 2
    %The channel must be stopped.
    stopChannel(dm, name);
    
    warnmsg = sprintf('Request to disable channel %s has forced it to be automatically stopped.', name);
    warning(warnmsg);
    
    %Toggle the flag.
    gdm(dm.ptr).channels(index).state = 0; 
    
    count = count + 1;
else
    %Toggle the flag.
    gdm(dm.ptr).channels(index).state = 0; 
    
    count = count + 1;
end

return;