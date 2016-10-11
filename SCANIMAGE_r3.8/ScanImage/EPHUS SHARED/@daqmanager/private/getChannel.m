%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Retrieve a named channel.
%%
%%  CHANNEL = getChannel(OBJ, channelName)
%%
%%  Created - Tim O'Connor 11/6/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function channel = getChannel(dm, channelName)
global gdm;

if isempty(channelName)
    error('channelName must not be empty.');
end

channel = [];

for i = 1 : length(gdm(dm.ptr).channels)
    if strcmp(gdm(dm.ptr).channels(i).name, channelName)
        channel = gdm(dm.ptr).channels(i);
        
        return
    end
end

return;