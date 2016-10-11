%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Return a list (cell array of strings) of all existing channels.
%%
%%  names = getChannelNames(OBJ)
%%
%%  Created - Tim O'Connor 12/16/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function names = getChannelNames(dm)
global gdm;

names = {};

for i = 1 : length(gdm(dm.ptr).channels)
    names{i} = gdm(dm.ptr).channels(i).name;
end

return;