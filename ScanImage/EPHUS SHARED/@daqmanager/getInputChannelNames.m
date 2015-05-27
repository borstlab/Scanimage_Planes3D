%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Return a list (cell array of strings) of all existing input channels.
%%
%%  names = getInputChannelNames(OBJ)
%%
%%  Created - Tim O'Connor 11/12/04
%%
%%  Changed:
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2004
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function names = getInputChannelNames(dm)
global gdm;

names = {};

for i = 1 : length(gdm(dm.ptr).channels)
    if gdm(dm.ptr).channels(i).ioFlag == 1
        names{length(names) + 1} = gdm(dm.ptr).channels(i).name;
    end
end

return;