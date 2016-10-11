%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Return the number of named channels.
%%  Alternate forms allow collection of more counted variables.
%%
%%  namedChannels = size(OBJ)
%%
%%  [namedChannels analogOuputObjects] = size(OBJ)
%%
%%  Created - Tim O'Connor 11/11/03
%%
%%  Changed:
%%           1/27/04 Tim O'Connor TO12704a: Use "pointers". See daqmanager.m for details.
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function varargout = size(dm)
global gdm;

varargout{1} = length(gdm(dm.ptr).channels);
varargout{2} = length(gdm(dm.ptr).aos);

return;