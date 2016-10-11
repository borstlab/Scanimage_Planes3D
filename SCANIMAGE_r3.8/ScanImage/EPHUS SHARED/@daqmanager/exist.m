%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Test for existence of this object.
%%
%%  existence = exist(OBJ)
%%
%%  existence == 1 if the object exists, 0 otherwise.
%% 
%%  Created - Tim O'Connor 11/27/04
%%
%%  Changed:
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function existence = exist(dm)
global gdm;

existence = 0;

%The global repository exists.
%   The class is 'daqmanager'.
%       The pointer is valid.
%           The repository contains an entry for the pointer.
if exist('gdm') == 1 & ...
    strcmpi(class(dm), 'daqmanager') & ...
        dm.ptr > 0 & ...
            length(gdm) >= dm.ptr
    existence = 1;
end

return;