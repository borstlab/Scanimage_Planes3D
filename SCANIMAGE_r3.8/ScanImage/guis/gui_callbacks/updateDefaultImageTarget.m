function updateDefaultImageTarget(handle)
%% function updateDefaultImageTarget(handle)
% Callback function that handles update to the default image target popup menu
%
%% NOTES
%   Being an INI-named callback allows this to be called during either a GUI control or CFG/USR file loading event
%
%% CREDITS
%   Created 2/20/09, by Vijay Iyer
%% ******************************************************************
global state gh

switch state.internal.defaultImageTargetGUI
    case 1
        state.internal.defaultImageTarget = [];
    case {2 3 4 5}
        state.internal.defaultImageTarget = state.internal.defaultImageTargetGUI - 1;
    case 6
        state.internal.defaultImageTarget = inf; %this encodes 'merge' possibility
end



        
        