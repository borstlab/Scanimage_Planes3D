function updateRootPathIsDefault(handle)
%% function updateRootPathIsDefault(handle)
% Callback function that handles update to 'Root Path is Default Path' user setting
%
%% NOTES
%   Function is an INI-file callback, so it is invoked either upon adjusting the GUI control or loading a USR file
%
%% CREDITS
%   Created 7/12/10, by Vijay Iyer
%% ******************************************************************
global state 

if state.files.rootPathIsDefault
    if isempty(state.files.savePath)
        if ~isempty(state.files.rootSavePath)
            state.files.savePath = state.files.rootSavePath;
        end
    end
end



        
        