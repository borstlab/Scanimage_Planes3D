%% function updateUserFcnPath(varargin)
% Handle changes to User Fcn path
%% NOTES
%   Being an INI-named callback allows this to be called during either a GUI control or CFG/USR file loading event
%
%   This function consolidates duplicate code that was in userFcnGUI() and openAndLoadUserSettings() -- Vijay Iyer 12/01/09
%% CREDITS
%   Created 12/1/09 by Vijay Iyer
%% ********************************

function updateUserFcnPath(varargin)

global state gh

if isdir(state.userFcnGUI.UserFcnPath)    
   
    files=dir([state.userFcnGUI.UserFcnPath '*.m']);
    state.userFcnGUI.UserFcnFiles = sortrows({files.name}'); % Sort names
    
    if ~isempty(state.userFcnGUI.UserFcnFiles)
        set(gh.userFcnGUI.UserFcnBrowser,'String',state.userFcnGUI.UserFcnFiles);
    else
        set(gh.userFcnGUI.UserFcnBrowser,'String',{});
    end
end
    
%%% Following approach was considered, but rejected, as it's too much hassle to actually use path to determine userFcn to execute -- Vijay Iyer 12/1/09
%     if ~isempty(state.userFcnGUI.UserFcnSelected)
%         set(gh.userFcnGUI.UserFcnBrowser,'Value',1,'String',state.userFcnGUI.UserFcnFiles);
%         
%         if ~isempty(state.userFcnGUI.UserFcnSelected)
%             selFuncsCell = transformStringListType(state.userFcnGUI.UserFcnSelected);
%             badIndices = [];
%             for i=1:length(selFuncsCell)
%                 if ~ismember(selFuncsCell{i},state.userFcnGUI.UserFcnFiles)
%                     badIndices(end+1) = i;
%                 end
%             end
%             selFuncsCell(badIndices) = [];              
%                       
%             set(gh.userFcnGUI.UserFcnSelected,'Value',1,'String', selFuncsCell);
%         else
%             set(gh.userFcnGUI.UserFcnSelected,'Value',1,'String','');
%         end
%         
%     else
%         set(gh.userFcnGUI.UserFcnBrowser,'Value',1,'String','');        
%         set(gh.userFcnGUI.UserFcnSelected,'Value',1,'String','');
%     end    
%        


