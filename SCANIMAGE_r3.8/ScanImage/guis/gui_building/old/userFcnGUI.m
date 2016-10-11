function varargout = userFcnGUI(varargin)
% userFcnGUI Application M-file for userFcnGUI.fig
%    FIG = userFcnGUI launch userFcnGUI GUI.
%    userFcnGUI('callback_name', ...) invoke the named callback.

% Last Modified by GUIDE v2.5 23-Feb-2008 15:13:48

if nargin == 0  % LAUNCH GUI
    
    fig = openfig(mfilename,'reuse');
    
    
    % Generate a structure of handles to pass to callbacks, and store it. 
    handles = guihandles(fig);
    guidata(fig, handles);
    
    if nargout > 0
        varargout{1} = fig;
    end
    
elseif ischar(varargin{1}) % INVOKE NAMED SUBFUNCTION OR CALLBACK
    
    try
        if (nargout)
            [varargout{1:nargout}] = feval(varargin{:}); % FEVAL switchyard
        else
            feval(varargin{:}); % FEVAL switchyard
        end
    catch
        disp(lasterr);
    end
    
end


%| ABOUT CALLBACKS:
%| GUIDE automatically appends subfunction prototypes to this file, and 
%| sets objects' callback properties to call them through the FEVAL 
%| switchyard above. This comment describes that mechanism.
%|
%| Each callback subfunction declaration has the following form:
%| <SUBFUNCTION_NAME>(H, EVENTDATA, HANDLES, VARARGIN)
%|
%| The subfunction name is composed using the object's Tag and the 
%| callback type separated by '_', e.g. 'slider2_Callback',
%| 'figure1_CloseRequestFcn', 'axis1_ButtondownFcn'.
%|
%| H is the callback object's handle (obtained using GCBO).
%|
%| EVENTDATA is empty, but reserved for future use.
%|
%| HANDLES is a structure containing handles of components in GUI using
%| tags as fieldnames, e.g. handles.figure1, handles.slider2. This
%| structure is created at GUI startup using GUIHANDLES and stored in
%| the figure's application data using GUIDATA. A copy of the structure
%| is passed to each callback.  You can store additional information in
%| this structure at GUI startup, and you can change the structure
%| during callbacks.  Call guidata(h, handles) after changing your
%| copy to replace the stored original so that subsequent callbacks see
%| the updates. Type "help guihandles" and "help guidata" for more
%| information.
%|
%| VARARGIN contains any extra arguments you have passed to the
%| callback. Specify the extra arguments by editing the callback
%| property in the inspector. By default, GUIDE sets the property to:
%| <MFILENAME>('<SUBFUNCTION_NAME>', gcbo, [], guidata(gcbo))
%| Add any extra arguments after the last argument, before the final
%| closing parenthesis.

% --------------------------------------------------------------------
function varargout = UserFcnBrowser_Callback(h, eventdata, handles, varargin)
global gh state
if strcmp(get(gcf,'SelectionType'),'open')
    val=get(h,'Value'); %Note this is a scalar value, as only single-item selection is permitted
    str=get(h,'String');
    
    if ischar(str) %VI120109B: Convert from string to cell string array  %Shouldn't ever be stored as string ever though  
        str = si_transformStringListType(str);
    end

    if iscell(str) %This is no longer necessary, but left for posterity -- Vijay Iyer 12/01/09
        str=deblank(str{val});  %VI120109C: Use deblank()
    end
       
    if ~isempty(str) %length(str)>2 %VI120109C: Used deblank() above instead
        if isempty(state.userFcnGUI.UserFcnSelected)
            state.userFcnGUI.UserFcnSelected=str; %VI120109B: Store as string now, not cell array
        else %if iscellstr(state.userFcnGUI.UserFcnSelected) %VI120109B: Store as string now, not cell array
            state.userFcnGUI.UserFcnSelected = [state.userFcnGUI.UserFcnSelected '|' str];
            %%%VI120109B:Removed%%%
            %             state.userFcnGUI.UserFcnSelected{length(state.userFcnGUI.UserFcnSelected)+1}=...
            %                 str;
            %%%%%%%%%%%%%%%%%%%%%
        end
        set(gh.userFcnGUI.UserFcnSelected,'String',state.userFcnGUI.UserFcnSelected);
    end
end



% --------------------------------------------------------------------
function varargout = UserFcnSelected_Callback(h, eventdata, handles, varargin)
global gh state
if strcmp(get(gcf,'SelectionType'),'open')
    val=get(h,'Value');
    %str=get(h,'String');  
    str = state.userFcnGUI.UserFcnSelected; %VI120109B -- use this, as it's always stored as a pipe-delimited string
    
    if ischar(str) %VI120109B: Convert from string to cell string array    
        str = si_transformStringListType(str);
    end
    if iscell(str) %This is no longer necessary, but left for posterity -- Vijay Iyer 12/01/09
        str=str{val};
    end
    evalin('base',['edit ' str(1:end-2)]);
end



% --------------------------------------------------------------------
function varargout = UserFcnPath_Callback(h, eventdata, handles, varargin)
global state gh
genericCallback(h); %This now invokes updateUserFcnPath() -- Vijay Iyer 12/1/09

%%%VI120109A: Removed%%%%%%%%%
% if isdir(state.userFcnGUI.UserFcnPath)
%     files=dir([state.userFcnGUI.UserFcnPath '*.m']);
%     state.userFcnGUI.UserFcnFiles = sortrows({files.name}'); % Sort names
%     if ~isempty(state.userFcnGUI.UserFcnFiles)
%         set(gh.userFcnGUI.UserFcnBrowser,'String',state.userFcnGUI.UserFcnFiles);
%     else
%         set(gh.userFcnGUI.UserFcnBrowser,'String',' ');
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% --------------------------------------------------------------------
function varargout = changePath_Callback(h, eventdata, handles, varargin)
global state gh
path='';
if isdir(state.userFcnGUI.UserFcnPath)
    path=state.userFcnGUI.UserFcnPath;
end

[fname,pname]=uiputfile([path 'Save.m'],'Select Path To User Functions');
if isnumeric(fname)
    return
else
    state.userFcnGUI.UserFcnPath=pname;
end

updateGUIByGlobal('state.userFcnGUI.UserFcnPath','Callback',1); %VI120109A

%%%VI120109A:Removed%%%%%%%%%%
% if isdir(state.userFcnGUI.UserFcnPath)
%     files=dir([state.userFcnGUI.UserFcnPath '*.m']);
%     state.userFcnGUI.UserFcnFiles = sortrows({files.name}'); % Sort names
%     if ~isempty(state.userFcnGUI.UserFcnFiles)
%         set(gh.userFcnGUI.UserFcnBrowser,'Value',1,'String',state.userFcnGUI.UserFcnFiles);
%     else
%         set(gh.userFcnGUI.UserFcnBrowser,'Value',1,'String',' ');
%     end
% end
% updateGUIByGlobal('state.userFcnGUI.UserFcnPath');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --------------------------------------------------------------------
function varargout = clearAll_Callback(h, eventdata, handles, varargin)
global state gh
state.userFcnGUI.UserFcnSelected=''; %VI120109B: Store as strings;process as cell arrays if needed
set(gh.userFcnGUI.UserFcnSelected,'Value',1,'String',''); %VI120109B: Store as strings;process as cell arrays if needed



% --------------------------------------------------------------------
function varargout = clearSelected_Callback(h, eventdata, handles, varargin)
global gh state
val=get(gh.userFcnGUI.UserFcnSelected,'Value');

if ~isempty(state.userFcnGUI.UserFcnSelected)
    %%%VI120109B%%%%%%%%%
    selFunCell = si_transformStringListType(state.userFcnGUI.UserFcnSelected); %Convert to cell array
    selFunCell(val) = [];
    state.userFcnGUI.UserFcnSelected = si_transformStringListType(selFunCell); %Convert back to pipe-delimited string
    %%%%%%%%%%%%%%%%%%%%%     
    %state.userFcnGUI.UserFcnSelected(val)=[]; %VI120109B:Removed
    
    if val==1
        newval=1;
    else
        newval=val-1;
    end

    set(gh.userFcnGUI.UserFcnSelected,'Value',newval,'String',state.userFcnGUI.UserFcnSelected);
end


% --------------------------------------------------------------------
function varargout = addAll_Callback(h, eventdata, handles, varargin)
global state gh
state.userFcnGUI.UserFcnSelected=si_transformStringListType(state.userFcnGUI.UserFcnFiles);
set(gh.userFcnGUI.UserFcnSelected,'String',state.userFcnGUI.UserFcnSelected);


% --------------------------------------------------------------------
function varargout = UserFcnOn_Callback(h, eventdata, handles, varargin)
genericCallback(h);


