function output=genericPropertyEditor(Title,Param_names,Default_Values,Input_Type)
% GENERICPROPERTYEDITOR   - method for editing grahically param/value pairs.
% 	GENERICPROPERTYEDITOR(Title,Param_names,Default_Values) accepts a cell array
% 	of string parameter names (Param_names) and a mixed cell array of
% 	strings/numerics (Default_Values) and displays an input that can be
% 	used to edit them and output the new properties. Title is the name of
% 	the GUI figure made.  The outputs have the same class as the default
% 	values.
%
% 	GENERICPROPERTYEDITOR(Title,Param_names,Default_Values,Input_Type) accepts a cell array
% 	of string parameter names (Param_names) and a mixed cell array of
% 	strings/numerics (Default_Values) and displays an input that can be
% 	used to edit them and output the new properties. Input_Type is an array
% 	of 1's and 0's, 1 if the value is numeric, or 0 if it is not.  By
% 	default, all values are cast as strings. Title is the name of
% 	the GUI figure made.
% 	
% 	See also INPUTDLG

% Parse the inputs.
output={};
if nargin < 3
    error('genericPropertyEditor: too few inputs');
elseif nargin == 3
    % Read default class from the inputs.
    Input_Type=~cellfun('isclass',Default_Values,'char');
end

% Check the inputs for type and class.
if ~ischar(Title)
    error('genericPropertyEditor: Title must be a string.');
elseif ~iscellstr(Param_names)
    error('genericPropertyEditor: Param_names must be a cell array of strings.');
elseif ~iscell(Default_Values)
    error('genericPropertyEditor: Default_Values must be a cell array.');
elseif (~isnumeric(Input_Type) & ~islogical(Input_Type)) | ~isequal(size(Input_Type),[1 length(Default_Values)])
    error('genericPropertyEditor: Input_Type must be a numeric array of same length as Default_Values.');
end

% Setup the Figure.
% Maximum size of input parameter.
maxstringsize=max(cellfun('size',Param_names,2));

% Default uicontrol spacing.
starth=0;    %Space to leave at bottom of figure.
startv=3;    %Space to leave at left of figure.
spacerh=1;  %Space between buttons horizontally.
spacerv=.5;  %Space between buttons vertically..
extenth=maxstringsize+10;  %Extent of uicontrol horizontally..
extentv=1.4; %Extent of uicontrol vertically..

% Default figure spacing.
bufferv=3;
bufferh=1;

% Construct Figure.
fig_handle = figure('Units','characters',...
    'Position',[92 32 (2*extenth+spacerh+bufferh) 2*spacerv+extentv*length(Default_Values)+bufferv],'Name',Title,'DoubleBuffer','On','Resize','on',...
    'Color',get(0,'DefaultUicontrolBackgroundColor'),'HandleVisibility','Callback','NumberTitle','off',...
    'HandleVisibility','callback','MenuBar','None','Tag',Title,'CloseRequestFcn',@cancel,...
    'CloseRequestFcn','closereq','WindowStyle','modal');
% Construct OK Pushbutton.
uicontrol(fig_handle,'Style','pushbutton','Units','characters','Callback',@ok,...
    'Position',[extenth/2 0 10 2],'Tag','done','String','Done','FontWeight','Bold');
% Construct Cancel Pushbutton.
uicontrol(fig_handle,'Style','pushbutton','Units','characters','Callback',@cancel,...
    'Position',[extenth/2+11 0 10 2],'Tag','cancel','String','Cancel','FontWeight','Bold');

% Display all the param/value pairs in a lsitbox.
Param_names=fliplr(Param_names);    % Flip stuff so they list top to bottom
Default_Values=fliplr(Default_Values);
for paramCounter=1:length(Param_names)
    % Setup name of parameter
    uicontrol(fig_handle,'Style','text','Units','characters',...
        'Position',[starth startv+extentv*(paramCounter-1)+spacerv extenth extentv],'Tag',Param_names{paramCounter},...
        'String',Param_names{paramCounter},'FontWeight','Bold','HorizontalAlignment','right');
    % Setup edit box
    uicontrol(fig_handle,'Style','edit','Units','characters',...
        'Position',[starth+spacerh+extenth startv+extentv*(paramCounter-1)+spacerv extenth extentv],'Tag',Param_names{paramCounter},...
        'String',num2str(Default_Values{paramCounter}),'FontWeight','Bold','BackgroundColor','White');
end
Param_names=fliplr(Param_names);    % Flip params back
% Wait for usresume to be called from figure.
uiwait(fig_handle); 

% Now compute the outputs fromt he dialog box.
if ishandle(fig_handle)
    output=cell(size(Default_Values));
    % if were are here they hit ok.
    for paramCounter=1:length(Param_names)
        if Input_Type(paramCounter) == 1 % Numeric so convert from string
            output{paramCounter}=str2num(get(findobj(fig_handle,'type','uicontrol','style','edit','Tag',Param_names{paramCounter}),'String'));
        else
            output{paramCounter}=get(findobj(fig_handle,'type','uicontrol','style','edit','Tag',Param_names{paramCounter}),'String');
        end
    end
    delete(fig_handle);
end

   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cancel(obj,eventdata)
% Cancel action
uiresume(gcbf);
delete(gcbf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ok(obj,eventdata)
% Ok action
uiresume(gcbf);
