function progmanagerdisp(program_manager)
% PROGMANAGERDISP   - @progmanager method for displaying and editing progmanager options and programs.
% 	PROGMANAGERDISP(program_manager) will display a GUI interface for changing properties of the program manager 
% 	specified by program_manager.  Graphical Version of
% 	SETPROGMANAGERDEFAULTS.
% 	
% 	See also PROGMANAGER, SETPROGMANAGERDEFAULTS

% TP030504: removed error checking in for setting, and placed it in
% SETPROGMANAGERDEFAULTS function.


currentfigure=findobj(allchild(0),'Tag','program_manager_display');

% if the progmanager display is on, update it.
setProgmanagerDefaults(program_manager,'ProgmanagerDisplayOn',1);
if ~isempty(currentfigure)
    figure(currentfigure);
    handles=guihandles(currentfigure);
    updatelistbox(handles.listbox);
    return
end

% Setup the Figure.
fig_handle = figure('Units','characters',...
    'Position',[92 32 58 13],'Name','Program Manager Display','DoubleBuffer','On','Resize','off',...
    'Color',get(0,'DefaultUicontrolBackgroundColor'),'HandleVisibility','Callback',...
    'HandleVisibility','callback','MenuBar','None','Tag','program_manager_display',...
    'CloseRequestFcn','closereq,setProgmanagerDefaults(progmanager,''ProgmanagerDisplayOn'',0)');
listbox_label = uicontrol(fig_handle,'Style','text','Units','characters',...
    'Position',[1 12 32 1],'Tag','listbox_label',...
    'String','Currently Loaded Programs','FontWeight','Bold');
listbox_handle = uicontrol(fig_handle,'Style','listbox','Units','characters',...
    'Position',[1 1 33 10.5],'ToolTipString','Double Click to see Main GUI',...
    'BackgroundColor','white','Tag','listbox',...
    'Max',10,'Min',1,'Callback',@listbox_callback);
close_program_handle = uicontrol(fig_handle,'Style','pushbutton','Units','characters',...
    'Position',[36 8 20 2], 'String','Close Program','FontWeight','Bold','Tag','close_program',...
    'BackgroundColor',get(0,'DefaultUicontrolBackgroundColor'),'Callback',{@close_program,listbox_handle});
open_program_handle = uicontrol(fig_handle,'Style','pushbutton','Units','characters',...
    'Position',[36 5 20 2], 'String','Open Program','FontWeight','Bold','Tag','open_program',...
    'BackgroundColor',get(0,'DefaultUicontrolBackgroundColor'),'Callback',{@open_program,listbox_handle});
edit_program_handle = uicontrol(fig_handle,'Style','pushbutton','Units','characters',...
    'Position',[36 2 20 2], 'String','Properties','FontWeight','Bold','Tag','edit_program',...
    'BackgroundColor',get(0,'DefaultUicontrolBackgroundColor'),'Callback',@editParams,...
    'ToolTipString','Edit Program Manager Properties');

addmenu(fig_handle,1,{'File&','Close Matlab'},{'','exit'});
addIconMenuToMainFigure(fig_handle);
movegui(fig_handle,'onscreen');

% Initialize list box
updatelistbox(listbox_handle);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function listbox_callback(obj,eventdata,programs)
% Listbox callback.  if user double clicks on program name, it brings the
% main gui for that program to the front.

switch get(gcbf,'SelectionType')
    case 'open'
        val=get(obj,'Value');
        string=get(obj,'String');
        if ~isempty(string)
            selectedprogram=string{val};
            figure(getHandleFromName(progmanager,getProgramProp(progmanager,selectedprogram,'mainGUIname'),selectedprogram));
        end
end        

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function close_program(obj,eventdata,listbox)
% Close program callback.
val=get(listbox,'Value');
string=get(listbox,'String');
if ~isempty(string)
    selectedprogram=string{val};
    closeprogram(progmanager,selectedprogram)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function open_program(obj,eventdata,listbox)
% Open program callback.
selectedprogram='';
if ishandle(listbox)
    val=get(listbox,'Value');
    string=get(listbox,'String');
    if ~isempty(string)
        selectedprogram=string{val};
    end
end
filename=loadprogram(progmanager,selectedprogram);
path=fileparts(filename);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updatelistbox(listbox)
% Updates listbox with programs.
global progmanagerglobal
if ~isempty(progmanagerglobal.programs)
    programs=fieldnames(progmanagerglobal.programs);
    set(listbox,'String',programs);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function editParams(obj,eventdata,listbox)
% This function will display the editable fields of the program manager in
% a input dialog box.  The values can be changed, and the user can save
% the changes for use next time.

% Assemble the global program manager structure.
program_manager=progmanager;
names=getProgmanagerDefaults(program_manager,'editable_fields');

% check to make sure we have somethign to edit.
if isempty(names)
    beep,disp(['No Editable Properties for Program manager']),return;
end

% Loop through all the editable fields, and convert any numerics to
% strings.
for val_counter=1:length(names)
    values{val_counter}=getProgmanagerDefaults(program_manager,names{val_counter});
end

% Open the dialog box
answer=genericPropertyEditor('Edit Program Manager Defaults',names,values);

% Update the program manager with any updates.
if ~isempty(answer)
    for val_counter=1:length(names)
        setProgmanagerDefaults(progmanager,names{val_counter},answer{val_counter});
    end
    savedefaults;   % Save defaults to programmanger file.
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handles=addIconMenuToMainFigure(fig_handle)
% Adds icons for savign and loading to GUI figure.

%load little icons for the buttons.
images=load('mwtoolbaricons');    

% Toolbar properties.
type={'push','push','push'};
ccb={{@open_program,''},@editParams,'helpwin(''progmanager'')'};
tags={'openprogram','editprogmanager','helpprogmanager'};
ttip={,'Open Program','Edit Program Manager','Help on Program Manager'};
sep={'off','off','on'};
cicons={images.opendoc,images.newdoc,images.help};

% Make toolbar and one button.
h=uitoolbar(fig_handle);
uipushtool('Parent',h);

% Add the rest of the buttons.
addtoolbarbtn(fig_handle,1,type,cicons,ccb,tags,sep,ttip);





