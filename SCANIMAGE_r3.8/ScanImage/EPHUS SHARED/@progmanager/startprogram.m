function startprogram(obj,program_id,varargin)
%STARTPROGRAM   - @progmanager method for starting a program.
%   STARTPROGRAM method for progmanager to start a program.
%   Must supply a progmanager object and either a string program name or program object.  
% 	If the program is not added to the manager already, an error occurs.
%
%   The main GUI of the program is renamed as 'PROGRAM_NAME Main GUI' while
%   all the other GUIs in that program are named according to their parent
%   program and their alias as 'PROGRAM_NAME: Alias Name'.  In addition,
%   the main GUI will have its background color changed.
%
%   A menu is also added to the main GUI with submenus for each GUI in the
%   program.  This menu will allow the program to bring any GUI to the
%   screen.  This is useful after the GUI was closed using the X button.
%
% See also PARSEGLOBALCELLARRAY, ADDPROGRAM, GUIHANDLES, CLOSEPROGRAM,
% ISSTARTED, ADDMENUTOFIGURE, GETPARENT
%
% Changes:
%     TP030804a: Made it an error to load a bad m-file. -- Tom Pologruto 3/8/04
%     TO061104a: Store program-specific, but not GUI-specific, variables. -- Tim O'Connor 6/11/04
%     TO061504a: Add checkmarks to the window menu items. -- Tim O'Connor 6/15/04
%     TO061604c: Link visibilities to checkmarks. -- Tim O'Connor 6/16/04
%     TO061604d: Make sure to put tags on menu items, so they can be found later. -- Tim O'Connor 6/16/04
%     TO101304b: Got rid of errant ';'. -- Tim O'Connor 10/13/04
%     TO071805A: Turn of HandleVisibility for all GUIs. -- Tim O'Connor 7/18/05
%     TO093005A: Completely reworked menus to be much more useful. Made a programmanger submenu on file. -- Tim O'Connor 9/30/05
%                This change introduces the genericOpenData, genericSaveData, and genericSaveDataAs callbacks to programs.
%     TO100505E: Handle the case of closing the last GUI, which should then prompt for close/hide. -- Tim O'Connor 10/5/05
%     TO102405A: Don't look for subguis when checking for windows remaing open, only consider main ones. -- Tim O'Connor 10/24/05
%     TO102405B: Make the menu name for showing sub-guis a little more descriptive. -- Tim O'Connor 10/24/05
%     TO111805A: Added support for configuration saving/loading (see progmanager/saveConfigurations and progmanager/loadConfigurations). -- Tim O'Connor 11/18/05
%     TO011305B: Make figure background colors more consistent across Matlab versions. -- Tim O'Connor 1/13/06
%     TO062306F: Allow individual program configurations to be saved. -- Tim O'Connor 6/23/06
%     TO071906B: Clean up errors when closing all programs. -- Tim O'Connor 7/19/06
%     TO080406C: Matlab 7.0 changed the underlying unit that defines font sizes. Compensate by setting all fonts to be smaller. -- Tim O'Connor 8/4/06
%     TO081606F: Made GUI changes manually, making TO080406C unnecessary (it had proved ineffective). -- Tim O'Connor 8/16/06
%     TO082506A: Added 'New' option to the file menu. Created genericNewData function. -- Tim O'Connor 8/25/06
%     TO082506B: Remove ampersand ('&') on menu names in Matlab 7+. -- Tim O'Connor 8/25/06
%     TO090506I: Changed program manager sub menu position index, as a consequence of TO082506A. -- Tim O'Connor 9/5/06
%     TO041207B: The genericStartFcn may cause the shared variables to be accessed, so initialize them prior to those calls. -- Tim O'Connor 4/12/07
%     TO071807B: Remove dependency on `addmenu` from the signal processing toolbox (which may also be causing menus to disappear on Matlab 7.1). -- Tim O'Connor 7/18/07
%     TO010808A: Factored addUiMenuItems out into it's own file. -- Tim O'Connor 1/8/08
%     VI053108A: Implement 'constructor' argument passing to genericStartFcn() of the program to start -- Vijay Iyer 5/31/08
%     VI053108B: Only pass arguments to the mainGUI genericStartFcn 'constructor' -- Vijay Iyer 5/31/08
%     VI073008A: For 2008a/b compatibility, avoid call to movegui(). This causes problems with the subsequent call to uimenu(). (Matlab Service Request 1-6O6AKO/Bug Report 460763) -- Vijay Iyer 7/30/08
%     VI073008B: For 2008a/b compatibility, set figure units to normalized for the duration of adding menus via uimenu(). (Matlab Service Request 1-606AKO/Bug Report 460763)
%     TO111908G: Added 'About'->'Version' option, to give version info. -- Tim O'Connor 11/19/08
%     TO111908K: Added 'About'->'Online Help' option, with links to the JFRC wiki. -- Tim O'Connor 11/19/08
%
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2004
    
global progmanagerglobal

% Parse inputs.
if nargin < 2
    error(['@progmanager/startprogram: requires 2 input variables.']);
else
    [program_name,program_obj]=parseProgramID(program_id);
end

if ~isfield(progmanagerglobal.programs,program_name)
    error(['@progmanager/startprogram: program ' program_name ' does not exist.']);
elseif progmanagerglobal.programs.(program_name).started
    error(['@progmanager/startprogram: program ' program_name ' already running.']);
else
    %TO061104a - Many variables are specific to a program, not a GUI.
    %            These should be stored at the program level, and be accessible by all
    %            GUIs, conveniently.
    %            See setProgramVariable and getProgramVariable.
    %            --Tim O'Connor 6/11/04
    progmanagerglobal.programs.(program_name).variables = [];
    
    %TO080406C - Matlab 7.0 changed the underlying unit that defines font sizes. Compensate by setting all fonts to be smaller. -- Tim O'Connor 8/4/06
    matlabVersionString = version;
    resizeFonts = 0;
    if str2num(matlabVersionString(1:3)) >= 7
        %resizeFonts = 1;%TO081606F - Had to be changed manually, programatic changes will not work.
    end
    
    guinames=fieldnames(progmanagerglobal.programs.(program_name).guinames);
    UserData.pmobj=obj; % Remember the progmanager object.
    UserData.progname=program_name; % Remember the program name.
    UserData.program_obj=program_obj; % Remember the program object in each figure.
    UserData.main_gui_name=progmanagerglobal.programs.(program_name).mainGUIname; % Remember the main GUI name in each figure.
    fig_handles=[]; %array of handles to figures created.
    for counter=1:length(guinames)
        % Check m_filename of GUI.  
        m_filename=progmanagerglobal.programs.(program_name).guinames.(guinames{counter}).m_filename;   % Extract the mfilename of the GUI.  Can be the same for many GUIs.
        
        % If one of the mfiles does not exis t or is bad, stop loading
        % program and read an error and remove loaded rpogram from memory
        % and delete figures that were created.
        if ~exist(m_filename,'file') > 0
            delete(fig_handles);
            progmanagerglobal.programs=rmfield(progmanagerglobal.programs,program_name);
            error(sprintf('@progmanager/startprogram: GUI %s with m-file %s for program %s does \nnot exist/is not on MATLAB Path. Aborting startprogram.',...
                guinames{counter},m_filename,program_name));
        end

        % Open GUI and load it.
        currentFigHandle=openfig(m_filename,'new');   % Execute the call to open the GUI.
        fig_handles=[fig_handles currentFigHandle];  % Add figure handle to list of opened figure handles.
        singleton=guideopts(currentFigHandle); % Set GUI options from programmanager defaults.
        if verLessThan('matlab','7.6') || ~strcmpi(progmanagerglobal.programs.(program_name).mainGUIname,guinames{counter}) %VI073008A
            movegui(currentFigHandle,'onscreen'); %Add movegui command to GUI opening so all windows are on screen
        else          
            %find something else to accomplish the movescreen() functionality /IF/ it's really needed?!
             %Set units to normalized for duration of this function (VI073008B)
             figUnits = get(currentFigHandle,'Units');
             set(currentFigHandle,'Units','normalized');
        end
        set(currentFigHandle,'Tag',lower(guinames{counter}));   %Set the TAG property of the Figure to the GUI alias name.
        program_fig_properties=getfigprops(program_obj,guinames{counter}); % Get program default figure properties for GUI.
        if ~any(cellfun('isempty',program_fig_properties))  %If they are all valid and not empty...
            set(currentFigHandle,program_fig_properties{:});   % Set the GUI properties to the ones from the program object.
        end

        % Pack the userdata for the current figure into the structure
        % UserData. A copy of the UserData is passed to each GUI in the
        % program.
        UserData.guiname=guinames{counter}; % Remember the gui name.
        set(currentFigHandle,'UserData',UserData);  % Set the figure UserData.

        progmanagerglobal.programs.(program_name).guinames.(guinames{counter}).fighandle=currentFigHandle;  % Remember handle to GUI
        progmanagerglobal.programs.(program_name).guinames.(guinames{counter}).funchandle=str2func(m_filename);  % Create function handle to GUI function.
        progmanagerglobal.programs.(program_name).(guinames{counter}).guihandles=guihandles(currentFigHandle);  % Set GUI object handles.
        progmanagerglobal.programs.(program_name).(guinames{counter}).variables=getvariables(program_obj,guinames{counter});  % Initialize location of variables for this gui from program object
        progmanagerglobal.programs.(program_name).(guinames{counter}).variableGUIs=[];  % Initialize connection of GUIs to variable.
        progmanagerglobal.programs.(program_name).(guinames{counter}).configflags=[];   % Initialize flags used for sorting variables.
        
        %TO080406C - Matlab 7.2 changed the underlying unit that defines font sizes. Compensate by setting all fonts to be smaller. -- Tim O'Connor 8/4/06
        if resizeFonts
% fprintf(1, '@progmanager/startprogram: Resizing all fonts greater than size 10 on %s::%s, shrinking by 2...\n', program_name, guinames{counter});
            fontkids = get(progmanagerglobal.programs.(program_name).(guinames{counter}).guihandles.(lower(guinames{counter})), 'Children');
            fonttypes = get(fontkids, 'Type');
            fontUicontrolIndices = find(strcmpi(fonttypes, 'uicontrol') | strcmpi(fonttypes, 'text') | strcmpi(fonttypes, 'axes'));
            if ~isempty(fontUicontrolIndices)
                fontkids = fontkids(fontUicontrolIndices);
                fontSizes = get(fontkids, 'FontSize');
                for z = 1 : length(fontSizes)
                    if fontSizes{z} >= 10
% fprintf(1, '\t%s: %s --> %s\n', get(fontkids(z), 'Tag'), num2str(fontSizes{z}), num2str(fontSizes{z} - 2));
                        set(fontkids(z), 'FontSize', max(2, fontSizes{z} - 2));
                    end
                end
                fontkids = [];
                fontUicontrolIndices = [];
                fontSizes = {};
            end
        end
        
        % read variable values from GUI.  If the program object containe
        % dany variables, these values are not overwritten.
        %TO101304b - There had been a semicolon at the end of the if statement here. -- Tim O'Connor 10/13/04
        if ~parseGlobalCellArray(progmanager,currentFigHandle) % Parse the global cell array.
            warning(['@progmanager/startprogram: error in initializing GUI ' guinames{counter} '.  No globalCellArray Defined.']);
        end

        % Set the root userdata to be the array of image handles used in
        % the program manager.  Could also set(0,'ShowHiddenHandles','on') to allow access to these fig handles. 
        currentRootUserData=get(0,'UserData');
        currentRootUserData=[currentRootUserData currentFigHandle];
        currentRootUserData(~ishandle(currentRootUserData))=[];
        set(0,'UserData',currentRootUserData);
    end

    %TO041207B - The genericStartFcn may cause these variables to be accessed, so initialize them prior to those calls. -- Tim O'Connor 4/12/07
    if ~isfield(progmanagerglobal.internal, 'shared')
        progmanagerglobal.internal.shared.progmanagerMenus = [];
        progmanagerglobal.internal.shared.windowMenus = [];
    end
    
    % Call start function for GUI if it exists and works.
    visible_on_callbacks={};   % This is a cell array of menu names/callbacks for the main GUI to see other GUIs in that program.
    for counter=1:length(guinames)
        fig_handle=progmanagerglobal.programs.(program_name).guinames.(guinames{counter}).fighandle;
        set(fig_handle, 'HandleVisibility', 'Off', 'Color', get(0,'defaultUicontrolBackgroundColor'));%TO071805A, TO011305B
        if strcmpi(progmanagerglobal.programs.(program_name).mainGUIname,guinames{counter}) %Is this main GUI?
            main_fig_handle=fig_handle;
            main_fig_name=guinames{counter};
            main_mfilename=progmanagerglobal.programs.(program_name).guinames.(guinames{counter}).m_filename;
            %If the main GUI is closes, the program will shut down....
            %set(fig_handle,'CloseRequestFcn',['closeprogram(progmanager,gcbf)'],'Name',[program_name ' Main GUI']);
            %TO093005A - Set the visibility to off, update the menus. Also got rid of that 'Main GUI' part of the string, that nobody cares about.
            set(fig_handle, 'CloseRequestFcn', @closeRequestFcnCallback, 'Name', [program_name]);
            
            constructorArgs = varargin; %VI053108B
        else
            %If other GUis are closed, their Visible property is set to
            %off, but they are not destroyed.
%             set(fig_handle,'CloseRequestFcn','set(gcf,''Visible'',''off'')','Name',[program_name ': ' guinames{counter}]);
%             visible_on_callbacks=[visible_on_callbacks {['setGUIProps(progmanager,gcbf,''' guinames{counter} ''',''Visible'',''On''), toggleCheckMark(gcbo)']}];
            %TO061604c - Link visibilities to checkmarks. -- Tim O'Connor 6/16/04
            set(fig_handle, 'CloseRequestFcn', ['toggleGuiVisibility(progmanager, gcbf, ''' guinames{counter} ''', ''Off'');'], 'Name', [program_name ': ' guinames{counter}]);
            visible_on_callbacks = [visible_on_callbacks {['toggleGuiVisibility(progmanager, gcbf, ''' guinames{counter} ''');']}];
            
            constructorArgs = {}; %VI053108B
        end
           
        % Evaluate the genericStartFunction for the GUI if it exists, passing along required input arguments, if any
        try
            feval(progmanagerglobal.programs.(program_name).guinames.(guinames{counter}).funchandle,'genericStartFcn',fig_handle,[],fig_handle,constructorArgs{:}); %VI053108A, VI053108B
        catch
            fprintf(2, '@progmanager/startprogram: GUI %s has a malfunctioning genericStartFcn. Skipping. Error: %s\n', guinames{counter}, getLastErrorStack);
        end
    end

     % Add File Menu for opening, saving, and closing program.
     %TO093005A - Completely changed this.
%     labels={'File&','Open Program','Close Program','Save Program','Save Program As...','See Program Manager'};
%     cbs={'','loadprogram(progmanager,gcf)','closeprogram(progmanager,gcf)',...
%            'saveprogram(progmanager,gcf,getProgramProp(progmanager,gcf,''program_object_filename''))',...
%            'saveprogram(progmanager,gcf)','progmanagerdisp(progmanager)'};
%     tags={'','open','close','save','saveas','progmanagerdisp'};
%     sep={'Off','Off','On','On','Off','On'};
%     accel={'','O','W','S','','P'};
%     addmenu(main_fig_handle,1,labels,cbs,tags,sep,accel);
    
    %TO082506A - Added 'New' option.
    %TO111908G - Added 'About' option.
    labels = {'File&', 'New', 'Open...', 'Save', 'Save As...', 'Program Manager', 'About', 'Exit', 'Exit All'};
    %TO082506B - No ampersand needed in Matlab 7.
    if str2num(matlabVersionString(1:3)) >= 7
        labels{1} = 'File';
    end
%     cbs = {'', 'disp(''Open...'')', 'disp(''Save'')','disp(''Save As...'')', 'disp(''Program Manager'')', 'disp(''Exit'')', 'disp(''Exit All'')'};
    cbs = {'', {@genericNewData, main_fig_handle}, {@genericOpenData, main_fig_handle}, {@genericSaveProgramData, main_fig_handle}, ...
        {@genericSaveProgramDataAs, main_fig_handle}, '', '', 'closeprogram(progmanager, gcbf)', @closeAllPrograms};
    tags = {'fileMenu', 'newMenuItem', 'openMenuItem', 'closeMenuItem', 'saveMenuItem', 'saveAsMenuItem', 'progmanagerSubMenuMenuItem', 'aboutSubMenu', 'exitMenuItem', 'exitAllMenuItem'};
    sep = {'Off', 'Off','Off','Off','Off','On', 'Off', 'On', 'Off'};
    accel = {'', 'N', 'O', 'S', 'A', '', '', 'Q', ''};
    %TO071807B - Remove dependency on `addmenu` from the signal processing toolbox.
    %menuHandles = addmenu(main_fig_handle, 1, labels, cbs, tags, sep, accel);
    menuHandles = addUiMenuItems(main_fig_handle, 1, labels, cbs, tags, sep, accel);

    %TO093005A - Hang on to these handles, since they'll get updated frequently.
    progmanagerglobal.internal.shared.progmanagerMenus(length(progmanagerglobal.internal.shared.progmanagerMenus) + 1) = menuHandles(5);

    % Add menu for seeing other GUIs from main GUI window.
    %TO102405B: Make this name a little more descriptive. -- Tim O'Connor 10/24/05
    %labels=[{'Show GUI'} guinames(~strcmpi(guinames,main_fig_name))'];%TO102405B
    labels=[{'Sub GUI(s)'} guinames(~strcmpi(guinames,main_fig_name))'];
    cbs=[{''} visible_on_callbacks];
    
    %TO093005A
    progmanagerMenuProgramListPosition = 2;
    
    % Only add this menu if there are other GUIs
    if length(cbs)>1
        %TO061604d: Make sure to put tags on these items, so they can be found later. -- Tim O'Connor 6/16/04
        %addmenu(main_fig_handle,2,labels,cbs, labels);
        %TO071807B - Remove dependency on `addmenu` from the signal processing toolbox.
        emptyArray = cell(size(labels));
        sepArray = emptyArray;
        for i = 1 : length(emptyArray)
            emptyArray{i} = '';
            sepArray{i} = 'Off';
        end
        addUiMenuItems(main_fig_handle, 2, labels, cbs, tags, sepArray, emptyArray);
        %TO093005A
        progmanagerMenuProgramListPosition = 3;
    end
    
    %TO093005A
    %progmanagerglobal.internal.shared.windowMenus(length(progmanagerglobal.internal.shared.windowMenus) + 1) = ...
    %    addmenu(main_fig_handle, progmanagerMenuProgramListPosition, 'Programs', '', 'programsMenu', 'Off', '');
    %TO071807B - Remove dependency on `addmenu` from the signal processing toolbox.
    progmanagerglobal.internal.shared.windowMenus(length(progmanagerglobal.internal.shared.windowMenus) + 1) = ...
        addUiMenuItems(main_fig_handle, progmanagerMenuProgramListPosition, 'Programs', '', 'programsMenu', 'Off', '');
    %TO111805A, TO062306F
    submenuLabels = {'ProgramManagerGUI', 'Save Configuration', 'Load Configuration', 'Save Configuration Set', 'Load Configuration Set'};
    submenuCallbacks = {'progmanagerdisp(progmanager)', 'saveConfig(progmanager, gcbo)', 'loadConfig(progmanager, gcbo)', 'saveConfigurations(progmanager)', 'loadConfigurations(progmanager)'};
    submenuTags = {'programManagerGuiMenuItem', 'saveConfigurationMenuItem', 'loadConfigurationMenuItem', 'saveConfigurationSetMenuItem', 'loadConfigurationSetMenuItem'};
    submenuSep = {'Off', 'On', 'Off', 'On', 'Off'};
    submenuAccel = {'', '', '', '', ''};
%     progmanagerMenu = addmenu(main_fig_handle, 1, 'ProgramManagerGUI', 'progmanagerdisp(progmanager)', 'programManagerGuiMenuItem', 'Off', '');
    %progmanagerMenu = addmenu(main_fig_handle, 1, submenuLabels, submenuCallbacks, submenuTags, submenuSep, submenuAccel);
    progmanagerMenu = addUiMenuItems(main_fig_handle, 1, submenuLabels, submenuCallbacks, submenuTags, submenuSep, submenuAccel);%TO071807B
    set(progmanagerMenu, 'Parent', menuHandles(6));%TO090506I
    %This isn't all that useful/intuitive for a lot of our software. For now, it's best to leave it out.
    %% Add icon menu for saving and opening.
    %addIconMenuToMainFigure(main_fig_handle,main_mfilename); % Add graphical bar for loading and saving programs.

    %TO111908G - Added 'About'->'Version' option.
    %TO111908K - Added 'About'->'Online Help' option.
    submenuLabels = {'Version', 'Online Help'};
    submenuCallbacks = {'displayVersionDialog(progmanager, gcbf)', sprintf('web http://openwiki.janelia.org/wiki/display/ephus/%s/ -new -browser', program_name)};
    submenuTags = {'versionMenuItem', 'onlineHelpMenuItem'};
    submenuSep = {'Off', 'Off'};
    submenuAccel = {'', ''};
    aboutMenu = addUiMenuItems(main_fig_handle, 1, submenuLabels, submenuCallbacks, submenuTags, submenuSep, submenuAccel);%TO071807B
    set(aboutMenu, 'Parent', menuHandles(7));%TO090506I
    
    % declare that the program has started.
    progmanagerglobal.programs.(program_name).started=1;
    
    %TO093005A
    setWindowsMenuItems(obj);    
    
    if ~verLessThan('matlab','7.6') %VI073008B
        set(main_fig_handle,'Units',figUnits);
    end
    
          
end

%-------------------------------------------------------------------------------------------
%TO100505E
function closeRequestFcnCallback(varargin)
global progmanagerglobal;

%Find out if this is the last visible Gui.
visibleGuis = 0;
programNames = fieldnames(progmanagerglobal.programs);
for i = 1 : length(programNames)
    %TO102405A: Don't look for subguis here, just main ones. -- Tim O'Connor 10/24/05
    %guiNames = fieldnames(progmanagerglobal.programs.(programNames{i}).guinames);%TO102405A
    guiNames = {progmanagerglobal.programs.(programNames{i}).mainGUIname};
    for j = 1 : length(guiNames)
        if strcmpi(get(progmanagerglobal.programs.(programNames{i}).guinames.(guiNames{j}).fighandle, 'Visible'), 'On')
            visibleGuis = visibleGuis + 1;
        end
    end
end

if visibleGuis > 1
    %There are others still open, so this one can always be recovered through them.
    %Just make it invisible.
    set(gcbf, 'Visible', 'Off');
    setWindowsMenuItems(progmanager, 'toggle');
else
    option = questdlg(sprintf('No other progams are currently visible. You may not be able to regain access to this gui if it is hidden.\n\nWhat action should be taken?'), ...
        'CloseRequestFcn', 'Exit Program', 'Exit All Programs', 'Cancel', 'Exit All Programs');

    switch lower(option)
        case 'cancel'
            return;
        case 'exit program'
            closeprogram(progmanager, gcbf);
        case 'exit all programs'
            closeAllPrograms(varargin{:});
        otherwise
            errordlg(sprintf('Unknown option: ''%s''', option));
    end
end

return;

%-------------------------------------------------------------------------------------------
%TO082506A
function genericNewData(varargin)
global progmanagerglobal;

[programName, programObj] = parseProgramID(gcbf);

try
    feval(progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).funchandle, 'genericNewData', ...
        progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle, [], ...
        progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle);
catch
    warning('Encountered a malfunctioning genericNewData function for %s:%s - %s', programName, ...
        progmanagerglobal.programs.(programName).mainGUIname, getLastErrorStack);
end

return;

%-------------------------------------------------------------------------------------------
%TO093005A
function genericOpenData(varargin)
global progmanagerglobal;

[programName, programObj] = parseProgramID(gcbf);

try
    feval(progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).funchandle, 'genericOpenData', ...
        progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle, [], ...
        progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle);
catch
    warning('Encountered a malfunctioning genericOpenData function for %s:%s - %s', programName, ...
        progmanagerglobal.programs.(programName).mainGUIname, getLastErrorStack);
end

return;

%-------------------------------------------------------------------------------------------
%TO093005A
function genericSaveProgramData(varargin)
global progmanagerglobal;

[programName, programObj] = parseProgramID(gcbf);

try
    feval(progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).funchandle, 'genericSaveProgramData', ...
        progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle, [], ...
        progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle);
catch
    warning('Encountered a malfunctioning genericSaveProgramData function for %s:%s - %s', programName, ...
        progmanagerglobal.programs.(programName).mainGUIname, getLastErrorStack);
end

return;

%-------------------------------------------------------------------------------------------
%TO093005A
function genericSaveProgramDataAs(varargin)
global progmanagerglobal;

[programName, programObj] = parseProgramID(gcbf);

try
    feval(progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).funchandle, 'genericSaveProgramDataAs', ...
        progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle, [], ...
        progmanagerglobal.programs.(programName).guinames.(progmanagerglobal.programs.(programName).mainGUIname).fighandle);
catch
    warning('Encountered a malfunctioning genericSaveProgramDataAs function for %s:%s - %s', programName, ...
        progmanagerglobal.programs.(programName).mainGUIname, getLastErrorStack);
end

return;

%-------------------------------------------------------------------------------------------
%TO093005A
function closeAllPrograms(varargin)
global progmanagerglobal;

fnames = fieldnames(progmanagerglobal.programs);
for i = 1 : length(fnames)
    try
        closeprogram(progmanager, progmanagerglobal.programs.(fnames{i}).guinames.(progmanagerglobal.programs.(fnames{i}).mainGUIname).fighandle);
    catch
        warning('Failed to properly close program ''%s'' - %s', fnames{i}, getLastErrorStack);%TO071906B
    end
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handles=addIconMenuToMainFigure(fig_handle,main_mfilename)
% Adds icons for savign and loading to GUI figure.
program_name=parseProgramID(fig_handle);

%load little icons for the buttons.
images=load('mwtoolbaricons');    

% Toolbar properties.
type={'push','push','push','push','push'};
ccb={'loadprogram(progmanager,gcf)','saveprogram(progmanager,gcf,getProgramProp(progmanager,gcf,''program_object_filename''))'...
        ,'printpreview(gcf)','progmanagerdisp(progmanager)',['helpwin(''' main_mfilename ''')']};
tags={'load','save','print','progmanagerdisp','help'};
ttip={'Load Program','Save Program','Print Main GUI','Display Program Manager','Help'};
sep={'off','off','on','on','on'};
cicons={images.opendoc,images.savedoc,images.printdoc,images.newdoc,images.help};

% Make toolbar and one button.
h=uitoolbar(fig_handle);
uipushtool('Parent',h);

% Add the rest of the buttons.
addtoolbarbtn(fig_handle,1,type,cicons,ccb,tags,sep,ttip);

%-------------------------------------------------------------------------------------------
%TO010808A - Factored addUiMenuItems out into it's own file. -- Tim O'Connor 1/8/08
%TO071807B
% function handles = addUiMenuItems(parent, pos, labels, callbacks, tags, separators, accelerators)
% 
% if ~strcmpi(class(labels), 'cell')
%     handles = uimenu(parent, 'Label', labels, 'Position', pos, 'Callback', callbacks, 'Tag', tags, 'Separator', separators, 'Accelerator', accelerators);
%     return;
% end
% 
% if strcmpi(get(parent, 'Type'), 'figure')
%     handles(1) = uimenu(parent, 'Label', labels{1}, 'Position', pos, 'Callback', callbacks{1}, 'Tag', tags{1}, 'Separator', separators{1}, 'Accelerator', accelerators{1});
% else
%     handles(1) = uimenu(parent, 'Label', labels{1}, 'Position', 1, 'Callback', callbacks{1}, 'Tag', tags{1}, 'Separator', separators{1}, 'Accelerator', accelerators{1});
% end
% for i = 2 : length(labels)
%     handles(i) = uimenu(handles(1), 'Label', labels{i}, 'Position', i - 1, 'Callback', callbacks{i}, 'Tag', tags{i}, 'Separator', separators{i}, 'Accelerator', accelerators{i});
% end
% 
% return;