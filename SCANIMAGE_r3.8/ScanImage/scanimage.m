function scanimage(defFile,iniFileName)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function that starts the ScanImage software.
% 
% defFile is the default file name of a .usr file if called as a function.
%
% Software Description:
%
% ScanImage controls a laser-scanning microscope (Figure 1A). It is written 
% entirely in MATLAB and makes use of standard multifunction boards 
% (e.g. National Instruments, Austin, TX) for data acquisition and 
% control of scanning. The software generates the analog voltage 
% waveforms to drive the scan mirrors, acquires the raw data from 
% the photomultiplier tubes (PMTs), and processes these signals to 
% produce images. ScanImage controls three input channels 
% (12-bits each) simultaneously, and the software is written to be 
% easily expandable to the maximum number of channels the data acquisition 
% (DAQ) board supports and that the CPU can process efficiently. 
% The computer bus speed dictates the number of samples that can be 
% acquired before an overflow of the input buffer occurs, while the 
% CPU speed and bus speed combine to determine the rate of data 
% processing and ultimately the refresh rate of images on the screen. 
% Virtually no customized data acquisition hardware is required for 
% either scan mirror motion or data acquisition.
%
% Reference: Pologruto, T.A., Sabatini, B.L., and Svoboda, K. (2003)exit
%            ScanImage: Flexible software for operating laser scanning microscopes.
%            Biomedical Engineering Online, 2:13.
%            Link: www.biomedical-engineering-online.com/content/2/1/13
%
% Copyright 2003 Cold Spring harbor Laboratory
%
%% CHANGES
%   11/24/03 Tim O'Connor - Start using the daqmanager object.
%   12/18/03 Tim O'Connor - Initialize the uncagingPulseImporter tool.
%   TPMOD_1: Modified 12/30/03 Tom Pologruto - Handles defFile Input correctly now
%   TPMOD_2: Modified 12/31/03 Tom Pologruto - Sets the GUI for power control
%            to the scan laser beam by default.
%   TPMOD_3: Modified 1/12/04 Tom Pologruto - Allows passing in of INI file also now.
%   TO051704a Tim O'Connor 5/17/04 - Public release formatting.
%   VI021808A Vijay Iyer 2/18/08 - Make global vars (state,gh) available in base workspace--needed (for now) for GUI callback dispatches (e.g. via updateGUIByGlobal())
%   VI030408A Vijay Iyer 3/4/08 - Prompt for .ini file location if no .usr file specified
%   VI042108A Vijay Iyer 4/21/08 - Initialize file writing function handle
%   VI050608A Vijay Iyer 5/6/08 - Use installation 'standard_model.ini' file when no user-created file was identified
%   VI081308A Vijay Iyer 8/13/08 - Do PowerControl figure hiding stuff /after/ opening user file, because that action can re-open the GUI
%   VI091608A Vijay Iyer 9/16/08 - Include updateSaveDuringAcq in startup tasks 
%   VI092408A Vijay Iyer 9/24/08 - Handle unified configuration GUI -- Vijay Iyer 9/24/08
%   VI101708A Vijay Iyer 10/17/08 - Eliminate the use of scanLaserBeam variable
%   VI103108A Vijay Iyer 10/31/08 - Handle pockels calibration via openusr() or calibrateBeams(), depending on whether USR file is selected
%   VI110208A Vijay Iyer 11/02/08 - Handle state.init.eom.started flag here now, after first chance to calibrate any beams the user is employing
%   VI110708A Vijay Iyer 11/07/08 - Verify that DAQmx drivers are present and exclusively installed
%   VI111708A Vijay Iyer 11/17/08 - Allow bidirectional scanning to be enabled via an INI file setting
%   VI111708B Vijay Iyer 11/17/08 - Ensure ChannelGUI is in consistent state following all INI/USR loading
%   VI112208A Vijay Iyer 11/22/08 - Create state.userSettingsPath here now, since it's not done in INI file anymore
%   VI121208A Vijay Iyer 12/12/08 - Call setPockelsAcqParameters() rather than Configuration GUI callback
%   VI010209A Vijay Iyer 1/02/09 - Remove call to setPockelsAcqParameters()
%   VI011609A Vijay Iyer 1/16/09 - Changed state.init.pockelsOn to state.init.eom.pockelsOn
%   VI011709A Vijay Iyer 1/17/09 - Add PowerBox GUI to list of figures
%   VI012709A Vijay Iyer 1/27/09 - Load configurationGUI, rather than basicConfigurationGUI
%   VI012909A Vijay Iyer 1/29/09 - Create section to initialize cell array variables (as these can not be initialized within the INI file, with current INI parser)
%   VI020709A Vijay Iyer 2/7/09 - Store version information immediately following INI file loading 
%   VI021309A Vijay Iyer 2/13/09 - Add Align GUI to list of figures
%   VI022709A Vijay Iyer 2/27/09 - Revert VI110708A
%   VI082909A Vijay Iyer 8/29/09 - Use scim_parkLaser() instead of makeAndPutDataPark() 
%   VI083109A Vijay Iyer 8/31/09 - Use scim_parkLaser() instead of parkLaserCloseShutter()
%   VI090109A Vijay Iyer 9/1/09 - No need to store which NI driver
%   VI091709A Vijay Iyer 9/17/09 -- Add newly created Trigger GUI
%   VI111109A Vijay Iyer 11/11/09 - Remove code referencing now-deprecated 'specialty' EOM features
%   VI112309A Vijay Iyer 11/23/09 - Revert VI020709A: No longer place version information here -- do this in internal.ini instead
%   VI011310A Vijay Iyer 1/13/10 - Remove calls to deprecated updateXXXCheckMark() functions. 
%   VI031010A Vijay Iyer 3/10/10 - Use general motorConfig(), in lieu of MP285Config()
%   VI032310A Vijay Iyer 3/23/10 - motorGetPosition(), via motorAction(), replaces updateMotorPosition()
%   TO091210B Tim O'Connor 9/12/10 - Created the clockExportGUI.
%   VI092210A Vijay Iyer 9/22/10 -- The 'saveDuringAcquisition' control/variable is now defunct; state.files.autoSave var is no longer a USR variable
%   VI092710A Vijay Iyer 9/27/10 -- Handle the new fastConfigurationGUI & associated variables
%   VI092910A Vijay Iyer 9/29/10 -- Bind figure CloseRequestFcn() to (modified) HideGUI programatically, rather than with GUIDE property editor
%   VI092910B Vijay Iyer 9/29/10 -- Initialize new EventManager class; add new UserFunctions GUI (in lieu of older UserFcns GUI); initialize EventMap data structure
%   VI101910A Vijay Iyer 10/19/10 -- Take care not to override CloseRequestFcn() where 
%   VI110410A Vijay Iyer 11/4/10 -- Load default window positions from internal.INI if no USR file is supplied
%   VI111110A Vijay Iyer 11/11/10 -- Use Matlab installation drive as default directory; save INI path as user path, if no user path specified 
%   VI112310A Vijay Iyer 11/23/10 -- Handle disabling of Power and/or Motor controls from here, if these features are deactivated by INI file
%   VI010511A: Replace deprecated applyModeCycleAndConfigSettings() with simple opencfg() -- Vijay Iyer 1/5/11
%   VI020111A: Signal start of ScanImage with event -- Vijay Iyer 2/1/11
%   VI042411A: React to changes in motorConfig() -- Vijay Iyer 4/24/11
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import scanimage.* %VI092910B

% Global declaration: all variables used by ScanImage are contained in the structure
% state, while all the handles to the grpahics objects (GUIs) are contained in
% the structure gh
global state gh
evalin('base','global state gh;'); %VI021808A

%Create waitbar to track process of application.
h = waitbar(0, 'Starting ScanImage...', 'Name', 'ScanImage Software Initialization', 'WindowStyle', 'modal', 'Pointer', 'watch');            

try   
%User File Manipulations from outside the function....
if nargin == 0
    defFile = '';
    iniFileName = '';
elseif nargin == 1
    iniFileName = '';
end

if isempty(defFile) % start TPMOD_1 12/31/03
    % Select user file if one exists....
    % Remembers the path to the last one loaded from the last session if possible....
    scanimagepath=fileparts(which('scanimage'));
    if isdir(scanimagepath) && exist(fullfile(scanimagepath, 'lastUserPath.mat'), 'file') == 2
        temp = load(fullfile(scanimagepath,'lastUserPath.mat'));
        userpath = getfield(temp, char(fieldnames(temp)));
%         if isdir(userpath) %VI030408A
%             cd(userpath);
%         end
    else
        userpath = most.idioms.startPath(); %VI111110A
        %userpath = cd; %VI030408A -- use current path if no lastUserPath exists
    end
    
    [fname, pname] = uigetfile('*.usr', 'Choose user file (cancel if none)', userpath);
    if isnumeric(fname)
        fname = '';
        pname = '';
        full = [];
        selectIni = true;
    else
        %Use 'standard.ini' if found in .usr directory, otherwise prompt to select
        if exist(fullfile(pname,'standard.ini'), 'file') %VI030408A 
            iniFileName = fullfile(pname, 'standard.ini'); 
            selectIni = false;
        else
            selectIni = true;            
        end           
        %cd(pname); %VI030408A -- removed
        full = fullfile(pname, fname);
        defFile = full;
    end
        
    if selectIni %VI030408A -- prompt for .ini 
        [ini_f ini_p] = uigetfile('*.ini', 'Select .ini file (usually ''standard.ini''; cancel to use installation standard.ini file)', userpath); %VI030408A
        if isnumeric(ini_f)
            iniFileName = '';
        else
            iniFileName = fullfile(ini_p, ini_f);
        end
    end    
            
end % end TPMOD_1 12/31/03

%Create new-style SI model objects, responsible for newer functionality
state.hSI = SI3(); 

%Build GUIs 
state.internal.guinames={'channelGUI','triggerGUI','configurationControls', 'alignGUI', ... %VI092408A, VI011709A, VI012709A, VI021309A, VI091709A
        'mainControls', 'cycleGUI', 'metaStackGUI', 'userFcnGUI', 'userPreferenceGUI', 'powerControl', 'powerTransitions', ...
        'powerBox','uncagingPulseImporter', 'powerBoxStepper', 'uncagingMapper', 'laserFunctionPanel', 'clockExportGUI', ...
		'fastConfigurationGUI' 'userFunctionsGUI' 'roiGUI' 'roiDisplayGUI' ... %VI092910A %VI092710A %TO091210B
        'positionGUI','imageControls','motorControls'};  %Show GUIs with modifications last..to make modifications less apparent

for guicounter=1:length(state.internal.guinames)
    gh = setfield(gh,state.internal.guinames{guicounter},eval(['guidata(' state.internal.guinames{guicounter} ')']));
end

%%%VI092910A\VI101910A%%%
permanentGUIs = setdiff(state.internal.guinames, 'mainControls');
for i=1:length(permanentGUIs)
    hFig = gh.(permanentGUIs{i}).figure1;
	
	% ensure the right figure color
	set(hFig,'Color',get(0,'defaultUIControlBackgroundColor'));
end

%Create new-style SI controller object, responsible for newer GUIs & functionality
state.hSICtl = SI3Controller(state.hSI);
                
%Handle GUI variations from 'master' copies
modMainControls();
modImageControls();
modMotorControls(); 
% Show primary GUIs by default
guiNamesToOpen = {'mainControls' 'powerControl' 'motorControls' 'imageControls'};
for i = 1:length(guiNamesToOpen)
    set(gh.(guiNamesToOpen{i}).figure1,'Visible','On');
end

% Open the waitbar for loading
waitbar(.1,h, 'Reading Initialization File...');

%%%VI012909A%%%%%%%%
state.internal.executeButtonFlags = {};
%%%%%%%%%%%%%%%%%%%%

% start TPMOD_3 1/12/04
if isempty(iniFileName)
    openini('standard_model.ini'); %VI050608A
else
    openini(iniFileName);
end
% end TPMOD_3 1/12/04

%%%SECTION OF INI-FILE DEPENDENT ACTIONS%%%%%%%%%%%%%%%%%%%%%

setStatusString('Initializing...');

if state.init.allowFasterLineScans
    hMpl = gh.configurationControls.pmMsPerLine;
    mplCellStr = get(hMpl,'String');
    mplCellStr = [{'0.125';' 0.25'} ; mplCellStr];

    set(hMpl,'String',mplCellStr);    
    
    state.internal.msPerLineGUI = state.internal.msPerLineGUI + 2;
    updateGUIByGlobal('state.internal.msPerLineGUI');
end    

waitbar(.2,h, 'Configuring Motor Controller(s)...');
motorConfig(); %VI042411A

%Compute scanAmplitudeFast/Slow vals from INI-file vars
updateScanAmplitude();

waitbar(.3,h, 'Creating Figures for Imaging');
makeImageFigures;	% config independent...relies only on the .ini file for maxNumberOfChannles.

setStatusString('Initializing...');
waitbar(.4,h, 'Setting Up Data Acquisition Devices...');
setupDAQDevices_Common;				% config independent 

%%%%VI103008: Prepare shutter state
state.shutter.open = logical(state.shutter.open);
state.shutter.epiShutterOpen = logical(state.shutter.epiShutterOpen);

%Close Shutter and Park Beam
%makeAndPutDataPark;					% config independent
scim_parkLaser;                     % VI082909A, config independent

setStatusString('Initializing...');

if state.video.videoOn
    waitbar(.6, h, 'Starting Video Controls...');
    videoSetup;
end

%Activate Pockels Cell.
if state.init.eom.pockelsOn %VI011609A
    startEomGui;
    % start TPMOD_2 12/31/03
    %state.init.eom.beamMenu=state.init.eom.scanLaserBeam; %VI101708A
    state.init.eom.beamMenu = 1; %VI101708A    
    updateGUIByGlobal('state.init.eom.beamMenu');
    powerControl('beamMenu_Callback', gh.powerControl.beamMenu);
     % end TPMOD_2 12/31/03
    powerControl('usePowerArray_Callback', gh.powerControl.usePowerArray);
    initializeUncagingPulseImporter;
    set(gh.uncagingMapper.figure1, 'Visible', 'Off');
    
    %Do 'park' again, as this will set beam to calibration-determined minimum power
    scim_parkLaser();   %VI083109A
else
    enableEomGui(0); %VI112310A: Disable Power Controls
end
%%%%END INI FILE DEPENDENT ACTIONS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%parkLaserCloseShutter;	%VI083109A: Removed -- this is handled via scim_parkLaser() in pockelsOn section above

if length(defFile) == 0
    waitbar(.7, h, 'No user settings file chosen.');
    state.userSettingsPath = ''; %VI112208A
    calibrateBeams(true); %VI103108A: Calibrate beams according to default beam selection settings (beam 1 on, others off)
    %applyModeCycleAndConfigSettings; %VI010511A
    opencfg(); %VI010511A
    
    updateFastConfigTable(); %VI092710A
    loadWindowPositions(); %VI110410A: Load default window positions (from internal.INI)
    
    %%%VI111110A%%%
    state.userSettingsPath = state.iniPath; 
    saveUserSettingsPath();
    %%%%%%%%%%%%%%%
    
    %updateAutoSaveCheckMark; %VI092210A: Removed % BSMOD
    %updateSaveDuringAcq; %VI092210A: Removed %VI091608A
    %updateKeepAllSlicesCheckMark; %VI011310A: Removed % BSMOD
    %updateAutoOverwriteCheckMark; %VI011310A: Removed
else
    waitbar(.7, h, 'Reading User Settings...');
    openusr(defFile, true); %VI103108A: Signal that this is on startup        
end
%%%%%%%%%%%%%%%%

if state.init.eom.pockelsOn %VI081308A, VI011609A
    state.init.eom.started = 1; %VI110208A
end

setStatusString('Initializing...');
waitbar(.8, h, 'Updating motor position...');

%updateMotorPosition; %VI032310A
motorGetPosition(); %VI032310A

setStatusString('Initializing...');
state.internal.startupTime = clock;
state.internal.startupTimeString = clockToString(state.internal.startupTime);
updateHeaderString('state.internal.startupTimeString');
state.internal.imageWriter = getfield(imformats('tif'), 'write'); %VI042108A 
%state.internal.niDriver = whichNIDriver; %VI090109A

waitbar(.9, h, 'Initialization Done');

setStatusString('Ready to use');
state.initializing = 0;
setStatusString('Ready to use');
waitbar(1, h, 'Ready To Use');

%Ugly...call several callbacks to ensure/initialize various states. These could be handled by updateGuiByGlobal as well.
%basicConfigurationGUI('pockelsClosedOnFlyback_Callback',gh.basic.pockelsClosedOnFlyback); %VI092408A, VI121208A
%setPockelsAcqParameters(); %VI121208A, VI010209A
%roiCycleGUI('roiCyclePosition_Callback', gh.roiCycleGUI.roiCyclePosition);   %setup initial cycle....
updateGUIByGlobal('state.acq.channelMerge','Callback',1); %VI111708B

updateGUIByGlobal('state.internal.figureColormap1','Callback',1);
updateGUIByGlobal('state.internal.figureColormap2','Callback',1);
updateGUIByGlobal('state.internal.figureColormap3','Callback',1);
updateGUIByGlobal('state.internal.figureColormap4','Callback',1);

%Initialize application and signal application start
state.hSI.initialize();
notify(state.hSI, 'appOpen'); %VI020111A

close(h);

catch ME
    if ishandle(h)
        close(h);
    end
    try
        scim_exit();
    catch ME1
        throwAsCaller(MException('','Error occurred during ScanImage startup:\n%s\n\nUnable to clean up! Please exit Matlab.\n(Kill Matlab application/process using Windows Task Manager, if needed).',most.idioms.reportError(ME)));
    end
    ME.rethrow();
end

function modMainControls()

global gh

hFig = gh.mainControls.figure1;

%Hide menu items not used currently in SI 3.x
set(findobj(hFig,'Tag','mnu_View_FastZControls'),'Enable','off');


return;

function modImageControls()

global gh

hFig = gh.imageControls.figure1;

% %Hide menubar -- use checkboxes directly in SI38
% pixPosn = getpixelposition(hFig);
% pixPosn(4) = 299; %Original height set in FIG file, withou menubar added
% delete(findobj(hFig,'Tag','mnuSettings'));
% setpixelposition(hFig,pixPosn);

showControls = {'cbAverageSamples' 'cbShowCrosshair'};
hideControls = {'etFrameSelFactor' 'etFrameSelections' 'cbLockFrameSel2RollAvg' 'cbUseLastSelFrame' 'text29' 'text28'};

cellfun(@(x)set(findobj(hFig,'Tag',x),'Visible','on'), showControls);
cellfun(@(x)set(findobj(hFig,'Tag',x),'Visible','off'), hideControls);

%Hide all but rolling average controls from averaging/selection panel
rollAveControls = {'stRollingAverage' 'etRollingAverage' 'cbLockRollAvg2AcqAvg'};
cellfun(@(x)set(findobj(hFig,'Tag',x),'Parent',hFig), rollAveControls);
set(findall(findobj(hFig,'Tag','pnlAveragingAndSelection')),'Visible','off');

%Compress controls
compressionOffset = 5;
for i=1:4 %Hard-code # of channels here
    hPnl = findobj(hFig,'Tag',sprintf('pnlChan%d',i));
    currPosn = get(hPnl,'position');
    set(hPnl,'position', currPosn + [0 -compressionOffset 0 0]);
end

currPosn = get(hFig,'position');
set(hFig,'position',currPosn + [0 0 0 -compressionOffset]);
%set(findobj(hFig,'Tag','pnlAveragingAndSelection'),'Visible','off');

rollAveHOffset = 1;
rollAveVOffset = -0.1;
for i=1:length(rollAveControls)
    hCtl = findobj(hFig,'Tag',rollAveControls{i});
    currPosn = get(hCtl,'position');
    set(hCtl,'position',currPosn + [rollAveHOffset rollAveVOffset 0 0]);
end

return;

function modMotorControls()

global state gh

showControls = {'pbGrabOneStack' 'cbLockSliceVals'};
hideControls = {'stPower' 'etStartPower' 'etEndPower' 'pbClearStartEnd' 'pbClearEnd' 'cbUseStartPower'};

cellfun(@(x)set(gh.motorControls.(x),'Visible','on'), showControls);
cellfun(@(x)set(gh.motorControls.(x),'Visible','off'), hideControls);

toMoveControls = {'stStart' 'stEnd' 'stPosn' 'etStackStart' 'etStackEnd'};

pixShift = getpixelposition(gh.motorControls.etStartPower) - getpixelposition(gh.motorControls.etStackStart);
pixShift = pixShift(1);

for i=1:length(toMoveControls)
    hCtl = gh.motorControls.(toMoveControls{i});
    pixPosn = getpixelposition(hCtl);
    setpixelposition(hCtl,pixPosn + [pixShift 0 0 0]);
end

state.motor.excludeControls = cellfun(@(x)gh.motorControls.(x),hideControls);


return;
