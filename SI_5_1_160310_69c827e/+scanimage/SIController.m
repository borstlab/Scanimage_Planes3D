classdef SIController < most.Controller & scanimage.interfaces.Class
    %%% SIController Controller class for the ScanImage application
    %% USER PROPS
    properties
        beamDisplayIdx=1;       %Index of beam whose properties are currently displayed/controlled
        channelsTargetDisplay;  %A value indicating 'active' channel display, or Inf, indicating the merge display figure. If empty, no channel is active.
        enablePhotostimHotkeys = false;     % enable hotkeys for on demand photostim while that gui is in the foreground
    end
    
    %%% Read-only sub-controller handles
    %properties (SetAccess=immutable,Transient)
    properties 
        hCycleManagerCtrl;
    end

    %% FRIEND PROPS
    properties (Hidden)
        motorStepSize = [0.1 0.1 0.1];  %Step size to use, in um, for motor increment/decrement operations in X,Y,Z axes. Z axis value pertains to active Z controller, if secondary is present.
        pbIdx;
    end
    
    properties (Hidden, Dependent,SetAccess={?scanimage.interfaces.Class})
        mainControlsStatusString;
    end
    
    properties (Hidden, SetAccess=?scanimage.interfaces.Class)
        beamProp2Control;       %Scalar struct. Fields: SI beam property name. values: uicontrol handle arrays for that prop. The properties in this struct must be beam-indexed (with round brackets).
        waitCursorProps = {'acqInitInProgress'};
        cancelStart = false;
    end
    
    %% INTERNAL PROPS
    properties (SetAccess=private,Hidden)
        motorListeners = [];
        addedPaths = {};                % cell array of paths that were added to the Matlab search path by scanimage
        defaultGuis = {};               % cell array of guis that are displayed by default on startup
        initComplete = false;
        
        pshk_zeroshit = 0;              % love this variable name.
    end
    
    properties (Constant,Hidden)
        motorMaxNumUserDefinedPositions = 100;
        WINDOW_BORDER_SPACING = 8; % [pixels] space between tethered guis
    end
    
    properties(Hidden,SetAccess=private)
        usrSettingsPropListeners; % col vec of listener objects used for userSettingsV4
        hSliderListener = [];
        hWaitCursorListeners = [];
        
        hPowbAx;
        hPowbCtxIm;
        hPowbBoxSurf;
        hPowbBoxTL;
        hPowbBoxTR;
        hPowbBoxBL;
        hPowbBoxBR;
        hPowbBoxT;
        hPowbBoxB;
        hPowbBoxL;
        hPowbBoxR;
        hPowbBoxCtr;
        hText;
        hPowbOthers;
        hOthTexts;
        
        hThorBScope2 = [];
        hThorBScope2MC = [];
        hThorBScope2LSC = [];
        hBScopeListeners = {};
        
        hPmtListener = [];
        gPowers;
        gGains;
        gTrips;
        gOffs;
        gBands;
    end
    
    properties(Hidden,Dependent,SetAccess=private)
        hMainPbFastCfg;  % 6x1 vector of MainControls fastCfg buttons
    end
    
    %%% USER FUNCTION RELATED PROPERTIES
    properties(Hidden,Dependent)
        userFunctionsViewType; % string enum; either 'CFG', 'USR', or 'none'.
        userFunctionsCurrentEvents;
        userFunctionsCurrentProp;
    end
    
    %%% ABSTRACT PROPERTY REALIZATIONS (most.Controller)
    properties (SetAccess=protected)
        propBindings = [];
    end
    
    %% LIFECYCLE
    methods (Hidden)
        function obj = SIController(hModel)
            baseDirectory = fileparts(which('scanimage'));
            requiredPaths{1} = fullfile(baseDirectory, 'guis');
            requiredPaths{2} = fullfile(baseDirectory, 'guis', 'icons');
            addedPaths_ = most.idioms.addPaths(requiredPaths);
                        
            visibleGuis = {'mainControlsV4' 'imageControlsV4' 'configControlsV4' 'channelControlsV4'};
            hiddenGuis = {'zDisplayV1', 'motorControlsV5' 'posnControlsV5' 'fastZControlsV4' 'powerControlsV4' 'powerBoxControlsV4'...
                'fastConfigurationV4' 'userFunctionControlsV4' 'triggerControlsV5' 'userSettingsV4' 'mroiControlsV5'...
                'photostimControlsV5' 'alignmentControlsV5' 'pmtControlsV5' 'pointClickInterfaceV5'};
            hiddenGuis(cellfun(@(x)~exist(x),hiddenGuis)) = [];
            allGuis = union(visibleGuis, hiddenGuis);
            
            for i = 1:numel(hModel.OptionalComponents)
                compName = hModel.OptionalComponents{i};
                if isprop(hModel.(compName), 'guis') && ~isempty(hModel.(compName).guis)
                    allGuis = union(allGuis, hModel.(compName).guis);
                end
            end
            obj = obj@most.Controller(hModel,{},unique(allGuis));
            obj.addedPaths = addedPaths_; % this call has to occur after the superclass constructor
            obj.defaultGuis = visibleGuis;
            
            %Capture keypresses for FastCfg F-key behavior. At moment, set
            %KeyPressFcn for all figures, uicontrols, etc so that all
            %keypresses over SI guis are captured. This can be modified
            %if/when certain figures/uicontrols need their own KeyPressFcns.
            structfun(@(handles)obj.ziniSetKeyPressFcn(handles),obj.hGUIData);
            
            %GUI Initializations
            obj.ziniMainControls();
            obj.ziniConfigControls();
            obj.ziniImageControls();
            obj.ziniPowerControls();
            obj.ziniPowerBoxControls();
            obj.ziniBScope2Controls();
            obj.ziniPmtControls();
            obj.ziniMotorControls();
            obj.ziniPosnControls();
            obj.ziniUsrSettingsGUI();
            obj.ziniTriggers();
            obj.ziniFastZControls();
            obj.ziniChannelControls();
            obj.ziniRegisterFigs();
            obj.ziniZDisplay();

            obj.hCycleManagerCtrl = scanimage.ctrl.CycleManagerController(hModel.hCycleManager);
            obj.registerGUI(obj.hCycleManagerCtrl.view.gui);
            
            %Listener Initializations
            for i = 1:numel(obj.waitCursorProps)
                lobj = obj.hModel;
                c = strsplit(obj.waitCursorProps{i}, '.');
                if numel(c) > 1
                    for j = 1:numel(c)-1
                        lobj = lobj.(c{j});
                    end
                end
                obj.hWaitCursorListeners{end+1} = addlistener(lobj,c{end},'PostSet',@(varargin)waitCursorUpdate);
            end
            obj.hWaitCursorListeners = [obj.hWaitCursorListeners{:}];
            
            %Initialize controller properties with set-access side-effects
            obj.motorStepSize = obj.motorStepSize;
            
            function waitCursorUpdate
                persistent curscache
                wt = cellfun(@(x)evalin('caller',x),strcat('obj.hModel.', obj.waitCursorProps));
                if any(wt)
                    if isempty(curscache)
                        nms = fieldnames(obj.hGUIs);
                        for k = 1:numel(nms)
                            try
                                curscache.(nms{k}) = get(obj.hGUIs.(nms{k}), 'pointer');
                            catch
                            end
                        end
                    end
                    set(obj.hGUIsArray, 'pointer', 'watch');
                    drawnow
                else
                    if ~isempty(curscache)
                        nms = fieldnames(curscache);
                        for k = 1:numel(nms)
                            try
                                set(obj.hGUIs.(nms{k}), 'pointer', curscache.(nms{k}));
                            catch
                            end
                        end
                    end
                    
                    drawnow
                    curscache = [];
                end
            end
        end
        
        function initialize(obj,varargin)
            initialize@most.Controller(obj);
            
            %Load user file (which adjusts figure positions). If no user
            %file is loaded raise default guis in default positions
            if ~obj.hModel.hConfigurationSaver.usrLoadUsr(varargin{:})
                obj.ziniFigPositions();
                cellfun(@(gui)obj.showGUI(obj.hGUIs.(gui)),obj.defaultGuis)
                arrayfun(@(figNum)figure(obj.hModel.hDisplay.hFigs(figNum)),obj.hModel.hChannels.channelDisplay)
            end
            
            %Do final pass of row labelling on channels GUI.
            obj.hGUIData.channelControlsV4.pcChannelConfigg.resize(obj.hModel.hChannels.channelsAvailable);
            
            %Mark initialization as complete.
            obj.initComplete = true;
            
            obj.hModel.hUserFunctions.notify('applicationOpen');
        end
        
        function exit(obj)
            obj.hModel.exit();
        end
        
        function delete(obj)
            delete(obj.hCycleManagerCtrl)
            delete@most.Controller(obj);
            delete(obj.hSliderListener);
            delete(obj.hWaitCursorListeners);
            delete(obj.motorListeners);
            most.idioms.safeDeleteObj(obj.hBScopeListeners);
            most.idioms.safeDeleteObj(obj.hPmtListener);
            
            % most.idioms.removePaths(obj.addedPaths);
        end
        
        function resetScanImage(obj)
            ans_ = questdlg('ScanImage needs to exit to reset. Do you want to proceed?',...
                'Exit ScanImage Confirmation','Yes','No','No');
            if strcmpi(ans_,'No')
                return; %Abort this exit function
            end
            classDataDir_ = obj.hModel.classDataDir; % obj.exit will delete the model cache the classDataDir property
            obj.exit()
            scanimage.util.resetClassDataFiles(classDataDir_);
        end
        
        function resetDaqDevices(obj)
            ans_ = questdlg('ScanImage needs to exit to reset all NI DAQ devices. Do you want to proceed?',...
                'Exit ScanImage Confirmation','Yes','No','No');
            if strcmpi(ans_,'No')
                return; %Abort this exit function
            end
            obj.exit()
            scanimage.util.resetDaqDevices();
        end
        
        function ziniChannelControls(obj)
            %obj.hGUIData.channelControlsV4.pcChannelConfig.resize(obj.hModel.MAX_NUM_CHANNELS);
            %numChan = obj.hModel.hScan2D.channelsAvailable;
            numChan = obj.hModel.hChannels.channelsAvailable;
            
           % obj.hGUIData.channelControlsV4.pcChannelConfig.resize(numChan);
            obj.hGUIData.channelControlsV4.pcChannelConfigg.resize(numChan);
            obj.hGUIData.channelControlsV4.channelImageHandler.initColorMapsInTable(); % re-init to deal with resize

            % set the input ranges to the available ranges of hScan2D
            cellNumRanges = obj.hModel.hChannels.channelAvailableInputRanges;
            cellStringRanges = cellfun(@(numRange)sprintf('[%s %s]',num2str(numRange(1)),num2str(numRange(2))),cellNumRanges,'UniformOutput',false);
% 
%             hTable = obj.hGUIData.channelControlsV4.tblChanConfig;
%             colFormat = get(hTable,'ColumnFormat');
%             colFormat{3} = cellStringRanges;
%             set(hTable,'ColumnFormat',colFormat);

            hTable = obj.hGUIData.channelControlsV4.tblChanConfigg;
            colFormat = get(hTable,'ColumnFormat');
            colFormat{3} = cellStringRanges;
            set(hTable,'ColumnFormat',colFormat);
                       
            % This re-registers figure windows with specific channels in the channel window. Not sure what this does
            % as virtually everything is handled via most MVC. Why change
            % use numChan?
            %
            %obj.hModel.hDisplay.prepareDisplayFigs();
            obj.hGUIData.channelControlsV4.channelImageHandler.registerChannelImageFigs(obj.hModel.hDisplay.hFigs(1:numChan));
            % obj.hGUIData.channelControlsV4.channelImageHandler.registerChannelImageFigs(obj.hModel.hDisplay.hFigs);
        end
        
        function ziniFigPositions(obj)
%             movegui(obj.hGUIs.mainControlsV4,'northwest');
%             drawnow expose % otherwise the main gui is not always moved to the correct position
            most.gui.tetherGUIs([],obj.hGUIs.mainControlsV4,'northwest',[]);
            most.gui.tetherGUIs(obj.hGUIs.mainControlsV4,obj.hGUIs.configControlsV4,'righttop',obj.WINDOW_BORDER_SPACING);
            most.gui.tetherGUIs(obj.hGUIs.mainControlsV4,obj.hGUIs.imageControlsV4,'bottomleft',obj.WINDOW_BORDER_SPACING);
            most.gui.tetherGUIs(obj.hGUIs.configControlsV4,obj.hGUIs.channelControlsV4,'righttop',obj.WINDOW_BORDER_SPACING);
            most.gui.tetherGUIs(obj.hGUIs.imageControlsV4,obj.hGUIs.powerControlsV4,'bottomleft',obj.WINDOW_BORDER_SPACING);
            most.gui.tetherGUIs(obj.hGUIs.imageControlsV4,obj.hGUIs.motorControlsV5,'righttop',obj.WINDOW_BORDER_SPACING);
            most.gui.tetherGUIs(obj.hGUIs.motorControlsV5,obj.hGUIs.fastZControlsV4,'bottomleft',obj.WINDOW_BORDER_SPACING);
            most.gui.tetherGUIs(obj.hGUIs.fastZControlsV4,obj.hGUIs.pmtControlsV5,'bottomleft',obj.WINDOW_BORDER_SPACING);
            
            if isfield(obj.hGUIs, 'bScope2ControlsV5')
                most.gui.tetherGUIs(obj.hGUIs.motorControlsV5,obj.hGUIs.bScope2ControlsV5,'righttop',obj.WINDOW_BORDER_SPACING);
            end
            
            % stack channel display figures
            initialPosition = [700 300];
            offset = 30;
            numFigs = length(obj.hModel.hDisplay.hFigs);
            for i = 1:numFigs
                figNum = numFigs - i + 1;
                offset_ = offset * (i-1);
                position = [initialPosition(1)+offset_, initialPosition(2)-offset_];
                setpixelposition(obj.hModel.hDisplay.hFigs(figNum),[position(1,:) 408 408]);
            end
            setpixelposition(obj.hModel.hDisplay.hMergeFigs,[700 250 490 490]);     %Invisible by default
        
            % ensure no figure is located outside the visible part of the screen
            allFigs = [obj.hGUIsArray(:)' obj.hModel.hDisplay.hFigs(:)' obj.hModel.hDisplay.hMergeFigs(:)'];
            for hFig = allFigs
               most.gui.moveOntoScreen(hFig);
            end
        end
        
        function ziniRegisterFigs(obj)
            % makes channel windows 'managed' figures so that they are
            % saved in the user settings file
            for i = 1:obj.hModel.hChannels.channelsAvailable
                hFig = obj.hModel.hDisplay.hFigs(i);
                obj.registerGUI(hFig);
            end
%             keyboard
%             for hDisp = obj.hModel.hDisplay.scanfieldDisplays
%                 obj.registerGUI(hDisp.hFig);
%             end
            obj.registerGUI(obj.hModel.hDisplay.hMergeFigs);
        end
        
        function zcbkKeyPress(obj,~,evt)
            % Currently this handles keypresses for all SI guis
            switch evt.Key
                % Keys that should be captured over all guis go in this top level case structure
                case {'f1' 'f2' 'f3' 'f4' 'f5' 'f6'}
                    idx = str2double(evt.Key(2));
                    tfRequireCtrl = get(obj.hGUIData.fastConfigurationV4.cbRequireControl,'Value');
                    tfLoadFastCfg = ~tfRequireCtrl || ismember('control',evt.Modifier);
                    tfBypassAutoStart = ismember('shift',evt.Modifier);
                    if tfLoadFastCfg
                        obj.hModel.hConfigurationSaver.fastCfgLoadConfig(idx,tfBypassAutoStart);
                    end
                    
                % Gui specific keys
                otherwise
                    [tf, i] = ismember(gcf, obj.hGUIsArray);
                    if tf
                        switch obj.guiNames{i}
                        end
                    end
            end
        end
        
        function ziniSetKeyPressFcn(obj,handles)
            tags = fieldnames(handles);
            for c = 1:numel(tags)
                h = handles.(tags{c});
                if isprop(h,'KeyPressFcn')
                    set(h,'KeyPressFcn',@(src,evt)obj.zcbkKeyPress(src,evt));
                end
            end
        end
        
        function ziniMainControls(obj)
            %Disable controls for currently unimplemented features
            disabledControls = {'stCycleIteration' 'stCycleIterationOf' ...
                'etIterationsDone' 'etIterationsTotal' ...
                'tbCycleControls' ...
                'centerOnSelection' 'zoomhundredsslider' ...
                'zoomhundreds' 'pbLastLine' ...
                'pbLastLineParent' 'snapShot' 'numberOfFramesSnap' ...
                'pbBase' 'pbSetBase' 'pbRoot'};
            disabledControls = [disabledControls {'cbEnableMroi' 'pbPhotostim' 'pbEditRoiGroup' 'mnu_View_PhotostimControls' 'mnu_View_AlignmentControls'}];
            cellfun(@(s)set(obj.hGUIData.mainControlsV4.(s),'Enable','off'),disabledControls);
            
            hiddenControls = {'xstep' 'ystep'};
            cellfun(@(s)set(obj.hGUIData.mainControlsV4.(s),'Visible','off'),hiddenControls);
            
            %Disable menu items for currently unimplemented features
            disabledMenuItems = {};
            
            cellfun(@(s)set(obj.hGUIData.mainControlsV4.(s),'Enable','off'),disabledMenuItems);
            set(obj.hGUIData.mainControlsV4.figure1,'closeRequestFcn',@lclCloseEventHandler);

			%+++
            set(obj.hGUIData.mainControlsV4.mnu_Settings_YokeWS,'Checked','off');
            
            function lclCloseEventHandler(src,evnt)
                ans_ = questdlg('Are you sure you want to exit ScanImage?','Exit ScanImage Confirmation','Yes','No','No');
                if strcmpi(ans_,'No')
                    return; %Abort this exit function
                end
                set(src,'CloseRequestFcn',[]); % User clicked yes, don't ask again even if exit fails
                obj.exit();
            end
        end
        
        function ziniZDisplay(obj)
            
        end
        
        function ziniConfigControls(obj)
            %Configure imaging system list
            nms = cellfun(@(x)x.name,obj.hModel.hScanners, 'UniformOutput', false);
            set(obj.hGUIData.configControlsV4.pmImagingSystem, 'string', nms);
            
            %Hide controls not used
            hideControls = {'rbScanPhaseHardware' 'stShutterDelay' 'stShutterDelayMs' 'etShutterDelay'};
            cellfun(@(s)set(obj.hGUIData.configControlsV4.(s),'Visible','off'), hideControls);
            
            %Disable controls with features not supported
            disableControls = {'stShutterDelay' 'stShutterDelayMs' 'etShutterDelay' 'rbScanPhaseSoftware'};
            disableControls = [disableControls {'pbAlignment'}];
            cellfun(@(s)set(obj.hGUIData.configControlsV4.(s),'Enable','off'), disableControls);
            
            
            %Set properties of line phase slider
            obj.cfgLinePhaseSlider();
        end
        
        function ziniImageControls(obj)            
            %Initialize channel LUT controls
            for i=1:4
                if i > obj.hModel.hChannels.channelsAvailable %Disable controls for reduced channel count devices
                    set(findobj(obj.hGUIData.imageControlsV4.(sprintf('pnlChan%d',i)),'Type','uicontrol'),'Enable','off');
                    set(obj.hGUIData.imageControlsV4.(sprintf('blackSlideChan%d',i)),'Min',0,'Max',1,'SliderStep',[.01 .1],'Value',0);
                    set(obj.hGUIData.imageControlsV4.(sprintf('whiteSlideChan%d',i)),'Min',0,'Max',1,'SliderStep',[.01 .1],'Value',0);
                    set(obj.hGUIData.imageControlsV4.(sprintf('blackEditChan%d',i)),'String',num2str(0));
                    set(obj.hGUIData.imageControlsV4.(sprintf('whiteEditChan%d',i)),'String',num2str(100));
                else
                    %Allow 10-percent of negative range, if applicable
                    set(findobj(obj.hGUIData.imageControlsV4.(sprintf('pnlChan%d',i)),'Type','uicontrol'),'Enable','on');
                    chanLUTMin = round(obj.hModel.hChannels.channelLUTRange(1) * 0.1);
                    chanLUTMax = obj.hModel.hChannels.channelLUTRange(2);
                    blackVal = max(chanLUTMin,obj.hModel.hChannels.channelLUT{i}(1));
                    whiteVal = min(chanLUTMax,obj.hModel.hChannels.channelLUT{i}(2));
                    set(obj.hGUIData.imageControlsV4.(sprintf('blackSlideChan%d',i)),'Min',chanLUTMin,'Max',chanLUTMax,'SliderStep',[.001 .05],'Value',blackVal);
                    set(obj.hGUIData.imageControlsV4.(sprintf('whiteSlideChan%d',i)),'Min',chanLUTMin,'Max',chanLUTMax,'SliderStep',[.001 .05],'Value',whiteVal);
                end
            end
            
            %Move Frame Averaging/Selection panel up if there are 2 or less channels
            if obj.hModel.MAX_NUM_CHANNELS <= 2
                charShift = (obj.hModel.MAX_NUM_CHANNELS - 2) * 5;
                
                for i=3:obj.hModel.MAX_NUM_CHANNELS
                    hPnl = obj.hGUIData.imageControlsV4.(sprintf('pnlChan%d',i));
                    set(hPnl,'Visible','off');
                    set(findall(hPnl),'Visible','off');
                end
                
                for i=1:2
                    hPnl = obj.hGUIData.imageControlsV4.(sprintf('pnlChan%d',i));
                    set(hPnl,'Position',get(hPnl,'Position') + [0 -charShift 0 0]);
                end
                
                hFig = obj.hGUIs.imageControlsV4;
                set(hFig,'Position',get(hFig,'Position') + [0 charShift 0 -charShift]);
            end
        end
        
        function ziniPowerControls(obj)
            if obj.hModel.hBeams.numInstances
                znstConnectBeamPropToBeamControl('powers',[findobj(obj.hGUIs.powerControlsV4,'Tag','etBeamPower');...
                    findobj(obj.hGUIs.powerControlsV4,'Tag','sldBeamPower')]);
                znstConnectBeamPropToBeamControl('powerLimits',[findobj(obj.hGUIs.powerControlsV4,'Tag','etMaxLimit');...
                    findobj(obj.hGUIs.powerControlsV4,'Tag','sldMaxLimit')]);
                znstConnectBeamPropToBeamControl('lengthConstants',findobj(obj.hGUIs.powerControlsV4,'Tag','etZLengthConstant'));
                znstConnectBeamPropToBeamControl('pzAdjust',findobj(obj.hGUIs.powerControlsV4,'Tag','cbPzAdjust'));
                znstConnectBeamPropToBeamControl('directMode',findobj(obj.hGUIs.powerControlsV4,'Tag','cbDirectMode'));
                znstConnectBeamPropToBeamControl('interlaceDecimation',findobj(obj.hGUIs.powerControlsV4,'Tag','etInterlaceDecimation'));
                znstConnectBeamPropToBeamControl('interlaceOffset',findobj(obj.hGUIs.powerControlsV4,'Tag','etInterlaceOffset'));
                set(obj.hGUIData.powerControlsV4.pumBeamIdx,'Value',1);
                set(obj.hGUIData.powerControlsV4.pumBeamIdx,'String',obj.hModel.hBeams.displayNames);
                obj.changedBeamPowerUnits();
                obj.beamDisplayIdx = 1;
                obj.defaultGuis{end+1} = 'powerControlsV4';
            else
                most.gui.disableAll(obj.hGUIs.powerControlsV4);
            end
            
            %TODO: Support this 'dynamic' binding of control to a property as a Controller method OR support a Pcontrol for binding to array vals with display/control of 1 index at a time determined by an index control
            function znstConnectBeamPropToBeamControl(propName,hControls)
                propName = sprintf('hBeams___%s',propName);
                obj.beamProp2Control.(propName) = hControls;
                set(hControls,'UserData',propName);
            end
        end
        
        function ziniPowerBoxControls(obj)
            if obj.hModel.hBeams.numInstances
                if ~most.idioms.isValidObj(obj.hPowbAx)
                    obj.hPowbAx = obj.hGUIData.powerBoxControlsV4.axBoxPos;
                    set(obj.hPowbAx,'XLim',[0 1],'YLim',[0 1],'ButtonDownFcn',@(varargin)obj.powbPanFcn(true))
                    obj.hPowbCtxIm = surface([0 1],[0 1],zeros(2),'Parent',obj.hPowbAx,'Hittest','off','FaceColor','texturemap',...
                        'CData',zeros(2,2,3),'EdgeColor','none','FaceLighting','none','FaceAlpha',1);
                    obj.hPowbBoxSurf = surface([.25 .75],[.25 .75],ones(2),'Parent',obj.hPowbAx,'Hittest','off','FaceColor','r',...
                        'EdgeColor','none','FaceLighting','none','FaceAlpha',0.2);
                    
                    args = {'Parent',obj.hPowbAx,'ZData',2,'Color','r','Hittest','on','Marker','.','MarkerSize',25};
                    obj.hPowbBoxTL = line('XData',.25,'YData',.25,'ButtonDownFcn',@(varargin)obj.powbCpFunc([1 1 0 0],true),args{:});
                    obj.hPowbBoxTR = line('XData',.75,'YData',.25,'ButtonDownFcn',@(varargin)obj.powbCpFunc([0 1 1 0],true),args{:});
                    obj.hPowbBoxBL = line('XData',.25,'YData',.75,'ButtonDownFcn',@(varargin)obj.powbCpFunc([1 0 0 1],true),args{:});
                    obj.hPowbBoxBR = line('XData',.75,'YData',.75,'ButtonDownFcn',@(varargin)obj.powbCpFunc([0 0 1 1],true),args{:});
                    obj.hPowbBoxCtr = line('XData',.5,'YData',.5,'ButtonDownFcn',@(varargin)obj.powbCpFunc([1 1 1 1],true),args{:});
                    obj.hText = text(.25,.25,2,'Power Box','Parent',obj.hPowbAx,'color','y','Hittest','off');
                    if obj.graphics2014b
                        obj.hText.PickableParts = 'none';
                    end
                    
                    args = {'Parent',obj.hPowbAx,'ZData',[1.5 1.5],'Color','r','Hittest','on','LineWidth',1.5,'ButtonDownFcn',@(varargin)obj.powbCpFunc([1 1 1 1],true)};
                    obj.hPowbBoxT = line('XData',[.25 .75],'YData',[.25 .25],args{:});
                    obj.hPowbBoxB = line('XData',[.25 .75],'YData',[.75 .75],args{:});
                    obj.hPowbBoxL = line('XData',[.25 .25],'YData',[.25 .75],args{:});
                    obj.hPowbBoxR = line('XData',[.75 .75],'YData',[.25 .75],args{:});
                    
                    set(obj.hGUIs.powerBoxControlsV4,'WindowScrollWheelFcn',@obj.powbScrollWheelFcn)
                end
                
                %hide unusable controls
                for iterChannels = 1:4
                    if iterChannels <= obj.hModel.hChannels.channelsAvailable
                        set(obj.hGUIData.powerBoxControlsV4.(sprintf('pbCopy%d',iterChannels)),'Enable','on');
                    else
                        set(obj.hGUIData.powerBoxControlsV4.(sprintf('pbCopy%d',iterChannels)),'Enable','off');
                    end
                end
                
                %power box dropdown
                nms = {};
                for pb = obj.hModel.hBeams.powerBoxes
                    nms{end+1} = pb.name;
                    if isempty(nms{end})
                        nms{end} = sprintf('Power Box %d', numel(nms));
                    end
                end
                nms{end+1} = 'New Power Box';
                set(obj.hGUIData.powerBoxControlsV4.pmPbSel,'String',nms);
                
                i = obj.pbIdx;
                if i <= numel(obj.hModel.hBeams.powerBoxes)
                    set([obj.hPowbBoxSurf obj.hPowbBoxTL obj.hPowbBoxTR obj.hPowbBoxBL obj.hPowbBoxBR obj.hText...
                        obj.hPowbBoxCtr obj.hPowbBoxT obj.hPowbBoxB obj.hPowbBoxL obj.hPowbBoxR],'visible','on');
                    set([obj.hGUIData.powerBoxControlsV4.etPowers...
                        obj.hGUIData.powerBoxControlsV4.etPosition],'enable','on');
                    set(obj.hGUIData.powerBoxControlsV4.pnPbSettings,'Title',['Power Box Settings (' nms{i} ')']);
                else
                    set([obj.hPowbBoxSurf obj.hPowbBoxTL obj.hPowbBoxTR obj.hPowbBoxBL obj.hPowbBoxBR obj.hText...
                        obj.hPowbBoxCtr obj.hPowbBoxT obj.hPowbBoxB obj.hPowbBoxL obj.hPowbBoxR],'visible','off');
                    set([obj.hGUIData.powerBoxControlsV4.etPowers...
                        obj.hGUIData.powerBoxControlsV4.etPosition],'enable','off');
                    set([obj.hGUIData.powerBoxControlsV4.etPowers...
                        obj.hGUIData.powerBoxControlsV4.etPosition],'string','');
                    set(obj.hGUIData.powerBoxControlsV4.pnPbSettings,'Title','Power Box Settings');
                end
            else
                most.gui.disableAll(obj.hGUIs.powerBoxControlsV4);
            end
        end
        
        function ziniBScope2Controls(obj)
            if ismember('bScope2ControlsV5', obj.guiNames)
                showGui = false;
                hasGGMirror = false;
                hasGRMirror = false;
                hasFlipperMirror = false;
                hasRotation = false;
                disabledControls = {};
                
                if isprop(obj.hModel, 'hBScope2') && isa(obj.hModel.hBScope2, 'dabs.thorlabs.BScope2')
                    obj.hThorBScope2 = obj.hModel.hBScope2;
                    if obj.hModel.hBScope2.ecuInitSuccessful
                        showGui = true;
                    else
                        disabledControls = {'etScanAlign' 'slScanAlign'};
                    end
                    
                    hasGRMirror = ~isempty(obj.hThorBScope2.galvoResonantMirrorInPath);
                    hasGGMirror = ~isempty(obj.hThorBScope2.galvoGalvoMirrorInPath);
                    hasFlipperMirror = ~isempty(obj.hThorBScope2.flipperMirrorPosition);
                    
                    if hasGRMirror || hasGGMirror || hasFlipperMirror
                        obj.hThorBScope2MC = obj.hThorBScope2;
                    end
                end
                
                if most.idioms.isValidObj(obj.hModel.hMotors.hMotor) && isa(obj.hModel.hMotors.hMotor.hLSC, 'dabs.thorlabs.MCM5000')
                    showGui = true;
                    obj.hThorBScope2LSC = obj.hModel.hMotors.hMotor.hLSC;
                    hasRotation = obj.hThorBScope2LSC.hasRotation;
                    
                    hasGGMirror = true;
                    hasGRMirror = true;
                    hasFlipperMirror = true;
                    obj.hThorBScope2MC = obj.hThorBScope2LSC;
                end
                
                if hasRotation
                    obj.hBScopeListeners{end+1} = obj.hThorBScope2LSC.addlistener('rotationAngleAbsolute', 'PostSet', @obj.changedBScope2RotationAngle);
                else
                    disabledControls = union(disabledControls, {'etRotationAngle' 'pbUpdateRotationAngle' 'pbResetLSC'...
                        'pbRotationAngle_Dec' 'etRotationAngleStepSize' 'pbRotationAngle_Inc'});
                end
                
                if hasGGMirror
                    obj.hBScopeListeners{end+1} = obj.hThorBScope2MC.addlistener('galvoGalvoMirrorInPath', 'PostSet', @obj.changedGalvoGalvoMirrorInPath);
                    obj.changedGalvoGalvoMirrorInPath();
                else
                    disabledControls = union(disabledControls, {'pbGG_In' 'pbGG_Out'});
                end
                
                if hasGRMirror
                    obj.hBScopeListeners{end+1} = obj.hThorBScope2MC.addlistener('galvoResonantMirrorInPath', 'PostSet', @obj.changedGalvoResonantMirrorInPath);
                    obj.changedGalvoResonantMirrorInPath();
                else
                    disabledControls = union(disabledControls, {'pbGR_In' 'pbGR_Out'});
                end
                
                if hasFlipperMirror
                    obj.hBScopeListeners{end+1} = obj.hThorBScope2MC.addlistener('flipperMirrorPosition', 'PostSet', @obj.changedFlipperMirrorPosition);
                    obj.changedFlipperMirrorPosition();
                else
                    disabledControls = union(disabledControls, {'pbPmt' 'pbCamera'});
                end
                
                cellfun(@(s)set(obj.hGUIData.bScope2ControlsV5.(s),'Enable','off'),disabledControls);
                
                % Only show the gui if some part of the bscope2 component is working
                if showGui
                    obj.defaultGuis{end+1} = 'bScope2ControlsV5';
                end
                
                obj.hBScopeListeners = [obj.hBScopeListeners{:}];
            end
        end
        
        function ziniPmtControls(obj)
            numPmts = obj.hModel.hPmts.numPmts;
            if numPmts < 1
                most.gui.disableAll(obj.hGUIs.pmtControlsV5);
            else
                %Leaving pmt names general for now
%                 for i = 1:numPmts
%                     stTag = sprintf('stPmt%d',i);
%                     pmtName = obj.hModel.hPmts.names{i};
%                     set(obj.hGUIData.pmtControlsV5.(stTag),'String',pmtName);
%                 end
                
                % disable extra controls
                for i = numPmts+1:4                    
                    pbTag = sprintf('pbPmt%dPower',i);
                    set(obj.hGUIData.pmtControlsV5.(pbTag),'Enable','off');
                    
                    etTag = sprintf('etPmt%dGain',i);
                    set(obj.hGUIData.pmtControlsV5.(etTag),'Enable','off');
                    
                    pbTag = sprintf('etPmt%dStatus',i);
                    set(obj.hGUIData.pmtControlsV5.(pbTag),'Enable','off');
                    
                    pbTag = sprintf('pbResetPmt%d',i);
                    set(obj.hGUIData.pmtControlsV5.(pbTag),'Enable','off');
                    
                    etTag = sprintf('etPmt%dOffset',i);
                    set(obj.hGUIData.pmtControlsV5.(etTag),'Enable','off');
                    
                    etTag = sprintf('etPmt%dBandwidth',i);
                    set(obj.hGUIData.pmtControlsV5.(etTag),'Enable','off');
                end
                
                for i = 1:numPmts
                    stTag = sprintf('stPmt%d',i);
                    pmtName = obj.hModel.hPmts.names{i};
                    set(obj.hGUIData.pmtControlsV5.(stTag),'String',pmtName);
                end
                
                % listen for updates
                obj.gPowers = nan(1, numPmts);
                obj.gGains = nan(1, numPmts);
                obj.gTrips = nan(1, numPmts);
                obj.gOffs = nan(1, numPmts);
                obj.gBands = nan(1, numPmts);
                obj.changedPmtsStatus();
                obj.hPmtListener = obj.hModel.hPmts.addlistener('statusChanged', @obj.changedPmtsStatus);
                obj.defaultGuis{end+1} = 'pmtControlsV5';
            end
        end
        
        function ziniMotorControls(obj)
            %Disable all if motor is disabled
            if obj.hModel.hMotors.numInstances <= 0
                most.gui.disableAll(obj.hGUIs.motorControlsV5);
                return;
            end
            
            if obj.hModel.hMotors.numInstances >= 2 || obj.hModel.hFastZ.numInstances
                set(obj.hGUIData.motorControlsV5.pbZeroXY,'Visible','off');
                set(obj.hGUIData.motorControlsV5.pbZeroZ,'Visible','off');
                set(obj.hGUIData.motorControlsV5.pbAltZeroXY,'Visible','on');
                set(obj.hGUIData.motorControlsV5.pbAltZeroZ,'Visible','on');
                set(obj.hGUIData.motorControlsV5.etPosZZ,'Visible','on');
                set(obj.hGUIData.motorControlsV5.etPosZZTarget,'Visible','on');
                set(obj.hGUIData.motorControlsV5.etPosZZ,'Enable','off');
                
                switch obj.hModel.hMotors.motorDimensionConfiguration
                    case 'xyz-z'
                        set(obj.hGUIData.motorControlsV5.etPosZZTarget,'Enable','on');
                        set(obj.hGUIData.motorControlsV5.cbSecZ,'Visible','on');
                    otherwise
                        set(obj.hGUIData.motorControlsV5.cbSecZ,'Visible','off');
                        if obj.hModel.hFastZ.numInstances
                            set(obj.hGUIData.motorControlsV5.etPosZZTarget,'Enable','on');
                        else
                            set(obj.hGUIData.motorControlsV5.etPosZZTarget,'Enable','off');
                        end
                end
            else
                set(obj.hGUIData.motorControlsV5.pbZeroXY,'Visible','on');
                set(obj.hGUIData.motorControlsV5.pbZeroZ,'Visible','on');
                set(obj.hGUIData.motorControlsV5.pbAltZeroXY,'Visible','off');
                set(obj.hGUIData.motorControlsV5.pbAltZeroZ,'Visible','off');
                set(obj.hGUIData.motorControlsV5.cbSecZ,'Visible','off');
                set(obj.hGUIData.motorControlsV5.etPosZZ,'Visible','off');
                set(obj.hGUIData.motorControlsV5.etPosZZTarget,'Visible','off');
            end
            
            %initialize listeners
            listnrs = obj.hModel.hMotors.hMotor.addlistener('LSCError',...
                @(src,evt)obj.motorErrorCbk(src,evt));
            if obj.hModel.hMotors.numInstances >= 2
                listnrs(end+1,1) = obj.hModel.hMotors.hMotorZ.addlistener('LSCError',...
                    @(src,evt)obj.motorErrorCbk(src,evt));
            end
            % listnrs(end+1,1) = addlistener(obj.hModel.hMotors,'motorPosition','PostGet',@(src,evnt)obj.changedMotorPosition) fix SCIM-1061401 GJ 2015-12-01
            obj.motorListeners = listnrs;
            
            obj.hModel.hUserFunctions.addlistener('motorPositionUpdate',@(src,evnt)obj.changedMotorPosition);
            
            obj.defaultGuis{end+1} = 'motorControlsV5';
        end
        
        function ziniPosnControls(obj)
            %Disable all if motor is disabled
            if obj.hModel.hMotors.numInstances <= 0
                most.gui.disableAll(obj.hGUIs.posnControlsV5);
                return;
            else
                gd = obj.hGUIData.posnControlsV5;
                cbs = [gd.cbX gd.cbY gd.cbZ gd.cbZZ];
                nDims = numel(obj.hModel.hMotors.motorPosition);
                set(cbs(1:nDims), 'enable', 'on');
                set(cbs(nDims+1:end), 'enable', 'off');
                set(cbs(nDims+1:end), 'Value', false);
                set(gd.lbPosns, 'Value', 1);
            end
        end
        
        function ziniFastZControls(obj)
            if obj.hModel.hFastZ.numInstances <= 0
                most.gui.disableAll(obj.hGUIs.fastZControlsV4);
            else
                disabledControls = {'pmImageType'};
                cellfun(@(s)set(obj.hGUIData.fastZControlsV4.(s),'Enable','off'),disabledControls);
                obj.defaultGuis{end+1} = 'fastZControlsV4';
            end
            
            hiddenCtls = {'cbSpecifyZs' 'lblUserZs' 'etUserZs'};
            cellfun(@(s)set(obj.hGUIData.fastZControlsV4.(s),'Visible','off'),hiddenCtls);
        end
        
        
        function ziniTriggers(obj)
            set(obj.hGUIData.triggerControlsV5.pmTrigAcqInTerm,'String',obj.hModel.hScan2D.trigAcqInTermAllowed,'Value',1);
            set(obj.hGUIData.triggerControlsV5.pmTrigStopInTerm,'String',obj.hModel.hScan2D.trigStopInTermAllowed,'Value',1);
            set(obj.hGUIData.triggerControlsV5.pmTrigNextInTerm,'String',obj.hModel.hScan2D.trigNextInTermAllowed,'Value',1);
        end
        
        function ziniUsrSettingsGUI(obj)
            availableUsrProps = obj.hModel.mdlGetConfigurableProps()';
            % Throw a warning if any available user prop is not
            % SetObservable. This can happen b/c SetObservable-ness of usr
            % properties is required neither by the Model:mdlConfig
            % infrastructure nor by SI (this is arguably the right
            % thing to do). Meanwhile, the userSettings GUI provides a view
            % (via a propTable) into the current usrProps; this is
            % implemented via listeners. (Side note: ML silently allows
            % adding a listener to an obj for a prop that is not
            % SetObservable.)
            %
            % At the moment I believe all available usr props for SI3/4 are
            % indeed SetObservable, but this warning will be good for
            % maintenance moving forward.
            data(:,1) = sort(availableUsrProps); %#ok<TRSRT>
            data(:,2) = {false};                 %will get initted below
            set(obj.hGUIData.userSettingsV4.tblSpecifyUsrProps,'Data',data);
            obj.changedUsrPropList();
        end
    end
    
    %% PROP ACCESS
    methods
        function viewType = get.userFunctionsViewType(obj)
            viewBtn = get(obj.hGUIData.userFunctionControlsV4.bgView,'SelectedObject');
            if ~isempty(viewBtn)
                switch get(viewBtn,'Tag')
                    case 'tbUsr'
                        viewType = 'USR';
                    case 'tbCfg'
                        viewType = 'CFG';
                end
            else
                viewType = 'none';
            end
        end
        
        function evtNames = get.userFunctionsCurrentEvents(obj)
            switch obj.userFunctionsViewType
                case 'none'
                    evtNames = cell(0,1);
                case 'CFG'
                    evtNames = unique(obj.hModel.hUserFunctions.userFunctionsEvents);
                case 'USR'
                    evtNames = unique([obj.hModel.hUserFunctions.userFunctionsEvents;...
                                     obj.hModel.hUserFunctions.userFunctionsUsrOnlyEvents]);
            end
        end
        
        function propName = get.userFunctionsCurrentProp(obj)
            switch obj.userFunctionsViewType
                case 'none'
                    propName = '';
                case 'CFG'
                    propName = 'userFunctionsCfg';
                case 'USR'
                    propName = 'userFunctionsUsr';
            end
        end
        
        function changedUserFunctionsCfg(obj,~,~)
            switch obj.userFunctionsViewType
                case 'CFG'
                    obj.hGUIData.userFunctionControlsV4.uft.refresh();
            end
        end
        
        function changedUserFunctionsUsr(obj,~,~)
            switch obj.userFunctionsViewType
                case 'USR'
                    obj.hGUIData.userFunctionControlsV4.uft.refresh();
            end
        end
        
        function changedUserFunctionsOverride(obj,~,~)
            obj.hGUIData.userFunctionControlsV4.uftOverride.refresh();
        end
        
        function val = get.propBindings(obj)
            if isempty(obj.propBindings)
                obj.propBindings = lclInitPropBindings(obj.hModel);
            end
            
            val = obj.propBindings;
        end
        
        function val = get.pbIdx(obj)
            val = get(obj.hGUIData.powerBoxControlsV4.pmPbSel,'Value');
        end
        
        function set.pbIdx(obj,val)
            set(obj.hGUIData.powerBoxControlsV4.pmPbSel,'Value',val);
            obj.changedPowerBoxes();
        end
    end
    
    methods
        function val = get.hMainPbFastCfg(obj)
            val = [obj.hGUIData.mainControlsV4.pbFastConfig1; ...
                obj.hGUIData.mainControlsV4.pbFastConfig2; ...
                obj.hGUIData.mainControlsV4.pbFastConfig3; ...
                obj.hGUIData.mainControlsV4.pbFastConfig4; ...
                obj.hGUIData.mainControlsV4.pbFastConfig5; ...
                obj.hGUIData.mainControlsV4.pbFastConfig6];
        end
        
        % This sets the GUI-displayed status string, NOT the hModel status string.
        function set.mainControlsStatusString(obj,val)
            set(obj.hGUIData.mainControlsV4.statusString,'String',val);
        end
        
        % This gets the GUI-displayed status string, NOT the hModel status
        % string.
        function val = get.mainControlsStatusString(obj)
            val = get(obj.hGUIData.mainControlsV4.statusString,'String');
        end
        
        %%% Beams
        function set.beamDisplayIdx(obj,val)
            if obj.hModel.hBeams.numInstances <= 0
                return;
            end
            
            assert(ismember(val,1:obj.hModel.hBeams.totalNumBeams));
            if val~=obj.beamDisplayIdx
                obj.beamDisplayIdx = val;
                beamPropNames = fieldnames(obj.beamProp2Control);
                for i = 1:numel(beamPropNames)
                   obj.changedBeamParams(beamPropNames{i});
                end
                
                set(obj.hGUIData.powerControlsV4.pumBeamIdx,'Value',val); %#ok<*MCSUP>
            end
            
            data = get(obj.hGUIData.powerControlsV4.tblBeams,'Data');
            data(:,1) = {false};
            data(val,1) = {true};
            set(obj.hGUIData.powerControlsV4.tblBeams,'Data',data);
        end
        
        %%% Motors
        function set.motorStepSize(obj,val)
            currVal = obj.motorStepSize;
            assert(numel(val) == numel(currVal),'The motorStepSize value must have %d elements',numel(currVal));
            
            %Only change dimensions with valid values (positive, finite, smaller than fastMotionThreshold)
            val(val <= 0 | val > obj.hModel.hMotors.motorFastMotionThreshold | isinf(val)) = nan;
            unchangedDims = isnan(val);
            val(unchangedDims) = currVal(unchangedDims);
            
            %Set property & update view
            obj.motorStepSize = val;
            
            set(obj.hGUIData.motorControlsV5.etStepSizeX,'String',num2str(val(1),'%0.5g'));
            set(obj.hGUIData.motorControlsV5.etStepSizeY,'String',num2str(val(2),'%0.5g'));
            set(obj.hGUIData.motorControlsV5.etStepSizeZ,'String',num2str(val(3),'%0.5g'));
            
        end
    end
    
    %% USER METHODS
    %%% ACTION CALLBACKS
    methods
        %%% MAIN %%%
        function focusButton(obj)
            if strcmpi(obj.hModel.acqState,'idle')
                obj.hModel.startFocus();
                
                if obj.cancelStart
                    obj.cancelStart = false;
                    obj.hModel.abort();
                end
            end
        end
        
        function grabButton(obj)
            if strcmpi(obj.hModel.acqState,'idle')
                obj.hModel.startGrab();
                
                if obj.cancelStart
                    obj.cancelStart = false;
                    obj.hModel.abort();
                end
            end
        end
        
        function loopButton(obj)
            if strcmpi(obj.hModel.acqState,'idle')
                if ~obj.hModel.hCycleManager.enabled
                    obj.hModel.startLoop();
                else
                    obj.hModel.startCycle();
                end
                
                if obj.cancelStart
                    obj.cancelStart = false;
                    obj.hModel.abort();
                end
            end
        end
        
        function abortButton(obj)
            if obj.hModel.hCycleManager.enabled
                obj.hModel.hCycleManager.abort();
            end

            if obj.hModel.acqInitInProgress
                fAbort = obj.hGUIData.mainControlsV4.fAbort;
                gAbort = obj.hGUIData.mainControlsV4.gAbort;
                lAbort = obj.hGUIData.mainControlsV4.lAbort;
                
                obj.cancelStart = true;
                set([fAbort gAbort lAbort],'Enable','off');
            else
                obj.hModel.abort();
            end
        end
        
        %%% BEAM FUNCTION CALLBACKS
        function calibrateBeam(obj)
            beamIdx = obj.beamDisplayIdx;
            obj.hModel.hBeams.beamsCalibrate(beamIdx);
        end
        
        function showCalibrationCurve(obj)
            beamIdx = obj.beamDisplayIdx;
            obj.hModel.hBeams.beamsShowCalibrationCurve(beamIdx);
        end
        
        function measureCalibrationOffset(obj)
            beamIdx = obj.beamDisplayIdx;
            offset = obj.hModel.hBeams.beamsMeasureCalOffset(beamIdx,true);
            if ~isnan(offset)
                msg = sprintf('Calibration offset voltage: %.3g. Result saved to Machine Data file.',offset);
                msgbox(msg,'Calibration offset measured');
            end
        end
        
        %%% IMAGE FUNCTION CALLBACKS
        function showChannelDisplay(obj,channelIdx)
            set(obj.hModel.hDisplay.hFigs(channelIdx),'visible','on');
        end
        
        function showMergeDisplay(obj,channelIdx)
            if ~obj.hModel.hDisplay.channelsMergeEnable
                obj.hModel.hDisplay.channelsMergeEnable = true;
            end
        end
        
        function linePhaseImageFunction(obj,fcnName)
            hFig = obj.zzzSelectImageFigure();
            if isempty(hFig)
                return;
            end
            
            allChannelFigs = [obj.hModel.hDisplay.hFigs(1:obj.hModel.hChannels.channelsAvailable)];
            [tf,chanIdx] = ismember(hFig,allChannelFigs);
            if tf
                feval(fcnName,obj.hModel.hScan2D,chanIdx);
            end

        end

        %%% MOTOR CALLBACKS
        function motorZeroAction(obj,action)
            feval(action,obj.hModel.hMotors);
            obj.changedMotorPosition();
        end
        
        function motorClearZero(obj)
            obj.hModel.hMotors.hMotor.hLSC.clearSoftZero();
            obj.changedMotorPosition();
        end
        
        function motorStepPosition(obj,incOrDec,stepDim)
            if ~obj.hModel.hMotors.hMotor.hLSC.nonblockingMoveInProgress
                posn = obj.hModel.hMotors.motorPosition;
                switch incOrDec
                    case 'inc'
                        stepSign = 1;
                    case 'dec'
                        stepSign = -1;
                    otherwise
                        assert(false);
                end

                switch stepDim
                    case 'x'
                        posn(1) = posn(1) + (stepSign * obj.motorStepSize(1));
                    case 'y'
                        posn(2) = posn(2) + (stepSign * obj.motorStepSize(2));
                    case 'z'

                        if obj.hModel.hMotors.motorSecondMotorZEnable
                            if strcmpi(obj.hModel.hMotors.motorDimensionConfiguration,'xyz-z')
                                posnIdx = 4;
                            else
                                posnIdx = 3;
                            end

                            %Make 'decrement' = 'down'/'deeper'
                            if obj.hModel.hMotors.mdfData.motor2ZDepthPositive
                                stepSign = stepSign * -1;
                            end
                        else
                            posnIdx = 3;

                            %Make 'decrement' = 'down'/'deeper'
                            if obj.hModel.hMotors.mdfData.motorZDepthPositive
                                stepSign = stepSign * -1;
                            end
                        end

                        posn(posnIdx) = posn(posnIdx) + (stepSign * obj.motorStepSize(3));

                    otherwise
                        assert(false);
                end

                obj.hModel.hMotors.motorPosition = posn;
            else
                most.idioms.warn('Motor is currently executing a move operation. New command ignored.')
            end
        end
        
        function motorRecover(obj)
            if obj.hModel.hMotors.numInstances >= 1 && obj.hModel.hMotors.hMotor.lscErrPending
                obj.hModel.hMotors.hMotor.recover();
            end
            if obj.hModel.hMotors.numInstances >= 2 && obj.hModel.hMotors.hMotorZ.lscErrPending
                obj.hModel.hMotors.hMotorZ.recover();
            end
            
            % if we made it this far, then assume the error is fixed
            structfun(@nstEnable,obj.hGUIData.motorControlsV5);
            set(obj.hGUIData.motorControlsV5.pbRecover,'Visible','off');
            set(obj.hGUIData.motorControlsV5.pbRecover,'Enable','off');
            
            function nstEnable(h)
                if isprop(h,'Enable')
                    set(h,'Enable','on');
                end
            end
        end
        
        function stackSetStackStart(obj)
            obj.hModel.hStackManager.stackSetStackStart();
            % xxx DOC why it would be a bad idea for hModel to have a
            % dependent, setAccess=private, setobservable prop called
            % "tfStackStartEndPowersDefined" and for appC to listen to that
            % prop.
            if obj.hModel.hBeams.stackStartEndPowersDefined
                set(obj.hGUIData.motorControlsV5.cbOverrideLz,'Enable','on');
            end
        end
        
        function stackSetStackEnd(obj)
            obj.hModel.hStackManager.stackSetStackEnd();
            if obj.hModel.hBeams.stackStartEndPowersDefined
                set(obj.hGUIData.motorControlsV5.cbOverrideLz,'Enable','on');
            end
        end
        
        function stackClearStartEnd(obj)
            obj.hModel.hStackManager.stackClearStartEnd();
            set(obj.hGUIData.motorControlsV5.cbOverrideLz,'Enable','off');
        end
        
        function stackClearEnd(obj)
            obj.hModel.hStackManager.stackClearEnd();
            set(obj.hGUIData.motorControlsV5.cbOverrideLz,'Enable','off');
        end
        
        function changedPosnID(obj,guiObj,goButton)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            v = get(guiObj, 'string');
            if ~all(isstrprop(v, 'digit'))
                set(guiObj, 'string', '');
                set(goButton, 'enable', 'off');
            else
                v = str2double(v);
                if v > 0 && v <= numel(obj.hModel.hMotors.userDefinedPositions)
                    set(goButton, 'enable', 'on');
                else
                    set(goButton, 'enable', 'off');
                end
            end
        end
        
        function changedPosns(obj,~,~)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.motorControlsV5;
            ets = [gd.etPosnID1 gd.etPosnID2 gd.etPosnID3];
            pbs = [gd.pbGo1 gd.pbGo2 gd.pbGo3];
            N = numel(obj.hModel.hMotors.userDefinedPositions);
            for i = 1:3
                guiObj = ets(i);
                goButton = pbs(i);
                v = str2double(get(guiObj, 'string'));
                if v > 0 && v <= N
                    set(goButton, 'enable', 'on');
                else
                    set(goButton, 'enable', 'off');
                end
            end
            
            gd = obj.hGUIData.posnControlsV5;
            nms = cell(1,N+1);
            for i = 1:N+1
                if i == N+1
                    nms{i} = 'New Position';
                else
                    n = obj.hModel.hMotors.userDefinedPositions(i).name;
                    c = obj.hModel.hMotors.userDefinedPositions(i).coords;
                    if ~isempty(n)
                        n = [n ': '];
                    end
                    nms{i} = sprintf('%d: %s[%s]', i, n, num2str(c,'%.1f '));
                end
            end
            v = get(gd.lbPosns, 'Value');
            set(gd.lbPosns, 'Value', min(v, numel(nms)));
            set(gd.lbPosns, 'String', nms);
            obj.changedSelectedPosn();
        end
        
        function changedSelectedPosn(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            posns = obj.hModel.hMotors.userDefinedPositions;
            N = numel(posns);
            v = get(gd.lbPosns, 'Value');
            
            motorPos = obj.hModel.hMotors.motorPosition;
            nDims = numel(motorPos);
            cbs = [gd.cbX gd.cbY gd.cbZ gd.cbZZ];
            cbs(nDims+1:end) = [];
            
            if v > N
                enDims = get(cbs, 'Value');
                enDims = [enDims{:}];
                motorPos(~enDims) = nan;
                
                set(gd.etName, 'String', '');
                set(gd.etPosn, 'String', sprintf('[%s]', num2str(motorPos,'%.1f  ')));
                set(gd.pbAdd, 'String', 'Add');
            else
                set(gd.etName, 'String', posns(v).name);
                set(gd.etPosn, 'String', sprintf('[%s]', num2str(posns(v).coords,'%.1f  ')));
                arrayfun(@(a,b)set(a, 'Value', b), cbs, ~isnan(posns(v).coords));
                set(gd.pbAdd, 'String', 'Set');
            end
        end
        
        function changedDimCbs(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            motorPos = obj.hModel.hMotors.motorPosition;
            nDims = numel(motorPos);
            cbs = [gd.cbX gd.cbY gd.cbZ gd.cbZZ];
            cbs(nDims+1:end) = [];
            enDims = get(cbs, 'Value');
            enDims = logical([enDims{:}]);
            
            posn = get(gd.etPosn, 'String');
            posn = str2num(posn(2:end-1));
            
            posn(~enDims) = nan;
            posn(and(enDims, isnan(posn))) = motorPos(and(enDims, isnan(posn)));
            
            set(gd.etPosn, 'String', sprintf('[%s]', num2str(posn,'%.1f  ')));
        end
        
        function changedPosnEt(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            motorPos = obj.hModel.hMotors.motorPosition;
            nDims = numel(motorPos);
            cbs = [gd.cbX gd.cbY gd.cbZ gd.cbZZ];
            cbs(nDims+1:end) = [];
            enDims = get(cbs, 'Value');
            enDims = logical([enDims{:}]);
            motorPos(~enDims) = nan;
            
            nwStr = get(gd.etPosn, 'String');
            if nwStr(1) == '['
                nwStr = nwStr(2:end-1);
            end
            nwVal = str2num(nwStr);
            if isempty(nwVal)
                nwVal = motorPos;
            else
                nwVal(nDims+1:end) = [];
                nwVal(end+1:nDims) = motorPos(numel(nwVal)+1:end);
                arrayfun(@(a,b)set(a, 'Value', b), cbs, ~isnan(nwVal));
            end
            
            set(gd.etPosn, 'String', sprintf('[%s]', num2str(nwVal,'%.1f  ')));
        end
        
        function addPosn(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            posns = obj.hModel.hMotors.userDefinedPositions;
            N = numel(posns);
            v = get(gd.lbPosns, 'Value');
            
            nm = get(gd.etName, 'String');
            posn = get(gd.etPosn, 'String');
            posn = str2num(posn(2:end-1));
            
            if v > N
                obj.hModel.hMotors.defineUserPosition(nm,posn);
                set(gd.lbPosns, 'Value', v+1);
            else
                posns(v).name = nm;
                posns(v).coords = posn;
                obj.hModel.hMotors.userDefinedPositions = posns;
            end
        end
        
        function readPosn(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            motorPos = obj.hModel.hMotors.motorPosition;
            nDims = numel(motorPos);
            cbs = [gd.cbX gd.cbY gd.cbZ gd.cbZZ];
            cbs(nDims+1:end) = [];
            enDims = get(cbs, 'Value');
            enDims = logical([enDims{:}]);
            motorPos(~enDims) = nan;
            
            set(gd.etPosn, 'String', sprintf('[%s]', num2str(motorPos,'%.1f  ')));
        end
        
        function movePosnUp(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            posns = obj.hModel.hMotors.userDefinedPositions;
            N = numel(posns);
            v = get(gd.lbPosns, 'Value');
            
            if v <= N && v > 1
                tmp = posns(v-1);
                posns(v-1) = posns(v);
                posns(v) = tmp;
                obj.hModel.hMotors.userDefinedPositions = posns;
                set(gd.lbPosns, 'Value', v-1);
            end
        end
        
        function movePosnDown(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            posns = obj.hModel.hMotors.userDefinedPositions;
            N = numel(posns);
            v = get(gd.lbPosns, 'Value');
            
            if v < N
                tmp = posns(v);
                posns(v) = posns(v+1);
                posns(v+1) = tmp;
                obj.hModel.hMotors.userDefinedPositions = posns;
                set(gd.lbPosns, 'Value', v+1);
            end
        end
        
        function removePosn(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            posns = obj.hModel.hMotors.userDefinedPositions;
            N = numel(posns);
            v = get(gd.lbPosns, 'Value');
            
            if v <= N
                obj.hModel.hMotors.userDefinedPositions(v) = [];
                if v > 1
                    set(gd.lbPosns, 'Value', v-1);
                end
            end
        end
        
        function dblClickPosn(obj)
            if obj.hModel.hMotors.numInstances <= 0
                return;
            end
            
            gd = obj.hGUIData.posnControlsV5;
            posns = obj.hModel.hMotors.userDefinedPositions;
            N = numel(posns);
            v = get(gd.lbPosns, 'Value');
            
            if v <= N
                obj.hModel.hMotors.gotoUserDefinedPosition(v)
            end
        end
        
        function toggleLineScan(obj,src,evnt)
            lineScanEnable = get(src,'Value');
            if lineScanEnable
                obj.hModel.lineScanCacheParams();
                obj.hModel.hRoiManager.scanAngleMultiplierSlow = 0;
                obj.hModel.hRoiManager.forceSquarePixels = false;
                set(obj.hGUIData.mainControlsV4.etScanAngleMultiplierSlow,'Enable','inactive');
            else
                set(obj.hGUIData.mainControlsV4.etScanAngleMultiplierSlow,'Enable','on');
                obj.hModel.lineScanRestoreParams();
            end
        end
        
    end
    
    %% FRIEND METHODS
    %%%  APP PROPERTY CALLBACKS 
    %%%  Methods named changedXXX(src,...) respond to changes to model, which should update the controller/GUI
    %%%  Methods named changeXXX(hObject,...) respond to changes to GUI, which should update the model %}
    methods (Hidden)
        %%% IMAGING SYSTEM METHODS
        function changedImagingSystem(obj,~,~)
            nms = cellfun(@(x)x.name,obj.hModel.hScanners, 'UniformOutput', false);
            [~,id] = ismember(obj.hModel.imagingSystem, nms);
            set(obj.hGUIData.configControlsV4.pmImagingSystem, 'Value', id);
            
            persistent hImagingSystem_
            if ~isempty(hImagingSystem_) && isequal(hImagingSystem_,obj.hModel.hScan2D)
                return
            end
            
            obj.ziniChannelControls();
            obj.ziniTriggers();
            
            obj.reprocessSubMdlPropBindings('hScan2D');
            
            if isa(obj.hModel.hScan2D, 'scanimage.components.scan2d.ResScan')
                set(obj.hGUIData.configControlsV4.cbKeepScannerOn, 'Visible', 'on');
            else
                set(obj.hGUIData.configControlsV4.cbKeepScannerOn, 'Visible', 'off');
            end
            
            obj.ziniImageControls();
            obj.ziniPowerBoxControls();
            obj.cfgLinePhaseSlider();
            
            hImagingSystem_ = obj.hModel.hScan2D;
        end
        
        function changeImagingSystem(obj,hObject)
            sys = get(hObject,'String');
            obj.hModel.imagingSystem = sys{get(hObject,'Value')};
        end
        
        %%% TIMER METHODS
        function changedSecondsCounter(obj,~,~)
            %TODO: make value of 0 'sticky' for 0.3-0.4s using a timer object here
            hSecCntr = obj.hGUIData.mainControlsV4.secondsCounter;
            
            switch obj.hModel.secondsCounterMode
                case 'up' %countup timer
                    set(hSecCntr,'String',num2str(max(0,floor(obj.hModel.secondsCounter))));
                case 'down'  %countdown timer
                    set(hSecCntr,'String',num2str(max(0,ceil(obj.hModel.secondsCounter))));
                otherwise
                    set(hSecCntr,'String','0');
            end
        end
        
        %%% DISPLAY METHODS
        function changedDisplayRollingAverageFactorLock(obj,~,~)
            if obj.hModel.hDisplay.displayRollingAverageFactorLock
                set(obj.hGUIData.imageControlsV4.etRollingAverage,'Enable','off');
            else
                set(obj.hGUIData.imageControlsV4.etRollingAverage,'Enable','on');
            end
        end

        function updateFrameBatchOptions(obj,~,~)
            if obj.hModel.hRoiManager.mroiEnable
                set(obj.hGUIData.mainControlsV4.cbEnableMroiFocus,'Enable','on');
            else
                set(obj.hGUIData.mainControlsV4.cbEnableMroiFocus,'Enable','off');
            end
            
            if obj.hModel.hRoiManager.mroiEnable || obj.hModel.hDisplay.displayFrameBatchFactorLock
                set(obj.hGUIData.imageControlsV4.cbLockVolume,'Enable','off');
            else
                set(obj.hGUIData.imageControlsV4.cbLockVolume,'Enable','on');
            end
            
            if obj.hModel.hDisplay.displayVolumeLock || obj.hModel.hRoiManager.mroiEnable || obj.hModel.hDisplay.displayFrameBatchFactorLock
                set(obj.hGUIData.imageControlsV4.etFrameSelFactor,'Enable','off');
            else
                set(obj.hGUIData.imageControlsV4.etFrameSelFactor,'Enable','on');
            end
            
            if obj.hModel.hDisplay.displayFrameBatchSelectLast || obj.hModel.hDisplay.displayFrameBatchSelectAll
                set(obj.hGUIData.imageControlsV4.etFrameSelections,'Enable','off');
            else
                set(obj.hGUIData.imageControlsV4.etFrameSelections,'Enable','on');
            end
            
            if obj.hModel.hDisplay.displayVolumeLock || obj.hModel.hRoiManager.mroiEnable
                set(obj.hGUIData.imageControlsV4.cbLockFrameSel2RollAvg,'Enable','off');
            else
                set(obj.hGUIData.imageControlsV4.cbLockFrameSel2RollAvg,'Enable','on');
            end
        end
        
        function saStep(obj,fast,slow)
            mult = 0.1 / obj.hModel.hRoiManager.scanZoomFactor;
            if fast ~= 0
                obj.hModel.hRoiManager.scanAngleShiftFast = obj.hModel.hRoiManager.scanAngleShiftFast + fast*mult;
            end
            if slow ~= 0
                obj.hModel.hRoiManager.scanAngleShiftSlow = obj.hModel.hRoiManager.scanAngleShiftSlow + slow*mult;
            end
        end
        
        function zeroScanAngle(obj)
            obj.hModel.hRoiManager.scanAngleShiftFast = 0;
            obj.hModel.hRoiManager.scanAngleShiftSlow = 0;
        end
        
        %%% BEAM METHODS
        function changeBeamParams(obj,src,~,~)
            %Change occurred to beam-indexed params in view/controller
            switch get(src,'Style')
                case 'edit'
                    newVal = str2double(get(src,'String'));
                case {'slider' 'checkbox'}
                    newVal = get(src,'Value');
                otherwise
                    assert(false,'Unsupported control style.');
            end
            propName = get(src,'UserData');
            propNameBeams = strrep(propName,'hBeams___','');
            
            try
                obj.hModel.hBeams.(propNameBeams)(obj.beamDisplayIdx) = newVal;
            catch ME
                % Error setting beam-indexed model prop; revert GUI
                obj.changedBeamParams(propName);
                
                % TODO what is the right thing here
                switch ME.identifier
                    % currently don't throw any warnings/errs
                end
            end
        end
        
        function changeBeamPowersDisplay(obj,src,~,~)
            if obj.hModel.hBeams.numInstances <= 0
                return;
            end
            
            switch get(src,'Style')
                case 'edit'
                    newVal = str2double(get(src,'String'));
                case 'slider'
                    newVal = get(src,'Value');
                otherwise
                    assert(false,'Unsupported control style.');
            end
            
            try
                obj.hModel.hBeams.powers(obj.beamDisplayIdx) = newVal;
            catch ME
                switch ME.identifier
                    % currently don't throw any warnings/errs
                end
            end
        end
        
        function changedBeamPowersDisplay(obj,src,evnt)
            %Do nothing.
        end
        
        function changedBeamParams(obj,src,evnt)
            % Change occurred to beam-indexed property in model; refresh
            % controls tied to that prop.
            % src: either a meta.prop object (when changedBeamParams used as
            % prop listener), or a propName string
            if obj.hModel.hBeams.numInstances <= 0
                return;
            end
            
            if ischar(src)
                propName = src;
            elseif isa(src,'meta.property')
                propName = sprintf('hBeams___%s',src.Name);
            else
                assert(false,'Invalid src input arg.');
            end
            
            propNameBeams = strrep(propName,'hBeams___','');
            newVal = obj.hModel.hBeams.(propNameBeams)(obj.beamDisplayIdx);
            
            hControls = obj.beamProp2Control.(propName);
            for c = 1:numel(hControls)
                switch get(hControls(c),'Style')
                    case 'edit'
                        set(hControls(c),'String',num2str(newVal));
                    case {'slider' 'checkbox'}
                        set(hControls(c),'Value',newVal);
                    otherwise
                        assert(false,'Unsupported control style.');
                end
            end
            
            v = 0.9412 * double(~any(obj.hModel.hBeams.directMode)) * ones(1,3) + [double(any(obj.hModel.hBeams.directMode)) 0 0];
            set(obj.hGUIData.powerControlsV4.cbDirectMode, 'BackgroundColor', v)
        end
        
       function changedBeamPowerUnits(obj,src,evnt) %#ok<*INUSD>
            switch obj.hModel.hBeams.powerUnits
                case 'percent'
                    set(obj.hGUIData.powerControlsV4.rbPercentBeamPower,'Value',1);
                    set(obj.hGUIData.powerControlsV4.rbMilliwattBeamPower,'Value',0);
                case 'milliwatts'
                    set(obj.hGUIData.powerControlsV4.rbPercentBeamPower,'Value',0);
                    set(obj.hGUIData.powerControlsV4.rbMilliwattBeamPower,'Value',1);
                otherwise
                    assert(false,'Unsupported value of beamPowerUnits.');
            end
        end
        
        function changedBeamPzAdjust(obj,src,evnt)
            if obj.hModel.hBeams.numInstances <= 0
                return;
            end
            
            currBeamActive = obj.hModel.hBeams.pzAdjust(obj.beamDisplayIdx);
            
            set(obj.hGUIData.powerControlsV4.cbPzAdjust,'Value',currBeamActive);
            
            if currBeamActive
                set(obj.hGUIData.powerControlsV4.etZLengthConstant,'Enable','on');
            else
                set(obj.hGUIData.powerControlsV4.etZLengthConstant,'Enable','off');
            end
        end
        
        function changedPowerBoxes(obj,~,~)
            obj.ziniPowerBoxControls();
            
            i = obj.pbIdx;
            if i <= numel(obj.hModel.hBeams.powerBoxes)
                pb = obj.hModel.hBeams.powerBoxes(i);
                set(obj.hGUIData.powerBoxControlsV4.etPowers,'String',num2str(pb.powers));
                if obj.hGUIData.powerBoxControlsV4.rbFraction == get(obj.hGUIData.powerBoxControlsV4.unitPanel, 'SelectedObject')
                    %units are fraction
                    r = pb.rect;
                    s = num2str(r,'%.3f ');
                else
                    %units are pixels
                    sz = [obj.hModel.hRoiManager.pixelsPerLine obj.hModel.hRoiManager.linesPerFrame];
                    r = floor(pb.rect .* [sz sz]);
                    s = num2str(r,'%d   ');
                    disp(sz)
                    disp(r)
                    disp(s)
                end
                set(obj.hGUIData.powerBoxControlsV4.etPosition,'String',s);
                obj.powerBoxUpdateBoxFigure();
            end
            
            obj.updateOtherPbs();
        end
        
        function updateOtherPbs(obj)
            i = obj.pbIdx;
            n = numel(obj.hModel.hBeams.powerBoxes);
            nOth = n - (i <= n);
            while numel(obj.hPowbOthers) < (nOth)
                obj.hPowbOthers(end+1) = surface([.25 .75],[.25 .75],0.5*ones(2),'Parent',obj.hPowbAx,'Hittest','off','FaceColor',[.5 .5 .5],...
                    'EdgeColor',[.5 .5 .5],'LineWidth',1.5,'FaceLighting','none','FaceAlpha',0.2,'visible','off');
                obj.hOthTexts(end+1) = text(.25,.25,.5,'Power Box','Parent',obj.hPowbAx,'visible','off','color','y','Hittest','on');
            end
            delete(obj.hPowbOthers(nOth+1:end));
            delete(obj.hOthTexts(nOth+1:end));
            obj.hPowbOthers(nOth+1:end) = [];
            obj.hOthTexts(nOth+1:end) = [];
            
            nms = {};
            for pb = obj.hModel.hBeams.powerBoxes
                nms{end+1} = pb.name;
                if isempty(nms{end})
                    nms{end} = sprintf('Power Box %d', numel(nms));
                end
            end
            
            oths = setdiff(1:n,i);
            for i = 1:nOth
                r = obj.hModel.hBeams.powerBoxes(oths(i)).rect;
                set(obj.hPowbOthers(i), 'XData', [r(1) r(1)+r(3)]);
                set(obj.hPowbOthers(i), 'YData', [r(2) r(2)+r(4)]);
                set(obj.hPowbOthers(i), 'visible','on');
                set(obj.hOthTexts(i), 'Position', [r(1)+.01 r(2)+.03 .75],'visible','on');
                set(obj.hOthTexts(i), 'String', nms{oths(i)});
                set(obj.hOthTexts(i), 'ButtonDownFcn', @(varargin)selPb(oths(i)));
            end
            
            function selPb(n)
                obj.pbIdx = n;
            end
        end
        
        function changePowerBoxRect(obj,~,~)
            i = obj.pbIdx;
            if i <= numel(obj.hModel.hBeams.powerBoxes)
                pb = obj.hModel.hBeams.powerBoxes(i);
                u = str2num(get(obj.hGUIData.powerBoxControlsV4.etPosition,'String'));
                if obj.hGUIData.powerBoxControlsV4.rbFraction == get(obj.hGUIData.powerBoxControlsV4.unitPanel, 'SelectedObject')
                    %units are fraction
                    pb.rect = u;
                else
                    %units are pixels
                    sz = [obj.hModel.hRoiManager.pixelsPerLine obj.hModel.hRoiManager.linesPerFrame];
                    pb.rect = u ./ [sz sz];
                end
                obj.hModel.hBeams.powerBoxes(i) = pb;
            end
        end
        
        function changePowerBoxPowers(obj,~,~)
            i = obj.pbIdx;
            if i <= numel(obj.hModel.hBeams.powerBoxes)
                v = str2num(get(obj.hGUIData.powerBoxControlsV4.etPowers,'String'));
                obj.hModel.hBeams.powerBoxes(i).powers = v;
            end
        end
        
        function powerBoxUpdateBoxFigure(obj)
            i = obj.pbIdx;
            if i <= numel(obj.hModel.hBeams.powerBoxes)
                pb = obj.hModel.hBeams.powerBoxes(i);
                x1 = pb.rect(1);
                x2 = pb.rect(1)+pb.rect(3);
                y1 = pb.rect(2);
                y2 = pb.rect(2)+pb.rect(4);
                
                set(obj.hPowbBoxSurf,'XData',[x1 x2],'YData',[y1 y2]);
                set(obj.hPowbBoxCtr,'XData',(x1+x2)*.5,'YData',(y1+y2)*.5);
                set([obj.hPowbBoxTL obj.hPowbBoxBL],'XData',x1);
                set([obj.hPowbBoxTR obj.hPowbBoxBR],'XData',x2);
                set([obj.hPowbBoxTL obj.hPowbBoxTR],'YData',y1);
                set([obj.hPowbBoxBL obj.hPowbBoxBR],'YData',y2);
                
                set([obj.hPowbBoxT obj.hPowbBoxB],'XData',[x1 x2]);
                set(obj.hPowbBoxL,'XData',[x1 x1]);
                set(obj.hPowbBoxR,'XData',[x2 x2]);
                set([obj.hPowbBoxL obj.hPowbBoxR],'YData',[y1 y2]);
                set(obj.hPowbBoxT,'YData',[y1 y1]);
                set(obj.hPowbBoxB,'YData',[y2 y2]);
                
                if isempty(pb.name)
                    nm = sprintf('Power Box %d', i);
                else
                    nm = pb.name;
                end
                set(obj.hText, 'Position', [x1+.01 y1+.03 2]);
                set(obj.hText, 'String', nm);
            end
        end
        
        function deletePowerBox(obj)
            i = obj.pbIdx;
            if i <= numel(obj.hModel.hBeams.powerBoxes)
                if i > 1
                    set(obj.hGUIData.powerBoxControlsV4.pmPbSel,'Value',i-1);
                end
                obj.hModel.hBeams.powerBoxes(i) = [];
                obj.ziniPowerBoxControls();
            end
        end
        
        function selectPowerBox(obj)
            if obj.pbIdx > numel(obj.hModel.hBeams.powerBoxes)
                obj.hModel.hBeams.powerBoxes(obj.pbIdx) = struct('rect', [.25 .25 .5 .5], 'powers', NaN, 'name', '','oddLines',true,'evenLines',true);
                obj.ziniPowerBoxControls();
            else
                obj.changedPowerBoxes();
            end
        end
        
        function powerBoxGuiCopyChannel(obj,idx)
            try
                imdata = single(obj.hModel.hDisplay.rollingStripeDataBuffer{1}{1}.roiData{1}.imageData{idx}{1})' ./ obj.hModel.hDisplay.displayRollingAverageFactor; %Transpose image data in SI2015.
                lut = single(obj.hModel.hChannels.channelLUT{idx});
                maxVal = single(255);
                scaledData = uint8((imdata - lut(1)) .* (maxVal / (lut(2)-lut(1))));
                set(obj.hPowbCtxIm, 'cdata', repmat(scaledData,1,1,3));
            catch
                most.idioms.warn('No image data found.');
                set(obj.hPowbCtxIm, 'cdata', zeros(2,2,3,'uint8'));
            end
        end
        
        function p = getPbPt(obj)
            p = get(obj.hPowbAx,'CurrentPoint');
            p = p([1 3]);
        end
        
        function powbScrollWheelFcn(obj, ~, evt)
            mv = double(evt.VerticalScrollCount) * 1;%evt.VerticalScrollAmount;
            
            % find old range and center
            xlim = get(obj.hPowbAx,'xlim');
            ylim = get(obj.hPowbAx,'ylim');
            rg = xlim(2) - xlim(1);
            ctr = 0.5*[sum(xlim) sum(ylim)];
            
            % calc and constrain new half range
            nrg = min(1,rg*.75^-mv);
            nrg = max(0.0078125,nrg);
            nhrg = nrg/2;
            
            %calc new center based on where mouse is
            pt = obj.getPbPt;
            odfc = pt - ctr; %original distance from center
            ndfc = odfc * (nrg/rg); %new distance from center
            nctr = pt - [ndfc(1) ndfc(2)];
            
            %constrain center
            nctr = max(min(nctr,1-nhrg),nhrg);
            
            % new lims
            xlim = [-nhrg nhrg] + nctr(1);
            ylim = [-nhrg nhrg] + nctr(2);
            set(obj.hPowbAx,'xlim',xlim,'ylim',ylim);
        end
        
        function powbPanFcn(obj,starting,stopping)
            persistent prevpt;
            persistent ohrg;
            
            if starting
                if strcmp(get(obj.hGUIs.powerBoxControlsV4,'SelectionType'), 'normal')
                    % left click
                    prevpt = obj.getPbPt;
                    
                    xlim = get(obj.hPowbAx,'xlim');
                    ohrg = (xlim(2) - xlim(1))/2;
                    
                    set(obj.hGUIs.powerBoxControlsV4,'WindowButtonMotionFcn',@(varargin)obj.powbPanFcn(false,false),'WindowButtonUpFcn',@(varargin)obj.powbPanFcn(false,true));
                    waitfor(obj.hGUIs.powerBoxControlsV4,'WindowButtonMotionFcn',[]);
                end
            else
                % find prev center
                xlim = get(obj.hPowbAx,'xlim');
                ylim = get(obj.hPowbAx,'ylim');
                octr = 0.5*[sum(xlim) sum(ylim)];
                
                % calc/constrain new center
                nwpt = obj.getPbPt;
                nctr = octr - (nwpt - prevpt);
                nctr = max(min(nctr,1-ohrg),ohrg);
                
                nxlim = nctr(1) + [-ohrg ohrg];
                nylim = nctr(2) + [-ohrg ohrg];
                
                set(obj.hPowbAx,'xlim',nxlim);
                set(obj.hPowbAx,'ylim',nylim);

                prevpt = obj.getPbPt;
                
                if stopping
                    set(obj.hGUIs.powerBoxControlsV4,'WindowButtonMotionFcn',[],'WindowButtonUpFcn',[]);
                end
            end
        end
        
        function powbCpFunc(obj,chgng,starting,stopping)
            persistent prevpt;
            
            if starting
                if strcmp(get(obj.hGUIs.powerBoxControlsV4,'SelectionType'), 'normal')
                    % left click
                    prevpt = obj.getPbPt;
                    
                    set(obj.hGUIs.powerBoxControlsV4,'WindowButtonMotionFcn',@(varargin)obj.powbCpFunc(chgng,false,false),'WindowButtonUpFcn',@(varargin)obj.powbCpFunc(chgng,false,true));
                    waitfor(obj.hGUIs.powerBoxControlsV4,'WindowButtonMotionFcn',[]);
                end
            else
                nwpt = obj.getPbPt;
                mv = nwpt - prevpt;
                i = obj.pbIdx;
                if i <= numel(obj.hModel.hBeams.powerBoxes)
                    pb = obj.hModel.hBeams.powerBoxes(i);
                    r = pb.rect;
                    osz = r([3 4]);
                    r([3 4]) = osz + r([1 2]);
                    
                    if chgng(1)
                        r(1) = r(1) + mv(1);
                    end
                    
                    if chgng(2)
                        r(2) = r(2) + mv(2);
                    end
                    
                    if chgng(3)
                        r(3) = r(3) + mv(1);
                    end
                    
                    if chgng(4)
                        r(4) = r(4) + mv(2);
                    end
                    
                    if all(chgng)
                        r([3 4]) = osz;
                        lims = 1 - osz;
                        r([1 2]) = min(lims,max(0,r([1 2])));
                    else
                        r([3 4]) = r([3 4]) - r([1 2]);
                    end
                    
                    r(3) = max(0,r(3)); % prevent negative width
                    r(4) = max(0,r(4)); % prevent negative height
                    
                    pb.rect = r;
                    obj.hModel.hBeams.powerBoxes(i) = pb;
                end
                
                if stopping
                    set(obj.hGUIs.powerBoxControlsV4,'WindowButtonMotionFcn',[],'WindowButtonUpFcn',[]);
                else
                    prevpt = nwpt;
                end
            end
        end
        
        %%% Trigger Methods
        function changedTrigNextStopEnable(obj,src,evnt)
            if obj.hModel.hScan2D.trigNextStopEnable
                buttonEnable = 'on';
            else
                buttonEnable = 'off';
            end
            set(obj.hGUIData.triggerControlsV5.pbAcqStop,'Enable',buttonEnable);
            set(obj.hGUIData.triggerControlsV5.pbNextFileMarker,'Enable',buttonEnable);
        end
        
        function changedTrigAcqInTerm(obj,src,evnt)
            if isempty(obj.hModel.hScan2D.trigAcqInTerm)
                triggerButtonEnable = 'off';
            else
                triggerButtonEnable = 'on';
            end
            set(obj.hGUIData.mainControlsV4.tbExternalTrig,'Enable',triggerButtonEnable);
            set(obj.hGUIData.triggerControlsV5.pbAcqStart,'Enable',triggerButtonEnable);
        end
        
        %%% CHANNEL METHODS        
        function changedChannelsMergeEnable(obj,src,evt)
            val = obj.hModel.hDisplay.channelsMergeEnable;
            if val
                set(obj.hGUIData.channelControlsV4.cbChannelsMergeFocusOnly,'Enable','on');
                set(obj.hModel.hDisplay.hMergeFigs,'visible','on');
            else
                set(obj.hGUIData.channelControlsV4.cbChannelsMergeFocusOnly,'Enable','off');
            end
        end
        
        
        %%% CHANNEL METHODS
        function changedChanLUT(obj,src,evnt)
            chanNum = str2double(regexpi(src.Name,'[0-9]*','Match','Once'));
            
            chanProp = sprintf('chan%dLUT',chanNum);            
            blackVal = obj.hModel.hDisplay.(chanProp)(1);
            whiteVal = obj.hModel.hDisplay.(chanProp)(2);
            
            blackValSlider = coerceToSliderRange(blackVal,chanNum);
            whiteValSlider = coerceToSliderRange(whiteVal,chanNum);
            
            set(obj.hGUIData.imageControlsV4.(sprintf('blackSlideChan%d',chanNum)),'Value',blackValSlider);
            set(obj.hGUIData.imageControlsV4.(sprintf('whiteSlideChan%d',chanNum)),'Value',whiteValSlider);
            
            set(obj.hGUIData.imageControlsV4.(sprintf('blackEditChan%d',chanNum)),'String',num2str(blackVal));
            set(obj.hGUIData.imageControlsV4.(sprintf('whiteEditChan%d',chanNum)),'String',num2str(whiteVal));
            
            function val = coerceToSliderRange(val,i)
                minSliderVal = get(obj.hGUIData.imageControlsV4.(sprintf('blackSlideChan%d',i)),'Min');
                maxSliderVal = get(obj.hGUIData.imageControlsV4.(sprintf('blackSlideChan%d',i)),'Max');
                val = max(val,fix(minSliderVal));
                val = min(val,fix(maxSliderVal));
            end
        end
        
        function changeChannelsLUT(obj,src,blackOrWhite,chanIdx)
            %blackOrWhite: 0 if black, 1 if white
            %chanIdx: Index of channel whose LUT value to change
            switch get(src,'Style')
                case 'edit'
                    newVal = str2double(get(src,'String'));
                case 'slider'
                    newVal = get(src,'Value');
                    %Only support integer values, from slider controls
                    newVal = round(newVal);         
            end
            
            %Erroneous entry
            if isempty(newVal)
                %refresh View
                obj.changedChanLUT(); 
            else
                chanProp = sprintf('chan%dLUT',chanIdx);
                try
                    obj.hModel.hDisplay.(chanProp)(2^blackOrWhite) = newVal;
                catch ME
                    obj.changedChanLUT();
                    obj.updateModelErrorFcn(ME);
                end
            end
        end
        
        function changedExtTrigEnable(obj,src,evnt)            
            h=findobj(obj.hGUIs.mainControlsV4,'tag','tbExternalTrig');
            if obj.hModel.extTrigEnable
                set(h,'BackgroundColor','green');
            else
                set(h,'BackgroundColor',[0.9412 0.9412 0.9412]);
            end
        end
        
        function changedAcqState(obj,src,evnt)
            hFocus = obj.hGUIData.mainControlsV4.focusButton;
            hGrab = obj.hGUIData.mainControlsV4.grabOneButton;
            hLoop = obj.hGUIData.mainControlsV4.startLoopButton;
            fAbort = obj.hGUIData.mainControlsV4.fAbort;
            gAbort = obj.hGUIData.mainControlsV4.gAbort;
            lAbort = obj.hGUIData.mainControlsV4.lAbort;
            hPoint = obj.hGUIData.mainControlsV4.tbPoint;
            
            switch obj.hModel.acqState
                case 'idle'
                    set([hFocus hGrab hLoop],'Enable','on');
                    set([hFocus hGrab hLoop],'Visible','on');
                    set([fAbort gAbort lAbort],'Visible','off');
                    set([fAbort gAbort lAbort],'Enable','on');
                    
                    set(hPoint,'String','POINT','ForegroundColor',[0 .6 0],'Enable','on');
                    set(hPoint,'Value',false);
                    
                case 'focus'
                    set([hFocus hGrab hLoop],'Visible','off');
                    set([fAbort gAbort lAbort],'Visible','off');
                    set([fAbort gAbort lAbort],'Enable','on');
                    set(hPoint,'Enable','off');
                    set(fAbort,'Visible','on');
                    
                case 'grab'
                    set([hFocus hGrab hLoop],'Visible','off');
                    set([fAbort gAbort lAbort],'Visible','off');
                    set([fAbort gAbort lAbort],'Enable','on');
                    set(hPoint,'Enable','off');
                    set(gAbort,'Visible','on');
                    
                case {'loop' 'loop_wait'}
                    set([hFocus hGrab hLoop],'Visible','off');
                    set([fAbort gAbort lAbort],'Visible','off');
                    set([fAbort gAbort lAbort],'Enable','on');
                    set(hPoint,'Enable','off');
                    set(lAbort,'Visible','on');
                    
                case 'point'
                    set(hPoint,'String','PARK','ForegroundColor','r');
                    set(hPoint,'Value',true);
                    set([hFocus hGrab hLoop],'enable','off');
                    
                    %TODO: Maybe add 'error' state??
            end
            
            drawnow
        end
        
        function changedScanAngleMultiplierSlow(obj,~,~)
            s = obj.hGUIData.configControlsV4;
            hForceSquareCtls = [s.cbForceSquarePixel s.cbForceSquarePixelation];
            
            if obj.hModel.hRoiManager.scanAngleMultiplierSlow == 0
                set(obj.hGUIData.mainControlsV4.tbToggleLinescan,'Value',1);
                set(hForceSquareCtls,'Enable','off');
            else
                set(obj.hGUIData.mainControlsV4.tbToggleLinescan,'Value',0);
                set(hForceSquareCtls,'Enable','on');
            end
        end
        
        function changedisprope(obj,~,~)
            obj.cfgLinePhaseSlider();
        end
        
        function changeScanPhaseSlider(obj,src)
            val = get(src,'Value');
            obj.hModel.hScan2D.linePhase = val;
        end
        
        function changedScanPhase(obj,~,~)
            val = obj.hModel.hScan2D.linePhase;
            
            minSliderVal = get(obj.hGUIData.configControlsV4.scanPhaseSlider,'Min');
            maxSliderVal = get(obj.hGUIData.configControlsV4.scanPhaseSlider,'Max');
            
            % enforce limits
            if val < minSliderVal
                val = minSliderVal;
            elseif val > maxSliderVal
                val = maxSliderVal;
            end
            
            set(obj.hGUIData.configControlsV4.scanPhaseSlider,'Value',val);
        end
        
        function changedScanFramePeriod(obj,~,~)
            if isnan(obj.hModel.hRoiManager.scanFramePeriod)
                set(obj.hGUIData.fastZControlsV4.etFramePeriod,'BackgroundColor',[0.9 0 0]);
                set(obj.hGUIData.configControlsV4.etFrameRate,'BackgroundColor',[0.9 0 0]);
                set(obj.hGUIData.fastZControlsV4.etVolumeRate,'BackgroundColor',[0.9 0 0]);
            else
                set(obj.hGUIData.fastZControlsV4.etFramePeriod,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
                set(obj.hGUIData.configControlsV4.etFrameRate,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
                set(obj.hGUIData.fastZControlsV4.etVolumeRate,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
            end
        end
        
        function changedChannelDisplay(obj,~,~)
            for chan = 1:obj.hModel.hChannels.channelsAvailable
                hFig = obj.hModel.hDisplay.hFigs(chan);
                wasActive = strcmp(get(hFig,'UserData'),'active');
                wasVisible = strcmp(get(hFig,'visible'),'on');
                activate = ismember(chan,obj.hModel.hChannels.channelDisplay);
                
                if activate
                    if ~wasVisible && obj.initComplete
                        set(hFig,'visible','on'); % only set property when it is changed to reduce flickering of the figure window
                    end
                    set(hFig,'UserData','active');
                else
                    set(hFig,'UserData','');
                end
            end
        end
        
        function changedForceSquarePixelation(obj,~,~)
            if obj.hModel.hRoiManager.forceSquarePixelation
                set(obj.hGUIData.configControlsV4.etLinesPerFrame,'Enable','off');
            else
                set(obj.hGUIData.configControlsV4.etLinesPerFrame,'Enable','on');
            end
        end
        
        function changeScanZoomFactor(obj,hObject,absIncrement,lastVal)
            newVal = get(hObject,'Value');
            currentZoom = obj.hModel.hRoiManager.scanZoomFactor;
            
            if newVal > lastVal
                if currentZoom + absIncrement > 99.9
                    newZoom = 99.9;
                else
                    newZoom = currentZoom + absIncrement;
                end
            elseif newVal < lastVal
                if currentZoom - absIncrement < 1
                    newZoom = 1;
                else
                    newZoom = currentZoom - absIncrement;
                end
            else
                newZoom = currentZoom;
            end
            
            obj.hModel.hRoiManager.scanZoomFactor = newZoom;
        end
        
        function changedStatusString(obj,~,~)
            %For now, just display the string
            ss = obj.hModel.statusString;
            obj.mainControlsStatusString = ss;
        end
        
        function changedLogEnable(obj,~,~)
            hAutoSaveCBs = [obj.hGUIData.mainControlsV4.cbAutoSave obj.hGUIData.configControlsV4.cbAutoSave];
            hLoggingControls = [obj.hGUIData.mainControlsV4.baseName obj.hGUIData.mainControlsV4.baseNameLabel ...
                obj.hGUIData.mainControlsV4.fileCounter obj.hGUIData.mainControlsV4.fileCounterLabel];
            
            if obj.hModel.hChannels.loggingEnable
                set(hAutoSaveCBs,'BackgroundColor',[0 .8 0]);
                set(hLoggingControls,'Enable','on');
            else
                set(hAutoSaveCBs,'BackgroundColor',[1 0 0]);
                set(hLoggingControls,'Enable','off');
            end
        end
        
        function setSavePath(obj,~,~)
            folder_name = uigetdir(obj.hModel.hScan2D.logFilePath);
            
            if folder_name ~= 0
                obj.hModel.hScan2D.logFilePath = folder_name;
            end
        end
        
        %%% CFG CONFIG 
        function changedCfgFilename(obj,~,~)
            cfgFilename = obj.hModel.hConfigurationSaver.cfgFilename;
            [~,fname] = fileparts(cfgFilename);
            set([obj.hGUIData.mainControlsV4.configName obj.hGUIData.configControlsV4.configurationName],'String',fname);
        end
        
        %%% FASTCFG 
        function changedFastCfgCfgFilenames(obj,~,~)
            fastCfgFNames = obj.hModel.hConfigurationSaver.fastCfgCfgFilenames;
            tfEmpty = cellfun(@isempty,fastCfgFNames);
            set(obj.hMainPbFastCfg(tfEmpty),'Enable','off');
            set(obj.hMainPbFastCfg(~tfEmpty),'Enable','on');
            
            obj.changedFastCfgAutoStartTf();
        end
        
        function changedFastCfgAutoStartTf(obj,~,~)
            autoStartTf = obj.hModel.hConfigurationSaver.fastCfgAutoStartTf;
            
            defaultBackgroundColor = get(0,'defaultUicontrolBackgroundColor');
            set(obj.hMainPbFastCfg(autoStartTf),'BackGroundColor',[0 1 0]);
            set(obj.hMainPbFastCfg(~autoStartTf),'BackGroundColor',defaultBackgroundColor);
        end
        
        %%% USR CONFIG
        function changedUsrFilename(obj,~,~)
            usrFilename = obj.hModel.hConfigurationSaver.usrFilename;
            [~,fname] = fileparts(usrFilename);
            set(obj.hGUIData.mainControlsV4.userSettingsName,'String',fname);
        end
        
        function changedCfgLoading(obj,~,~)
        end
        
        function changedUsrPropList(obj,~,~)
            % This is done because the user is given the ability to modify
            % the values of properties in the "User Settings" GUI.
            usrPropSubsetCurrent = obj.hModel.hConfigurationSaver.usrPropList;
            usrPropSubsetCurrent_ = obj.hModel.hConfigurationSaver.usrPropList;
            NUsrPropSubsetCurrent = numel(usrPropSubsetCurrent);
            
            % remove previous listeners for userSettingsV4
            delete(obj.usrSettingsPropListeners);
            
            % add new listeners
            listenerObjs = event.proplistener.empty(0,1);
            for c = 1:NUsrPropSubsetCurrent
                pname = usrPropSubsetCurrent{c};
                %The problem here is that the function
                %changedCurrentUsrProp carries only the property name in
                %the object that holds the signal. What I really need to do
                %is encode the full property somehow.
                listenerObjs(c) = obj.hModel.mdlSetPropListenerFcn(pname,'PostSet',@(src,evt,fullname)obj.changedCurrentUsrPropCallback(src,evt,pname));
                usrPropSubsetCurrent_{c} = regexprep(pname,'\.','_');
            end
            obj.usrSettingsPropListeners = listenerObjs;
            
            % BEGIN CODE TO SET USER SETTINGS STRUCT AND PASS TO GUI.
            % Update currentUsrProps table to use new property subset
            obj.hGUIData.userSettingsV4.pcCurrentUSRProps.reset();
            formatStruct = struct('format','char','info',[]); % xxx explain char
            formatCell = num2cell(repmat(formatStruct,NUsrPropSubsetCurrent,1));
            
            % The following is used to create the struct that is passed
            % onto the User Settings GUI. This struct is used by
            % most.gui.control.PropertyTable to fill in the "Current USR
            % Properties" table. The current issue is that the names of the
            % properties are used as keys, and therefore they cause
            % cell2struct to break because the properties have '.'s in
            % their name.
            metadata = cell2struct(formatCell,usrPropSubsetCurrent_,1);
            obj.hGUIData.userSettingsV4.pcCurrentUSRProps.addProps(metadata);
            
            % Manually fire listeners for each prop in usrPropSubsetCurrent
            % so that the currentUsrProps table updates
            for c = 1:NUsrPropSubsetCurrent
                pname = usrPropSubsetCurrent{c};
                obj.changedCurrentUsrProp(pname);
            end
            
            % Update specifyCurrentUsrProps table
            data = get(obj.hGUIData.userSettingsV4.tblSpecifyUsrProps,'Data');
            availableUsrProps = data(:,1);
            tfInCurrentUsrSubset = ismember(availableUsrProps,usrPropSubsetCurrent);
            data(:,2) = num2cell(tfInCurrentUsrSubset);
            set(obj.hGUIData.userSettingsV4.tblSpecifyUsrProps,'Data',data);
        end
        
        function changedCurrentUsrPropCallback(obj,~,~,fullname)
            % propName = src.Name;
            % propObj = evt.AffectedObject;
            % src and evt are unused - they are only there so I can pass in
            % the constant property name 'fullname' in the callback
            val = lclRecursePropGet(obj.hModel,fullname);
            obj.hGUIData.userSettingsV4.pcCurrentUSRProps.encodeFcn(regexprep(fullname,'\.','_'),val);
            
            function val = lclRecursePropGet(obj, propName)
                %Detect if pname is a submodel reference by looking for a '.'.
                [baseName, propName] = strtok(propName,'.');
                if ~isempty(propName)
                    propName = propName(2:end);
                    val = lclRecursePropGet(obj.(baseName),propName);
                else
                    val = obj.(baseName);
                end
            end
        end
        
        function changedCurrentUsrProp(obj,varargin)
            switch nargin
                case 2
                    propName = varargin{1};
                    propObj  = [];
                case 3
                    src = varargin{1};
                    propName = src.Name;
                    propObj  = varargin{2}.AffectedObject;
                otherwise
                    assert(false,'Invalid number of args.');
            end
            propName = regexprep(propName,'_','\.');
            
            if isempty(propObj)
                val = lclRecursePropGet(obj.hModel, propName);
            else
                val = propObj.(propName);
            end
            
            obj.hGUIData.userSettingsV4.pcCurrentUSRProps.encodeFcn(regexprep(propName,'\.','_'),val);
            
            function val = lclRecursePropGet(obj, propName)
                %Detect if pname is a submodel reference by looking for a '.'.
                [baseName, propName] = strtok(propName,'.');
                if ~isempty(propName)
                    propName = propName(2:end);
                    val = lclRecursePropGet(obj.(baseName),propName);
                else
                    val = obj.(baseName);
                end
            end
        end
        
        % This looks similar to Controller.updateModel for PropControls.
        % However updateModel() does not quite work as when there is a
        % failure, it reverts using Controller.updateViewHidden. This will
        % not work as the currentUsrProps are not currently participating
        % in the prop2Control struct business.
        function changeCurrentUsrProp(obj,hObject,eventdata,handles)
            [status,propName,propVal] = ...
                obj.hGUIData.userSettingsV4.pcCurrentUSRProps.decodeFcn(hObject,eventdata,handles);
            propName = regexprep(propName,'_','\.');
            
            switch status
                case 'set'
                    try
                        % obj.hModel.(propName) = propVal;
                        lclRecursePropSet(obj.hModel, propName, propVal);
                    catch ME
                        obj.changedCurrentUsrProp(propName);
                        switch ME.identifier
                            case 'most:InvalidPropVal'
                                % no-op
                            case 'PDEPProp:SetError'
                                throwAsCaller(obj.DException('','ModelUpdateError',ME.message));
                            otherwise
                                ME.rethrow();
                        end
                    end
                case 'revert'
                    obj.changedCurrentUsrProp(propName);
                otherwise
                    assert(false);
            end
            
            function lclRecursePropSet(obj, propName, val)
                %Detect if pname is a submodel reference by looking for a '.'.
                [baseName, propName] = strtok(propName,'.');
                if ~isempty(propName)
                    propName = propName(2:end);
                    lclRecursePropSet(obj.(baseName),propName,val);
                else
                    obj.(baseName) = val;
                end
            end
        end
        
        function specifyCurrentUsrProp(obj,hObject,eventdata,handles)
            data = get(hObject,'data');
            availableUsrProps = data(:,1);
            tf = cell2mat(data(:,2));
            obj.hModel.hConfigurationSaver.usrPropList = availableUsrProps(tf);
        end
        
        %%% MOTOR 
        function changeMotorPosition(obj,src,coordinateIdx)
            if ~obj.hModel.hMotors.hMotor.hLSC.nonblockingMoveInProgress
                newVal = str2double(get(src,'String'));
                try
                    % NOTE: Indexing operation forces read of motorPosition prior to setting
                    % obj.hModel.hMotors.motorPosition(coordinateIdx) = newVal;
                    pos = obj.hModel.hMotors.motorPosition;
                    
                    if numel(pos) >= coordinateIdx
                        pos(coordinateIdx) = newVal;
                        obj.hModel.hMotors.motorPosition = pos;
                    elseif coordinateIdx == 4 && obj.hModel.hFastZ.numInstances
                        obj.hModel.hFastZ.positionTarget = newVal;
                        obj.changedMotorPosition();
                    end
                catch 
                    obj.changedMotorPosition(); % refreshes motor-Position-related GUI components
                end
            else
                most.idioms.warn('Motor is currently executing a move operation. New command ignored.')
            end
        end
        
        function changedMotorPosition(obj,~,~)
            formatStr = '%.2f';
            
            motorPos = obj.hModel.hMotors.motorPosition;
            if ~isempty(motorPos)
                set(obj.hGUIData.motorControlsV5.etPosX,'String',num2str(motorPos(1),formatStr));
                set(obj.hGUIData.motorControlsV5.etPosY,'String',num2str(motorPos(2),formatStr));
                set(obj.hGUIData.motorControlsV5.etPosZ,'String',num2str(motorPos(3),formatStr));
                set(obj.hGUIData.motorControlsV5.etPosR,'String',num2str(norm(motorPos(1:3)),formatStr));
                if numel(motorPos)==4
                    if ~isempty(obj.hModel.hMotors.hMotorZ)
                        zpos = obj.hModel.hMotors.hMotorZ.positionRelative;
                    else
                        zpos = motorPos;
                    end
                    set(obj.hGUIData.motorControlsV5.etPosZZTarget,'String',num2str(motorPos(4),formatStr));
                    set(obj.hGUIData.motorControlsV5.etPosZZ,'String',num2str(zpos(3),formatStr));
                elseif obj.hModel.hFastZ.numInstances
                    set(obj.hGUIData.motorControlsV5.etPosZZTarget,'String',num2str(obj.hModel.hFastZ.positionTarget,formatStr));
                    set(obj.hGUIData.motorControlsV5.etPosZZ,'String',num2str(obj.hModel.hFastZ.positionAbsolute,formatStr));
                end
            end
        end
        
        function changedStackStartEndPositionPower(obj,~,~)
            startPos = obj.hModel.hStackManager.stackZStartPos;
            endPos = obj.hModel.hStackManager.stackZEndPos;
            startPower = obj.hModel.hBeams.stackStartPower; % todo multibeam
            endPower = obj.hModel.hBeams.stackEndPower;     % todo multibeam
            
            set(obj.hGUIData.motorControlsV5.etStartPower,'String',num2str(startPower));
            set(obj.hGUIData.motorControlsV5.etEndPower,'String',num2str(endPower));
            
            if obj.hModel.hFastZ.enable
                hStartEndCtls = {'etStackStart' 'etStackEnd'};
                cellfun(@(x)set(obj.hGUIData.motorControlsV5.(x),'Enable','off'),hStartEndCtls);
            else
                zlclEnableUIControlBasedOnVal(obj.hGUIData.motorControlsV5.etStackStart,startPos,'inactive');
                zlclEnableUIControlBasedOnVal(obj.hGUIData.motorControlsV5.etStackEnd,endPos,'inactive');
            end
            
            if ~isnan(startPower)
                set(obj.hGUIData.motorControlsV5.cbUseStartPower,'Enable','on');
            else
                set(obj.hGUIData.motorControlsV5.cbUseStartPower,'Enable','off');
            end
            
            if obj.hModel.hStackManager.stackStartEndPointsDefined && obj.hModel.hBeams.stackStartEndPowersDefined
                set(obj.hGUIData.motorControlsV5.cbOverrideLz,'Enable','on');
                set(obj.hGUIData.motorControlsV5.pbOverrideLz,'Enable','on');
            else
                set(obj.hGUIData.motorControlsV5.cbOverrideLz,'Enable','off');
                set(obj.hGUIData.motorControlsV5.pbOverrideLz,'Enable','off');
            end
        end
        
        function changedStackUseStartPower(obj,~,~)
            tfUseStartPower = obj.hModel.hBeams.stackUseStartPower;
            if tfUseStartPower && ~obj.hModel.hFastZ.enable
                set(obj.hGUIData.motorControlsV5.etStartPower,'Enable','inactive');
            else
                set(obj.hGUIData.motorControlsV5.etStartPower,'Enable','off');
            end
        end
        
        %%% FAST Z
        function changedFastZDiscardFlybackFrames(obj,~,~)
            hFastZGUI = obj.hGUIData.fastZControlsV4;
            
            if obj.hModel.hFastZ.discardFlybackFrames
                set(hFastZGUI.etNumDiscardFrames,'Enable','inactive');
            else
                set(hFastZGUI.etNumDiscardFrames,'Enable','off');
            end
        end
        
        function changedOverrideLz(obj,~,~)
            tf = obj.hModel.hBeams.stackUserOverrideLz;
            if tf && ~obj.hModel.hFastZ.enable
                set(obj.hGUIData.motorControlsV5.etEndPower,'Enable','inactive');
            else
                set(obj.hGUIData.motorControlsV5.etEndPower,'Enable','off');
            end
        end
        
        function changedFastZEnable(obj,~,~)
            obj.changedStackStartEndPositionPower();
            obj.changedStackUseStartPower();
            obj.changedOverrideLz();
            if obj.hModel.hFastZ.enable
                set(obj.hGUIData.mainControlsV4.framesTotal,'Enable','off');
            else
                set(obj.hGUIData.mainControlsV4.framesTotal,'Enable','on');
            end
        end
        
        function changedWaveformType(obj,~,~)
            switch obj.hModel.hFastZ.waveformType
                case 'sawtooth'
                    set(obj.hGUIData.fastZControlsV4.pmScanType, 'Value', 1);
                    set(obj.hGUIData.fastZControlsV4.cbSpecifyZs, 'Visible', 'off');
                    
                case 'step'
                    set(obj.hGUIData.fastZControlsV4.pmScanType, 'Value', 2);
                    set(obj.hGUIData.fastZControlsV4.cbSpecifyZs, 'Visible', 'on');
            end
            
            obj.changedArbZs();
        end
        
        function changedArbZs(obj,~,~)
            uz = {'lblUserZs' 'etUserZs'};
            nz = {'stNumZSlices' 'etNumZSlices' 'stZStepPerSlice' 'etZStepPerSlice' 'cbCenteredStack'};
            
            if obj.hModel.hFastZ.useArbitraryZs && strcmp(obj.hModel.hFastZ.waveformType, 'step')
                cellfun(@(s)set(obj.hGUIData.fastZControlsV4.(s),'Visible','off'),nz);
                cellfun(@(s)set(obj.hGUIData.fastZControlsV4.(s),'Visible','on'),uz);
            else
                cellfun(@(s)set(obj.hGUIData.fastZControlsV4.(s),'Visible','off'),uz);
                cellfun(@(s)set(obj.hGUIData.fastZControlsV4.(s),'Visible','on'),nz);
            end
            
        end
        
        function changeWaveformType(obj, val)
            switch val
                case 1
                    obj.hModel.hFastZ.waveformType = 'sawtooth';
                    
                case 2
                    obj.hModel.hFastZ.waveformType = 'step';
            end
        end
        
        %%% Main Controls
        function changedPointButton(obj,src,~)
            if get(src,'Value')
                obj.hModel.scanPointBeam();
            else
                obj.hModel.abort();
            end
        end
        
        function changedLogFramesPerFileLock(obj,~,~)
            if obj.hModel.hScan2D.logFramesPerFileLock
                set(obj.hGUIData.configControlsV4.etFramesPerFile,'Enable','off');
            else
                set(obj.hGUIData.configControlsV4.etFramesPerFile,'Enable','on');
            end
        end
        
        %%% Cfg controls
        function cfgLinePhaseSlider(obj,varargin)
            sliderMin  = -1000 * obj.hModel.hScan2D.linePhaseStep;
            sliderMax  =  1000 * obj.hModel.hScan2D.linePhaseStep;
            sliderStep = obj.hModel.hScan2D.linePhaseStep / (sliderMax - sliderMin);
            Value  =  obj.hModel.hScan2D.linePhase;
            set(obj.hGUIData.configControlsV4.scanPhaseSlider,'Min',sliderMin,'Max',sliderMax,'SliderStep',[sliderStep 10*sliderStep],'Value',Value);
        end
        
        function calibrateLinePhase(obj)
            assert(strcmp(obj.hModel.acqState, 'focus'),'This operation is only available during focus.');
            assert(obj.hModel.hScan2D.bidirectional,'This operation must be done with bidirectional scanning enabled');
            assert(numel(obj.hModel.hChannels.channelDisplay) > 0,'At least one channel must be selected for display');
            
            if numel(obj.hModel.hChannels.channelDisplay) < 2
                chanIdx = obj.hModel.hChannels.channelDisplay;
            else
                hFig = obj.zzzSelectImageFigure();
                if isempty(hFig)
                    return;
                end
                allChannelFigs = [obj.hModel.hDisplay.hFigs(1:obj.hModel.hChannels.channelsAvailable)];
                [tf,chanIdx] = ismember(hFig,allChannelFigs);
                if ~tf
                    return;
                end
            end
            
            obj.hModel.hScan2D.calibrateLinePhase(chanIdx);
        end
        
        %%% PMTs
        function changePmtsPowersOn(obj,pmtNum,val)
            if nargin < 3 || isempty(val)
                val = false;
            end
            
            numPmts = obj.hModel.hPmts.numPmts;
            
            if numPmts > 0
                if isempty(pmtNum)
                    obj.hModel.hPmts.powersOn = repmat(val,1,numPmts);
                else
                    obj.hModel.hPmts.setPmtPower(pmtNum, ~obj.hModel.hPmts.powersOn(pmtNum));
                    %obj.hModel.hPmts.powersOn(pmtNum) = ~obj.hModel.hPmts.powersOn(pmtNum);
                end

                obj.hModel.hPmts.updateStatus();
            end
        end
        
        function changePmtsGains(obj,pmtNum,val)
            obj.gGains(:) = nan;
            obj.hModel.hPmts.setPmtGain(pmtNum, val);
            obj.hModel.hPmts.updateStatus();
            obj.changedPmtsStatus();
        end
        
        function changePmtsOffsets(obj,pmtNum,val)
            obj.gOffs(:) = nan;
            obj.hModel.hPmts.setPmtOffset(pmtNum, val);
            obj.hModel.hPmts.updateStatus();
            obj.changedPmtsStatus();
        end
        
        function changePmtsBandwidths(obj,pmtNum,val)
            obj.gBands(:) = nan;
            obj.hModel.hPmts.setPmtBandwidth(pmtNum, val);
            obj.hModel.hPmts.updateStatus();
            obj.changedPmtsStatus();
        end
        
        function pmtsResetTripped(obj,pmtNum)
            obj.hModel.hPmts.resetTripStatus(pmtNum);
            obj.hModel.hPmts.updateStatus();
        end
        
        function changedPmtsStatus(obj,~,~)
            [powersOn, gains, pmtsTripped, offs, bands] = obj.hModel.hPmts.getLastStatus();
            
            if any(powersOn ~= obj.gPowers) || any(pmtsTripped ~= obj.gTrips)
                obj.gPowers = powersOn;
                obj.gTrips = pmtsTripped;
                for i = 1:numel(pmtsTripped)
                    pbTag = sprintf('pbPmt%dPower',i);
                    etTag = sprintf('etPmt%dStatus',i);
                    
                    if ~isnan(powersOn(i)) && powersOn(i)
                        pbString = 'On';
                    else
                        pbString = 'Off';
                    end
                    
                    if ~isnan(pmtsTripped(i)) && pmtsTripped(i)
                        etString = 'Tripped';
                        bgColor = 'r';
                    else
                        etString = 'OK';
                        if ~isnan(powersOn(i)) && powersOn(i)
                            bgColor = 'g';
                        else
                            bgColor = 'w';
                        end
                    end
                    
                    set(obj.hGUIData.pmtControlsV5.(pbTag),'String',pbString);
                    set(obj.hGUIData.pmtControlsV5.(pbTag),'Value',~isnan(powersOn(i)) && powersOn(i));
                    
                    set(obj.hGUIData.pmtControlsV5.(etTag),'String',etString);
                    set(obj.hGUIData.pmtControlsV5.(etTag),'BackgroundColor',bgColor);
                end
                drawnow
            end
            
            if any(gains ~= obj.gGains)
                obj.gGains = gains;
                for i = 1:numel(gains)
                    etTag = sprintf('etPmt%dGain',i);
                    set(obj.hGUIData.pmtControlsV5.(etTag),'String',sprintf('%.3f',gains(i)));
                end
                drawnow
            end
            
            if any(offs ~= obj.gOffs)
                obj.gOffs = offs;
                for i = 1:numel(offs)
                    etTag = sprintf('etPmt%dOffset',i);
                    set(obj.hGUIData.pmtControlsV5.(etTag),'String',sprintf('%.3f',offs(i)));
                end
                drawnow
            end
            
            if any(bands ~= obj.gBands)
                obj.gBands = bands;
                for i = 1:numel(bands)
                    etTag = sprintf('etPmt%dBandwidth',i);
                    set(obj.hGUIData.pmtControlsV5.(etTag),'String',bands(i));
                end
                drawnow
            end
        end
        
        %%% BScope2
        function changedFlipperMirrorPosition(obj,varargin)
            if ~isempty(obj.hThorBScope2MC)
                switch obj.hThorBScope2MC.flipperMirrorPosition
                    case 'pmt'
                        set(obj.hGUIData.bScope2ControlsV5.pbCamera,'Value',false);
                        set(obj.hGUIData.bScope2ControlsV5.pbPmt,'Value',true);

                    case 'camera'
                        set(obj.hGUIData.bScope2ControlsV5.pbPmt,'Value',false);
                        set(obj.hGUIData.bScope2ControlsV5.pbCamera,'Value',true);
                end
            end
        end
        
        
        function changeFlipperMirrorPosition(obj, val)
            obj.hThorBScope2MC.flipperMirrorPosition = val;
        end
        
        
        function changedGalvoResonantMirrorInPath(obj,varargin)
            if ~isempty(obj.hThorBScope2MC)
                if obj.hThorBScope2MC.galvoResonantMirrorInPath
                    set(obj.hGUIData.bScope2ControlsV5.pbGR_Out,'Value',false);
                    set(obj.hGUIData.bScope2ControlsV5.pbGR_In,'Value',true);
                else
                    set(obj.hGUIData.bScope2ControlsV5.pbGR_In,'Value',false);
                    set(obj.hGUIData.bScope2ControlsV5.pbGR_Out,'Value',true);
                end
            end
        end
        
        function changeGalvoResonantMirrorInPath(obj, val)
            obj.hThorBScope2MC.galvoResonantMirrorInPath = val;
        end
        
        function changedGalvoGalvoMirrorInPath(obj,varargin)
            if ~isempty(obj.hThorBScope2MC)
                if obj.hThorBScope2MC.galvoGalvoMirrorInPath
                    set(obj.hGUIData.bScope2ControlsV5.pbGG_Out,'Value',false);
                    set(obj.hGUIData.bScope2ControlsV5.pbGG_In,'Value',true);
                else
                    set(obj.hGUIData.bScope2ControlsV5.pbGG_In,'Value',false);
                    set(obj.hGUIData.bScope2ControlsV5.pbGG_Out,'Value',true);
                end
            end
        end
        
        function changeGalvoGalvoMirrorInPath(obj, val)
            obj.hThorBScope2MC.galvoGalvoMirrorInPath = val;
        end
        
        function setBScope2RotationAngle(obj, val)
            if ~isempty(obj.hThorBScope2LSC)
                if numel(val) == 1 && ~isnan(val)
                    validateattributes(val,{'numeric'},{'scalar', '>=',-180,'<=',180});
                    obj.hThorBScope2LSC.completeRotation(val);
                    obj.changedBScope2RotationAngle();
                end
            end
        end
        
        function stepBScope2RotationAngle(obj, mult)
            if ~isempty(obj.hThorBScope2LSC)
                val = obj.hThorBScope2LSC.rotationAngleAbsolute + obj.bScope2RotationAngleStepSize * mult;
                validateattributes(val,{'numeric'},{'scalar', '>=',-180,'<=',180});
                obj.hThorBScope2LSC.completeRotation(val);
                obj.changedBScope2RotationAngle();
            end
        end
        
        function changedBScope2RotationAngle(obj,~,evnt)
            if ~isempty(obj.hThorBScope2LSC)
                formatStr = '%.1f';
                rotationAngle = obj.hThorBScope2LSC.rotationAngleAbsolute;
                if ~isempty(rotationAngle)
                    set(obj.hGUIData.bScope2ControlsV5.etRotationAngle,'String',num2str(rotationAngle,formatStr));
                end
            end
        end
        
        function changeScanRotation(obj,src,inc)
            obj.hModel.hRoiManager.scanRotation = obj.hModel.hRoiManager.scanRotation + inc;
        end
        
        function zeroScanRotation(obj,src)
            obj.hModel.hRoiManager.scanRotation = 0;
        end
        
        function changeScanRotationX(obj,src,inc)
            obj.hModel.hRoiManager.scanRotationX = obj.hModel.hRoiManager.scanRotationX + inc;
        end
        
        function changeScanRotationY(obj,src,inc)
            obj.hModel.hRoiManager.scanRotationY = obj.hModel.hRoiManager.scanRotationY + inc;
        end
        
        function zeroScanRotationX(obj,src)
            obj.hModel.hRoiManager.scanRotationX = 0;
        end
        
        function zeroScanRotationY(obj,src)
            obj.hModel.hRoiManager.scanRotationY = 0;
        end
        
        function zeroShiftZ(obj)
            obj.hModel.hRoiManager.ShiftZ = 0;
        end
        
        function changeShiftZ(obj,src,inc)
            obj.hModel.hRoiManager.ShiftZ = obj.hModel.hRoiManager.ShiftZ + inc;
        end
        
        function changeRelTranslationZ(obj,src,inc)
            obj.hModel.hRoiManager.RelTranslationZ = obj.hModel.hRoiManager.RelTranslationZ + inc;
        end
        
        function changeRelTranslationY(obj,src,inc)
            obj.hModel.hRoiManager.RelTranslationY = obj.hModel.hRoiManager.RelTranslationY + inc;
        end
        
        function changeRelTranslationX(obj,src,inc)
            obj.hModel.hRoiManager.RelTranslationX = obj.hModel.hRoiManager.RelTranslationX + inc;
        end
        
        
        function updateZDisplay(obj,src,inc,state)
            figure(obj.hGUIData.zDisplayV1.figure1);
            subplot(3,4,[1,2,3,5,6,7,9,10,11],'replace');
            cla;
            
            state = sum(sign(get(inc.hController.hGUIData.zDisplayV1.setColorMap,'State')));
            units = get(inc.hController.hGUIData.zDisplayV1.changeUnits,'String');
            unit = units{get(inc.hController.hGUIData.zDisplayV1.changeUnits,'Value')};
            
            if strcmp(unit,'FOV')
                wave = inc.hModel.hScan2D.currentRoiGroupScannerCoords.scanStackFOV(inc.hModel.hScan2D.scannerset, 0, '',0);
                axis([-0.1 1.1 -0.1 1.1 -0.1 1.1])
            elseif strcmp(unit,'Microns')
                wave = inc.hModel.hScan2D.currentRoiGroupScannerCoords.scanStackFOV(inc.hModel.hScan2D.scannerset, 0, '',0);
                wave.G = wave.G*(obj.hModel.hScan2D.mdfData.xGalvoAngularRange/obj.hModel.hScan2D.mdfData.opticalDegreesPerMicronXY);
                obj.hModel.hScan2D.mdfData.xGalvoAngularRange
                axis([-10 300 -10 300 -10 300])
            elseif strcmp(unit,'Volt AO')
                wave = inc.hModel.hScan2D.currentRoiGroupScannerCoords.scanStackAO(inc.hModel.hScan2D.scannerset, 0, '',0);
                axis([-5 5 -5 5 -5 5])
            end
            
            len_AO = length(wave.G(:,2));
            if state == 2
                cmap = colormap(parula);
                colors = 1:len_AO;
            elseif state == 3
                cmap = colormap(copper);
                colors = wave.G(:,3);
                cmap(:,4) = 0.5;
            else
                cmap = colormap(copper);
                colors = wave.G(:,3);
            end   
            
            indices = 1:50:len_AO;
            scatter3(wave.G(indices,1),wave.G(indices,2),wave.G(indices,3),5,colors(indices),'Marker','o', 'LineWidth',5)
            axis equal
            xlabel('x'); ylabel('y'); zlabel('z')
            az = -20; el = -40; view(az,el)
            
            %axis([-5 5 -5 5 -5 5])
            rotate3d on
            hold on
            first_notnan = wave.G(~isnan(wave.G(:,3)),3);
            first_notnan = first_notnan(1);
            s = plot3(wave.G(1,1),wave.G(1,2),first_notnan,'r.','MarkerSize',5);
            set(gca,'zdir','reverse')
            
            hold off
            h = colorbar('location','westoutside','Box','off');
            if state == 2
                set(h,'YDir','reverse');
                set(get(h,'YLabel'),'String','scan order (blue to yellow)');
                set(h,'YTickLabel',{})
            elseif state == 3
                if strcmp(unit,'FOV')
                    set(get(h,'YLabel'),'String','FOV (relative corrdinates)');
                elseif strcmp(unit,'Volt AO')
                    set(get(h,'YLabel'),'String','Volt AO');
                elseif strcmp(unit,'Microns')
                    set(get(h,'YLabel'),'String','microns z');
                end
            end 
            
            h = subplot(3,4,4);
            plot(wave.G(:,1),'k');
            set(h,'XTickLabel',{})
            xlim([0 len_AO+len_AO/15])
            xlabel('x')
                
            h = subplot(3,4,8);
            plot(wave.G(:,2),'k');
            set(h,'XTickLabel',{})
            xlim([0 len_AO+len_AO/15])
            ylabel('AO Voltage')
            xlabel('y')
            
            h = subplot(3,4,12);
            plot(wave.G(:,3),'k');
            set(h,'XTickLabel',{})
            xlim([0 len_AO+len_AO/15])
            xlabel('z')
        end
    end
    
    %% INTERNAL METHODS
    %%% MOTOR ERROR CALLBACKS
    methods (Hidden, Access=private)
        function motorErrorCbk(obj,src,evt) 
            structfun(@nstDisable,obj.hGUIData.motorControlsV5);
            set(obj.hGUIData.motorControlsV5.pbRecover,'Visible','on');
            set(obj.hGUIData.motorControlsV5.pbRecover,'Enable','on');
            uistack(obj.hGUIData.motorControlsV5.pbRecover,'top');
            
            function nstDisable(h)
                if isprop(h,'Enable')
                    set(h,'Enable','off');
                end
            end
        end
    end
    
    %%% CONTROLLER PROPERTY CALLBACKS    
    methods (Hidden, Access=private)
        function hFig = zzzSelectImageFigure(obj)
            %Selects image figure, either from channelsTargetDisplay property or by user-selection
            if isempty(obj.channelsTargetDisplay)
                obj.mainControlsStatusString = 'Select image...';
                chanFigs = [ obj.hModel.hDisplay.hFigs obj.hModel.hDisplay.hMergeFigs ] ;
                hFig = most.gui.selectFigure(chanFigs);
                obj.mainControlsStatusString = '';
            elseif isinf(obj.channelsTargetDisplay)
                hFig = obj.hModel.hDisplay.hMergeFigs;
            else
                hFig = obj.hModel.hDisplay.hFigs(obj.channelsTargetDisplay);
            end
        end
    end
end

%% LOCAL
function v = zlclShortenFilename(v)
assert(ischar(v));
[~,v] = fileparts(v);
end

%helper for changedStackStartEndPositionPower
function zlclEnableUIControlBasedOnVal(hUIC,val,enableOn)
if isnan(val)
    set(hUIC,'Enable','off');
else
    set(hUIC,'Enable',enableOn);
end
end

function s = lclInitPropBindings(hModel)
    %NOTE: In this prop metadata list, order does NOT matter!
    %NOTE: These are properties for which some/all handling of model-view linkage is managed 'automatically' by this class
    %TODO: Some native approach for dependent properties could be specified here, to handle straightforward cases where change in one property affects view of another -- these are now handled as 'custom' behavior with 'Callbacks'
    %For example: scanLinePeriodUS value depends on scanMode
    s = struct();

    %%SI Root Model
    s.imagingSystem             = struct('Callback','changedImagingSystem');
    s.acqsPerLoop               = struct('GuiIDs',{{'mainControlsV4','repeatsTotal'}});
    s.loopAcqInterval           = struct('GuiIDs',{{'mainControlsV4','etRepeatPeriod'}});
    s.extTrigEnable             = struct('GuiIDs',{{'mainControlsV4' 'tbExternalTrig'}},'Callback','changedExtTrigEnable');

    % acquisition State
    s.frameCounterForDisplay = struct('GuiIDs',{{'mainControlsV4','framesDone'}});
    s.loopAcqCounter         = struct('GuiIDs',{{'mainControlsV4','repeatsDone'}});
    s.acqState               = struct('Callback','changedAcqState','GuiIDs',{{'mainControlsV4' 'statusString'}});
    s.acqInitInProgress      = struct('Callback','changedAcqState');
    s.secondsCounter         = struct('Callback','changedSecondsCounter');

    %%% Stack props
    s.hStackManager.framesPerSlice     = struct('GuiIDs',{{'mainControlsV4','framesTotal'}});
    s.hStackManager.slicesPerAcq       = struct('GuiIDs',{{'mainControlsV4','slicesTotal'}});
    s.hStackManager.numSlices          = struct('GuiIDs',{{'motorControlsV5','etNumberOfZSlices','fastZControlsV4','etNumZSlices'}});
    s.hStackManager.stackSlicesDone    = struct('GuiIDs',{{'mainControlsV4','slicesDone'}});
    s.hStackManager.stackZStartPos     = struct('GuiIDs',{{'motorControlsV5','etStackStart'}},'Callback','changedStackStartEndPositionPower');
    s.hStackManager.stackZEndPos       = struct('GuiIDs',{{'motorControlsV5','etStackEnd'}},'Callback','changedStackStartEndPositionPower');
    s.hStackManager.stackZStepSize     = struct('GuiIDs',{{'motorControlsV5','etZStepPerSlice','fastZControlsV4','etZStepPerSlice'}});
    s.hStackManager.stackReturnHome    = struct('GuiIDs',{{'motorControlsV5','cbReturnHome','fastZControlsV4','cbReturnHome'}});
    s.hStackManager.stackStartCentered = struct('GuiIDs',{{'motorControlsV5','cbCenteredStack','fastZControlsV4','cbCenteredStack'}});


    %%% Submodels (sub-components)
    %%% Display component
    s.hDisplay.displayRollingAverageFactor     = struct('GuiIDs',{{'imageControlsV4','etRollingAverage'}});
    s.hDisplay.displayRollingAverageFactorLock = struct('GuiIDs',{{'imageControlsV4','cbLockRollAvg2AcqAvg'}},'Callback','changedDisplayRollingAverageFactorLock');
    s.hDisplay.displayFrameBatchFactor         = struct('GuiIDs',{{'imageControlsV4','etFrameSelFactor'}});
    s.hDisplay.displayFrameBatchSelection      = struct('GuiIDs',{{'imageControlsV4','etFrameSelections'}});
    s.hDisplay.displayFrameBatchSelectLast     = struct('GuiIDs',{{'imageControlsV4','cbUseLastSelFrame'}},'Callback','updateFrameBatchOptions');
    s.hDisplay.displayFrameBatchSelectAll      = struct('GuiIDs',{{'imageControlsV4','cbUseAll'}},'Callback','updateFrameBatchOptions');
    s.hDisplay.displayFrameBatchFactorLock     = struct('GuiIDs',{{'imageControlsV4','cbLockFrameSel2RollAvg'}},'Callback','updateFrameBatchOptions');
    s.hDisplay.displayVolumeLock               = struct('GuiIDs',{{'imageControlsV4','cbLockVolume'}},'Callback','updateFrameBatchOptions');
    
    s.hDisplay.chan1LUT = struct('Callback','changedChanLUT');
    s.hDisplay.chan2LUT = struct('Callback','changedChanLUT');
    s.hDisplay.chan3LUT = struct('Callback','changedChanLUT');
    s.hDisplay.chan4LUT = struct('Callback','changedChanLUT');
    
    %s.hDisplay.channelsMergeColor      = struct('GuiIDs',{{'channelControlsV4','pcChannelConfig'}},'PropControlData',struct('columnIdx',6,'format','options','prettyOptions',{{'Green' 'Red' 'Blue' 'Gray' 'None'}}));
    s.hDisplay.channelsMergeEnable     = struct('GuiIDs',{{'channelControlsV4','cbMergeEnable'}},'Callback','changedChannelsMergeEnable');
    s.hDisplay.channelsMergeFocusOnly  = struct('GuiIDs',{{'channelControlsV4','cbChannelsMergeFocusOnly'}});
    
    
    %%% Scan2D component
    % channels
    %s.hScan2D.channelsAcquire          = struct('GuiIDs',{{'channelControlsV4','pcChannelConfig'}},'PropControlData',struct('columnIdx',2,'format','logicalindices','formatInfo',[]),'Callback','changedChannelsAcquire');
    %s.hScan2D.logChannels              = struct('GuiIDs',{{'channelControlsV4','pcChannelConfig'}},'PropControlData',struct('columnIdx',1,'format','logicalindices','formatInfo',[]));
    %s.hScan2D.channelsInputRanges      = struct('GuiIDs',{{'channelControlsV4','pcChannelConfig'}},'PropControlData',struct('columnIdx',3,'format','options'));
    %s.hScan2D.channelsOffsets          = struct('GuiIDs',{{'channelControlsV4','pcChannelConfig'}},'PropControlData',struct('columnIdx',4,'format','numeric'));
    %s.hScan2D.channelsSubtractOffsets  = struct('GuiIDs',{{'channelControlsV4','pcChannelConfig'}},'PropControlData',struct('columnIdx',5,'format','logical'));
    s.hScan2D.channelsAutoReadOffsets  = struct('GuiIDs',{{'channelControlsV4','cbAutoReadOffsets'}});

    s.hChannels.loggingEnable          = struct('GuiIDs',{{'mainControlsV4','cbAutoSave','configControlsV4','cbAutoSave'}},'Callback','changedLogEnable');
    s.hChannels.channelName            = struct('GuiIDs',{{'channelControlsV4','pcChannelConfigg'}},'PropControlData',struct('columnIdx',0,'format','cellstr'));
    s.hChannels.channelSave            = struct('GuiIDs',{{'channelControlsV4','pcChannelConfigg'}},'PropControlData',struct('columnIdx',1,'format','logicalindices','formatInfo',[]));
    s.hChannels.channelDisplay         = struct('GuiIDs',{{'channelControlsV4','pcChannelConfigg'}},'PropControlData',struct('columnIdx',2,'format','logicalindices','formatInfo',[]),'Callback','changedChannelDisplay');
    s.hChannels.channelInputRange      = struct('GuiIDs',{{'channelControlsV4','pcChannelConfigg'}},'PropControlData',struct('columnIdx',3,'format','options'));
    s.hChannels.channelOffset          = struct('GuiIDs',{{'channelControlsV4','pcChannelConfigg'}},'PropControlData',struct('columnIdx',4,'format','numeric'));
    s.hChannels.channelSubtractOffset  = struct('GuiIDs',{{'channelControlsV4','pcChannelConfigg'}},'PropControlData',struct('columnIdx',5,'format','logical'));
    s.hChannels.channelMergeColor      = struct('GuiIDs',{{'channelControlsV4','pcChannelConfigg'}},'PropControlData',struct('columnIdx',6,'format','options','prettyOptions',{{'Green' 'Red' 'Blue' 'Gray' 'None'}}));
    
    % SCAN
    s.hScan2D.bidirectional            = struct('GuiIDs',{{'configControlsV4','cbBidirectionalScan'}});
    s.hScan2D.fillFractionTemporal     = struct('GuiIDs',{{'configControlsV4','etFillFrac'}},'ViewPrecision','%0.3f');
    s.hScan2D.fillFractionSpatial      = struct('GuiIDs',{{'configControlsV4','etFillFracSpatial'}},'ViewPrecision','%0.3f');
    s.hScan2D.scanPixelTimeMean        = struct('GuiIDs',{{'configControlsV4','etPixelTimeMean'}},'ViewScaling',1e9,'ViewPrecision','%.1f');
    s.hScan2D.scanPixelTimeMaxMinRatio = struct('GuiIDs',{{'configControlsV4','etPixelTimeMaxMinRatio'}},'ViewPrecision','%.1f');
    s.hScan2D.linePhase                = struct('GuiIDs',{{'configControlsV4','etScanPhase'}},'Callback','changedScanPhase','ViewScaling',1e6);
    s.hScan2D.sampleRate               = struct('GuiIDs',{{'configControlsV4','etSampleRateMHz'}},'ViewPrecision','%.3f','ViewScaling',1e-6,'Callback','cfgLinePhaseSlider');
    s.hScan2D.trigAcqInTerm            = struct('GuiIDs',{{'triggerControlsV5','pmTrigAcqInTerm'}},'Callback','changedTrigAcqInTerm');
    s.hScan2D.trigStopInTerm           = struct('GuiIDs',{{'triggerControlsV5','pmTrigStopInTerm'}});
    s.hScan2D.trigNextInTerm           = struct('GuiIDs',{{'triggerControlsV5','pmTrigNextInTerm'}});
    s.hScan2D.trigAcqEdge              = struct('GuiIDs',{{'triggerControlsV5','pmTrigAcqEdge'}});
    s.hScan2D.trigStopEdge             = struct('GuiIDs',{{'triggerControlsV5','pmTrigStopEdge'}});
    s.hScan2D.trigNextEdge             = struct('GuiIDs',{{'triggerControlsV5','pmTrigNextEdge'}});
    s.hScan2D.trigNextStopEnable       = struct('GuiIDs',{{'triggerControlsV5','cbTrigNextStopEnable'}},'Callback', 'changedTrigNextStopEnable');
    s.hScan2D.pixelBinFactor           = struct('GuiIDs',{{'configControlsV4','etPixelBinFactor'}});
    s.hScan2D.flytoTimePerScanfield    = struct('GuiIDs',{{'configControlsV4','etFlytoTimePerScanfieldMs'}},'ViewPrecision','%.3f','ViewScaling',1e3);
    s.hScan2D.flybackTimePerFrame      = struct('GuiIDs',{{'configControlsV4','etFlybackTimePerFrameMs'}},'ViewPrecision','%.3f','ViewScaling',1e3);
    s.hScan2D.keepResonantScannerOn    = struct('GuiIDs',{{'configControlsV4','cbKeepScannerOn'}});
    
    % logging
    s.hScan2D.logFileStem           = struct('GuiIDs',{{'mainControlsV4' 'baseName'}});
    s.hScan2D.logFileCounter        = struct('GuiIDs',{{'mainControlsV4' 'fileCounter'}});
    s.hScan2D.logFramesPerFile      = struct('GuiIDs',{{'configControlsV4' 'etFramesPerFile'}});
    s.hScan2D.logFramesPerFileLock  = struct('GuiIDs',{{'configControlsV4' 'cbFramesPerFileLock'}},'Callback','changedLogFramesPerFileLock');
    s.hScan2D.logAverageFactor      = struct('GuiIDs',{{'mainControlsV4','etNumAvgFramesSave'}});
    
    %%% ROIMANAGER component
    s.hRoiManager.forceSquarePixelation    = struct('GuiIDs',{{'configControlsV4','cbForceSquarePixelation'}},'Callback','changedForceSquarePixelation');
    s.hRoiManager.forceSquarePixels        = struct('GuiIDs',{{'configControlsV4','cbForceSquarePixel'}});
    s.hRoiManager.linesPerFrame            = struct('GuiIDs',{{'configControlsV4','etLinesPerFrame'}},'Callback','changedPowerBoxes');
    s.hRoiManager.pixelsPerLine            = struct('GuiIDs',{{'configControlsV4','pmPixelsPerLine'}},'Callback','changedPowerBoxes');
    s.hRoiManager.scanAngleMultiplierFast  = struct('GuiIDs',{{'mainControlsV4','etScanAngleMultiplierFast'}});
    s.hRoiManager.scanAngleMultiplierSlow  = struct('GuiIDs',{{'mainControlsV4','etScanAngleMultiplierSlow'}});
    s.hRoiManager.scanAngleShiftSlow       = struct('GuiIDs',{{'mainControlsV4','scanShiftSlow'}});
    s.hRoiManager.scanAngleShiftFast       = struct('GuiIDs',{{'mainControlsV4','scanShiftFast'}});
    s.hRoiManager.scanFramePeriod          = struct('GuiIDs',{{'fastZControlsV4','etFramePeriod'}},'ViewPrecision','%.1f','ViewScaling',1000,'Callback','changedScanFramePeriod');
    s.hRoiManager.scanFrameRate            = struct('GuiIDs',{{'configControlsV4','etFrameRate'}},'ViewPrecision','%.2f');
    s.hRoiManager.linePeriod               = struct('GuiIDs',{{'configControlsV4','etLinePeriod'}},'ViewScaling',1e6,'ViewPrecision','%.2f');
    s.hRoiManager.scanRotation             = struct('GuiIDs',{{'mainControlsV4','scanRotation'}});
    s.hRoiManager.scanRotationX            = struct('GuiIDs',{{'mainControlsV4','scanRotationX'}});
    s.hRoiManager.scanRotationY            = struct('GuiIDs',{{'mainControlsV4','scanRotationY'}});
    s.hRoiManager.ShiftZ                   = struct('GuiIDs',{{'mainControlsV4','ShiftZ'}});
    s.hRoiManager.RelTranslationX          = struct('GuiIDs',{{'mainControlsV4','RelTranslationX'}});
    s.hRoiManager.RelTranslationY          = struct('GuiIDs',{{'mainControlsV4','RelTranslationY'}});
    s.hRoiManager.RelTranslationZ          = struct('GuiIDs',{{'mainControlsV4','RelTranslationZ'}});
    s.hRoiManager.scanZoomFactor           = struct('GuiIDs',{{'mainControlsV4' 'pcZoom'}});
    s.hRoiManager.scanVolumeRate           = struct('GuiIDs',{{'fastZControlsV4', 'etVolumeRate'}},'ViewPrecision','%.2f');
    s.hRoiManager.mroiEnable               = struct('GuiIDs',{{'mainControlsV4', 'cbEnableMroi'}},'Callback','updateFrameBatchOptions');
    s.hRoiManager.mroiFocusEnable          = struct('GuiIDs',{{'mainControlsV4', 'cbEnableMroiFocus'}});
    
    %%% FASTZ component
    s.hFastZ.enable                        = struct('GuiIDs',{{'fastZControlsV4','cbEnable'}},'Callback','changedFastZEnable');
    s.hFastZ.numVolumes                    = struct('GuiIDs',{{'fastZControlsV4','etNumVolumes'}});
    s.hFastZ.volumesDone                   = struct('GuiIDs',{{'fastZControlsV4','etVolumesDone'}});
    s.hFastZ.discardFlybackFrames          = struct('GuiIDs',{{'fastZControlsV4','cbDiscardFlybackFrames'}},'Callback','changedFastZDiscardFlybackFrames');
    s.hFastZ.settlingTime                  = struct('GuiIDs',{{'fastZControlsV4','etSettlingTime'}});
    s.hFastZ.numDiscardFlybackFrames       = struct('GuiIDs',{{'fastZControlsV4','etNumDiscardFrames'}},'Callback','updateFrameBatchOptions');
    s.hFastZ.volumePeriodAdjustment        = struct('GuiIDs',{{'fastZControlsV4','pcFramePeriodAdjust'}},'ViewPrecision',2,'ViewScaling',1e6);
    s.hFastZ.waveformType                  = struct('Callback','changedWaveformType');
    s.hFastZ.useArbitraryZs                = struct('GuiIDs',{{'fastZControlsV4','cbSpecifyZs'}},'Callback','changedArbZs');
    s.hFastZ.userZs                        = struct('GuiIDs',{{'fastZControlsV4','etUserZs'}});
    
    %%% ConfigurationSaver component
    s.hConfigurationSaver.cfgFilename          = struct('Callback','changedCfgFilename');
    s.hConfigurationSaver.usrFilename          = struct('Callback','changedUsrFilename');
    s.hConfigurationSaver.usrPropList   = struct('Callback','changedUsrPropList');
    s.hConfigurationSaver.cfgLoadingInProgress = struct('Callback','changedCfgLoading');
    s.hConfigurationSaver.fastCfgCfgFilenames  = struct('GuiIDs',{{'fastConfigurationV4','pcFastCfgTable'}},'PropControlData',struct('columnIdx',3,'format','cellstr','customEncodeFcn',@zlclShortenFilename),'Callback','changedFastCfgCfgFilenames');
    s.hConfigurationSaver.fastCfgAutoStartTf   = struct('GuiIDs',{{'fastConfigurationV4','pcFastCfgTable'}},'PropControlData',struct('columnIdx',4,'format','logical'),'Callback','changedFastCfgAutoStartTf');
    s.hConfigurationSaver.fastCfgAutoStartType = struct('GuiIDs',{{'fastConfigurationV4','pcFastCfgTable'}},'PropControlData',struct('columnIdx',5,'format','options'));
    
    %%% UserFcns component
    s.hUserFunctions.userFunctionsCfg      = struct('Callback','changedUserFunctionsCfg');
    s.hUserFunctions.userFunctionsUsr      = struct('Callback','changedUserFunctionsUsr');
    s.hUserFunctions.userFunctionsOverride = struct('Callback','changedUserFunctionsOverride');
    
    %%% Beam component
    s.hScan2D.beamClockDelay     = struct('GuiIDs',{{'powerControlsV4','etBeamLead'}});
    s.hBeams.flybackBlanking     = struct('GuiIDs',{{'powerControlsV4','cbBlankFlyback'}});
    s.hBeams.displayNames        = struct('GuiIDs',{{'powerControlsV4','powerTbl'}},'PropControlData',struct('columnIdx',0,'format','cellstr'));
    s.hBeams.powers              = struct('Callback','changedBeamParams','GuiIDs',{{'powerControlsV4','powerTbl'}},'PropControlData',struct('columnIdx',2,'format','numeric'));
    s.hBeams.pzAdjust            = struct('Callback','changedBeamParams','GuiIDs',{{'powerControlsV4','powerTbl'}},'PropControlData',struct('columnIdx',3,'format','logical'));
    s.hBeams.lengthConstants     = struct('Callback','changedBeamParams','GuiIDs',{{'powerControlsV4','powerTbl'}},'PropControlData',struct('columnIdx',4,'format','numeric'));
    s.hBeams.powerLimits         = struct('Callback','changedBeamParams');
    s.hBeams.directMode          = struct('Callback','changedBeamParams');
    s.hBeams.interlaceDecimation = struct('Callback','changedBeamParams');
    s.hBeams.interlaceOffset     = struct('Callback','changedBeamParams');
    s.hBeams.powerUnits          = struct('Callback','changedBeamPowerUnits');
    s.hBeams.stackStartPower     = struct('GuiIDs',{{'motorControlsV5','etStartPower'}},'Callback','changedStackStartEndPositionPower');
    s.hBeams.stackEndPower       = struct('GuiIDs',{{'motorControlsV5','etEndPower'}},'Callback','changedStackStartEndPositionPower');
    s.hBeams.stackUseStartPower  = struct('GuiIDs',{{'motorControlsV5','cbUseStartPower'}},'Callback','changedStackUseStartPower');
    s.hBeams.stackUserOverrideLz = struct('GuiIDs',{{'motorControlsV5','cbOverrideLz'}},'Callback','changedOverrideLz');
    s.hBeams.enablePowerBox      = struct('GuiIDs',{{'powerControlsV4','cbEnablePowerBox'}});
    
    %%% Power box
    s.hBeams.powerBoxes          = struct('Callback','changedPowerBoxes');
    s.hBeams.powerBoxStartFrame  = struct('GuiIDs',{{'powerBoxControlsV4','etStartFrame'}});
    s.hBeams.powerBoxEndFrame    = struct('GuiIDs',{{'powerBoxControlsV4','etEndFrame'}});

    %%% Motors component
    s.hMotors.motorSecondMotorZEnable = struct('GuiIDs',{{'motorControlsV5','cbSecZ'}});
    s.hMotors.motorPosition           = struct('Callback','changedMotorPosition');
    s.hMotors.userDefinedPositions    = struct('Callback','changedPosns');

    
    %%% Thor BScope2
    if isprop(hModel, 'hBScope2') && isa(hModel.hBScope2, 'dabs.thorlabs.BScope2')
        s.hBScope2.scanAlign                = struct('GuiIDs',{{'bScope2ControlsV5','etScanAlign','bScope2ControlsV5','slScanAlign'}});
    end
end


%--------------------------------------------------------------------------%
% SIController.m                                                           %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
