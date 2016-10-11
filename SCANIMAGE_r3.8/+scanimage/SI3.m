classdef SI3 < most.Model
    %Class encapsulating (newer) ScanImage 3.x state/behavior
    
    %% ABSTRACT PROPERTY REALIZATION (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = zlclInitPropMetadata(); %A structure effecting Map from property names to structures whose fields are Tags, with associated values, specifying attributes of each property
        
        %OPTIONAL (Can leave empty)
        mdlHeaderExcludeProps; %String cell array of props to forcibly exclude from header
    end
    
    %% USER PROPERTIES
    
    properties (SetObservable)
        
        % menu  props
        roiUseMIPForMRI = true; % If true, Max Image Projections will be stored as the Most Recent Image (in the EOA cache).
        roiShowMarkerNumbers = true; % If true, ROI/Position IDs will be displayed on the RDF and PDF.
        roiGotoOnAdd = true; % If true, ROI scan parameters are automatically updated to match each new ROI added using roiAddXXX() methods
        roiSnapOnAdd = false; % If true, and roiGotoOnAdd=true as well, a Snapshot image is automatically collected at each new ROI added using roiAddXXX() methods.
        roiGotoOnSelect = true; % If true, ROI scan parameters are automatically updated to match any ROI selected in the table.
        roiSnapOnSelect = false; % If true, and roiGotoOnAdd=true as well, a Snapshot image is automatically collected for any ROI selected in the table.
        roiShowAbsoluteCoords = false; % If true, Positions will be displayed using absolute coordinates.
        roiWarnOnMove = true; % If true, any ROI-goto operation involving a motor-move will prompt the user for confirmation.
        
        % RDF toolbar props
        roiDisplayedChannel = '1'; % Specifies which channel to use for ROIs; one of {'1' '2' '3' '4' 'merge'}
        roiDisplayDepth=inf;
        
        % Tolerances
        roiPositionToleranceX;% = state.motor.posnResolution;
        roiPositionToleranceY;% = state.motor.posnResolution;
        roiPositionToleranceZ;% = state.motor.posnResolution;
        roiPositionToleranceZZ;% = state.motor.posnResolutionZ;
        
        %Position properties
        posnIgnoreSecZ = false; %If true, secondary Z values of stored motor positions are ignored when 1) going to position or 2) updating active positionID
        
    end
    
    
    %% SUPERUSER PROPERTIES
    
    properties (Hidden,SetAccess=private)
        
        % Overridable function handles
        % NOTE: these are intended to be directly accessed, ignoring good OOP practice in favor of performance, i.e. feval(state.hSI.hMakeStripe)
        hMakeStripe = @makeStripe;
        hMakeFrameByStripes = @makeFrameByStripes;
        hEndAcquisition = @endAcquisition;
        hMakeMirrorDataOutput = @makeMirrorDataOutput;
        
        % USER-ADDED OVERRIDABLE FUNCTION HANDLES
        %    NOTE: Any added overridable functions/handles should be added to overridableFcns list
        
    end
    
    properties (Constant)
        overridableFcns = {'makeStripe' 'makeFrameByStripes' 'endAcquisition' 'makeMirrorDataOutput'};
    end
    
    
    %% DEVELOPER PROPERTIES
    
    properties (Access=private)
        %User/Override Function Handling
        listenerAbortFlag = false; % A flag indicating that a notify()-ed event generated an "error" condition.
    end
    
    
    properties (Hidden, SetAccess=protected, SetObservable)
        
        %ROI ColumnArrayTable bound props
        roiIDs = {};
        roiPositionIDs = [];
        roiTypes = {};
        roiZoomFactors = [];
        roiRotations = [];
        roiShifts = {};
        roiScanAngleMultipliers = {};
        
        %roiAspectRatios = [1 nan]; %TODO!
        
    end
    
    properties (Hidden, SetObservable)
        
        % Position Handling
        positionDataStructure;
        activePositionID = 0;
        selectedPositionID;
        
        % ROI Handling
        roiDataStructure;
        activeROIID;
        shownROI = scanimage.SI3.ROI_ROOT_ID;
        selectedROIID;
        roiLastLine = [];
        
        currentRSPStruct; % a struct containing the current RSPs
        
        % ROI GUI params
        roiBreadcrumbString = 'ROOT =>';
        roiAngleToMicronsFactor=1; % an unrealistic, but safe, default value...
        
        roiLastAcqCache;
        
        roiName='';
        roiPath='';
        roiLoading = false;
        
        % ColumnArrayTable bound props
        positionIDs = {};
        xVals = [];
        yVals = [];
        zVals = [];
        zzVals = [];
        
        roiSuppressUpdates = false; % if true, the ROI table & display figure will not be automatically updated
    end
    
    properties (Hidden, SetObservable)
        roiAutoUpdateConfig = true;
        
        %Line Scan Handling
        lineScanEnable = false;
        scanAngleMultiplierSlowCache = [];
        
        roiActiveUpdatePending = false; % if true, indicates that the active ROI has been changed, but the view has not yet been updated.
        %roiSuppressLinescanSideEffects = false; % if true, indicates that enabling/disabling LS mode should have no side effects.
        
        isSubUnityZoomAllowed = false;
        
        currentBaseZoom = 1;
        
        motorStepSizeX = 1;
        motorStepSizeY = 1;
        motorStepSizeZ = 1;
        motorStepSizeZZ = 1;
        %motorStepSizeZCache = 1; % allows caching of z-step value when secondary-z motor is enabled/disabled.
    end
    
    properties (Hidden, SetAccess=protected)
        hROIDisplayFig;
        hROIDisplayAx;
        hROIDisplayIm;
        
        hROIAcqIm;
    end
    
    properties (Hidden, Dependent, SetObservable)
        roiIsShownPositionNotActive; %Logical indicating if shown position does not match active position
        shownRotation; %Rotation associated with currently shownROI
        shownPositionID; %Motor position associated with currently shownROI
    end
    
    properties (Hidden, Dependent)
        roiBaseConfigStruct;
    end
    
    properties (Hidden, Constant)
        usrBoundProperties = {'roiAngleToMicronsFactor' 'roiUseMIPForMRI' 'roiShowMarkerNumbers' ...
            'roiGotoOnAdd' 'roiSnapOnAdd' 'roiGotoOnSelect' 'roiSnapOnSelect' 'roiShowAbsoluteCoords' ...
            'roiBaseConfigStruct' 'roiWarnOnMove' 'roiDisplayedChannel' 'roiDisplayDepth' ...
            'posnIgnoreSecZ' 'roiPositionToleranceX' 'roiPositionToleranceY' 'roiPositionToleranceZ' 'roiPositionToleranceZZ' ...
            };
        
        scanParameterNames = {'zoomFactor' 'scanShiftFast' 'scanShiftSlow' 'scanRotation' 'scanAngleMultiplierFast' 'scanAngleMultiplierSlow'};
        
        markerColors = containers.Map({'point' 'line' 'square' 'rect' 'selected' 'active'}, ...
            {[1 0 1] [1 0 1] [1 0 1] [1 0 1] [1 1 0] [0 1 0]});
        
        ROI_ROOT_ID = 0;
        ROI_BASE_ID = -1;
        
        ROI_FIG_CHECKERBOARD_SIZE = 256;
    end
    
    
    %% CLASS EVENTS
    events
        
        %% COMMONLY USED EVENTS
        %A list of the 'top-ten' events, most likely to be useful to general users
        
        acquisitionStarting; % Fires when a GRAB acqusition or LOOP acquisition is being started.
        acquisitionStarted; % Fires when a GRAB acqusition or LOOP repeat has been started
        acquisitionDone; %Fires when a GRAB acquisition or LOOP repeat has completed
        sliceDone; %Fires when single slice of a multi-slice GRAB/LOOP acquisition has completed
        cycleIterating; %Fires when next iteration of a cycle-mode LOOP acquisition is about to start
        loopModeDone; %Fires when a LOOP mode acquisition has fully completed, without aborting
        
        focusStart; % Fires when a FOCUS acquisition has been started.
        focusDone; %Fires when FOCUS acquisition is completed
        
        stripeAcquired; %Fires when acqusition of stripe has occurred
        frameAcquired; %Fires when acquisition of frame has been completed
        
        startTriggerReceived; %Fires when start trigger is received (only for GRAB/LOOP acquisitions)
        nextTriggerReceived; %Fires when a 'next' trigger is received
        
        %USR-file only events
        appOpen; %Fires when ScanImage starts
        appClose; %Fires when ScanImage closes
        
        motorPositionUpdated; %Fires when motorGetPosition() is called or motorSetPosition() is called with 'assume' option.
        
        %% USER-ADDED EVENTS
        %Add any events required by your application here
        %At appropriate point in application code, you must add the line:
        %   notify(state.userFcns.hEventManager,'<EVENT_NAME>');
        %   (Ensure the 'state' variable is in scope, by entering 'global state' in same function, if not there already)
        
        
        
        %% OTHER EVENTS
        %Other events, added by developers, e.g. for specific 'plugins', are entered here
        
        abortAcquisitionStart; %Fires at start of an abort acquisition operation (for GRAB/LOOP)
        abortAcquisitionEnd; %Event at end of an abort acquisition operation (for GRAB/LOOP)
        
        
        %TODO: Add following events, using regular notify (see DriftComp branch)
        %         executeFocusStart; %Event invoked at start of acquisition function execute<Focus/Grab/Loop>Callback()
        %         executeGrabStart;
        %         executeLoopStart
        %
        
        %TODO: Add following events,  using 'smart' notify (see DriftComp branch, si_notify())
        %         startGrabStart;
        %         startFocusStart;
        
    end
    
    
    
    %% CONSTRUCTOR/DESTRUCTOR
    methods
        function obj = SI3()
            
            %Initialize Class Data store
            obj.ensureClassDataFile(struct('userFcnsLastPath', most.idioms.startPath(), 'usrOnlyFcnsLastPath', most.idioms.startPath(), ...
                'overrideFcnsLastPath',most.idioms.startPath(), 'cycleLastPath',most.idioms.startPath(), ...
                'cycleCFGLastPath',most.idioms.startPath(), 'roiLastPath',most.idioms.startPath()));
            
            %Construction initialization fcns
            obj.ziniInitRoiAndPosnDataStructs();
            
            %Callback bindings
            addlistener(obj,'acquisitionDone',@obj.roiEOA_Listener);
            addlistener(obj,'focusDone',@obj.roiEOA_Listener);
            addlistener(obj,'motorPositionUpdated',@obj.roiMotorUpdate_Listener);
            
            %Miscellaneous initializations
            obj.notify('dummyEvent'); %Ensure persistent var gets initialized
        end
        
        function initialize(obj)
            initialize@most.Model(obj);
            
            global state
            
            obj.ziniInitializeFigures();
            obj.roiSetBaseConfig();
            
        end
    end
    
    methods (Access=protected)
        
        function ziniInitRoiAndPosnDataStructs(obj)
            
            % Initialize the ROI data-struct, adding the ROOT and BASE ROIs
            obj.roiDataStructure = containers.Map('KeyType','int32','ValueType','any');
            
            rootROIStruct = struct('type','square','positionID',0,'children',[]);
            rootROIStruct.RSPs.zoomFactor = 1;
            rootROIStruct.RSPs.scanAngleMultiplierFast = 1;
            rootROIStruct.RSPs.scanAngleMultiplierSlow = 1;
            rootROIStruct.RSPs.scanShiftFast = 0;
            rootROIStruct.RSPs.scanShiftSlow = 0;
            rootROIStruct.RSPs.scanRotation = 0;
            
            obj.roiDataStructure(obj.ROI_ROOT_ID) = rootROIStruct;
            
            baseROIStruct = struct('type','square','parentROIID',obj.ROI_ROOT_ID,'positionID',0);
            obj.roiDataStructure(obj.ROI_BASE_ID) = baseROIStruct;
            obj.roiAddChildToParent(obj.ROI_ROOT_ID, obj.ROI_BASE_ID);
            
            % Initialize the Position data-struct, adding a root all-NaN position
            obj.positionDataStructure = containers.Map('KeyType','int32','ValueType','any');
            obj.roiAddPosition(struct('motorX',nan,'motorY',nan,'motorZ',nan,'motorZZ',nan),0); % create a 'root' all-NaN PDO
            
        end
        
    end
    
    
    methods (Access=public)%protected
        
        function ziniInitializeFigures(obj,~,~)
            global state;
            
            obj.hROIDisplayFig = obj.hController{1}.hGUIData.roiDisplayGUI.figure1;
            
            set(obj.hController{1}.hGUIData.roiDisplayGUI.pnlROIDisplay,'BackgroundColor',get(state.hSI.hROIDisplayFig,'Color'));
            obj.hROIDisplayAx = obj.hController{1}.hGUIData.roiDisplayGUI.axROIDisplay;
            obj.hROIDisplayIm = image('Parent', obj.hROIDisplayAx,'CData',obj.roiDrawCheckeredBackground([repmat(obj.ROI_FIG_CHECKERBOARD_SIZE,1,2) 3]),'Visible','off');
            set(obj.hROIDisplayAx,'XTick',[],'YTick',[],'XLim',[0 obj.ROI_FIG_CHECKERBOARD_SIZE]+0.5,'YLim',[0 obj.ROI_FIG_CHECKERBOARD_SIZE]+0.5,'Visible','off');
            
            obj.roiUpdateROITable();
            obj.roiUpdatePositionTable();
        end
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% PROPERTY ACCESS
    
    methods
        
        %% GET HETHODS
        
        function val = get.currentRSPStruct(obj)
            if ~obj.mdlInitialized
                return;
            end
            
            global state;
            
            val = struct();
            for i = 1:length(obj.scanParameterNames)
                val.(obj.scanParameterNames{i}) = state.acq.(obj.scanParameterNames{i});
            end
        end
        
        function val = get.roiBaseConfigStruct(obj)
            val = [];
            
            global state gh;
            if isempty(state) || isempty(gh)
                % must be startup--global structs don't exist yet...
                return;
            end
            
            roiStruct = obj.roiDataStructure(obj.ROI_BASE_ID);
            fieldsToRemove = {'MRI' 'hMarker' 'hMarkerLabel'};
            for i = 1:length(fieldsToRemove)
                if isfield(roiStruct,fieldsToRemove{i})
                    roiStruct = rmfield(roiStruct,fieldsToRemove{i});
                end
            end
            val = roiStruct;
        end
        
        
        %% SET METHODS
        
        function set.activePositionID(obj,val)
            
            if ~obj.mdlInitialized
                return;
            end
            
            prevVal = val;
            
            val = obj.validatePropArg('activePositionID',val);
            obj.activePositionID = val;
            
            %Side effects
            if ~isequal(val, prevVal) % Clear cache of last added/gone-to line ROI if motor position has changed
                obj.roiLastLine = [];
            end
        end
        
        function set.activeROIID(obj,val)
            
            if ~obj.mdlInitialized
                return;
            end
            
            % before we update the value, reset the marker color of the currently selected ROI
            if ~isempty(obj.activeROIID)
                if ~isempty(obj.selectedROIID) && obj.activeROIID == obj.selectedROIID
                    % do nothing
                else
                    if obj.roiDataStructure.isKey(obj.activeROIID)
                        obj.roiSetMarkerColor(obj.activeROIID); % sets color to default
                    end
                end
            end
            
            if ~isempty(val) && (isempty(obj.activeROIID) || val ~= obj.activeROIID)
                obj.roiActiveUpdatePending = true;
            end
            
            % boilerplate set code
            val = obj.validatePropArg('activeROIID',val);
            obj.activeROIID = val;
            
            % highlight the active ROI ID marker in green
            if ~isempty(val)
                obj.roiSetMarkerColor(val,obj.markerColors('active'));
            end
            
            % update the asterisk in the ROI table
            if ~obj.roiSuppressUpdates
                obj.roiUpdateROITable();
            end
        end
        
        
        function val = get.shownPositionID(obj)
            if obj.shownROI > 0
                val = obj.roiGetPositionFromROIID(obj.shownROI);
            else
                val = [];
            end
        end
        
        function set.shownPositionID(obj,val)
            obj.mdlDummySetProp(val,'shownPositionID');
        end
        
        function set.shownROI(obj,val)
            
            if ~obj.mdlInitialized
                return;
            end
            
            global state;
            
            % Prevent points and lines from being shown
            if obj.roiDataStructure.isKey(val)
                newROIStruct = obj.roiDataStructure(val);
                if (isfield(newROIStruct,'type') && (strcmpi(newROIStruct.type,'point') || strcmpi(newROIStruct.type,'line')))
                    return;
                end
            else
                obj.consoleError('Invalid ROI ID.');
                return;
            end
            
            % "pre-set" logic:
            if ~obj.roiSuppressUpdates
                obj.roiSetMarkersVisibility('off');
            end
            
            if val ~= obj.shownROI
                didChange = true;
            else
                didChange = false;
            end
            
            % boilerplate "set" logic:
            val = obj.validatePropArg('shownROI',val);
            obj.shownROI = val;
            
            %Side effects
            if obj.roiSuppressUpdates
                return;
            end
            
            obj.roiUpdateView();
            
            if didChange
                obj.selectedROIID = [];
            end
            
            % update the ROI uitable
            obj.roiUpdateROITable();
            
        end
        
        function set.isSubUnityZoomAllowed(obj,val)
            val = obj.validatePropArg('isSubUnityZoomAllowed',val);
            obj.isSubUnityZoomAllowed = val;
            
            global state gh;
            
            if isempty(state) || isempty(gh)
                % must be startup--global structs don't exist yet...
                return;
            end
            
            % update dependant properties
            if ~obj.isSubUnityZoomAllowed %~get(gh.userPreferenceGUI.cbIsSubUnityZoomAllowed,'Value')
                if state.acq.zoomFactor < 1
                    setZoomValue(1);
                end
                
                if state.acq.minZoomFactor < 1
                    state.acq.minZoomFactor = 1;
                    updateGUIByGlobal('state.acq.minZoomFactor');
                end
            else
                state.acq.minZoomFactor = 0.1;
                updateGUIByGlobal('state.acq.minZoomFactor');
            end
        end
        
        function set.roiBaseConfigStruct(obj,val)
            global state gh;
            if isempty(state) || isempty(gh)
                % must be startup--global structs don't exist yet...
                return;
            end
            
            obj.roiDataStructure(obj.ROI_BASE_ID) = val;
        end
        
        function set.roiDisplayedChannel(obj,val)
            global state;
            
            if ~obj.mdlInitialized
                return;
            end
            
            obj.validatePropArg('roiDisplayedChannel',val);
            
            entryErrorString = sprintf('Must supply an integer 1-%d, corresponding to a channel number, or the string ''merge''',state.init.maximumNumberOfInputChannels);
            numVal = str2double(val);
            if isnan(numVal)
                assert(strcmpi(val,'merge'),entryErrorString)
            else
                assert(ismember(numVal,1:state.init.maximumNumberOfInputChannels),entryErrorString);
            end
            
            obj.roiDisplayedChannel = val;
            
            %Side-effects
            obj.roiUpdateView();
        end
        
        function set.roiDisplayDepth(obj,val)
            % boilerplate set code
            val = obj.validatePropArg('roiDisplayDepth',val);
            obj.roiDisplayDepth = val;
            
            % update the views
            obj.roiUpdateROITable();
            obj.roiUpdateView();
        end
        
        function set.roiGotoOnAdd(obj,val)
            % boilerplate set code
            val = obj.validatePropArg('roiGotoOnAdd',val);
            obj.roiGotoOnAdd = val;
        end
        
        function set.roiGotoOnSelect(obj,val)
            % boilerplate set code
            val = obj.validatePropArg('roiGotoOnSelect',val);
            obj.roiGotoOnSelect = val;
        end
        
        function val = get.roiIsShownPositionNotActive(obj)
            val = ~isempty(obj.shownPositionID) && obj.shownPositionID > 0 && ~isequal(obj.shownPositionID,obj.activePositionID);
        end
        
        function set.roiIsShownPositionNotActive(obj,val)
            obj.mdlDummySetProp(val,'roiIsShownPositionNotActive');
        end
        
        function set.roiSnapOnAdd(obj,val)
            % boilerplate set code
            val = obj.validatePropArg('roiSnapOnAdd',val);
            obj.roiSnapOnAdd = val;
        end
        
        function set.roiSnapOnSelect(obj,val)
            % boilerplate set code
            val = obj.validatePropArg('roiSnapOnSelect',val);
            obj.roiSnapOnSelect = val;
        end
        
        function val = get.shownRotation(obj)
            if obj.shownROI > 0
                val = obj.roiDataStructure(obj.shownROI).RSPs.scanRotation;
            else
                val = [];
            end
        end
        
        function set.shownRotation(obj,val)
            obj.mdlDummySetProp(val,'shownRotation');
        end
        
        function set.selectedROIID(obj,val)
            % Sets the currently-selected ROI.
            
            global state gh;
            if isempty(state) || isempty(gh)
                % must be startup--global structs don't exist yet...
                return;
            end
            
            % before we update the value, reset the marker color of the currently selected ROI
            if ~isempty(obj.selectedROIID)
                if ~isempty(obj.activeROIID) && isscalar(obj.selectedROIID) && obj.selectedROIID == obj.activeROIID
                    obj.roiSetMarkerColor(obj.selectedROIID,obj.markerColors('active'));
                else
                    obj.roiSetMarkerColor(obj.selectedROIID);
                end
            end
            
            % boilerplate set code
            val = obj.validatePropArg('selectedROIID',val);
            obj.selectedROIID = val;
            
            if ~isempty(val)
                % highlight the selected ROI marker in yellow
                obj.roiSetMarkerColor(val,obj.markerColors('selected'));
                
                if obj.roiGotoOnSelect
                    obj.roiGotoROI(obj.selectedROIID);
                    
                    % only take a snapshot if 'goto-on-select' is enabled
                    if obj.roiSnapOnSelect
                        snapShot();
                        while state.internal.snapping
                            pause(0.1);
                        end
                    end
                end
            end
        end
        
        
        function set.roiPositionToleranceX(obj,val)
            val = obj.zprpRoiClampPositionTolerance(val,'X');
            
            val = obj.validatePropArg('roiPositionToleranceX',val);
            obj.roiPositionToleranceX = val;
            obj.roiMotorUpdate_Listener();
        end
        
        function set.roiPositionToleranceY(obj,val)
            val = obj.zprpRoiClampPositionTolerance(val,'Y');
            
            val = obj.validatePropArg('roiPositionToleranceY',val);
            obj.roiPositionToleranceY = val;
            obj.roiMotorUpdate_Listener();
        end
        
        function set.roiPositionToleranceZ(obj,val)
            val = obj.zprpRoiClampPositionTolerance(val,'Z');
            
            val = obj.validatePropArg('roiPositionToleranceZ',val);
            obj.roiPositionToleranceZ = val;
            obj.roiMotorUpdate_Listener();
        end
        
        function set.roiPositionToleranceZZ(obj,val)
            val = obj.zprpRoiClampPositionTolerance(val,'ZZ');
            
            val = obj.validatePropArg('roiPositionToleranceZZ',val);
            obj.roiPositionToleranceZZ = val;
            obj.roiMotorUpdate_Listener();
        end
        
        function set.roiShowAbsoluteCoords(obj,val)
            val = obj.validatePropArg('roiShowAbsoluteCoords',val);
            obj.roiShowAbsoluteCoords = val;
            
            obj.roiUpdatePositionTable();
        end
        
        function set.roiShowMarkerNumbers(obj,val)
            val = obj.validatePropArg('roiShowMarkerNumbers',val);
            obj.roiShowMarkerNumbers = val;
            
            if val
                visibleState = 'on';
            else
                visibleState = 'off';
            end
            
            rois = obj.roiDataStructure.keys();
            for i = 1:length(rois)
                roiStruct = obj.roiDataStructure(rois{i});
                
                if isfield(roiStruct,'hMarkerLabel')
                    set(roiStruct.hMarkerLabel,'Visible',visibleState);
                end
            end
        end
        
        function set.posnIgnoreSecZ(obj,val)
            val = obj.validatePropArg('posnIgnoreSecZ',val);
            obj.posnIgnoreSecZ = val;
            
            obj.roiUpdatePositionTable();
        end
        
        
    end
    
    %Property-access helpers
    methods (Hidden)
        
        
        function val = zprpRoiClampPositionTolerance(obj,val,dimension)
            % Enforces position tolerance/resolution constraints.
            
            global state;
            
            if state.motor.motorOn && (isempty(val) || val < state.motor.(['resolution' dimension]))
                val = state.motor.(['resolution' dimension]);
            elseif isempty(val) %motorOn=0
                val = inf;
            end
            
        end
    end
    
    %% USER METHODS
    methods
        
        function cycAddIteration(obj,rowArray)
            % Adds a new iteration row to the end of the cycle table.
            %
            % rowArray: A cell array representing the new row.
            %
            
            global state;
            
            if nargin < 2 || isempty(rowArray)
                rowArray = state.cycle.cycleTableColumnDefaults;
            end
            
            columnNames = state.cycle.cycleTableColumns;
            if length(rowArray) ~= length(columnNames)
                obj.consoleError('Invalid row data.');
                return;
            end
            
            if isempty(fieldnames(state.cycle.cycleTableStruct))
                iterationIndex = 1;
            else
                iterationIndex = length(state.cycle.cycleTableStruct) + 1;
            end
            
            for i = 1:length(rowArray)
                state.cycle.cycleTableStruct(iterationIndex).(columnNames{i}) = rowArray{i};
            end
            
            state.cycle.cycleConfigPaths{iterationIndex} = '';
            if isempty(state.cycle.cycleLength)
                state.cycle.cycleLength = 1;
            else
                state.cycle.cycleLength = state.cycle.cycleLength + 1;
            end
            updateGUIByGlobal('state.cycle.cycleLength'); % TODO: move this to PropControl
            
            if ~obj.roiSuppressUpdates
                obj.hController{1}.cycTableUpdateView();
            end
        end
        
        function cycRemoveIteration(obj,iterationIndex)
            % Removes the specified iteration (row) from the cycle table.
            %
            % iterationIndex: an integer specifying the row to be removed.
            % If empty, the last row will be removed.
            %
            
            global state;
            
            if nargin < 2 || isempty(iterationIndex)
                iterationIndex = length(state.cycle.cycleTableStruct);
            end
            
            state.cycle.cycleTableStruct(iterationIndex) = [];
            state.cycle.cycleConfigPaths(iterationIndex) = [];
            state.cycle.cycleLength = state.cycle.cycleLength - 1;
            updateGUIByGlobal('state.cycle.cycleLength'); %TODO: move this to PropControl
            
            if ~obj.roiSuppressUpdates
                obj.hController{1}.cycTableUpdateView();
            end
        end
        
        function roiID = roiAddCurrent(obj)
            % Adds a new ROI using the current RSPs.
            %
            % roiID: the ID of the newly created ROI.
            
            hAx = obj.zprvRoiAddBegin(true);
            if isempty(hAx)
                return;
            end
            
            % calling roiAddNew() without args captures the current config state
            roiID = obj.roiAddNew();
        end
        
        function roiID = roiAddRect(obj,doForceSquare)
            % Adds a new Rectanglular ROI.
            %
            % doForceSquare: a boolean that, if true, indicates to constrain graphical selection to a square.
            %
            % roiID: the ID of the newly created ROI.
            
            global state
            
            roiID = [];
            done = 0;
            
            hAx = obj.zprvRoiAddBegin();
            if isempty(hAx)
                return;
            end
            
            if nargin < 2 || isempty(doForceSquare)
                doForceSquare = false;
            end
            
            % Extract ROI coordinates from target figure
            pos=getRectFromAxes(hAx, 'Cursor', 'crosshair', 'nomovegui', 1, 'forcesquare', doForceSquare,'LineColor',[0.5 0.5 0.5],'LineWidth',2); %VI071310A %VI021809B
            if pos(3) == 0 || pos(4) == 0
                return;
            end
            
            sizeImage = [diff(get(hAx,'XLim')) diff(get(hAx,'YLim'))];
            
            % Determine original scan rotation and whether ROI can in fact be added
            if hAx == obj.hROIDisplayAx
                shownROIStruct = obj.roiDataStructure(obj.shownROI);
                originalRotation = shownROIStruct.RSPs.scanRotation;
                originalZoomFactor = shownROIStruct.RSPs.zoomFactor;
                
                originalSAMFast = shownROIStruct.RSPs.scanAngleMultiplierFast;
                originalSAMSlow = shownROIStruct.RSPs.scanAngleMultiplierSlow;
            else
                originalRotation = obj.roiLastAcqCache.RSPs.scanRotation;
                originalZoomFactor = obj.roiLastAcqCache.RSPs.zoomFactor;
                
                originalSAMFast = obj.roiLastAcqCache.RSPs.scanAngleMultiplierFast;
                originalSAMSlow = obj.roiLastAcqCache.RSPs.scanAngleMultiplierSlow;
                
                % make sure it makes sense to add a new ROI, given the current scan configuration
                currentType = obj.computeROIType(originalSAMFast,originalSAMSlow);
                if strcmp(currentType,'line') || strcmp(currentType,'point')
                    obj.fullError('Cannot create ROI', 'Unable to add ROI under current scan configuration.');
                    roiID = [];
                    return;
                end
            end
            
            if pos(1) + pos(3) > sizeImage(1) || pos(2) + pos(4) > sizeImage(2)
                obj.fullError('Invalid ROI','The drawn ROI exceeds the bounds of the current acquisition.');
                return;
            end
            
            %Quad ROIs inherit same rotation as their parent
            s.RSPs.scanRotation = originalRotation;
            
            %Determine scan shift
            xc = pos(1) + pos(3)/2;
            yc = pos(2) + pos(4)/2;
            angularCoords = obj.zprvRoiConvertPixels2Angle([xc yc],hAx,true);
            
            s.RSPs.scanShiftFast = angularCoords(1);
            s.RSPs.scanShiftSlow = angularCoords(2);
            
            %Compute zoom & SAM
            R = [cosd(originalRotation) -sind(originalRotation); sind(originalRotation) cosd(originalRotation)];
            %scanAngleRefRotated = R * [state.init.scanAngularRangeReferenceFast;state.init.scanAngularRangeReferenceSlow];
            
            zoomFactorFast = originalZoomFactor * (sizeImage(1) / pos(3)) / originalSAMFast;
            zoomFactorSlow = originalZoomFactor * (sizeImage(2) / pos(4)) / originalSAMSlow;
            
            s.RSPs.zoomFactor = ceil(10 * min(zoomFactorFast,zoomFactorSlow))/10;
            
            if zoomFactorFast == zoomFactorSlow
                s.RSPs.scanAngleMultiplierFast = 1;
                s.RSPs.scanAngleMultiplierSlow = 1;
            elseif zoomFactorFast > zoomFactorSlow
                s.RSPs.scanAngleMultiplierFast = zoomFactorSlow/zoomFactorFast;
                s.RSPs.scanAngleMultiplierSlow = 1 ;
            elseif zoomFactorSlow > zoomFactorFast
                s.RSPs.scanAngleMultiplierFast = 1;
                s.RSPs.scanAngleMultiplierSlow = zoomFactorFast/zoomFactorSlow;
            end
            %Add to ROI Spec Table
            roiID = obj.roiAddNew(s,[],hAx);
        end
        
        function roiID = roiAddSquare(obj)
            % Adds a new Square ROI.
            
            roiID = obj.roiAddRect(true);
        end
        
        function roiIDs = roiAddPoints(obj,numPoints)
            % Opens a dialog to add one or more 'point' ROIs.
            
            global state;
            
            if nargin < 2 || isempty(numPoints)
                numPoints = inf;
            end
            
            hAx = obj.zprvRoiAddBegin();
            if isempty(hAx)
                return;
            end
            
            roiIDs = [];
            
            %Extract ROI coordinates from target figure
            [x, y]= getPointsFromAxes(hAx, 'Cursor', 'crosshair', 'nomovegui', 1,'numberOfPoints',numPoints,'LineStyle','none','MarkerEdgeColor',[0.5 0.5 0.5],'MarkerSize',8,'EraseMode','normal');
            if isempty(x) || isempty(y)
                return;
            end
            
            % convert from pixel to angular coordinates
            angularPointCoords = obj.zprvRoiConvertPixels2Angle([x y],hAx,true);
            
            if isempty(angularPointCoords)
                return;
            end
            
            % Add the new Point ROIs...
            roiIDs = zeros(1,size(angularPointCoords,1));
            gotoOnAddCache = obj.roiGotoOnAdd;
            obj.roiGotoOnAdd = false;
            obj.roiSuppressUpdates = true;
            hWaitbar = waitbar(0,'Adding Point ROIs');
            for i=1:size(angularPointCoords,1)
                if i == size(angularPointCoords,1)
                    obj.roiGotoOnAdd = gotoOnAddCache;
                    obj.roiSuppressUpdates = false; % don't update the view until the last point
                end
                roiIDs(i) = obj.roiAddPoint(angularPointCoords(i,:),hAx);
                waitbar(i/size(angularPointCoords,1),hWaitbar);
            end
            close(hWaitbar);
            
            obj.roiSuppressUpdates = false;
            obj.roiUpdateROITable();
            obj.roiUpdatePositionTable();
        end
        
        function roiID = roiAddPoint(obj,posn,hAx)
            % Adds a single 'point' ROI at the given position.
            %
            % posn: the position (given in angular coordinates) of the new Point
            % hAx: a handle to the Axes object that the point was "drawn" on.
            %
            % roiID: the ID of the newly created ROI.
            
            % if called without args, prompt user for graphical selection
            if nargin < 2 || isempty(posn)
                roiID = obj.roiAddPoints(1);
                return;
            end
            
            global state;
            roiID = [];
            
            % 			% ensure we have EOA data
            % 			if isempty(obj.roiLastAcqCache)
            % 				obj.fullError('Cannot create ROI', 'Unable to create ROI: No acquired data.')
            % 				return;
            % 			end
            
            if nargin < 3 || isempty(hAx)
                % if no Axes handle given, assume channel-1 display
                hAx = state.internal.axis(1);
            end
            
            % construct the ROI structure
            s = struct();
            s.RSPs.scanAngleMultiplierFast = 0;
            s.RSPs.scanAngleMultiplierSlow = 0;
            s.RSPs.scanShiftFast = posn(1);
            s.RSPs.scanShiftSlow = posn(2);
            
            if ismember(hAx,state.internal.axis)
                s.RSPs.scanRotation = obj.roiLastAcqCache.RSPs.scanRotation;
            else
                shownROIStruct = obj.roiDataStructure(obj.shownROI);
                if isfield(shownROIStruct,'RSPs') && isfield(shownROIStruct.RSPs,'scanRotation')
                    s.RSPs.scanRotation = shownROIStruct.RSPs.scanRotation;
                else
                    obj.fullError('Can''t add ROI; invalid scan parameters.');
                end
            end
            
            roiID = obj.roiAddNew(s,[],hAx);
            if roiID < 1
                return;
            end
        end
        
        function roiIDs = roiAddGrid(obj,gridSize,gridShift,isGridShiftInMicrons,scanAngleMultiplier,zoomFactor,rotation,angleToMicronsFactor)
            % Adds a grid of ROIs, which can be points, lines, or rectangles/square
            %
            % gridSize: a 2-vector specifying the MxN size of the grid.
            % gridShift: <Default=[0 0]> a 2-vector specifying the x,y offset to be added to the grid. TODO: MxN next to x,y feels bad
            % isGridShiftInMicrons: <Default=false> a boolean that, if true, specifies that the gridShift units are in microns.
            % scanAngleMultiplier: <Default=[0 0]> a 2-vector specifying scanAngleMultiplierFast/Slow for ROIs to add
            % zoomFactor: <Default=1> zoom factor for ROIs to add
            % rotation: <Default=0> rotation, in degrees, for ROIs to add
            % angleToMicronsFactor: an integer specifying the angle->micron conversion factor for the current rig.
            %
            % roiIDs: a vector of created ROI IDs.
            
            roiIDs = [];
            
            global state;
            
            if nargin < 8 || isempty(angleToMicronsFactor)
                angleToMicronsFactor = obj.roiAngleToMicronsFactor;
            end
            
            if nargin < 7 || isempty(rotation)
                rotation = 0;
            end
            
            if nargin < 6 || isempty(zoomFactor)
                zoomFactor = 1;
            end
            
            if nargin < 5 || isempty(scanAngleMultiplier)
                scanAngleMultiplier = [0 0];
            end
            
            if nargin < 4 || isempty(isGridShiftInMicrons)
                isGridShiftInMicrons = false;
            end
            
            if nargin < 3 || isempty(gridShift)
                gridShift = [0 0];
            end
            
            if nargin < 2
                obj.hController{1}.macroGrid();
                return;
            end
            
            padX = state.acq.pixelsPerLine/(gridSize(2));
            padY = state.acq.linesPerFrame/(gridSize(1));
            
            if isGridShiftInMicrons
                angleMultiplier = [state.acq.scanAngleMultiplierFast*state.init.scanAngularRangeReferenceFast ...
                    state.acq.scanAngleMultiplierSlow*state.init.scanAngularRangeReferenceSlow ];
                gridShift = ((gridShift./angleToMicronsFactor)./angleMultiplier).*[state.acq.pixelsPerLine state.acq.linesPerFrame];
            end
            
            roiType = obj.computeROIType(scanAngleMultiplier(1),scanAngleMultiplier(2));
            
            % all ROIs that are part of a 'grid' get assigned to an all-NaN PDO.
            % Temporarily make this PDO the 'active', so that all created ROIs
            % get assigned to it. (And create this PDO, if necessary.)
            nanPositionID = obj.roiAddPosition(struct('motorX',nan,'motorY',nan,'motorZ',nan,'motorZZ',nan));
            
            %Use current RSPs as parent if of lower effective zoom than grid-placed ROIs
            %Otherwise, use ROOT ROI RSPs
            rsps = struct();
            for i = 1:length(obj.scanParameterNames)
                rsps.(obj.scanParameterNames{i}) = state.acq.(obj.scanParameterNames{i});
            end
            effZoomCUR = rsps.zoomFactor/(rsps.scanAngleMultiplierFast * rsps.scanAngleMultiplierSlow);
            effZoomGrid = zoomFactor/(scanAngleMultiplier(1)*scanAngleMultiplier(2));
            
            if effZoomCUR > effZoomGrid   %Use ROOT ROI RSPs instead
                rsps = obj.roiDataStructure(obj.ROI_ROOT_ID).RSPs;
            end
            
            %If grid ROIs are area ROIs, ensure rotation of abstract parent ROI matches that of these child area ROIs on grid
            if all(scanAngleMultiplier > 0) %Grid ROIs are area ROIs
                rsps.scanRotation = rotation;
            end
            
            %Add grid parent ROI, with abstract/negative/all-nan positionID
            allNanROIID = obj.roiAddNew(struct('positionID',nanPositionID,'RSPs',rsps));
            
            % temporarily disable 'roiXXXOnAdd'
            gotoOnAddCache = obj.roiGotoOnAdd;
            snapOnAddCache = obj.roiSnapOnAdd;
            obj.roiGotoOnAdd = false;
            obj.roiSnapOnAdd = false;
            
            roiIDs = zeros(1,gridSize(1)*gridSize(2));
            obj.roiSuppressUpdates = true;
            hWaitbar = waitbar(0,'Adding ROIs on grid...');
            for j = 1:gridSize(1)
                for i = 1:gridSize(2)
                    
                    %Determine grid point in angular coordinates for fast/slow axes
                    fsPixels= [i*padX - padX/2 + gridShift(1), j*padY - padY/2 + gridShift(2)];
                    fsAngular = obj.zprvRoiConvertPixels2Angle(fsPixels);
                    
                    s.RSPs.scanShiftFast = fsAngular(1);
                    s.RSPs.scanShiftSlow = fsAngular(2);
                    s.RSPs.scanAngleMultiplierFast = scanAngleMultiplier(1);
                    s.RSPs.scanAngleMultiplierSlow = scanAngleMultiplier(2);
                    s.RSPs.zoomFactor = zoomFactor;
                    s.RSPs.scanRotation = rotation;
                    
                    s.parentROIID = allNanROIID;
                    s.type = roiType;
                    
                    roiIDs((j-1)*gridSize(2) + i) = obj.roiAddNew(s);
                    
                    waitbar(((j-1)*gridSize(2) + i)/(gridSize(1)*gridSize(2)),hWaitbar);
                end
            end
            close(hWaitbar);
            
            %Store cached last-acquired image for parent of grid ROI, if available
            if ~isempty(obj.roiLastAcqCache) && obj.roiIsEqualRSPs(rsps,obj.roiLastAcqCache.RSPs)
                s = obj.roiDataStructure(allNanROIID);
                s.MRI = obj.roiLastAcqCache.MRI;
                obj.roiDataStructure(allNanROIID) = s;
            end
            
            % restore the state
            obj.roiSuppressUpdates = false;
            %             obj.activePositionID = activePositionIDCache;
            obj.roiGotoOnAdd = gotoOnAddCache;
            obj.roiSnapOnAdd = snapOnAddCache;
            
            obj.roiUpdateROITable();
            obj.roiUpdatePositionTable();
            obj.roiUpdateView();
        end
        
        function roiAddCenterPoint(obj)
            % Adds a single 'point' ROI centered in the acquisition window.
            
            s = struct();
            s.scanAngleMultiplierFast = 0;
            s.scanAngleMultiplierSlow = 0;
            s.scanShiftFast = 0;
            s.scanShiftSlow = 0;
            s.scanRotation = 0;
            
            %Add to ROI Spec Table
            roiID = obj.roiAddNew(s);
            
            if roiID < 1
                return;
            end
        end
        
        
        function roiID = roiAddLine(obj)
            % Adds a new Line ROI.
            
            global state gh;
            
            hAx = obj.zprvRoiAddBegin();
            if isempty(hAx)
                return;
            end
            
            done = 0;
            roiID = [];
            
            %Extract ROI coordinates from target figure
            [x, y]= getPointsFromAxes(hAx, 'Cursor', 'crosshair', 'nomovegui', 1,'numberOfPoints',2,'LineColor',[0.5 0.5 0.5],'LineWidth',2,'Marker','x','MarkerEdgeColor',[0.5 0.5 0.5],'MarkerSize',8,'EraseMode','normal');
            if isempty(x) || isempty(y) || length(x) ~= 2 || length(y) ~= 2
                return
            end
            
            % initialize RSP structure
            s = struct();
            s.type = 'line';
            
            % compute normalized x/y coordinates (normalized to scanAngularRangeReferenceFast/Slow
            angularPointCoords = obj.zprvRoiConvertPixels2Angle([x y],hAx,true);
            midPointNormalized = [sum(angularPointCoords(:,1))/2 sum(angularPointCoords(:,2))/2];
            
            % determine scan rotation (in degrees) -- exactly as drawn, before rescaling
            dx = diff(angularPointCoords(:,1))/state.init.scanAngularRangeReferenceFast;
            dy = diff(angularPointCoords(:,2))/state.init.scanAngularRangeReferenceSlow;
            delta = sqrt(power(dx,2) + power(dy,2));
            
            scanRotation = -asind(dy/delta);
            if abs(atan2(dy,dx)) > pi/2
                scanRotation = -scanRotation;
            end
            s.RSPs.scanRotation = scanRotation;
            
            % calculate zoom
            s.RSPs.zoomFactor = round((1/delta) * 10) / 10;
            
            % determine scan shift (determined by the mid-point of the line)
            s.RSPs.scanShiftFast = midPointNormalized(1);
            s.RSPs.scanShiftSlow = midPointNormalized(2);
            
            % Update SAM settings (use normalized 1, 0 values -- all magnification is via the zoom parameter)
            s.RSPs.scanAngleMultiplierFast = 1;
            s.RSPs.scanAngleMultiplierSlow = 0;
            
            % add to ROI Spec Table
            roiID = obj.roiAddNew(s,[],hAx);
        end
        
        
        function positionID = roiAddPosition(obj,position,positionID)
            % Adds a new Position (PDO).
            %
            % position: 3 or 4 vector specifying *absolute* position, or a positionStruct with fields motorX/Y/Z/ZZ
            % positionID: Integer identifying position.
            
            global state;
            
            
            if nargin < 3
                positionID = [];
            end
            
            if nargin < 2 || isempty(position)
                
                if ~state.motor.motorOn
                    obj.fullError('No motor.','No motor is currently configured; adding positions is disabled.');
                    return;
                end
                
                %Update the motor position
                motorGetPosition();
                
                position = [state.motor.absXPosition state.motor.absYPosition state.motor.absZPosition];
                
                if state.motor.dimensionsXYZZ
                    position(end+1) = state.motor.absZZPosition;
                end
                
                doUpdateActivePosition = true;
            else
                doUpdateActivePosition = false;
            end
            
            if ~isstruct(position)
                
                % make sure the given position doesn't already exist...
                positionIDExisting = obj.zprvPosn2PositionID(position);
                if positionIDExisting > 0
                    positionID = positionIDExisting;
                    obj.fullError('Position exists.',sprintf('Specified position vector matches that of existing Position ID #%d (to within current roiPosnTolerance).',positionID));
                    return;
                end
                
                % populate all fields with current values
                positionStruct = struct('motorX',position(1),'motorY',position(2),'motorZ',position(3));
                if state.motor.dimensionsXYZZ
                    positionStruct.motorZZ = position(4);
                end
            else
                positionStruct = position;
            end
            
            suffixes = {'X' 'Y' 'Z' 'ZZ'};
            isAllNan = true;
            for i = 1:length(suffixes)
                if ~isnan(positionStruct.(['motor' suffixes{i}]))
                    isAllNan = false;
                    break;
                end
            end
            
            % if there's no motor, only allow all-NaN Positions
            if (~isfield(state,'motor') || ~state.motor.motorOn) && ~isAllNan
                obj.fullError('No motor','Unable to add Position: no motor.');
                return;
            end
            
            % determine the new Position ID.
            if isempty(positionID)
                existingKeys = obj.positionDataStructure.keys();
                existingKeys = [existingKeys{:}];
                
                if isempty(existingKeys)
                    positionID = (-1)^(isAllNan);
                else
                    if isAllNan
                        existingKeys(existingKeys > 0) = [];
                        positionID = min(existingKeys) - 1;
                    else
                        existingKeys(existingKeys < 0) = [];
                        positionID = max(existingKeys) + 1;
                    end
                end
            end
            
            obj.positionDataStructure(positionID) = positionStruct;
            
            % update the current position
            if doUpdateActivePosition
                obj.activePositionID = positionID;
            end
            
            if ~obj.roiSuppressUpdates
                obj.roiUpdatePositionTable();
            end
        end
        
        function wasPositionApplied = roiGotoPosition(obj,posnID)
            % Moves the motor to the given Position ID.
            %
            % posnID: the PDO to go to.  If empty, the currently selected Position will be used.
            %
            % wasPositionApplied: a boolean that, if true, indicates that the motor position was successfully applied.
            
            wasPositionApplied = false;
            
            global state;
            if ~state.motor.motorOn
                obj.fullError('No motor','No motor is currently configured; motor moves are disabled.');
                return;
            end
            
            if nargin < 2 || isempty(posnID)
                if isempty(obj.selectedPositionID)
                    obj.consoleError('No Position selected.');
                    return; % should never get here
                end
                posnID = obj.selectedPositionID;
            else
                if posnID ~= obj.selectedPositionID
                    % ideally, in this case we could programmatically select the given posnID in the table, but in lieu of that,
                    % just set 'selectedPositonID' to empty.
                    obj.selectedPositionID = [];
                end
            end
            
            motorPositionGoto(posnID);
            
            wasPositionApplied = true;
        end
        
        function roiGotoLastLine(obj,doGotoParent)
            % Applies the 'last line' ROI.
            %
            % doGotoParent: a boolean that, if true, indicates to go the the *parent* of the last line.
            
            if nargin < 2 || isempty(doGotoParent)
                doGotoParent = false;
            end
            
            if isempty(obj.roiLastLine)
                if doGotoParent
                    obj.consoleError('No ''last line'' defined.');
                    return;
                end
                
                % if a line doesn't exist, create one...
                roiID = obj.roiAddLine();
                if obj.roiGotoOnAdd
                    % the line has already been gone to...
                    return;
                end
            else
                roiID = obj.roiLastLine;
            end
            
            if doGotoParent
                lastLineStruct = obj.roiDataStructure(obj.roiLastLine);
                if isfield(lastLineStruct,'parentROIID')
                    obj.roiGotoROI(lastLineStruct.parentROIID);
                end
            else
                obj.roiGotoROI(roiID);
            end
            
        end
        
        
        function wasROIApplied = roiGotoROI(obj,roiID,doSuppressMoveWarning)
            % Applies the selected ROI: applies all ROI scan parameters and goes to the associated motor position.
            %
            % roiID: the ROI to be applied.
            % doSuppressMoveWarning: a boolean that, if true, indicates to ignore 'roiWarnOnMove'. (useful for cycle-mode)
            %
            % wasROIApplied: a boolean that, if true, indicates that then ROI was successfully applied.
            
            wasROIApplied = false;
            
            if obj.roiLoading
                return;
            end
            
            if nargin < 3 || isempty(doSuppressMoveWarning)
                doSuppressMoveWarning = false;
            end
            
            if nargin < 2 || isempty(roiID)
                if isempty(obj.selectedROIID) || length(obj.selectedROIID) > 1
                    obj.consoleError('Invalid ROI.');
                    return;
                end
                roiID = obj.selectedROIID;
            end
            
            global state;
            
            roiType = '';
            
            % 			if obj.lineScanEnable
            % 				obj.roiSuppressLinescanSideEffects = true;
            % 				obj.lineScanEnable = false;
            % 				obj.roiSuppressLinescanSideEffects = false;
            % 			end
            
            try
                if ~obj.roiDataStructure.isKey(roiID)
                    obj.consoleError('Invalid ROI ID.');
                    return;
                end
                
                roiStruct = obj.roiDataStructure(roiID);
                rspStruct = roiStruct.RSPs;
                
                roiType = roiStruct.type;
                
                % Apply RSPs
                if ~isinf(rspStruct.zoomFactor)
                    setZoomValue(rspStruct.zoomFactor,true);
                end
                
                scanParams = {'scanShiftFast' 'scanShiftSlow' 'scanRotation' 'scanAngleMultiplierFast' 'scanAngleMultiplierSlow'};
                changedParams = {};
                
                
                for i = 1:length(scanParams)
                    paramName = scanParams{i};
                    
                    if state.acq.(scanParams{i}) ~= rspStruct.(paramName)
                        changedParams{end+1} = paramName;
                    end
                    
                    state.acq.(scanParams{i}) = rspStruct.(paramName);
                    updateGUIByGlobal(['state.acq.' paramName]);
                end
                
                
                %Execute appropriate INI callbacks (not invoked
                %automatically by updateGUIByGlobal). Note we could/would
                %call updateRSPs() here for all the RSP cases other than
                %SAMFast/Slow -- but its present logic (updating active
                %ROI) is entirely duplicated by subsequent logic below
                %updating activeROIID
                if ismember('scanAngleMultiplierFast',changedParams)
                    updateScanAngleMultiplier();
                elseif ismember('scanAngleMultiplierSlow',changedParams)
                    updateScanAngleMultiplierSlow();
                end
                
                %Flag change in RSPs
                if ~isempty(changedParams)
                    state.internal.updatedZoomOrRot = 1;
                end
                
                % Apply the motor position and update activeROIID
                noMotorMove = true;
                
                if state.motor.motorOn
                    
                    positionID = obj.roiGetPositionFromROIID(roiID);
                    
                    % make the move if necessary,
                    if positionID > 0 && positionID ~= obj.activePositionID
                        noMotorMove = false;
                        
                        if obj.roiWarnOnMove && ~(doSuppressMoveWarning || state.cycle.cycling)
                            % prompt the user for confirmation
                            choice = questdlg('This GOTO operation involves a motor-move; would you like to proceed?', ...
                                'GOTO warning', ...
                                'Yes','No','Don''t warn me again','Yes');
                            switch choice
                                case 'No'
                                    return;
                                case 'Don''t warn me again'
                                    obj.roiWarnOnMove = false;
                            end
                        end
                        motorPositionGoto(positionID); %leads to roiUpdateActiveROI() call
                    end
                end
                
                %Update activeROI (if not done so as part of motor move), since RSPs may have changed
                if noMotorMove
                    obj.roiUpdateActiveROI(roiID);
                end
                
            catch ME
                ME.throwAsCaller();
            end
            
            % if this is a line, cache the ID
            if strcmpi(roiType,'line')
                obj.roiLastLine = roiID;
            end
            
            wasROIApplied = true;
        end
        
        function roiEOA_Listener(obj,~,~)
            % Updates the internal 'End Of Acqusition' (EOA) cache.
            
            if si_isAcquiring()
                return;
            end
            
            global state;
            
            % clear any existing cached data
            obj.roiLastAcqCache = struct();
            
            % Read motor position, cache as 'last acquired' motor position
            if ~state.motor.motorOn
                obj.roiLastAcqCache.position = [nan nan nan];
            else
                obj.roiLastAcqCache.position = motorGetPosition();
            end
            
            % Cache the RSPs that were used as 'last acquired' RSPs
            for i = 1:length(obj.scanParameterNames)
                obj.roiLastAcqCache.RSPs.(obj.scanParameterNames{i}) = state.acq.(obj.scanParameterNames{i});
            end
            
            % prepare the max projections, if necessary
            if obj.roiUseMIPForMRI && state.acq.numberOfZSlices > 1
                calculateMaxProjections();
            end
            
            % Cache the last acquired data – all channels.
            obj.roiLastAcqCache.MRI = cell(1,state.init.maximumNumberOfInputChannels);
            for i = 1:state.init.maximumNumberOfInputChannels
                if state.acq.imagingChannel(i)
                    if obj.roiUseMIPForMRI && state.acq.numberOfZSlices > 1 && state.acq.maxImage(i)
                        obj.roiLastAcqCache.MRI{i} = get(state.internal.maximagehandle(i),'CData');%state.acq.maxData(i);
                    else
                        obj.roiLastAcqCache.MRI{i} = get(state.internal.imagehandle(i),'CData');%state.acq.acquiredData{1}(i);
                    end
                end
            end
            
            % Cache the merged data
            if state.acq.channelMerge
                obj.roiLastAcqCache.MRI{5} = state.acq.acquiredDataMerged;
            end
            
            %Determining any matching ROIs (both 'abstract' and regular ROIs)
            [isDefined, definedROI, definedPosition,matchingAbstractROIs] = obj.isCurrentROIDefined();
            
            %Update MRIs of abstract ROIs with matching RSPs
            if ~isempty(matchingAbstractROIs)
                for i=1:length(matchingAbstractROIs)
                    s = obj.roiDataStructure(matchingAbstractROIs(i));
                    s.MRI = obj.roiLastAcqCache.MRI;
                    obj.roiDataStructure(matchingAbstractROIs(i)) = s;
                end
                
                if ~isempty(obj.shownROI) && ismember(obj.shownROI,matchingAbstractROIs)
                    obj.roiUpdateView();
                end
            end
            
            % If the current RSPs (and maybe Position) match an existing RDO, then:
            if isDefined && ~isempty(definedROI) && definedROI ~= obj.ROI_BASE_ID
                % Update stored MRI for that RDO to this recently acquired data
                roiStruct = obj.roiDataStructure(definedROI);
                roiStruct.MRI = obj.roiLastAcqCache.MRI;
                obj.roiDataStructure(definedROI) = roiStruct;
                
                % if this is a top-level ROI, display it--otherwise, display the parent...
                if roiStruct.parentROIID == obj.ROI_ROOT_ID
                    obj.shownROI = definedROI;
                else
                    obj.shownROI = roiStruct.parentROIID;
                end
                
                % cache the matched ROI
                obj.roiLastAcqCache.definedROI = definedROI;
                
                obj.roiUpdateView();
            else %no defined ROI matches current RSPs/position
                
                obj.roiLastAcqCache.definedROI = [];
                if ~isempty(definedPosition)
                    % we matched a position, but the RSPs didn't match; get this PDO's top-level RDO (if one exists).
                    
                    topLevelROIIDExisting = obj.roiGetTopLevelROIID(definedPosition,obj.roiLastAcqCache.RSPs.scanRotation);
                    if ~isempty(topLevelROIIDExisting)
                        % if the effective zoom of the last-acquired data is "more informational" than that of the top-level ROI,
                        % create a new top-level ROI (using the last-acq data) and re-link the existing ROI in the tree.
                        topLevelROIStruct = obj.roiDataStructure(topLevelROIIDExisting);
                        effectiveZoomCached = obj.roiLastAcqCache.RSPs.zoomFactor/(obj.roiLastAcqCache.RSPs.scanAngleMultiplierFast*obj.roiLastAcqCache.RSPs.scanAngleMultiplierSlow);
                        effectiveZoomExisting = topLevelROIStruct.RSPs.zoomFactor/(topLevelROIStruct.RSPs.scanAngleMultiplierFast*topLevelROIStruct.RSPs.scanAngleMultiplierSlow);
                        
                        if effectiveZoomCached < effectiveZoomExisting
                            topLevelROIIDNew = obj.roiAddNewFromAcqCache('positionID',definedPosition);
                            
                            % re-link the existing top-level ROI
                            obj.roiRemoveChildFromParent(topLevelROIIDExisting);
                            obj.roiAddChildToParent(topLevelROIIDNew,topLevelROIIDExisting)
                            
                            topLevelROIID = topLevelROIIDNew;
                        else
                            topLevelROIID = topLevelROIIDExisting;
                        end
                        
                        % SHOW the top-level ROI
                        %obj.shownROI = topLevelROIID;
                    end
                end
            end
        end
        
        function roiShowActive(obj)
            %Sets shownROI to activeROIID, if it exists
            
            if ~isempty(obj.activeROIID)
                obj.shownROI = obj.activeROIID;
            else
                obj.fullError('No active ROI','No activeROIID is currently specified. Ignoring command.')
            end
            
        end
        
    end
    
    
    %% DEVELOPER METHODS
    
    %Superclass overrides
    methods
        function tf = notify(obj,eventName,eventData)
            %Event notification method that appends supplied scimData struct to the event's eventData struct, supplied to the event listener(s)
            
            persistent hEventData
            
            if isempty(hEventData)
                hEventData = scanimage.EventData();
            end
            
            if nargin < 3
                hEventData.scimData = [];
            else
                hEventData.scimData = eventData;
            end
            
            notify@handle(obj,eventName,hEventData);
            
            if obj.listenerAbortFlag
                tf = false;
                obj.listenerAbortFlag = false;
            else
                tf = true;
            end
        end
    end
    
    methods  (Hidden)
        
        function hAx = zprvRoiAddBegin(obj,isCUR)
            % Handles necessary checks before adding a new ROI.
            %
            % isCur: a boolean that, if true, indicates that a new CUR ROI is being created.
            
            if nargin < 2 || isempty(isCUR)
                isCUR = false;
            end
            
            
            
            %Determine target figure
            if isCUR
                hAx = 1; % roiAddCurrent() expecting a non-empty value.
            else
                hAx = si_selectImageFigure();
                if isempty(hAx)
                    return;
                end
            end
            
            if ~isCUR && hAx ~= obj.hROIDisplayAx %an acquisition window
                % ensure we have EOA data
                if isempty(obj.roiLastAcqCache)
                    hAx = [];
                    obj.fullError('Cannot add ROI', 'Unable to create ROI: Data must be acquired before ROI selection can occur.');
                    return;
                end
                
                % make sure it makes sense to add a new ROI, given the RSPs of the last acquired scan
                lastAcqRSPs = obj.roiLastAcqCache.RSPs;
                lastAcqType = obj.computeROIType(lastAcqRSPs.scanAngleMultiplierFast,lastAcqRSPs.scanAngleMultiplierSlow);
                if ismember(lastAcqType,{'line' 'point'})
                    obj.fullError('Cannot add ROI', sprintf('Can only select child ROI from area-type ROI parent. Last acquisition was of ''%s'' type.',lastAcqType));
                    hAx = [];
                    return;
                end
            end
            
            % force the RDF to be visible % NOTE: this is to prevent a bug that seems to occur when modifying a figure's colormap when it isn't visible.
            set(obj.hROIDisplayFig,'Visible','on');
        end
        
        function zprvMacroError(obj,errString,varargin)
            if ~isempty(errString)
                setStatusString('Invalid Entry!');
                ME = MException('',errString,varargin{:});
                ME.throwAsCaller();
            end
        end
        
        function fsAngular = zprvRoiConvertPixels2Angle(obj,fsPixels,hAx,doUseCachedRSPs)
            % Converts pixel coordinates obtained within a particular ROI (at current shift, multiplier, zoom, and rotation) into angle coordinates
            %
            % fsPixels: an Mx2 array of pixel coordinates.
            % hAx: an optional argument specifying the axes.
            % doUseCachedRSPS: an optional boolean argument that, if true, specifies to use the scan parameters cached at last acquisition.
            %
            % fsAngular: the computed angular coordinates
            %
            
            global state
            
            scanAngularRangeReference = [state.init.scanAngularRangeReferenceFast state.init.scanAngularRangeReferenceSlow];
            
            if nargin < 3 || isempty(hAx) %Use current RSPs (case reached for adding Grid ROIs)
                sizeImage = [state.acq.pixelsPerLine  state.internal.storedLinesPerFrame];
                
                scanShift = [state.acq.scanShiftFast state.acq.scanShiftSlow];
                scanAngleMultiplier = [state.acq.scanAngleMultiplierFast state.acq.scanAngleMultiplierSlow];
                zoomFactor = state.acq.zoomFactor;
                scanRotation = state.acq.scanRotation;
            else
                if nargin < 4 || isempty(doUseCachedRSPs)
                    doUseCachedRSPs = false;
                end
                
                sizeImage = [diff(get(hAx,'XLim')) diff(get(hAx,'YLim'))];
                
                if hAx == obj.hROIDisplayAx
                    % if the coords came from the RDF, use the shown ROI's params
                    shownROIStruct = obj.roiDataStructure(obj.shownROI);
                    scanShift = [shownROIStruct.RSPs.scanShiftFast shownROIStruct.RSPs.scanShiftSlow];
                    scanAngleMultiplier = [shownROIStruct.RSPs.scanAngleMultiplierFast shownROIStruct.RSPs.scanAngleMultiplierSlow];
                    zoomFactor = shownROIStruct.RSPs.zoomFactor;
                    scanRotation = shownROIStruct.RSPs.scanRotation;
                elseif doUseCachedRSPs
                    scanShift = [obj.roiLastAcqCache.RSPs.scanShiftFast obj.roiLastAcqCache.RSPs.scanShiftSlow];
                    scanAngleMultiplier = [obj.roiLastAcqCache.RSPs.scanAngleMultiplierFast obj.roiLastAcqCache.RSPs.scanAngleMultiplierSlow];
                    zoomFactor = obj.roiLastAcqCache.RSPs.zoomFactor;
                    scanRotation = obj.roiLastAcqCache.RSPs.scanRotation;
                end
            end
            
            m = size(fsPixels,1);
            fsAngular = zeros(m,2);
            
            R = [cosd(scanRotation) -sind(scanRotation); sind(scanRotation) cosd(scanRotation)];
            
            for i = 1:m
                %Rotate coordinates clockwise, i.e. moving coordinates ccw back to reference orientation
                %VVV: Should scanAngularRangeReference actually be lumped in with in scanAngleMultiplier, as done now?
                fsNormalized = fsPixels(i,:)./sizeImage - 0.5;
                fsAngular(i,:) = scanShift + ((fsNormalized.*scanAngleMultiplier.*scanAngularRangeReference)/zoomFactor)*R;
            end
        end
        
        function positionID = zprvDisplayedPosn2PositionID(obj)
            % Determines if the current motor position matches any of the defined Position IDs.
            % NOTE: Assumes motor position has already/recently been read
            %
            % isDefined: a boolean that, if true, indicates that the current motor position is defined.
            % definedPosition: the ID of the defined Position, if one exists.
            
            global state;
            
            if ~state.motor.motorOn
                positionID = 0;
                return;
            end
            
            currentPos = [state.motor.absXPosition state.motor.absYPosition state.motor.absZPosition];
            if state.motor.dimensionsXYZZ
                currentPos(end+1) = state.motor.absZZPosition;
            end
            positionID = obj.zprvPosn2PositionID(currentPos);
        end
        
        function [isDefined, definedROI, definedPosition, matchingAbstractROIs] = isCurrentROIDefined(obj)
            % Determines if the current RSPs (and possibly position) match an existing ROI (or PDO).
            %
            % isDefined: a boolean that, if true, indicates an existing ROI matches the current RSPs.
            % definedROI: the integer ID of the existing ROI (if one exists).
            % definedPosition: the integer ID of the existing Position (if one exists).
            % matchingAbstractROIs: list of ROI IDs which have an abstract (negative) Position ID and whose RSPs match the current RSPs
            
            global state
            
            isDefined = false;
            definedROI = [];
            
            %First determine any 'abstract' ROI IDs (ROIs with abstract positions) whose RSPs match
            matchingAbstractROIs = [];
            topLevelROIs = obj.roiDataStructure(obj.ROI_ROOT_ID).children;
            rspStructCurrent = struct();
            for i = 1:length(obj.scanParameterNames)
                rspStructCurrent.(obj.scanParameterNames{i}) = state.acq.(obj.scanParameterNames{i});
            end
            
            for i=1:length(topLevelROIs)
                currROI = obj.roiDataStructure(topLevelROIs(i));
                if currROI.positionID < 0 && obj.roiIsEqualRSPs(currROI.RSPs,rspStructCurrent)
                    matchingAbstractROIs(end+1) = topLevelROIs(i);  %#ok<AGROW>
                end
            end
            
            % first determine if this position is defined
            definedPosition = obj.zprvDisplayedPosn2PositionID();
            if definedPosition == 0
                return;
            end
            
            topLevelROI = obj.roiGetTopLevelROIID(definedPosition,state.acq.scanRotation);
            if isempty(topLevelROI)
                return;
            end
            
            % start at the top-level ROI and walk down, looking for a matching ROI
            [isDefined, definedROI] = obj.roiDoCurrentRSPsMatchExistingROI(topLevelROI,rspStructCurrent);
        end
        
        function [doesMatch, matchingROI] = roiDoCurrentRSPsMatchExistingROI(obj,existingROIID,rspStructCurrent)
            % Determines if the current scan-parameters match the given ROI (or, if not, any of its descendants).
            %
            % existingROIID: the integer ID of the ROI to be checked against.
            % rspStructCurrent: <OPTIONAL> structure of current RSPs. If omitted, it is determined within this method.
            %
            % doesMatch: a boolean that, if true, indicates that the RSPs match the given ROI.
            % matchingROI: the integer ID of the matching ROI (if one exists).
            
            global state;
            
            doesMatch = false;
            matchingROI = [];
            
            if nargin < 2 || isempty(existingROIID) || ~obj.roiDataStructure.isKey(existingROIID) || ~isfield(obj.roiDataStructure(existingROIID),'RSPs')
                return;
            end
            
            if nargin < 3 || isempty(rspStructCurrent)
                % construct a struct out of the current RSPs
                rspStructCurrent = struct();
                for i = 1:length(obj.scanParameterNames)
                    rspStructCurrent.(obj.scanParameterNames{i}) = state.acq.(obj.scanParameterNames{i});
                end
            end
            
            roiStruct = obj.roiDataStructure(existingROIID);
            rspStructExisting = roiStruct.RSPs;
            
            
            
            doAllRSPsMatch = obj.roiIsEqualRSPs(rspStructExisting,rspStructCurrent);
            
            if doAllRSPsMatch
                doesMatch = true;
                matchingROI = existingROIID;
            elseif isfield(roiStruct,'children')
                % if we don't have a match, check this ROI's children
                children = roiStruct.children;
                for i = 1:length(children)
                    [doesMatch, matchingROI] = obj.roiDoCurrentRSPsMatchExistingROI(children(i));
                    if doesMatch
                        return;
                    end
                end
            end
        end
        
        function positionID = zprvPosn2PositionID(obj, posnVector)
            % Returns positionID matching the supplied posnVector (to within current position tolerances). positionID 0 is returned if no coordinate match is found.
            
            global state;
            
            positionID = 0;
            
            % iterate over all Position entries with positionID > 0
            keys = cell2mat(state.hSI.positionDataStructure.keys());
            keys(keys <=0) = []; %Remove root & abstract position IDs
            
            matchingPosnIDs = [];
            for i = 1:length(keys)
                currentStruct = obj.positionDataStructure(keys(i));
                currentVector = [currentStruct.motorX currentStruct.motorY currentStruct.motorZ];
                if state.motor.dimensionsXYZZ
                    if state.hSI.posnIgnoreSecZ
                        currentVector(4) = nan;
                    else
                        currentVector(4) = currentStruct.motorZZ;
                    end
                end
                
                
                if obj.roiIsEqualPosition(posnVector,currentVector)
                    if ~state.motor.dimensionsXYZZ || ~state.hSI.posnIgnoreSecZ
                        positionID = keys(i);
                        return;
                    else
                        matchingPosnIDs(end+1) = keys(i); %#ok<AGROW>
                    end
                end
            end
            
            closestZZ = inf;
            closestZZIdx = [];
            if ~isempty(matchingPosnIDs)
                for i=1:length(matchingPosnIDs)
                    currentStruct = obj.positionDataStructure(matchingPosnIDs(i));
                    currentVector = [currentStruct.motorX currentStruct.motorY currentStruct.motorZ currentStruct.motorZZ];
                    
                    zzDiff =  abs(currentVector(4) - posnVector(4));
                    if zzDiff < closestZZ
                        closestZZ = zzDiff;
                        closestZZIdx = i;
                    end
                end
                
                positionID = matchingPosnIDs(closestZZIdx);
            end
            
        end
        
        function roiID = roiAddNew(obj,roiStruct,roiID,hAx,doSuppressAutoActions)
            % Adds a new ROI.
            %
            % roiStruct: Structure containing one or more of the fields: {'zoomFactor' 'scanShiftFast' 'scanShiftSlow' 'scanRotation' 'scanAngularRangeFast' 'scanAngularRangeSlow'}
            % roiID: an optional ID to use for this ROI (useful when loading from a file).
            % hAx: a valid handle to the Axes on which this ROI was 'drawn'
            % doSuppressAutoActions: a boolean that, if true, indicates to ignore 'roiGotoOnAdd' and 'roiSnapOnAdd'
            
            global state
            
            if nargin < 2 || isempty(roiStruct)
                % no arguments means user clicked 'CUR': fill the empty roiStruct with current scan values
                roiStruct = struct();
                isCUR = true;
            else
                if isfield(roiStruct,'RSPs')
                    isCUR = false;
                else
                    isCUR = true;
                end
            end
            
            if nargin < 3 || isempty(roiID)
                roiID = [];
            end
            
            if nargin < 4 || isempty(hAx)
                hAx = [];
            else
                if hAx == obj.hROIDisplayAx && (isempty(obj.shownROI) || obj.shownROI < 1)
                    obj.fullError('Cannot add ROI','Unable to add ROI: no shown ROI.');
                    return;
                end
            end
            
            if nargin < 5 || isempty(doSuppressAutoActions)
                doSuppressAutoActions = false;
            end
            
            childROIID = [];
            % determine the target parent
            if ~isfield(roiStruct,'parentROIID')
                if isfield(roiStruct,'positionID') %Occurs when adding a grid
                    roiStruct.parentROIID = obj.roiGetTopLevelROIID(roiStruct.positionID,roiStruct.RSPs.scanRotation); %VVV: This seems like it should always return empty -- the newly added positionID to serve as parent of grid does not yet have an associated ROI
                    if isempty(roiStruct.parentROIID)
                        roiStruct.parentROIID = obj.ROI_ROOT_ID;
                    end
                else
                    if nargin < 4
                        hAx = [];
                    end
                    
                    [parentROIID, childROIID] = obj.roiEnsureHierarchy(hAx);
                    if isempty(parentROIID)
                        return;
                    else
                        roiStruct.parentROIID = parentROIID;
                    end
                end
            end
            
            % Update roiType -- based on the RSPs
            if ~isfield(roiStruct,'RSPs') || ~isfield(roiStruct.RSPs,'scanAngleMultiplierFast') || ~isfield(roiStruct.RSPs,'scanAngleMultiplierSlow')
                roiStruct.type = obj.computeROIType(state.acq.scanAngleMultiplierFast,state.acq.scanAngleMultiplierSlow);
            else
                newROIType = obj.computeROIType(roiStruct.RSPs.scanAngleMultiplierFast,roiStruct.RSPs.scanAngleMultiplierSlow);
                
                if isfield(roiStruct,'type') && ~isempty(roiStruct.type)
                    assert(strcmpi(roiStruct.type,newROIType),'Supplied roiStruct value has inconsistent ''type'' and ''scanAngleMultiplier'' values specified');
                else
                    roiStruct.type = newROIType;
                end
            end
            
            % if this is a Line, cache this ROI as the 'last line'
            if strcmpi(roiStruct.type,'line')
                obj.roiLastLine = roiID;
            end
            
            % Fill in any fields unspecified in roiStruct
            roiFields = obj.scanParameterNames;
            if strcmp(roiStruct.type,'point')
                roiFields = setdiff(roiFields, 'zoomFactor');
                roiStruct.RSPs.zoomFactor = 1;
            end
            for i=1:length(roiFields)
                if (~isfield(roiStruct,'RSPs') || ~isfield(roiStruct.RSPs,roiFields{i}))
                    if isfield(obj.roiLastAcqCache,'RSPs') && isfield(obj.roiLastAcqCache.RSPs,roiFields{i})
                        roiStruct.RSPs.(roiFields{i}) = obj.roiLastAcqCache.RSPs.(roiFields{i});
                    else
                        %VVV052512: Do we ever want/need to do this??
                        roiStruct.RSPs.(roiFields{i}) = state.acq.(roiFields{i});
                    end
                end
            end
            
            % Determine the new roiID value:
            if isempty(roiID)
                existingIDs = obj.roiDataStructure.keys();
                existingIDs([existingIDs{:}] < 1) = [];
                if isempty(existingIDs)
                    roiID = 1;
                else
                    roiID = max([existingIDs{:}]) + 1;
                end
            end
            
            % CUR ROIs need some extra attention...
            if isCUR
                if roiStruct.parentROIID == obj.ROI_ROOT_ID
                    roiStruct.positionID = obj.activePositionID;
                end
                
                roiStruct.MRI = obj.roiLastAcqCache.MRI;
                
                % update the EOA cache to reflect the addition of this ROI
                obj.roiLastAcqCache.definedROI = roiID;
            end
            
            % insert our struct into the master map
            obj.roiDataStructure(roiID) = roiStruct;
            
            % update the parent's list of children
            obj.roiAddChildToParent(roiStruct.parentROIID, roiID);
            
            % re-link existing ROI, if necessary
            if ~isempty(childROIID)
                obj.roiAddChildToParent(roiID,childROIID);
            end
            
            obj.roiSuppressUpdates = true;
            
            % update the shown ROI
            obj.shownROI = roiStruct.parentROIID;
            
            if obj.roiGotoOnAdd && ~doSuppressAutoActions && ~obj.roiLoading
                obj.roiGotoROI(roiID);
                
                % only take a snapshot if 'goto-on-add' is enabled
                if obj.roiSnapOnAdd
                    snapShot();
                    while state.internal.snapping
                        pause(0.1);
                    end
                end
            end
            
            obj.roiSuppressUpdates = false;
            obj.roiUpdateROITable();
            obj.roiUpdateView();
        end
        
        function roiID = roiAddNewFromAcqCache(obj,varargin)
            % Adds a new ROI using the RSPs cached from the last acquisition.
            %
            % varargin: an optional list of key/val pairs specifying additional struct fields to add.
            %
            
            if nargin > 1
                propMap = obj.extractPropValArgMap(varargin,{'parentROIID','positionID'});
                propKeys = propMap.keys;
            else
                propKeys = [];
            end
            
            roiID = [];
            
            %Ensure that ROI can be created
            if isempty(obj.roiLastAcqCache) || ~isfield(obj.roiLastAcqCache,'RSPs')
                obj.consoleError('Can''t create ROI; no existing acquisition data.');
                return;
            end
            
            if obj.roiLastAcqCache.RSPs.scanAngleMultiplierFast == 0 || obj.roiLastAcqCache.RSPs.scanAngleMultiplierSlow == 0
                obj.consoleError('Can''t add ROI under current scan parameters.');
                return;
            end
            
            %Create the roiStruct to add
            roiStruct = struct('RSPs',obj.roiLastAcqCache.RSPs,'MRI',{obj.roiLastAcqCache.MRI(:)'}); % DEQ20110811 - not sure about this goofy array indexing...but it works
            
            % assign all optional args
            for i = 1:length(propKeys)
                roiStruct.(propKeys{i}) = propMap(propKeys{i});
            end
            
            % if we have a position ID, but no parent ID, force this new ROI to be top-level
            if isfield(roiStruct,'positionID') && ~isfield(roiStruct,'parentROIID')
                roiStruct.parentROIID = obj.ROI_ROOT_ID;
            end
            
            roiID = obj.roiAddNew(roiStruct,[],[],true);
        end
        
        function roiAddChildToParent(obj, parentROIID, childROIID)
            % Assigns a parent/child hierarchy between the two given ROIs.
            %
            % parentROIID: a valid ROI ID representing the parent.
            % childROIID: a valid ROI ID representing the child to be added.
            %
            
            % update the parent's 'children' field, adding the new child ID
            if isfield(obj.roiDataStructure(parentROIID),'children')
                children = obj.roiDataStructure(parentROIID).children;
            else
                children =[];
            end
            parentROIStruct = obj.roiDataStructure(parentROIID);
            parentROIStruct.children = [children childROIID];
            obj.roiDataStructure(parentROIID) = parentROIStruct;
            
            % update the child's parent field
            childROIStruct = obj.roiDataStructure(childROIID);
            childROIStruct.parentROIID = parentROIID;
            
            % if the child has a position reference, but is not a direct
            % child of ROOT, clear the position reference.
            if isfield(childROIStruct, 'positionID') && parentROIID ~= obj.ROI_ROOT_ID
                childROIStruct = rmfield(childROIStruct,'positionID');
            end
            
            obj.roiDataStructure(childROIID) = childROIStruct;
        end
        
        function roiRemoveChildFromParent(obj, childROIID)
            % Deletes the parent/child hierarchy for the given ROI
            %
            % parentROIID: a valid ROI ID representing the parent.
            % childROIID: a valid ROI ID representing the child to be removed.
            
            existingChildROIStruct = obj.roiDataStructure(childROIID);
            existingParentROIID = existingChildROIStruct.parentROIID;
            existingChildROIStruct.parentROIID = []; % NOTE: this is now a dangling ROI--the user is responsible for re-linking.
            obj.roiDataStructure(childROIID) = existingChildROIStruct;
            
            parentROIStruct = obj.roiDataStructure(existingParentROIID);
            parentROIStruct.children(parentROIStruct.children == childROIID) = [];
            obj.roiDataStructure(existingParentROIID) = parentROIStruct;
        end
        
        function roiID = roiFindMatchingROIAtPosition(obj, posnID, rspStruct)
            % Returns the ROI ID matching the given RSPs at the given position.
            %
            % posnID: the position ID to search at
            % rspStruct: an RDO struct
            %
            % roiID: a valid ROI ID matching 'rspStruct', (empty if no matching ROI exists).
            
            roiID = [];
            
            topLevelROIIDs = obj.roiGetTopLevelROIID(posnID,[]);
            if isempty(topLevelROIIDs)
                return;
            end
            
            % starting at 'topLevelROIID', walk down all branches of the tree, looking for the first matching node.
            for i=1:length(topLevelROIIDs)
                roiID = obj.roiFindMatchingRSPs(topLevelROIIDs(i),rspStruct);
                if ~isempty(roiID)
                    break;
                end
            end
        end
        
        function roiID = roiFindMatchingRSPs(obj, startingROIID, rspStruct)
            
            roiID = [];
            
            startingROIStruct = obj.roiDataStructure(startingROIID);
            if isfield(startingROIStruct,'children')
                children = startingROIStruct.children(startingROIStruct.children > obj.ROI_BASE_ID);
            else
                children = []; % causes execution to fall through if RSP test fails
            end
            
            candidates = [startingROIID children];
            
            % go through all of the top-level ROI's children, returning
            % the first one to match 'rspStruct'
            for i = 1:length(candidates)
                if obj.roiIsEqualRSPs(rspStruct,obj.roiDataStructure(candidates(i)).RSPs)
                    roiID = candidates(i);
                    return;
                elseif isempty(candidates)
                    return;
                elseif candidates(i) ~= startingROIID
                    roiID = obj.roiFindMatchingRSPs(candidates(i),rspStruct);
                    if ~isempty(roiID)
                        return;
                    end
                end
            end
        end
        
        function childrenROIIDs = roiGetDisplayedChildren(obj)
            % Returns a list of descendants of the currently shown ROI, given the current display depth.
            %
            % childrenROIIDs: a list of descendant ROI IDs down to the current display depth.
            
            % construct a list of children, dependant on the current ROI Display Depth.
            childrenROIIDs = [];
            currentLevelIDs = obj.shownROI;
            % iterate to the specified display depth
            for i = 1:min(999,obj.roiDisplayDepth + 1) % DEQ20110830: I think it's safe to assume no hierarchy will be deeper than 999 levels...
                nextLevelIDs = [];
                for j = 1:length(currentLevelIDs)
                    % add the current child to the master list of children
                    childrenROIIDs = [childrenROIIDs currentLevelIDs(j)];
                    
                    % construct a list of the next level of IDs
                    currentChildStruct = obj.roiDataStructure(currentLevelIDs(j));
                    if ~isfield(currentChildStruct,'children') || isempty(currentChildStruct.children)
                        continue;
                    end
                    nextLevelIDs = [nextLevelIDs currentChildStruct.children];
                end
                currentLevelIDs = nextLevelIDs;
                if isempty(nextLevelIDs)
                    break;
                end
            end
            childrenROIIDs(childrenROIIDs == obj.shownROI) = [];
            childrenROIIDs(childrenROIIDs == obj.ROI_ROOT_ID) = [];
        end
        
        function [ancestorROIID  ancestorList] = roiGetOldestAncestor(obj, descendantROIID)
            % Walks up a branch of the ROI tree, returning the oldest ancestor of the given descendant ROI.
            
            ancestorROIID = [];
            ancestorList = [];
            
            if ~obj.roiDataStructure.isKey(descendantROIID)
                return;
            end
            
            descendantROIStruct = obj.roiDataStructure(descendantROIID);
            
            if ~isfield(descendantROIStruct,'parentROIID')
                ancestorROIID = obj.ROI_ROOT_ID;
                return;
            elseif descendantROIStruct.parentROIID == obj.ROI_ROOT_ID
                ancestorROIID = descendantROIID;
                return;
            end
            
            [parentROIID,nextParentROIID] = deal(descendantROIStruct.parentROIID);
            
            while nextParentROIID > obj.ROI_ROOT_ID
                parentROIID = nextParentROIID;
                nextParentROIID = obj.roiDataStructure(nextParentROIID).parentROIID;
                if nargout == 2
                    ancestorList = fliplr([fliplr(ancestorList) parentROIID]);
                end
            end
            
            ancestorROIID = parentROIID;
        end
        
        function positionID = roiGetPositionFromROIID(obj,roiID)
            % Returns the position associated with a given ROI.
            %
            % roiID: the RDO to return the position of.
            % positionID: the ID of the associated PDO. Value will be 0 if roiID is ROOT/BASE, or is some other abstract ROI (e.g. for grids).
            
            if nargin < 2
                error('Specify an ROI.');
            elseif isempty(roiID)
                positionID = [];
                return;
            end
            
            if roiID == obj.ROI_ROOT_ID
                positionID = 0;
                return;
            end
            
            roiStruct = obj.roiDataStructure(roiID);
            if roiStruct.parentROIID == obj.ROI_ROOT_ID && isfield(roiStruct,'positionID')
                positionID = roiStruct.positionID;
            else
                ancestorStruct = obj.roiDataStructure(obj.roiGetOldestAncestor(roiID));
                positionID = ancestorStruct.positionID;
            end
        end
        
        function isEqual = roiIsEqualPosition(obj,posnA,posnB)
            % Returns true if the given positions match (within the defined tolerances).
            %
            % posnA, posnB: valid position vectors.
            % isEqual: a boolean that, if true, indicates that the positions match.
            %
            % NaN values in either/both position vectors indicate to ignore that dimension
            
            
            if nargin < 3 || isempty(posnB) || nargin < 2 || isempty(posnA)
                error('Specify two valid Position IDs.');
            end
            
            global state;
            
            % pad the vectors to a length of 4
            posnA = [posnA nan(1,4-length(posnA))];
            posnB = [posnB nan(1,4-length(posnA))];
            
            tolerances = [obj.roiPositionToleranceX obj.roiPositionToleranceY obj.roiPositionToleranceZ];
            positionSuffixes = {'X' 'Y' 'Z'};
            if state.motor.dimensionsXYZZ
                positionSuffixes{end+1} = 'ZZ';
                tolerances = [tolerances obj.roiPositionToleranceZZ];
            end
            
            valid = false(1,length(positionSuffixes));
            
            % iterate over all dimensions
            for j = 1:length(positionSuffixes)
                
                % Ignore dimension if either value is NaN
                if  (isnan(posnA(j)) || isnan(posnB(j)))
                    valid(j) = true;
                    continue;
                end
                
                if posnA(j) - tolerances(j) <= posnB(j) && posnB(j) <= posnA(j) + tolerances(j)
                    valid(j) = true;
                else
                    isEqual = false;
                    return;
                end
            end
            
            if all(valid)
                isEqual = true;
            end
        end
        
        function isEqual = roiIsEqualRSPs(obj,rspStructA,rspStructB,ignoreScanRotation)
            % Tests the two given RSPs structs for equality.
            %
            % rspStructA: a valid ROI RSP structure.
            % rspStructB: a valid ROI RSP structure.
            % ignoreScanRotation: Logical, Default=false; if true, ignore scanRotation in comparison.
            %
            % isEqual: a boolean that, if true, indicates the two RSP structs have matching field values.
            
            isEqual = false;
            
            if nargin < 3 || isempty(rspStructA) || isempty(rspStructB)
                obj.fullError('','Please specify two valid RSP structs.');
                return;
            end
            
            if nargin < 4 ||  isempty(ignoreScanRotation)
                ignoreScanRotation = false;
            end
            
            if length(fieldnames(rspStructA)) ~= length(obj.scanParameterNames) || length(fieldnames(rspStructB)) ~= length(obj.scanParameterNames)
                obj.fullError('','Invalid RSP structure.');
                return;
            end
            
            % iterate through all fields, testing for equality.
            for i = 1:length(obj.scanParameterNames)
                fieldName = obj.scanParameterNames{i};
                if ~isfield(rspStructB,fieldName) || rspStructA.(fieldName) ~= rspStructB.(fieldName)
                    if ~strcmp(fieldName,'scanRotation') || ~ignoreScanRotation
                        return;
                    end
                end
            end
            
            isEqual = true;
            
        end
        
        function isValid = roiIsValidPosition(obj,posnID)
            if obj.positionDataStructure.isKey(posnID)
                isValid = true;
            else
                isValid = false;
            end
        end
        
        function isValid = roiIsValidROI(obj,roiID)
            if obj.roiDataStructure.isKey(roiID)
                isValid = true;
            else
                isValid = false;
            end
        end
        
        function roiClearAll(obj)
            % Removes all defined ROIs.
            
            rootROIStruct = obj.roiDataStructure(obj.ROI_ROOT_ID);
            
            hWaitbar = waitbar(0,'Clearing ROIs...');
            
            % remove all top-level ROIs (roiRemoveROI() will take care of deleting any children)
            topLevelROIIDs = rootROIStruct.children;
            topLevelROIIDs(topLevelROIIDs == obj.ROI_BASE_ID) = []; % don't delete the Base ROI
            numROIs = length(topLevelROIIDs);
            obj.roiSuppressUpdates = true;
            for i = 1:numROIs
                waitbar(i/numROIs,hWaitbar);
                obj.roiRemoveROI(topLevelROIIDs(i));
            end
            close(hWaitbar);
            
            obj.shownROI = obj.ROI_ROOT_ID;
            obj.roiSuppressUpdates = false;
            
            obj.hController{1}.disableROIControlButtons();
            obj.hController{1}.updateTableViews({'roi' 'cycle'});
            
            obj.roiUpdateView();
        end
        
        function roiClearShown(obj)
            % Clears all children of the currently displayed ROI.
            
            if isempty(obj.shownROI)
                return;
            end
            
            displayedROIStruct = obj.roiDataStructure(obj.shownROI);
            if ~isfield(displayedROIStruct,'children') || isempty(displayedROIStruct.children)
                return;
            end
            children = displayedROIStruct.children(displayedROIStruct.children ~= obj.ROI_BASE_ID);
            
            obj.roiSuppressUpdates = true;
            hWaitbar = waitbar(0,'Clearing ROIs...');
            for i = 1:length(children)
                waitbar(i/length(children),hWaitbar);
                obj.roiRemoveROI(children(i));
            end
            close(hWaitbar);
            obj.roiSuppressUpdates = false;
            obj.roiUpdateROITable();
        end
        
        function roiClearAllPositions(obj)
            % Removes all defined Positions (and, as a result, all ROIs).
            
            positions = obj.positionDataStructure.keys();
            positions([positions{:}] == 0) = [];
            numPositions = length(positions);
            
            obj.roiSuppressUpdates = true;
            hWaitbar = waitbar(0,'Clearing Positions...');
            for i = 1:numPositions
                waitbar(i/numPositions,hWaitbar);
                obj.roiRemovePosition(positions{i});
            end
            close(hWaitbar);
            obj.roiSuppressUpdates = false;
            
            obj.hController{1}.disableROIControlButtons();
            obj.hController{1}.updateTableViews();
        end
        
        function cdata = roiDrawCheckeredBackground(obj, dims, doAdjustForSAM)
            % Returns an image (cdata) containing a checkered background.
            %
            % dims: a vector ([m,n,c]) specifying the size of the image.  NOTE: if unspecified, the current acqusition size will be used.
            % doAdjustForSAM: a boolean that, if true, specifies to factor in the current scanAngleMultiplier values (so the checker displays
            %   correctly in rectangular acquisition windows).
            
            global state;
            
            if nargin < 3 || isempty(doAdjustForSAM)
                doAdjustForSAM = false;
            end
            
            if nargin < 2 || isempty(dims)
                dims = [state.acq.linesPerFrame state.acq.pixelsPerLine 1];
            else
                if length(dims) < 3
                    dims = [dims 1];
                end
            end
            
            % adjust for scanAngleMultiplierXXX if necessary (i.e. for acquisition windows)
            if doAdjustForSAM && (state.acq.scanAngleMultiplierSlow ~= 0 || state.acq.scanAngleMultiplierFast ~= 0)
                samFactor = state.acq.scanAngleMultiplierFast/state.acq.scanAngleMultiplierSlow;
                if samFactor == 0 || samFactor == Inf
                    samFactor = 1;
                end
            else
                samFactor = 1;
            end
            
            w = round(max(dims(2)/32,1));
            if samFactor < 1
                w = round(w/samFactor);
            end
            h = round(w*(dims(1)/dims(2))*samFactor);
            
            if dims(3) == 3
                bgColor = 0.8;
                fgColor = 0.95;
            else
                fgColor = 255;
                bgColor = 0;
            end
            
            checkerPattern = [repmat(bgColor,h,w) repmat(fgColor,h,w); repmat(fgColor,h,w) repmat(bgColor,h,w)];
            checkerPatternSize = [ceil(max(dims(1)/size(checkerPattern,1),1)) ceil(max(dims(2)/size(checkerPattern,2),1))];
            
            % make the checkerboard bigger than we need, if necessary...
            for i = 1:dims(3)
                cdata(:,:,i) = zeros(checkerPatternSize(1)*h*2,checkerPatternSize(2)*w*2);
            end
            
            cdata(:,:,1) = repmat(checkerPattern, checkerPatternSize(1), checkerPatternSize(2));
            
            for i = 2:dims(3)
                cdata(:,:,i) = cdata(:,:,1);
            end
            
            % ...and finally, truncate the data to the proper size.
            cdata = cdata(1:dims(1),1:dims(2),:);
        end
        
        function roiDrawMarker(obj,roiID)
            % Draws an ROI marker for the given roiID.
            % If a marker already exists for the ID, it is deleted and redrawn using the current scan parameters.
            %
            % roiID: the ID of the ROI to draw.
            %
            % TODO: this could probably be refactored a bit to clean things up.
            
            global state;
            
            roiStruct = obj.roiDataStructure(roiID);
            roiType = roiStruct.type;
            
            if ~isfield(roiStruct,'RSPs')
                return;
            end
            
            % delete any existing marker
            if isfield(roiStruct, 'hMarker') && ~isempty(roiStruct.hMarker)
                delete(roiStruct.hMarker);
                roiStruct.hMarker = [];
            end
            if isfield(roiStruct, 'hMarkerLabel') && ~isempty(roiStruct.hMarkerLabel)
                delete(roiStruct.hMarkerLabel);
                roiStruct.hMarkerLabel = [];
            end
            
            normalizedCoords = obj.roiCalculateNormalizedCoords(roiStruct);
            
            % compensate for the size of the roiDisplayGUI (or, more specifically, the size of the axes)
            xLim = get(obj.hROIDisplayAx,'XLim');
            yLim = get(obj.hROIDisplayAx,'YLim');
            sizeImage = [diff(xLim) diff(yLim)];
            
            posX1 = (normalizedCoords(1,1) + 0.5) * sizeImage(1);
            posY1 = (normalizedCoords(1,2) + 0.5) * sizeImage(2);
            
            if strcmpi(roiType,'line')
                posX2 = (normalizedCoords(2,1) + 0.5) * sizeImage(1);
                posY2 = (normalizedCoords(2,2) + 0.5) * sizeImage(2);
            else
                posX2 = normalizedCoords(2,1) * sizeImage(1);
                posY2 = normalizedCoords(2,2) * sizeImage(2);
            end
            
            % allow 'active' or 'selected' designations to supercede default color
            if ~isempty(obj.selectedROIID) && obj.selectedROIID == roiID
                markerType = 'selected';
            elseif ~isempty(obj.activeROIID) && obj.activeROIID == roiID
                markerType = 'active';
            else
                markerType = roiType;
            end
            
            EDGE_PADDING = 10;
            
            switch roiType
                case 'point'
                    % if we're near the right edge, switch the label to the left
                    flip = false;
                    if posX1 + EDGE_PADDING > sizeImage(2)
                        markerPosX = posX1 - 12;
                        flip = true;
                    else
                        markerPosX = posX1 + 7;
                    end
                    
                    % if we're near the bottom edge, switch the label to the top
                    if posY1 + EDGE_PADDING > sizeImage(1) || flip
                        markerPosY = posY1 - 7;
                        if ~flip
                            markerPosX = posX1 - 12;
                        end
                    else
                        markerPosY = posY1 + 7;
                    end
                    
                    shapeFun = @rectangle;
                    shapeArgs = {'Parent',obj.hROIDisplayAx,'Position',[posX1 posY1 posX2 posY2],'EdgeColor',obj.markerColors(markerType),'LineWidth',1,'Curvature',[1 1]};
                    
                case 'line'
                    % if we're near the right edge, switch the label to the left
                    flip = false;
                    if posX2 + EDGE_PADDING > sizeImage(2)
                        markerPosX = posX1 - 10;
                        flip = true;
                    else
                        markerPosX = posX2 + 4;
                    end
                    
                    % if we're near the bottom edge, switch the label to the top
                    if posY2 + EDGE_PADDING > sizeImage(1) || flip
                        markerPosY = posY1 - 7;
                        if ~flip
                            markerPosX = posX1 - 10;
                        end
                    else
                        markerPosY = posY2 + 2;
                    end
                    
                    shapeFun = @line;
                    shapeArgs = {'Parent',obj.hROIDisplayAx,'XData',[posX1 posX2],'YData',[posY1 posY2],'Color',obj.markerColors(markerType)};
                    
                case {'square' 'rect'}
                    % scale the normalized coords to the axes size
                    normalizedCoords(:,1) = round(sizeImage(1).*(normalizedCoords(:,1) + 0.5));
                    normalizedCoords(:,2) = round(sizeImage(2).*(normalizedCoords(:,2) + 0.5));
                    normalizedCoords(5,:) = normalizedCoords(1,:); % so we can wrap around with our for-loop below...
                    
                    % place the label on the opposite corner of whichever quadrant the quad is in--
                    % this does a decent job of keeping the labels visible...
                    shiftFast = ((roiStruct.RSPs.scanShiftFast/state.init.scanAngularRangeReferenceFast) + 0.5)*sizeImage(2);
                    shiftSlow = ((roiStruct.RSPs.scanShiftSlow/state.init.scanAngularRangeReferenceSlow) + 0.5)*sizeImage(1);
                    if shiftFast >= sizeImage(2)/2
                        markerPosX = normalizedCoords(1,1) - 1.5*EDGE_PADDING; % left edge
                    else
                        markerPosX = normalizedCoords(2,1) + EDGE_PADDING; % right edge
                    end
                    if shiftSlow >= sizeImage(1)/2
                        markerPosY = normalizedCoords(1,2); % top edge
                    else
                        markerPosY = normalizedCoords(4,2); % bottom edge
                    end
            end
            
            % draw the shape...
            if ismember(roiType,{'square' 'rect'})
                hMarker = zeros(1,4);
                % draw four lines to make a quad
                for i = 1:4
                    xData = normalizedCoords(i:i+1,1)';
                    yData = normalizedCoords(i:i+1,2)';
                    hMarker(i) = line('Parent',obj.hROIDisplayAx,'XData',xData,'YData',yData,'Color',obj.markerColors(markerType));
                end
            else
                hMarker = feval(shapeFun,shapeArgs{:});
            end
            
            % determine marker parameters
            if obj.roiShowMarkerNumbers
                isIDVisible = 'on';
            else
                isIDVisible = 'off';
            end
            
            % create the marker label
            hMarkerLabel = text('Parent',obj.hROIDisplayAx,'Position',[markerPosX markerPosY],'String',['#' num2str(roiID)],'FontSize',8,'Color',obj.markerColors(markerType),'Visible',isIDVisible);
            if ~obj.roiShowMarkerNumbers
                set(hMarkerLabel,'Visible','off');
            end
            
            % pack the marker data into the struct
            roiStruct.hMarker = hMarker;
            roiStruct.hMarkerLabel = hMarkerLabel;
            obj.roiDataStructure(roiID) = roiStruct;
        end
        
        function roiMacroCopyToCycle(obj,roiIDs,doClearCycleTable)
            % Copies the given ROIs to cycle table iterations.
            %
            % roiIDs: a vector of ROI IDs to copy.  If empty, all children of the currently displayed ROI will be copied.
            % doClearCycleTable: a boolean that, if true, indicates to clear the cycle table before copying.
            
            if nargin < 3 || isempty(doClearCycleTable)
                % prompt the user
                choice = questdlg('There is existing cycle data; would you like to overwrite it, or append the new data?', ...
                    'Existing Cycle Data', ...
                    'Overwrite','Append','Overwrite');
                if strcmp(choice,'Overwrite')
                    doClearCycleTable = true;
                else
                    doClearCycleTable = false;
                end
            end
            
            if nargin < 2 || isempty(roiIDs)
                if ~obj.roiDataStructure.isKey(obj.shownROI)
                    return;
                end
                
                displayedROIStruct = obj.roiDataStructure(obj.shownROI);
                if isfield(displayedROIStruct,'children')
                    roiIDs = displayedROIStruct.children;
                else
                    return;
                end
            end
            
            global state;
            state.cycle.cycleOn = 1;
            updateGUIByGlobal('state.cycle.cycleOn');
            toggleCycleGUI([],[],[],'on');
            
            if doClearCycleTable
                obj.hController{1}.cycClearTable();
            end
            
            obj.roiCopyROIsToCycle(roiIDs);
        end
        
        function roiCopyROIsToCycle(obj,roiIDs)
            % Inserts cycle iterations for the given ROIs.
            
            global state;
            
            MOTOR_ACTION_COL_IDX = 3;
            ROI_COL_IDX = 4;
            
            cycRow = state.cycle.cycleTableColumnDefaults;
            cycRow{MOTOR_ACTION_COL_IDX} = 'ROI #';
            obj.roiSuppressUpdates = true;
            hWaitbar = waitbar(0,'Inserting cycle iterations...');
            for i = 1:length(roiIDs)
                cycRow{ROI_COL_IDX} = roiIDs(i);
                obj.cycAddIteration(cycRow);
                waitbar(i/length(roiIDs),hWaitbar);
            end
            close(hWaitbar);
            obj.roiSuppressUpdates = false;
            obj.hController{1}.updateTableViews('cycle');
        end
        
        function roiMacroGrid(obj)
            
            global state
            
            name='Grid Macro';
            prompt={'Size (MxN):', ...
                'Grid Offset:', ...
                'Grid Offset Units (''degrees'' or ''microns''):', ...
                'Auto Populate Cycle Table:', ...
                'Auto Clear Cycle Table:', ...
                'Scan Angle Multiplier [fast slow]:', ...
                'Scan Zoom Factor:', ...
                'Scan Rotation:', ...
                'Angle to Microns Factor:'
                };
            numlines=1;
            defaultanswer={'[3 3]','[0 0]','degrees','0','1','[0 0]','1','0',num2str(obj.roiAngleToMicronsFactor)};
            answers=inputdlg(prompt,name,numlines,defaultanswer);
            
            if isempty(answers)
                return;
            end
            
            try
                gridSize = str2num(answers{1});
                validateattributes(gridSize,{'numeric'},{'integer' 'positive'},'roiAddGrid','gridSize');
                assert(numel(gridSize)==2,'GridSize must be array of two elements');
                
                gridOffset = str2num(answers{2});
                validateattributes(gridOffset,{'numeric'},{'finite'},'roiAddGrid','gridOffset');
                assert(numel(gridOffset) == 2,'GridOffset must be array of two elements');
                
                if strcmpi(answers{3},'microns')
                    isMicrons = true;
                else
                    isMicrons = false;
                end
                
                doCopyToCycle = str2double(answers{4});
                doOverwriteCycle = str2double(answers{5});
                
                scanAngleMultiplier = str2num(answers{6});
                validateattributes(scanAngleMultiplier,{'numeric'},{'nonnegative','<=',1.0},'roiAddGrid','scanAngleMultiplier');
                assert(numel(scanAngleMultiplier)==2,'scanAngleMultiplier must be array of two elements');
                
                zoomFactor = str2num(answers{7});
                validateattributes(zoomFactor,{'numeric'},{'scalar','positive','finite'},'roiAddGrid','zoomFactor');
                
                scanRotation = str2num(answers{8});
                validateattributes(scanRotation,{'numeric'},{'scalar','<=',45+state.acq.scanRotation ,'>=',-45+state.acq.scanRotation},'roiAddGrid','scanRotation');
                
                angleToMicronsFactor = str2num(answers{9});
                validateattributes(angleToMicronsFactor,{'numeric'},{'scalar','positive','finite'},'roiAddGrid','angleToMicronsFactor');
            catch ME
                most.idioms.reportError(ME);
                return;
            end
            
            roiIDs = obj.roiAddGrid(gridSize,gridOffset,isMicrons,scanAngleMultiplier,zoomFactor,scanRotation,angleToMicronsFactor);
            if doCopyToCycle
                obj.roiMacroCopyToCycle(roiIDs,doOverwriteCycle);
            end
        end
        
        function roiMacroMosaic(obj)
            global state;
            
            %             % ensure we have EOA data
            % 			if isempty(obj.roiLastAcqCache)
            % 				obj.fullError('Cannot create ROIs', 'Unable to create ROI: No acquired data.');
            % 				return;
            %             end
            %
            TILES_WARN_THRESHOLD = 50;
            
            persistent modeCache numValCache overlapCache zoomCache samCache angleToMicronsCache startCenteredCache autoPopulateCache autoClearCache;
            
            if isempty(modeCache)
                modeCache = 'Tiles';
            end
            if isempty(numValCache)
                numValCache = '[3 3]';
            end
            if isempty(overlapCache)
                overlapCache = '[0 0]';
            end
            if isempty(zoomCache)
                zoomCache = num2str(state.acq.zoomFactor);
            end
            if isempty(samCache)
                samCache = ['[' num2str(state.acq.scanAngleMultiplierFast) ' ' num2str(state.acq.scanAngleMultiplierSlow) ']'];
            end
            if isempty(angleToMicronsCache)
                angleToMicronsCache = num2str(obj.roiAngleToMicronsFactor);
            end
            if isempty(startCenteredCache)
                startCenteredCache = '0';
            end
            if isempty(autoPopulateCache)
                autoPopulateCache = '1';
            end
            if isempty(autoClearCache)
                autoClearCache = '1';
            end
            
            name='Mosaic Macro';
            prompt={'Specify ''Tiles'' or ''Span'':',...
                '# Tiles or Span (microns)', ...
                'Overlap (microns):', ...
                'Zoom Factor:', ...
                'Scan Angle MultiplierFast/Slow:', ...
                'Angle to Microns Factor:', ...
                'Start Posn Centered:', ...
                'Auto Populate Cycle Table:' ...
                'Auto Clear Cycle Table:'
                };
            
            numlines=1;
            defaultanswer={modeCache,numValCache,overlapCache,zoomCache,samCache,angleToMicronsCache,startCenteredCache,autoPopulateCache,autoClearCache};
            answers=inputdlg(prompt,name,numlines,defaultanswer);
            
            if isempty(answers)
                return;
            end
            
            if any(cellfun(@isempty,answers))
                obj.fullError('','Please specify all parameters.');
                return;
            end
            
            mode = answers{1};
            if ~(strcmpi(mode,'span') || strcmpi(mode,'tiles'))
                obj.zprvMacroError('Invalid mode: please specify ''Span'' or ''Tiles''.');
            end
            
            sizeParam = znstForceXYVar(str2num(answers{2}),'Mosaic Extent (Span or Tiles)');
            overlap = znstForceXYVar(str2num(answers{3}),'Overlap');
            sam = znstForceXYVar(str2num(answers{5}),'Scan Angle Multiplier');
            angleToMicronsFactor = str2double(answers{6});
            
            zoomFactor = str2double(answers{4});
            if isnan(zoomFactor) || zoomFactor < 0 || isinf(zoomFactor)
                obj.zprvMacroError('Invalid zoomFactor: please specify a scalar positive finite value');
            end
            
            fov = [(angleToMicronsFactor*(sam(1)*state.init.scanAngularRangeReferenceFast))/zoomFactor, ...
                (angleToMicronsFactor*(sam(2)*state.init.scanAngularRangeReferenceSlow))/zoomFactor];
            tileShift = fov - overlap;
            
            if strcmpi(mode,'tiles')
                sizeParam = round(sizeParam);
                
                if prod(sizeParam) > TILES_WARN_THRESHOLD
                    choice = questdlg('You have specified a very large number of tiles--did you mean to specify ''Span''?', ...
                        'Warning', ...
                        'Yes','No','No');
                    
                    if strcmp(choice,'Yes')
                        mode = 'span';
                    end
                end
            elseif strcmpi(mode,'span')
                if sizeParam(1) < fov(1) || sizeParam(2) < fov(2)
                    choice = questdlg('The specified Span is less than the specified FOV--did you mean to specify ''Tiles''?', ...
                        'Warning', ...
                        'Yes','No','No');
                    
                    if strcmp(choice,'Yes')
                        mode = 'tiles';
                    end
                end
            end
            
            if sam(1) < 0 || sam(1) > 1
                obj.fullError('','Invalid Parameter');
                return;
            end
            if sam(2) < 0 || sam(2) > 1
                obj.fullError('','Invalid Parameter');
                return;
            end
            
            isStartCentered = str2double(answers{7});
            doAutoPopulateCycleTable = str2double(answers{8});
            doOverwriteCycle = str2double(answers{9});
            
            if strcmpi(mode,'span')
                span = sizeParam;
                
                numTiles = ceil(span./tileShift);
                
                coverage = numTiles.*tileShift + overlap;
            elseif strcmpi(mode,'tiles')
                numTiles = sizeParam;
                
                span = numTiles.*tileShift + overlap;  % should this be (... - overlap)?
                
                coverage = span;
            end
            
            if isStartCentered
                centeredOffset = ceil(span./2);
            else
                centeredOffset = [0 0];
            end
            
            initialOffset = (span - coverage)./2 - centeredOffset;
            
            initialPosition = [state.motor.absXPosition + initialOffset(1), state.motor.absYPosition + initialOffset(2), state.motor.absZPosition];
            hStep = [tileShift(1) 0 0];
            vStep = [0 tileShift(2) 0];
            
            roiIDs = zeros(prod(numTiles));
            obj.roiSuppressUpdates = true;
            if isempty(obj.shownROI)
                obj.shownROI = obj.ROI_ROOT_ID;
            end
            hWaitbar = waitbar(0,'Creating ROIs...');
            for j = 1:numTiles(2)
                for i = 1:numTiles(1)
                    k = ((j-1)*numTiles(1) + i);
                    waitbar(k/prod(numTiles),hWaitbar);
                    
                    posn = initialPosition + ((i-1) * hStep) + ((j-1) * vStep);
                    
                    if state.motor.dimensionsXYZZ
                        posn(4) = state.motor.absZZPosition;
                    else
                        posn(4) = NaN;
                    end
                    
                    roiStruct = struct();
                    roiStruct.type = 'square';
                    roiStruct.parentROIID = obj.ROI_ROOT_ID;
                    roiStruct.positionID = obj.roiAddPosition(posn);
                    roiStruct.RSPs.zoomFactor = zoomFactor;
                    roiStruct.RSPs.scanAngleMultiplierFast = sam(1);
                    roiStruct.RSPs.scanAngleMultiplierSlow = sam(2);
                    roiStruct.RSPs.scanShiftFast = 0;
                    roiStruct.RSPs.scanShiftSlow = 0;
                    roiStruct.RSPs.scanRotation = 0;
                    
                    roiIDs((j-1)*numTiles(1) + i) = obj.roiAddNew(roiStruct,[],[],true);
                end
            end
            close(hWaitbar);
            
            if doAutoPopulateCycleTable
                obj.roiMacroCopyToCycle(roiIDs,doOverwriteCycle);
            end
            
            viewsToUpdate = {'roi','position'};
            if doAutoPopulateCycleTable
                viewsToUpdate{end+1} = 'cycle';
            end
            obj.shownROI = obj.ROI_ROOT_ID;
            obj.roiSuppressUpdates = false;
            obj.hController{1}.updateTableViews(viewsToUpdate); %TODO: fix this
            
            function val = znstForceXYVar(val,varName)
                if any(isnan(val)) || numel(val) > 2
                    obj.zprvMacroError('Invalid value supplied for ''%s'' -- must be a scalar or 2 element vector',varName);
                end
                
                if isscalar(val)
                    val = [val val];
                end
            end
            
            % cache the entered values
            cachedVarNames = {'modeCache' 'numValCache' 'overlapCache' 'zoomCache' 'samCache' 'angleToMicronsCache' 'startCenteredCache' 'autoPopulateCache' 'autoClearCache'};
            for i = 1:length(cachedVarNames)
                eval([cachedVarNames{i} ' = answers{i};']);
            end
        end
        
        function roiMotorUpdate_Listener(obj,~,~)
            % Handles necessary post motor-update logic.  Checks if the new motor position has defined position/ROI IDs.
            % Note this listener does nothing while in midst of LOOP (i.e. Cycle) acquisitions (until the end)
            
            global state
            
            if ~obj.mdlInitialized
                return;
            end
            
            % determine if we're at a defined position
            obj.activePositionID = obj.zprvDisplayedPosn2PositionID();
            
            %Defer shownROI/RDF updates until done looping
            if state.internal.looping
                return;
            end
            
            %Update activeROI indicator, if we match
            obj.roiUpdateActiveROI();
            
            shownPosition = obj.shownPositionID;
            
            %We only actively change anything in 1 case: activePosition
            %has a defined ID, which does not match the shown position.
            %In this case, update shownROIID to parent of one
            %matching position/rotation/RSPs if possible. Otherwise
            %update shownROIID to root ROI.
            activePosnDefined = (obj.activePositionID > 0);
            if activePosnDefined && ~isequal(shownPosition,obj.activePositionID) %shownPosition can be empty (when shownROI=ROOT) --> use isequal()
                %Active position does not match shown position -- update shown ROI to parent of one with matching position/rotation if such exists
                
                existingROIID = obj.roiFindMatchingROIAtPosition(obj.activePositionID,obj.currentRSPStruct); %VVV052912: Can probably use obj.activeROIID as the existingROIID -- it was just set by roiUpdateActiveROI(), apparently duplicating this call to roiFindMatchingROIAtPosition(). Can check at some point...
                if ~isempty(existingROIID)
                    %Show parent of existing ROI matching current RSPs and position, or top-level ROI if parent is ROOT
                    
                    existingROIParent = obj.roiDataStructure(existingROIID).parentROIID;
                    if existingROIParent == obj.ROI_ROOT_ID
                        obj.shownROI = existingROIID;
                    else
                        obj.shownROI = existingROIParent;
                    end
                    
                else %Show top-level ROI matching current position and rotation, if available
                    topLevelROIID = obj.roiGetTopLevelROIID(obj.activePositionID,state.acq.scanRotation);
                    if ~isempty(topLevelROIID) %there is a top-level ROI at active position and rotation
                        obj.shownROI = topLevelROIID;
                    end
                end
                
            else %activePosnDefined = false
                %Active position does not match any stored position -- do nothing
            end
            
            
            %obj.roiSuppressUpdates = false;
            obj.roiUpdateROITable();
            obj.roiUpdatePositionTable();
            
        end
        
        function normalizedCoords = roiCalculateNormalizedCoords(obj,roiStruct)
            % Calculates the normalized coordinates necessary to draw an ROI on the ROI Display Figure.
            % Normalized coordinates span range [-0.5 0.5] and are relative to the shownROI
            %
            %    roiStruct: structure with ROI data and RSP scan parameters
            %
            %    normalizedCoords: an Mx2 array of normalized coordinate pairs.
            
            normalizedCoords = [];
            
            global state;
            
            if ~isfield(roiStruct,'RSPs')
                error('Invalid ROI struct.');
            end
            
            assert(~isempty(obj.shownROI) && isfield(obj.roiDataStructure(obj.shownROI), 'RSPs'));
            rspStruct = roiStruct.RSPs;
            rspStructDisplayed = obj.roiDataStructure(obj.shownROI).RSPs;
            
            % zoomFactor is the one field we allow to be empty
            if isfield(rspStruct,'zoomFactor')
                zoomFactor = rspStruct.zoomFactor;
            else
                zoomFactor = 1;
            end
            scanShiftFast = rspStruct.scanShiftFast;
            scanShiftSlow = rspStruct.scanShiftSlow;
            scanAngleMultiplierFast = rspStruct.scanAngleMultiplierFast;
            scanAngleMultiplierSlow = rspStruct.scanAngleMultiplierSlow;
            scanRotation = rspStruct.scanRotation;
            
            % compute (normalized) relative RSPs, describing current ROI relative to currently shown ROI
            shiftFastRelative = scanShiftFast - rspStructDisplayed.scanShiftFast;
            shiftSlowRelative = scanShiftSlow - rspStructDisplayed.scanShiftSlow;
            zoomCoverageRelative = (1/zoomFactor)*rspStructDisplayed.zoomFactor; %zoom coverage is inverse of zoom factor
            rotationRelative = scanRotation - rspStructDisplayed.scanRotation;
            samFastRelative = scanAngleMultiplierFast/rspStructDisplayed.scanAngleMultiplierFast; %only used for drawing quad ROIs
            samSlowRelative = scanAngleMultiplierSlow/rspStructDisplayed.scanAngleMultiplierSlow; %only used for drawing quad ROIs
            
            %Shift central coordinates of ROI into the rotated reference frame which is displayed
            R = [cosd(rspStructDisplayed.scanRotation) -sind(rspStructDisplayed.scanRotation); sind(rspStructDisplayed.scanRotation) cosd(rspStructDisplayed.scanRotation)];
            scanShift = R * [shiftFastRelative; shiftSlowRelative];
            shiftFastRelative = scanShift(1);
            shiftSlowRelative = scanShift(2);
            
            %Normalize central coordinates relative to displayed parent ROI
            shiftFastRelative = shiftFastRelative /(state.init.scanAngularRangeReferenceFast * rspStructDisplayed.scanAngleMultiplierFast / rspStructDisplayed.zoomFactor);
            shiftSlowRelative = shiftSlowRelative /(state.init.scanAngularRangeReferenceSlow * rspStructDisplayed.scanAngleMultiplierSlow / rspStructDisplayed.zoomFactor);
            
            switch roiStruct.type
                case 'point'
                    normalizedCoords = zeros(2,2);
                    
                    
                    normalizedCoords(1,:) = [shiftFastRelative shiftSlowRelative];
                    normalizedCoords(2,:) = [0.01 0.01];
                    
                case 'line'
                    dispSAM = [rspStructDisplayed.scanAngleMultiplierFast rspStructDisplayed.scanAngleMultiplierSlow];
                    
                    x1 = shiftFastRelative - (zoomCoverageRelative/2)*(cosd(rotationRelative))/dispSAM(1);
                    y1 = shiftSlowRelative + (zoomCoverageRelative/2)*(sind(rotationRelative))/dispSAM(2); %Would be -sind(), but y axis is reversed
                    x2 = shiftFastRelative + (zoomCoverageRelative/2)*(cosd(rotationRelative))/dispSAM(1);
                    y2 = shiftSlowRelative - (zoomCoverageRelative/2)*(sind(rotationRelative))/dispSAM(2); %Would be -sind(), but y axis is reversed
                    
                    normalizedCoords = [x1 y1; x2 y2];
                    
                case {'square' 'rect'}
                    normalizedCoords = zeros(4,2);
                    
                    x1 = shiftFastRelative - (zoomCoverageRelative*samFastRelative)/2;
                    y1 = shiftSlowRelative - (zoomCoverageRelative*samSlowRelative)/2;
                    
                    w = zoomCoverageRelative * samFastRelative;
                    h = zoomCoverageRelative * samSlowRelative;
                    
                    normalizedCoords(1,:) = [x1, y1];
                    normalizedCoords(2,:) = [x1 + w, y1];
                    normalizedCoords(3,:) = [x1 + w, y1 + h];
                    normalizedCoords(4,:) = [x1, y1 + h];
                    
            end
        end
        
        function [parentROIID, childROIID] = roiEnsureHierarchy(obj,hAxes)
            % Inserts to-be-created ROI into the ROI hierarchy, returning parent and (if applicable) child ROI IDs
            %
            % hAxes: a handle to the Axes in which the user 'drew' the new ROI, if applicable. Otherwise current RSP settings are used.
            %
            % parentROIID = the ROI ID of the parent under which to create the new ROI.
            % childROIID = the ROI ID of an RDO to be linked as a child of the new ROI (if re-linking an existing ROI is necessary).
            %
            %
            
            global state;
            
            if nargin < 2 || isempty(hAxes)
                % if user didn't specify hAxes (for instance, when adding a CUR ROI), use the first acq figure.
                hAxes = state.internal.axis(1);
                isCUR = true;
            else
                isCUR = false;
            end
            
            childROIID = [];
            parentROIID = [];
            parentParentROIID = [];
            
            if hAxes == obj.hROIDisplayAx
                % User drew in the RDF; use the currently shown ROI as parent (or ROOT, if nothing is shown).
                
                if isempty(obj.shownROI)
                    parentROIID = obj.ROI_ROOT_ID;
                else
                    parentROIID = obj.shownROI;
                end
                
                if obj.roiGotoOnAdd
                    % update the motor position to match that of the shown ROI.
                    topLevelROIID = obj.roiGetOldestAncestor(parentROIID);
                    shownPositionID = obj.roiDataStructure(topLevelROIID).positionID;
                    if ~isempty(shownPositionID) && shownPositionID > 0
                        obj.roiGotoPosition(shownPositionID);
                    end
                end
                
            elseif ismember(hAxes, state.internal.axis) || hAxes == state.internal.mergeaxis
                % User drew in one of the acquisition figures...
                
                % ensure we have EOA data
                if isempty(obj.roiLastAcqCache) %TODO: This is probably redundant, can likely remove
                    obj.fullError('Cannot create ROIs','Unable to create ROI: No acquired data.');
                    return;
                end
                
                % ensure that the current motor position matches the cached EOA motor position
                cachedPosition = obj.roiLastAcqCache.position;
                infixes = {'X' 'Y' 'Z' 'ZZ'};
                isEqual = true;
                for i = 1:length(cachedPosition)
                    motorPos = state.motor.(['abs' infixes{i} 'Position']);
                    if (~isnan(cachedPosition(i)) && ~isnan(motorPos)) && cachedPosition(i) ~= motorPos % TODO: is this boolean logic correct?
                        isEqual = false;
                        break;
                    end
                end
                if ~isEqual
                    obj.fullError('Motor has moved.','Unable to create ROI: The motor has moved since the last acquisition.');
                    return;
                end
                
                % if adding a CUR, ensure that the cached EOA data matches the current RSPs
                if isCUR
                    currentRSPs = struct();
                    for i = 1:length(obj.scanParameterNames)
                        currentRSPs.(obj.scanParameterNames{i}) = state.acq.(obj.scanParameterNames{i});
                    end
                    if ~obj.roiIsEqualRSPs(obj.roiLastAcqCache.RSPs,currentRSPs)
                        obj.fullError('Cannot create ROIs','Unable to create ROI: Current scan parameters do not match cached scan parameters.');
                        return;
                    end
                end
                
                % if the EOA cache matched an existing ROI, use that as parent
                if ~isempty(obj.roiLastAcqCache.definedROI)
                    if isCUR
                        obj.fullError('Cannot create ROI',['Unable to create ROI: ROI with current scan parameters already exists (ROI # ' num2str(obj.roiLastAcqCache.definedROI) ').']);
                        return;
                    end
                    
                    parentROIID = obj.roiLastAcqCache.definedROI;
                    return;
                else
                    % no defined ROI: forcibly create a top-level ROI from the cached data
                    
                    % first, determine the positionID and parentParentROIID to use.
                    positionID = obj.zprvDisplayedPosn2PositionID();
                    isPosnDefined = (positionID > 0);
                    
                    if isCUR
                        if ~isPosnDefined
                            positionID = obj.roiAddPosition();
                        end
                        
                        topLevelROIID = obj.roiGetTopLevelROIID(positionID,currentRSPs.scanRotation);
                        effectiveZoomCUR = obj.roiLastAcqCache.RSPs.zoomFactor/(obj.roiLastAcqCache.RSPs.scanAngleMultiplierFast*obj.roiLastAcqCache.RSPs.scanAngleMultiplierSlow);
                        
                        if ~isempty(topLevelROIID)
                            topLevelROIStruct = obj.roiDataStructure(topLevelROIID);
                            effectiveZoomExisting = topLevelROIStruct.RSPs.zoomFactor/(topLevelROIStruct.RSPs.scanAngleMultiplierFast*topLevelROIStruct.RSPs.scanAngleMultiplierSlow);
                            
                            %Check that new ROI isn't /larger/ than current
                            %top-level ROI (can occur because of fractional
                            %zoom). Otherwise make this CUR ROI a direct child
                            %of the top-level ROI.
                            if effectiveZoomCUR < effectiveZoomExisting
                                % make the new CUR ROI top-level, and re-link the existing ROI
                                parentROIID = obj.ROI_ROOT_ID;
                                obj.roiRemoveChildFromParent(topLevelROIID);
                                childROIID = topLevelROIID;
                            elseif effectiveZoomCUR > effectiveZoomExisting
                                parentROIID = topLevelROIID;
                            elseif effectiveZoomCUR == effectiveZoomExisting
                                % TODO: Decide what to do in this case.
                                % Maybe auto-create a fractional-zoom ROI
                                % that spans the two ROIs.
                            end
                        else
                            if effectiveZoomCUR <= 1.0 %Make this ROI the top-level ROI
                                parentROIID = obj.ROI_ROOT_ID;
                            else % create a top-level ROI (with ROOT RSPs) to be used as the parent
                                
                                rspStruct = obj.roiDataStructure(obj.ROI_ROOT_ID).RSPs;
                                rspStruct.scanRotation = currentRSPs.scanRotation;
                                
                                parentROIStruct = struct('type','square','parentROIID',obj.ROI_ROOT_ID,'positionID',positionID,'RSPs',rspStruct);
                                parentROIID = obj.roiAddNew(parentROIStruct,[],[],true);
                            end
                        end
                        
                        return;
                    else
                        if isPosnDefined
                            parentParentROIID = obj.roiGetTopLevelROIID(positionID,state.acq.scanRotation); %The active scan rotation should match the rotation of image in channel display from which an ROI was just selected
                        else
                            if state.motor.motorOn
                                positionID = obj.roiAddPosition();
                            end
                        end
                        
                        if isempty(parentParentROIID)
                            % if RSPs don't match ROOT RSPs (all but scan rotation), create an additional ROI that is top-level
                            if obj.roiIsEqualRSPs(obj.roiLastAcqCache.RSPs,obj.roiDataStructure(obj.ROI_ROOT_ID).RSPs,true); %compare all but scanRotation
                                % if creating a CUR ROI, just use ROOT as the parent
                                if isCUR
                                    parentROIID = obj.ROI_ROOT_ID;
                                    return;
                                end
                                parentParentROIID = obj.ROI_ROOT_ID;
                            else
                                % create a top-level ROI (with ROOT RSPs, except scanRotation) to be used as the parent of the parent
                                
                                rspStruct = obj.roiDataStructure(obj.ROI_ROOT_ID).RSPs;
                                rspStruct.scanRotation = state.acq.scanRotation; %The active scan rotation should match the rotation of image in channel display from which an ROI was just selected
                                
                                parentParentROIStruct = struct('type','square','parentROIID',obj.ROI_ROOT_ID,'positionID',positionID,'RSPs',rspStruct);
                                parentParentROIID = obj.roiAddNew(parentParentROIStruct,[],[],true);
                            end
                        end
                        
                        % create the top-level parent ROI
                        parentROIID = obj.roiAddNewFromAcqCache('positionID',positionID,'parentROIID',parentParentROIID);
                        obj.roiLastAcqCache.definedROI = parentROIID;
                    end
                end
            else
                % Shouldn't ever get here...
                error('Invalid Axes handle.');
            end
        end
        
        function roiID = roiGetTopLevelROIID(obj,posnID,rotation)
            % Returns the top-level ROI ID(s) for the given motor position & (optionally) scan rotation, if one (or more)exists
            %
            % posnID: a valid Position ID for which to find the top-level ROI.  If empty, the currently selected Position will be used.
            % rotation: scan rotation for which to find the top-level ROI. If empty, a vector of roiIDs is returned.
            %
            % roiID: Empty, scalar, or vector array of ROIIDs. Vector case can arise iff supplied rotation is empty
            %
            % NOTES:
            %   roiID can be emtpy, a scalar, or a vector. The vector case arises for rotation
            
            roiID = [];
            
            if nargin < 2 || isempty(posnID) %VVV: Using the selectedPositionID seems highly questionable. Not sure this is ever actually reached though.
                if isempty(obj.selectedPositionID)
                    return;
                end
                posnID = obj.selectedPositionID;
            end
            
            matchAllRotations = nargin < 3 || isempty(rotation);
            
            rootROIStruct = obj.roiDataStructure(obj.ROI_ROOT_ID);
            topLevelROIIDs = rootROIStruct.children(rootROIStruct.children ~= obj.ROI_BASE_ID);
            
            for i = 1:length(topLevelROIIDs)
                childStruct = obj.roiDataStructure(topLevelROIIDs(i));
                
                if matchAllRotations
                    if childStruct.positionID == posnID
                        roiID(end+1) = topLevelROIIDs(i); %#ok<AGROW>
                    end
                else
                    if childStruct.positionID == posnID && childStruct.RSPs.scanRotation == rotation
                        roiID = topLevelROIIDs(i);
                        return;
                    end
                end
            end
        end
        
        function roiUpdatePositionTable(obj)
            
            global state;
            
            if isempty(state)
                return;
            end
            
            if obj.roiSuppressUpdates
                return;
            end
            
            positions = obj.positionDataStructure.keys();
            
            positionIDs = {};
            xVals = repmat(NaN,1,length(positions));
            yVals = repmat(NaN,1,length(positions));
            zVals = repmat(NaN,1,length(positions));
            zzVals = repmat(NaN,1,length(positions));
            
            for i = 1:length(positions)
                positionStruct = obj.positionDataStructure(positions{i});
                
                if isempty(positionStruct) || isnumeric(positionStruct)
                    continue;
                end
                
                positionIDs{i} = num2str(positions{i});
                
                if positions{i} == obj.activePositionID
                    positionIDs{i} = [positionIDs{i} ' *'];
                end
                
                % if displaying relative coords, apply the offset
                if ~obj.roiShowAbsoluteCoords
                    if ~state.motor.motorOn
                        relOrigin = [nan nan nan];
                    else
                        relOrigin = motorGetRelativeOrigin();
                    end
                    xVals(i) = positionStruct.motorX - relOrigin(1);
                    yVals(i) = positionStruct.motorY - relOrigin(2);
                    zVals(i) = positionStruct.motorZ - relOrigin(3);
                    
                    if state.motor.dimensionsXYZZ && ~obj.posnIgnoreSecZ
                        zzVals(i) = positionStruct.motorZZ - relOrigin(4);
                    end
                    
                else
                    xVals(i) = positionStruct.motorX;
                    yVals(i) = positionStruct.motorY;
                    zVals(i) = positionStruct.motorZ;
                    
                    if state.motor.dimensionsXYZZ && ~obj.posnIgnoreSecZ
                        zzVals(i) = positionStruct.motorZZ;
                    end
                end
                
            end
            
            % update the ColumnArrayTable-bound props
            boundProps = {'positionIDs' 'xVals' 'yVals' 'zVals' 'zzVals'};
            
            numProps = length(boundProps);
            for i = 1:numProps
                propName = boundProps{i};
                obj.(propName) = eval(propName);
            end
        end
        
        function roiUpdateROITable(obj)
            % Updates the data displayed in the ROI uitable, given the currently displayed ROI.
            %
            % doShowWaitbar: a boolean value that, if true, specifies to show a progress bar while updating the table (default=true).
            
            if obj.roiSuppressUpdates || ~obj.mdlInitialized
                return;
            end
            
            if obj.roiDataStructure.isKey(obj.shownROI)
                displayedROIStruct = obj.roiDataStructure(obj.shownROI);
            else
                displayedROIStruct = [];
            end
            
            roiIDs = {};
            roiPositionIDs = [];
            roiTypes = {};
            roiZoomFactors = [];
            roiRotations = [];
            roiShifts = {};
            roiScanAngleMultipliers = {};
            
            childrenROIIDs = obj.roiGetDisplayedChildren();
            
            for i = 1:length(childrenROIIDs)
                childROIStruct = obj.roiDataStructure(childrenROIIDs(i));
                
                if ~isfield(childROIStruct,'RSPs')
                    continue;
                end
                
                if childrenROIIDs(i) == obj.ROI_BASE_ID
                    roiIDs{i} = 'base';
                else
                    roiIDs{i} = num2str(childrenROIIDs(i));
                end
                
                if childrenROIIDs(i) == obj.activeROIID
                    roiIDs{i} = [roiIDs{i} ' *'];
                end
                
                roiTypes{i} = childROIStruct.type;
                
                % if this is a top-level ROI (i.e. parent is ROOT), use its positionID, otherwise walk up the tree and
                % determine the positionID of the top-level ROI.
                if childROIStruct.parentROIID == obj.ROI_ROOT_ID && isfield(childROIStruct,'positionID')
                    roiPositionIDs(i) = childROIStruct.positionID;
                else
                    ancestorID = obj.roiGetOldestAncestor(childrenROIIDs(i));
                    ancestorROIStruct = obj.roiDataStructure(ancestorID);
                    if isfield(ancestorROIStruct,'positionID')
                        roiPositionIDs(i) = ancestorROIStruct.positionID;
                    end
                end
                
                roiZoomFactors(i) = childROIStruct.RSPs.zoomFactor;
                roiShifts{i} = ['[' sprintf('%.2f',childROIStruct.RSPs.scanShiftFast) ' ' sprintf('%.2f',childROIStruct.RSPs.scanShiftSlow) ']'];
                roiRotations(i) = childROIStruct.RSPs.scanRotation;
                roiScanAngleMultipliers{i} = ['[' sprintf('%.2f',childROIStruct.RSPs.scanAngleMultiplierFast) ' ' ...
                    sprintf('%.2f',childROIStruct.RSPs.scanAngleMultiplierSlow) ']'];
            end
            
            % update the ColumnArrayTable-bound props
            boundProps = {'roiIDs' 'roiTypes' 'roiPositionIDs' 'roiZoomFactors' 'roiShifts' 'roiRotations' 'roiScanAngleMultipliers'};
            numProps = length(boundProps);
            for i = 1:numProps
                propName = boundProps{i};
                obj.(propName) = eval(propName);
            end
            
            % update the breadcrumbs
            breadcrumbString = 'ROOT';
            if obj.shownROI > obj.ROI_ROOT_ID
                [~, breadcrumbList] = obj.roiGetOldestAncestor(obj.shownROI);
                for i = 1:length(breadcrumbList)
                    breadcrumbString = [breadcrumbString ' => ' num2str(breadcrumbList(i))];
                end
                breadcrumbString = [breadcrumbString ' => ' num2str(obj.shownROI)];
            else
                breadcrumbString = [breadcrumbString ' => '];
            end
            obj.roiBreadcrumbString = breadcrumbString;
            
            if obj.roiActiveUpdatePending
                obj.roiActiveUpdatePending = false;
                obj.selectedROIID = [];
            end
        end
        
        function roiSetMarkersVisibility(obj,isVisible)
            % Sets all ROI markers belonging to the currently-displayed ROI to the specified state.
            %
            % isVisible: a boolean value specifiying if the markers are to be visible, OR one of {'on' 'off'}
            
            if nargin < 2 || isempty(isVisible)
                obj.fullError('','You must specify a state.');
                return;
            end
            
            if isempty(obj.shownROI)
                return;
            end
            
            if ischar(isVisible)
                if ~ismember(isVisible,{'on' 'off'})
                    obj.fullError('','Invalid state');
                    return;
                end
            elseif isVisible
                isVisible = 'on';
            else
                isVisible = 'off';
            end
            
            % if turning markers off, just turn off everything
            if strcmp(isVisible,'off')
                children = cell2mat(obj.roiDataStructure.keys());
                children(children < 1) = [];
            else
                % get all children of the currently-displayed ROI
                
                if ~obj.roiDataStructure.isKey(obj.shownROI)
                    return;
                end
                currentlyDisplayedROIStruct = obj.roiDataStructure(obj.shownROI);
                if ~isfield(currentlyDisplayedROIStruct,'children')
                    return;
                end
                children = currentlyDisplayedROIStruct.children;
            end
            
            % set all childrens' 'visible' property to the specified state.
            for i = 1:length(children)
                childROIStruct = obj.roiDataStructure(children(i));
                if isfield(childROIStruct,'hMarker')
                    set(childROIStruct.hMarker, 'Visible', isVisible);
                end
                if isfield(childROIStruct,'hMarkerLabel')
                    set(childROIStruct.hMarkerLabel, 'Visible', isVisible);
                end
            end
        end
        
        function roiUpdateActiveROI(obj,roiID)
            % Checks updated RSPs and updates activeROIID appropriately. If
            % roiID is specified, then activeROIID is simply updated to
            % this value, after some checks.
            
            if nargin > 1
                assert(obj.roiDataStructure.isKey(roiID),'Specified roiID not found');
                posnID = obj.roiGetPositionFromROIID(roiID);
                
                if posnID > 0
                    assert(roiID == obj.roiFindMatchingROIAtPosition(posnID,obj.currentRSPStruct),'Specified ROI does not match current position and/or RSPs');
                else %posnID < 0 ('abstract' position)
                    assert(obj.roiIsEqualRSPs(obj.roiDataStructure(roiID).RSPs,obj.currentRSPStruct),'Specified ROI does not match current RSPs');
                end
                obj.activeROIID = roiID;
            else
                posnID = obj.zprvDisplayedPosn2PositionID();
                
                if posnID == 0 %Displayed position matches no stored posnID coordinates
                    %Set activeROIID to null if 1) RSPs don't match or 2) (previous) activeROIID has a real associated posnID
                    %Otherwise, allow activeROIID to remain unchanged (i.e. grid point ROIs with abstract posnID)
                    if ~isempty(obj.activeROIID) && ...
                            (obj.roiGetPositionFromROIID(obj.activeROIID) > 0 || ~obj.roiIsEqualRSPs(obj.roiDataStructure(obj.activeROIID).RSPs,obj.currentRSPStruct))
                        obj.activeROIID = [];
                    end
                else
                    %Set activeROIID to matching ROI at the current position.
                    %Also allow current activeROIID to remain unchanged if the RSPs match and it has an 'abstract' associated posnID
                    matchingROI = obj.roiFindMatchingROIAtPosition(posnID,obj.currentRSPStruct);
                    if ~isempty(matchingROI) || isempty(obj.activeROIID) || ~obj.roiIsEqualRSPs(obj.roiDataStructure(obj.activeROIID).RSPs,obj.currentRSPStruct) || obj.roiGetPositionFromROIID(obj.activeROIID) >= 0 %Can maintain current activeROIID, if it has an abstract position and the RSPs match
                        obj.activeROIID = matchingROI;
                    end
                end
            end
        end
        
        function roiRSP_Listener(obj)
            % Handles necessary logic after a change to any ROI Scan Parameter (RSP).
            
            obj.roiUpdateActiveROI();
            
            % Clear the EOA cache. This effectively disallows drawing ROI
            % onto channel display figure (even if RSPs later change back
            % to what was last acquired).
            obj.roiLastAcqCache = [];
        end
        
        %         function roiUpdateShownPosition(obj)
        %             % Forces update of the shown position string in the RDF.
        %             obj.shownPositionID = obj.shownPositionID;
        %         end
        
        function roiUpdateView(obj)
            
            if obj.roiSuppressUpdates || ~obj.mdlInitialized
                return;
            end
            
            global state;
            
            if isempty(obj.shownROI) || ~obj.roiDataStructure.isKey(obj.shownROI)
                return;
            end
            
            %set(obj.hROIDisplayIm,'CData',obj.roiDrawCheckeredBackground([state.internal.storedLinesPerFrame state.acq.pixelsPerLine numColorChannels]));
            if obj.shownROI == obj.ROI_ROOT_ID
                %TODO -- allow selection of top-level ROI from positions/rotations lists
                return;
            end
            
            % determine the acquisition channel
            if strcmp(obj.roiDisplayedChannel,'merge')
                targetChannel = 5; % the merge-data is stored in the fifth index of the MRI cache
                targetColormap = colormap();
                %numColorChannels = 3;
            else
                targetChannel = str2double(obj.roiDisplayedChannel);
                targetColormap = eval(eval(['state.internal.figureColormap' num2str(targetChannel)]));
                %numColorChannels = 1;
            end
            
            % reset the display figure
            obj.roiSetMarkersVisibility('off');
            cData = obj.roiDrawCheckeredBackground([repmat(obj.ROI_FIG_CHECKERBOARD_SIZE,1,2) 3]);
            set(obj.hROIDisplayIm,'CData',cData);
            aspectRatio = 1;
            
            displayedROIStruct = obj.roiDataStructure(obj.shownROI);
            
            if isfield(displayedROIStruct,'MRI')
                if targetChannel <= length(displayedROIStruct.MRI) && ~isempty(displayedROIStruct.MRI{targetChannel})
                    cData = displayedROIStruct.MRI{targetChannel};
                    
                    % scale the merge-data
                    if targetChannel == 5
                        cData = double(cData)./max(double(cData(:)));
                    end
                end
                
                if targetChannel < 5
                    set(obj.hROIDisplayFig,'Colormap',targetColormap);
                    set(obj.hROIDisplayAx,'CLim',get(state.internal.axis(targetChannel),'CLim'));
                    set(obj.hROIDisplayIm,'CDataMapping','scaled');
                end
                
                set(obj.hROIDisplayIm,'CData',cData);
            end
            
            % handle rectangular ROIs
            if isfield(displayedROIStruct,'RSPs') && displayedROIStruct.RSPs.scanAngleMultiplierFast ~= displayedROIStruct.RSPs.scanAngleMultiplierSlow
                if displayedROIStruct.RSPs.scanAngleMultiplierFast == 0 || displayedROIStruct.RSPs.scanAngleMultiplierSlow == 0
                    aspectRatio = size(cData,2)/size(cData,1);
                else
                    dispROIAspectRatio = displayedROIStruct.RSPs.scanAngleMultiplierSlow/displayedROIStruct.RSPs.scanAngleMultiplierFast;
                    aspectRatio = (size(cData,2)/size(cData,1)) * dispROIAspectRatio;
                end
            else
                aspectRatio = 1;
            end
            
            %Update image/axes limits
            %axSize = [diff(get(obj.hROIDisplayAx,'XLim')) diff(get(obj.hROIDisplayAx,'YLim'))];
            %set(obj.hROIDisplayIm,'XData',[1 axSize(1)],'YData', [1 axSize(2)]);
            xData = [1 size(cData,2)];
            yData = [1 size(cData,1)];
            
            set(obj.hROIDisplayIm,'XData',xData,'YData',yData);
            set(obj.hROIDisplayAx,'XLim',xData + [-.5 .5],'YLim',yData + [-.5 .5]);
            set(obj.hROIDisplayAx,'DataAspectRatio', [aspectRatio 1 1]);
            
            % determine if this ROI has children (if so, recursively draw them.)
            children = obj.roiGetDisplayedChildren();
            children(children == obj.ROI_BASE_ID) = [];
            
            if isfield(displayedROIStruct, 'children')
                for i = 1:length(children)
                    obj.roiDrawMarker(children(i));
                end
            end
        end
        
        function roiUpdateViewCLim(obj)
            % Updates the CLim property of the RDF.
            
            global state;
            
            if strcmp(obj.roiDisplayedChannel,'merge')
                % TODO: this doesn't work.  It will require actively updating the EOA cached merge-data for the appropriate ROI.
            else
                targetChannel = str2double(obj.roiDisplayedChannel);
                set(obj.hROIDisplayAx,'CLim',[state.internal.lowPixelValue(targetChannel) state.internal.highPixelValue(targetChannel)])
            end
        end
        
        function roiRemoveROI(obj,roiID)
            % Removes an ROI from the list.
            %
            % 'roiID': A valid ROI to be removed.
            
            if nargin < 2 || isempty(roiID)
                obj.fullError('','Please specify a valid ROI.');
                return;
            end
            
            if roiID == obj.ROI_BASE_ID
                obj.fullError('','The Base ROI cannot be deleted.');
                return;
            end
            
            roiStruct = obj.roiDataStructure(roiID);
            
            % if this ROI has children, remove them as well...
            if isfield(roiStruct,'children') && ~isempty(roiStruct.children)
                children = roiStruct.children;
                for j = 1:length(children)
                    obj.roiRemoveROI(children(j));
                end
                
                % deleting the children modified the parent, so refresh the struct
                roiStruct = obj.roiDataStructure(roiID);
            end
            
            % delete the markers
            if isfield(roiStruct,'hMarker') && all(ishandle(roiStruct.hMarker))
                delete(roiStruct.hMarker);
            end
            if isfield(roiStruct,'hMarkerLabel') && ishandle(roiStruct.hMarkerLabel)
                delete(roiStruct.hMarkerLabel);
            end
                        
            % Now remove the ROI, including all references to it from parent ROI
            posnID = obj.roiGetPositionFromROIID(roiID);
            rotation = obj.roiDataStructure(roiID).RSPs.scanRotation;
            obj.roiRemoveChildFromParent(roiID);            
            obj.roiDataStructure.remove(roiID); 
            
            % remove any reference in the EOA cache
            if isfield(obj.roiLastAcqCache,'definedROI') && isequal(obj.roiLastAcqCache.definedROI,roiID)
                obj.roiLastAcqCache.definedROI = [];
            end
            
            if roiID == obj.shownROI
                obj.shownROI = obj.ROI_ROOT_ID;
            end
            
            if roiID == obj.activeROIID
                obj.activeROIID = [];
            end
            
            if ~obj.roiSuppressUpdates
                obj.roiUpdateROITable();
            end
            
            % remove any cycle-iterations referencing this ROI
            iterationIndices = obj.hController{1}.roiID2cycIterationIdx(roiID);
            for i = 1:length(iterationIndices)
                gridShift = length(iterationIndices(iterationIndices < iterationIndices(i)));
                obj.cycRemoveIteration(iterationIndices(i) - gridShift);
            end
            
            % remove associated posnID, if it was abstract and no remaining ROIs are associated with it
            if posnID < 0 
                topROI = obj.roiGetTopLevelROIID(posnID,rotation);
                if isempty(topROI) || isempty(obj.roiDataStructure(topROI).children)
                    if ~isempty(topROI)
                        obj.roiRemoveROI(topROI);
                        return;
                    end
                    
                    roiSuppressUpdates_ = obj.roiSuppressUpdates;
                    obj.roiSuppressUpdates = false;
                    obj.roiRemovePosition(posnID);     
                    obj.roiSuppressUpdates = roiSuppressUpdates_;
                    
                end
            end
           
        end
        
        function roiRemovePosition(obj,posnID)
            % Removes a Position from the list.
            %
            % 'posnID': A valid Position to be removed.
            
            if nargin < 2 || isempty(posnID)
                obj.fullError('','Please specify a valid Position.');
                return;
            end
            
            if posnID == 0
                obj.fullError('','The Root Position cannot be deleted.');
                return;
            end
            
            % delete the position
            if obj.positionDataStructure.isKey(posnID)
                obj.positionDataStructure.remove(posnID);
            end
            
            if posnID == obj.activePositionID
                obj.activePositionID = 0;
            end
            
            % remove any associated ROIs
            rootROIStruct = obj.roiDataStructure(obj.ROI_ROOT_ID);
            rootChildren = rootROIStruct.children;
            for j = 1:length(rootChildren)
                childROIStruct = obj.roiDataStructure(rootChildren(j));
                if childROIStruct.positionID == posnID
                    obj.roiRemoveROI(rootChildren(j));
                end
            end
            
            if ~obj.roiSuppressUpdates
                obj.roiUpdatePositionTable();
            end
        end
        
        
        function roiRenumberPositions(obj)
            % Renumbers Positions to be consecutive (starting with '1').
            
            positionIDs = obj.positionDataStructure.keys();
            for i = 1:length(positionIDs)
                if i == positionIDs{i}
                    continue;
                end
                
                obj.positionDataStructure(i) = obj.positionDataStructure(positionIDs{i});
                obj.positionDataStructure.remove(positionIDs{i});
                
                % update any ROIs referencing this Position
                doUpdateROITable = false;
                rootROIStruct = obj.roiDataStructure(obj.ROI_ROOT_ID);
                if isfield(rootROIStruct,'children') && ~isempty(rootROIStruct.children)
                    topLevelROIs = rootROIStruct.children;
                    for j = 1:length(topLevelROIs)
                        roiStruct = obj.roiDataStructure(topLevelROIs(j));
                        if roiStruct.positionID == positionIDs{i}
                            roiStruct.positionID = i;
                            obj.roiDataStructure(topLevelROIs(j)) = roiStruct;
                            doUpdateROITable = true;
                        end
                    end
                end
                
                if obj.activePositionID == positionIDs{i}
                    obj.activePositionID = i;
                end
            end
            
            % Update the data displayed in the Position/ROI table(s).
            obj.roiUpdatePositionTable();
            
            if doUpdateROITable
                obj.roiUpdateROITable();
            end
        end
        
        function roiRenumberROIs(obj)
            % Renumbers ROI IDs to be consecutive (starting with '1').
            
            global state;
            
            roiIDs = obj.roiDataStructure.keys();
            roiIDs(1) = []; % skip the root entry
            for i = 1:length(roiIDs)
                if i == roiIDs{i}
                    continue;
                end
                
                obj.roiDataStructure(i) = obj.roiDataStructure(roiIDs{i});
                obj.roiDataStructure.remove(roiIDs{i});
                
                % update the marker ID
                roiStruct = obj.roiDataStructure(i);
                set(roiStruct.hMarkerLabel,'String',['#' num2str(i)]);
                
                % update any childrens' 'parent' field
                if isfield(roiStruct,'children')
                    children = roiStruct.children;
                    for j = 1:length(children)
                        childROIStruct = obj.roiDataStructure(children(j));
                        childROIStruct.parentROIID = i;
                        obj.roiDataStructure(children(j)) = childROIStruct;
                    end
                end
                
                % udpate this ROI's parent's 'children' field
                parentROIStruct = obj.roiDataStructure(roiStruct.parentROIID);
                parentROIStruct.children(parentROIStruct.children == roiIDs{i}) = [];
                parentROIStruct.children = [parentROIStruct.children i];
                obj.roiDataStructure(roiStruct.parentROIID) = parentROIStruct;
                
                % update any cycle-iterations referencing this ROI
                cycIndices = obj.hController{1}.roiID2cycIterationIdx(roiIDs{i});
                for j = 1:length(cycIndices)
                    state.cycle.cycleTableStruct(cycIndices(j)).motorActionID = i;
                end
                
            end
            
            % Update the data displayed in the ROI table.
            obj.hController{1}.updateTableViews({'roi' 'cycle'});
        end
        
        function roiLoad(obj)
            % Loads ROI/Position data from the selected .roi file.
            %
            % TODO: this shares a lot of logic with loadCurrentCycle()...should these be refactored/merged?
            
            try
                %Prompt user to select file
                startPath = obj.getLastPath('roiLastPath');
                [fname, pname]=uigetfile({'*.roi'},'Choose ROI File...',startPath);
                if isnumeric(fname)
                    return
                else
                    [~,filenameNoExtension,~] = fileparts(fname);
                    
                    if ~strcmp(pname,startPath)
                        obj.setLastPath('roiLastPath',pname);
                    end
                end
            catch ME
                ME.throwAsCaller();
            end
            
            % handle a 'cancel' click
            if isnumeric(fname) && fname == 0 && isnumeric(pname) && pname == 0
                return;
            end
            
            obj.roiLoading = true;
            
            try
                [fID, message] = fopen(fullfile(pname,fname));
            catch ME
                error('Unable to open file.');
            end
            if fID < 0
                error('Unable to open file: %s.',message);
            end
            
            obj.roiPath = pname;
            [~,obj.roiName,~] = fileparts(fname);
            
            % clear any existing data...
            obj.roiClearAll();
            
            obj.roiSuppressUpdates = true;
            
            % initialize some regular expressions we'll need:
            headerExp = '^(\D*)$'; % matches the secton 'header' (i.e. 'ROI' or 'Position')
            rowExp = '^(\d+)\t((.+)\t(.+)\t)+'; % matches a row beginning with an integer ID
            rspExp = '^\t(RSPs)\t((.+)\t(.+)\t)+'; % matches an RSP row
            keyValExp = '([^\t]+)\t([^\t]+)\t'; % captures all key/val pairs in a row
            
            currentLine = fgetl(fID);
            while ischar(currentLine)
                tokens = regexp(currentLine,headerExp,'tokens','once');
                if ~isempty(tokens) % we have a section delimiter...
                    prefix = tokens{1};
                else % we have a row entry or an RSP entry
                    tokens = regexp(currentLine,rowExp,'tokens','once');
                    if ~isempty(tokens)
                        rowID = str2double(tokens{1});
                        rowStruct = struct();
                        isRSP = false;
                    else
                        tokens = regexp(currentLine,rspExp,'tokens','once');
                        if ~isempty(tokens)
                            isRSP = true;
                            rowStruct.RSPs = struct();
                        else
                            currentLine = fgetl(fID);
                            continue;
                        end
                    end
                    
                    keyValLine = tokens{2};
                    matches = regexp(keyValLine,keyValExp,'match');
                    for match = matches
                        keyVal = regexp(match{:},'(.+)\t(.+)\t','tokens');
                        key = keyVal{1}{1};
                        val = keyVal{1}{2};
                        
                        if strcmpi(val,'nan')
                            val = NaN;
                        elseif ~isnan(str2double(val))
                            val = str2double(val); % numeric value
                        else
                            % string value, do nothing
                        end
                        
                        if isRSP
                            rowStruct.RSPs.(key) = val;
                        else
                            rowStruct.(key) = val;
                        end
                    end
                    
                    % TODO: unify roiAddNew() and roiAddPosition() so that signatures match--thus avoiding this test?
                    if strcmpi(prefix,'roi') && isRSP
                        obj.roiAddNew(rowStruct,rowID);
                    elseif strcmpi(prefix,'position')
                        obj.roiAddPosition(rowStruct,rowID);
                    end
                end
                currentLine = fgetl(fID);
            end
            
            fclose(fID);
            
            obj.shownROI = obj.ROI_ROOT_ID;
            
            obj.roiLoading = false;
            
            obj.roiSuppressUpdates = false;
            obj.roiUpdateROITable();
            obj.roiUpdatePositionTable();
        end
        
        function roiSave(obj)
            % Saves all defined ROI and Position entries.
            
            if isempty(obj.roiName) || isempty(obj.roiPath)
                obj.roiSaveAs();
                return;
            end
            
            % open the file
            try
                [fID, message] = fopen(fullfile(obj.roiPath,[obj.roiName '.roi']), 'wt');
            catch ME
                error('Unable to open file.');
            end
            if fID < 0
                error('Unable to open file: %s.',message);
            end
            
            % Write the ROI data
            prefixes = {'ROI' 'Position'};
            for i = 1:length(prefixes)
                prefix = prefixes{i};
                
                fprintf(fID,'%s\n',prefix);
                
                dataStructure = obj.([lower(prefix) 'DataStructure']);
                ids = dataStructure.keys();
                ids([ids{:}] < 1) = []; % remove the ROOT entry
                
                for j = 1:length(ids)
                    fprintf(fID,'%d\t',ids{j});
                    
                    rowStruct = dataStructure(ids{j});
                    paramNames = setdiff(fieldnames(rowStruct),{'RSPs' 'children' 'hMarker' 'hMarkerLabel'}); % 'children' will be reconstructed, pointless to save marker handles...
                    
                    % write all params
                    for k = 1:length(paramNames)
                        paramName = paramNames{k};
                        if isfield(rowStruct,paramName) && ~isempty(rowStruct.(paramName))
                            obj.fprintfSmart(fID,paramName,rowStruct.(paramName));
                        end
                    end
                    
                    % now write any RSPS
                    if isfield(rowStruct,'RSPs')
                        fprintf(fID,'\n\tRSPs\t');
                        rspNames = fieldnames(rowStruct.RSPs);
                        for k = 1:length(rspNames)
                            obj.fprintfSmart(fID,rspNames{k},rowStruct.RSPs.(rspNames{k}));
                        end
                    end
                    
                    fprintf(fID,'\n');
                end
                fprintf(fID,'\n');
            end
            fclose(fID);
        end
        
        function roiSaveAs(obj)
            startPath = obj.getLastPath('roiLastPath');
            [fname, pname]=uiputfile({'*.roi'},'Choose ROI File...',startPath);
            if isnumeric(fname)
                return;
            end
            
            [~,fname,ext] = fileparts(fname);
            if isempty(ext) || ~strcmpi(ext,'.roi')
                fprintf(2,'WARNING: Invalid file extension found. Cannot open ROI file.\n');
                return;
            end
            
            obj.roiName =fname;
            obj.roiPath=pname;
            obj.roiSave();
        end
        
        function roiLoadBaseConfig(obj)
            % Calls through to roiGotoROI() to load the base configuration.
            
            obj.roiGotoROI(obj.ROI_BASE_ID);
        end
        
        function roiSetBaseConfig(obj,isInit)
            % Sets the 'Base' ROI.
            %
            % isInit: a boolean value that, if true, specifies that this function is being called during ScanImage initialization.
            %
            
            global state;
            
            if nargin < 2 || isempty(isInit)
                isInit = false;
            end
            
            baseROIStruct = obj.roiDataStructure(obj.ROI_BASE_ID);
            
            if isInit && isfield(baseROIStruct,'RSPs')
                % if the 'RSP' field exists, it means we've already loaded a base-config from a USR file.
                return;
            end
            
            % Cache all current scan parameters
            for i = 1:length(obj.scanParameterNames)
                baseROIStruct.RSPs.(obj.scanParameterNames{i}) = state.acq.(obj.scanParameterNames{i});
            end
            
            obj.roiDataStructure(obj.ROI_BASE_ID) = baseROIStruct;
            
            obj.roiUpdateROITable();
            
            %Hack invoking GUI side effects -- ideally should have a property update bound to SI3Controller
            updateScanAngleMultiplierSlow();
        end
        
        function roiSetMarkerColor(obj, roiID, color)
            % Sets/resets the color of the specified ROI marker.
            %
            % roiStruct: the ID of the ROI to be updated.
            % color: a 3-vector representing the new color to be applied.
            
            if nargin < 2 || isempty(roiID)
                return;
            end
            
            if ~obj.roiDataStructure.isKey(roiID)
                return;
            end
            roiStruct = obj.roiDataStructure(roiID);
            
            if nargin < 3 || isempty(color)
                color = obj.markerColors(roiStruct.type);
            end
            
            % make sure the marker handle is still valid
            if ~isfield(roiStruct,'hMarker') || ~isfield(roiStruct,'hMarkerLabel') ...
                    || ~all(ishandle(roiStruct.hMarker)) || ~ishandle(roiStruct.hMarkerLabel)
                return;
            end
            
            % update the markers
            set(roiStruct.hMarkerLabel,'Color',color);
            if strcmp(roiStruct.type,'point')
                set(roiStruct.hMarker,'EdgeColor',color);
            else
                set(roiStruct.hMarker,'Color',color);
            end
        end
        
        function roiShiftPosition(obj,axes)
            %
            
            global state;
            
            if ~state.motor.motorOn
                return;
            end
            
            if nargin < 2 || isempty(axes)
                error('Specify the axes.');
            end
            
            if isempty(obj.selectedPositionID) || ~obj.positionDataStructure.isKey(obj.selectedPositionID)
                error('Select a Position.');
            end
            
            selectedPositionStruct = obj.positionDataStructure(obj.selectedPositionID);
            
            % update current motor position, and then compute the delta against the selected position
            motorGetPosition();
            
            dx = state.motor.absXPosition - selectedPositionStruct.motorX;
            dy = state.motor.absYPosition - selectedPositionStruct.motorY;
            if strcmpi(axes,'xyz')
                dz = state.motor.absZPosition - selectedPositionStruct.motorZ;
                if state.motor.dimensionsXYZZ && ~obj.posnIgnoreSecZ
                    dzz = state.motor.absZZPosition - selectedPositionStruct.motorZZ;
                else
                    dzz = 0;
                end
            else
                dz = 0;
                dzz = 0;
            end
            
            % iterate through all Position entries, adding the offset
            positionIDs = obj.positionDataStructure.keys();
            for i = 1:length(positionIDs)
                positionStruct = obj.positionDataStructure(positionIDs{i});
                positionStruct.motorX = positionStruct.motorX + dx;
                positionStruct.motorY = positionStruct.motorY + dy;
                if strcmpi(axes,'xyz')
                    positionStruct.motorZ = positionStruct.motorZ + dz;
                    if state.motor.motorZEnable
                        positionStruct.motorZZ = positionStruct.motorZZ + dzz;
                    end
                end
                obj.positionDataStructure(positionIDs{i}) = positionStruct;
            end
            obj.roiUpdatePositionTable();
        end
        
        function roiType = computeROIType(~,scanAngularRangeFast,scanAngularRangeSlow)
            
            if scanAngularRangeFast == 0 && scanAngularRangeSlow == 0
                roiType = 'point';
            elseif scanAngularRangeSlow == 0
                roiType = 'line';
            elseif scanAngularRangeFast == scanAngularRangeSlow
                roiType = 'square';
            else
                roiType = 'rect';
            end
        end
        
        function vals = getOverridableFcns(obj)
            % Returns a cell array of all currently overridable ScanImage functions.
            vals = obj.overridableFcns;
        end
        
        function tf = isFcnOverridden(obj,overriddenFcn)
            if isequal(obj.(['h' upper(overriddenFcn(1)) overriddenFcn(2:end)]),eval(['@' overriddenFcn]))
                tf = false;
            else
                tf = true;
            end
        end
        
        function registerOverrideFcn(obj,overriddenFcn,hOverrideFcn)
            % Adds a user-defined override function to the global list of overrides.
            
            if nargin < 3 || isempty(overriddenFcn) || isempty(hOverrideFcn)
                error('Please specify all arguments.');
            end
            
            % Verify that that the function handle refers to a valid and overridable function.
            assert(strcmp(class(hOverrideFcn),'function_handle'),'Invalid function handle.');
            assert(ismember(overriddenFcn,obj.getOverridableFcns()),[overriddenFcn ' is not overridable.']);
            
            % Point to the new function handle
            obj.(['h' upper(overriddenFcn(1)) overriddenFcn(2:end)]) = hOverrideFcn;
        end
        
        function unregisterOverrideFcn(obj,overriddenFcn)
            
            if nargin < 2 || isempty(overriddenFcn)
                error('You must provide a function name to be unregistered.');
            end
            
            % Point our function handle to the original ScanImage function
            obj.(['h' upper(overriddenFcn(1)) overriddenFcn(2:end)]) = eval(['@' overriddenFcn]);
        end
        
        function flagListenerAbort(obj)
            obj.listenerAbortFlag = true;
            pause(0.2);
        end
        
        function path = getLastPath(obj,path)
            path = obj.getClassDataVar(path);
        end
        
        function setLastPath(obj,varName,val)
            obj.setClassDataVar(varName,val);
        end
        
        function viewAll_Callback(obj,hObject,eventdata,handles)
            [obj.roiCategoriesView(:)] = deal(logical(get(hObject,'Value')));
        end
        
        function autoSelectAll_Callback(obj,hObject,eventdata,handles)
            [obj.roiCategoriesAutoSelect(:)] = deal(logical(get(hObject,'Value')));
        end
        
        function loadUSRProperties(obj,filename)
            % Loads all USR-bound property values.
            %
            % filename: The USR file being opened.
            %
            
            if nargin < 2 || isempty(filename)
                error('Specify a filename.');
            end
            
            % open file and read in by line ignoring comments
            fid=fopen(filename, 'r');
            if fid==-1
                obj.fullError('Invalid file',['Error: Unable to open file: ' filename ]);
                return;
            end
            
            fileCell = textscan(fid, '%s', 'commentstyle', 'matlab', 'delimiter', '\n');
            fileCell = fileCell{1};
            
            % skip forward to the 'structure SI3' line...
            i = 1;
            while ~strcmp(fileCell{i},'structure SI3')
                i = i + 1;
                if i > length(fileCell)
                    return;
                end
            end
            i = i + 1;
            
            % concatenate all SI3 lines
            assignmentString = '';
            while ~strcmp(fileCell{i},'endstructure')
                assignmentString = sprintf('%s%s\n', assignmentString, fileCell{i});
                i = i + 1;
            end
            
            fclose(fid);
            
            most.util.assignments2StructOrObj(assignmentString, obj);
            
            obj.roiUpdateROITable();
        end
        
        function saveUSRProperties(obj,fid)
            % Saves all USR-bound properties (listed in 'usrBoundProperties'),
            % appending them to the end of the USR file represented by 'fid'.
            %
            % fid: A handle to the USR file being saved.
            %
            
            if nargin < 2 || isempty(fid)
                error('Invalid fid.');
            end
            
            % write a string containing prop/val assignments for all USR-bound properties.
            fprintf(fid, 'structure SI3\n');
            fprintf(fid, most.util.structOrObj2Assignments(obj,'obj',obj.usrBoundProperties));
            fprintf(fid, 'endstructure\n');
            
            % (don't close 'fid'--it's still in use...)
        end
        
    end
    
    
    methods (Hidden, Static)
        function fullError(statusString,messageString,doGenerateException)
            % Prints an error message to the SI3 status string, and optionally generates a Matlab error.
            %
            % statusString: the message to print to the SI3 status string.
            % messageString: the message to print to the Matlab command line.
            % doGenerateError: a boolean that, if true, will cause the function to generate a matlab exception.
            
            if nargin < 3 || isempty(doGenerateException)
                doGenerateException = false;
            end
            
            if nargin < 2
                messageString = '';
            end
            
            textColor = [1 0 0];
            
            if isempty(statusString)
                statusString = 'ERROR';
            end
            setStatusString(statusString,textColor);
            
            if isempty(messageString)
                errorString = statusString;
            else
                errorString = messageString;
            end
            
            ME = MException('SI3:ERROR',errorString);
            
            if doGenerateException
                ME.throwAsCaller();
            else
                most.idioms.reportError(ME);
            end
        end
        
        function consoleError(messageString)
            % Prints error information to the matlab command window (console).
            %
            % messageString: the string to print.
            
            global state;
            
            state.hSI.fullError('',messageString,false);
        end
        
    end
    
    
    methods(Static,Access=private)
        
        function fprintfSmart(fID,paramName,val)
            
            if isnumeric(val)
                formatString = '%s\t%d\t';
            elseif ischar(val)
                formatString = '%s\t%s\t';
            elseif islogical(val)
                formatString = '%s\t%s\t';
                if val
                    val = 'true';
                else
                    val = 'false';
                end
            else
                return;
            end
            
            fprintf(fID,formatString,paramName,val);
        end
        
    end
    
    %% DEVELOPER EVENTS
    
    events
        dummyEvent;
    end
    
    
end


function s = zlclInitPropMetadata()

s.activePositionID = struct('Classes','numeric');
s.shownPositionID = struct('Classes','numeric');
s.activeROIID = struct('Classes','numeric','AllowEmpty',1);
s.shownROI = struct('Classes','numeric');
s.lineScanEnable = struct('Classes','binaryflex');

s.roiName = struct('Classes','string');
s.roiAngleToMicronsFactor = struct('Classes','numeric');
s.roiDisplayDepth = struct('Classes','numeric');
s.roiDisplayedChannel = struct('Classes','string');

s.motorStepSizeX = struct('Classes','numeric');
s.motorStepSizeY = struct('Classes','numeric');
s.motorStepSizeZ = struct('Classes','numeric');
s.motorStepSizeZZ = struct('Classes','numeric');

s.roiIDs = struct('Classes','string');
s.roiPositionIDs = struct('Classes','numeric');
s.roiTypes = struct('Classes','string');
s.roiZoomFactors = struct('Classes','numeric');
s.roiRotations = struct('Classes','numeric');
s.roiShifts = struct('Classes','string');
s.roiScanAngleMultipliers = struct('Classes','string');
s.roiGotoOnAdd = struct('Classes','binaryflex');
s.roiSnapOnAdd = struct('Classes','binaryflex');

s.roiPositionToleranceX = struct('Classes','numeric');
s.roiPositionToleranceY = struct('Classes','numeric');
s.roiPositionToleranceZ = struct('Classes','numeric');
s.roiPositionToleranceZZ = struct('Classes','numeric');

% binary ROI menu settings
s.roiUseMIPForMRI = struct('Classes','binaryflex');
s.roiShowMarkerNumbers = struct('Classes','binaryflex');
s.roiGotoOnAdd = struct('Classes','binaryflex');
s.roiSnapOnAdd = struct('Classes','binaryflex');
s.roiGotoOnSelect = struct('Classes','binaryflex');
s.roiSnapOnSelect = struct('Classes','binaryflex');
s.roiWarnOnMove = struct('Classes','binaryflex');

%Other binary ROI/Posn settings
s.roiShowAbsoluteCoords = struct('Classes','binaryflex');
s.posnIgnoreSecZ = struct('Classes','binaryflex');

s.shownPositionID = struct('DependsOn',{{'shownROI'}});
s.shownRotation = struct('DependsOn',{{'shownROI'}});
s.roiIsShownPositionNotActive = struct('DependsOn',{{'activePositionID' 'shownPositionID'}});

end
