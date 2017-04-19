classdef Display < scanimage.interfaces.Component
    %% USER PROPS
    properties (SetObservable)
        displayRollingAverageFactor = 1;         % Number of frames averaged (using a simple moving average) for display purposes. Value must be greater or equal to acqNumAveragedFrames.
        displayRollingAverageFactorLock = false; % If true, lock acqNumAveragedFrames = displayRollingAverageFactor
        displayTiling = true;                    % Scalar logical. If false, arrays of values for displayFrameBatchSelection will be "flattened" into a single tile.
        displayFrameBatchFactor = 1;             % The number of frames to batch together for selective or tiled display
        displayFrameBatchSelection = 1;          % The frame or frames to display within each frame batch
        displayFrameBatchSelectLast = false;      % If true, lock displayFrameBatchFactor = displayFrameBatchSelection
        displayFrameBatchSelectAll = true;      % If true, lock displayFrameBatchFactor = displayFrameBatchSelection
        displayFrameBatchFactorLock = false;     % If true, lock displayFrameBatchFactor = displayRollingAverageFactor
        displayVolumeLock = 0;                   % If true, lock displayFrameBathFactor = # of slices in Volume

        channelsMergeEnable = false;             % Scalar logical. If true, the channels merge window is updated.
        channelsMergeFocusOnly = false;          % Scalar logical. If true, the channels merge image is not updated during GRAB/LOOP acquisitions.
        
        roiDisplayEdgeColor = 'blue';
        roiDisplayEdgeAlpha = 1;
        roiProjectionDisplayEdgeColor = 'blue';
        roiProjectionDisplayEdgeAlpha = 1;
        
        renderer = 'auto';                       % one of {'auto','painters','opengl'}
        hAxes = {};
        hFigs = [];
    end
    
    properties (SetObservable, Hidden)
        % hide for now
        scanfieldDisplays = [];
        frameRateDisplay = false;
        frameRateAverageFactor = 1;
        frameRateAverageBuffer = [];
        lastDisplayUpdate = [];
        lastFrame;
        lastFrameNumber = 0;
        frameRate;
    end
    
    %% FRIEND PROPS    
    properties (Hidden, SetAccess=private)
        
        hMergeFigs = [];
    end
    
    properties (SetObservable, Hidden)
        forceRoiDisplayTransform = false;        % If true, rois will be drawn at correct location within the normalized coordinate system. If false, rois will fill entire figure
        needReset = false;
        
        %Hidden because these should be set from hChannels
        chan1LUT = [0 100];                      % Channel 1 intensity lookup table extents.
        chan2LUT = [0 100];                      % Channel 2 intensity lookup table extents.
        chan3LUT = [0 100];                      % Channel 3 intensity lookup table extents.
        chan4LUT = [0 100];                      % Channel 4 intensity lookup table extents.
    end
        
    %% INTERNAL PROPS
    properties (Hidden, SetAccess=private)
        
        hMergeAxes = [];
        scanfieldDisplayFrameSelection;
        rollingStripeDataBufferDirty;           % Indicates that there is new data in the buffer that has not been displayed
        rollingStripeDataBuffer;                % Buffer used for stripe (display) averaging computation. Stored as double type.
        lastAcqStripeDataBuffer;                % Stores last stripe data at the end of acq for copy purposes
        lastAcqScannerName = '';                % Stores name of scanner that captured data in lastAcqStripeDataBuffer
        mergeStripeData;                        % Stripe data type containing merged contents of the latest stripeData displayed.
        mergeStripeDataBuffer;                  % Cell array containing mergedStripeData. This is for tiled display of stripes.
        frameAverageIndex = 0;                  % Index used to reference a specific RoiData object within in a rolling window.
        frameCounterForBuffer = 1;              % Number of frames acquired for this specific display averaging buffer configuration. This is reset when the buffers are resized.
        stripeDataBuffer = {};                  % Cell array containing most recently acquired stripeBufferLength stripeData elements.        
        stripeDataBufferPointer = 1;            % Index of current element in stripeDataBuffer
        displayLastFrameDisplayed = 0;          % Internal accounting of last frame displayed for frame display function.
        zLocked = true;                         % Indicates that the current batch selection factor matches the z series, therefore selected frames will be at consistent z planes
        
        resetReq = {};
        duringInit = false;
        displayZs = [];
    end
    
    properties (Hidden, SetAccess=?most.Model, Dependent)
        stripeBufferLength;                     % Length of running buffer used to store most-recently acquired stripes.
    end
    
    properties (Constant, Hidden, Access=private)
        MAXIMUM_CHANNELS = 10;                  % Maximum number of total channels including virtual, logical, and physical.
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = ziniInitPropAttributes();
        mdlInitSetExcludeProps;
        mdlHeaderExcludeProps = {'roiDisplayEdgeColor' 'roiDisplayEdgeAlpha'...
            'roiProjectionDisplayEdgeColor' 'roiProjectionDisplayEdgeAlpha'};
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (scanimage.interfaces.Component)
    properties (SetAccess = protected, Hidden)
        numInstances = 0;
    end
    
    properties (Constant, Hidden)
        COMPONENT_NAME = 'Display';                     % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
        PROP_TRUE_LIVE_UPDATE = {'displayRollingAverageFactor',... % Cell array of strings specifying properties that can be set while the component is active
            'displayRollingAverageFactorLock','displayFrameBatchFactor',...
            'displayFrameBatchSelection','displayFrameBatchSelectLast','displayFrameBatchSelectAll',...
            'displayFrameBatchFactorLock','displayVolumeLock',...
            'chan1LUT','chan2LUT','chan3LUT','chan4LUT',...
            'channelsMergeEnable','channelsMergeFocusOnly',...
            'renderer','forceRoiDisplayTransform'...
            'roiDisplayEdgeColor' 'roiDisplayEdgeAlpha'...
            'roiProjectionDisplayEdgeColor' 'roiProjectionDisplayEdgeAlpha'};
        PROP_FOCUS_TRUE_LIVE_UPDATE = {};               % Cell array of strings specifying properties that can be set while focusing
        DENY_PROP_LIVE_UPDATE = {};                     % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
        FUNC_TRUE_LIVE_EXECUTION = {'displayChannels','displayRaster','displayStripe','displayRoi'};  % Cell array of strings specifying functions that can be executed while the component is active
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {};            % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {};                  % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end
    
    %% LIFECYCLE
    methods (Hidden)
        function obj = Display(hSI)
            obj = obj@scanimage.interfaces.Component(hSI,[]);
        end
        
        function delete(obj)
            delete(obj.hFigs); % hFigs is parent to hAxes is parent to hImages. Deleting hFigs deletes its children too
            delete(obj.hMergeFigs);
        end
    end
    
    %% PROP ACCESS
    methods
        function set.renderer(obj,val)
            val = lower(val);
            val = obj.validatePropArg('renderer',val);
            if obj.componentUpdateProperty('renderer',val)
                switch val
                    case {'auto'}
                        set([obj.hFigs obj.hMergeFigs],'RendererMode','auto');                    
                    case {'painters','opengl'}
                        set([obj.hFigs obj.hMergeFigs],'Renderer',val,...
                                  'RendererMode','manual');
                    otherwise
                        %Note: zbuffer is not allowed, since it is
                        %deprecated in Matlab 2014b and later
                        error('Unknown renderer: %s',val);
                end
                obj.renderer = val;
            end
        end
        
        function set.forceRoiDisplayTransform(obj,val)
            val = obj.validatePropArg('forceRoiDisplayTransform',val);
            if obj.componentUpdateProperty('forceRoiDisplayTransform',val)
                oldval = obj.forceRoiDisplayTransform;
                obj.forceRoiDisplayTransform = val;
                
                if oldval ~= val
                    obj.resetDisplayFigs(obj.hSI.hChannels.channelDisplay,true);
                end
            end
        end
                
        function set.displayTiling(obj,val)
            val = obj.validatePropArg('displayTiling',val);
            changeVal = ~isequal(val,obj.displayTiling);
            
            % Set property value with validated value.
            obj.displayTiling = val;
            
            %Dependencies
            if changeVal
                obj.updateFrameBatchProps(true);
            end
        end
        
        function set.displayRollingAverageFactor(obj,val)
            if obj.componentUpdateProperty('displayRollingAverageFactor',val)
                %Enforce displayRollingAverageFactorLock constraint
                if obj.displayRollingAverageFactorLock
                    allowedVal = obj.zprpLockDisplayRollAvgFactor();
                    if val ~= allowedVal
                        return;
                    end
                end
                
                %Proceed with set
                val = obj.validatePropArg('displayRollingAverageFactor',val); %allow while running
                obj.displayRollingAverageFactor = val;
                obj.chan1LUT = obj.chan1LUT;
                obj.chan2LUT = obj.chan2LUT;
                obj.chan3LUT = obj.chan3LUT;
                obj.chan4LUT = obj.chan4LUT;
                
                obj.zprvResetBuffers();

                %Dependencies
                if obj.displayFrameBatchFactorLock
                    obj.updateFrameBatchProps();
                end
            end
        end
        
        function set.displayRollingAverageFactorLock(obj,val)
            val = obj.validatePropArg('displayRollingAverageFactorLock',val); %Allow while running
            if obj.componentUpdateProperty('displayRollingAverageFactorLock',val)
                obj.displayRollingAverageFactorLock = val;
                
                %Dependencies
                if val
                    obj.zprpLockDisplayRollAvgFactor();
                    obj.updateFrameBatchProps();
                end
            end
        end
        
        function set.displayFrameBatchFactor(obj,val)
            if obj.componentUpdateProperty('displayFrameBatchFactor',val)
                changeVal = ~isequal(val,obj.displayFrameBatchFactor);
                
                %Dependencies
                if changeVal
                    obj.displayFrameBatchFactor = val;
                    obj.updateFrameBatchProps();
                end
            end
        end
        
        function set.displayFrameBatchSelection(obj,val)
            if obj.componentUpdateProperty('displayFrameBatchSelection',val)
                changeVal = ~isequal(val,obj.displayFrameBatchSelection);
                
                %Dependencies
                if changeVal
                    obj.displayFrameBatchSelection = val;
                    obj.updateFrameBatchProps(true);
                end
            end
        end
        
        function set.displayFrameBatchSelectLast(obj,val)
            val = obj.validatePropArg('displayFrameBatchSelectLast',val);
            if obj.componentUpdateProperty('displayFrameBatchSelection',val)
                obj.displayFrameBatchSelectLast = val;
                
                %Dependencies
                if val
                    obj.displayFrameBatchSelectAll = false;
                    obj.updateFrameBatchProps();
                end
            end
        end
        
        function set.displayFrameBatchSelectAll(obj,val)
            val = obj.validatePropArg('displayFrameBatchSelectLast',val);
            if obj.componentUpdateProperty('displayFrameBatchSelection',val)
                obj.displayFrameBatchSelectAll = val;
                
                %Dependencies
                if val
                    obj.updateFrameBatchProps();
                end
            end
        end
        
        function set.displayFrameBatchFactorLock(obj,val)
            val = obj.validatePropArg('displayFrameBatchFactorLock',val);
            if obj.componentUpdateProperty('displayFrameBatchFactorLock',val)
                obj.displayFrameBatchFactorLock = val;
                
                %Dependencies
                if val
                    obj.updateFrameBatchProps();
                end
            end
        end
        
        function set.displayVolumeLock(obj,val)
            val = obj.validatePropArg('displayVolumeLock',val);
            if obj.componentUpdateProperty('displayVolumeLock',val)
                obj.displayVolumeLock = val;
                
                %Dependencies
                if val
                    obj.updateFrameBatchProps();
                end
            end
        end
        
        function set.chan1LUT(obj,val)
            val = obj.validatePropArg('chan1LUT',val);
            if obj.componentUpdateProperty('chan1LUT',val)
                val = obj.zprvCoerceLutToAdcRange(val);
                obj.chan1LUT = val;
                
                if obj.hSI.hChannels.channelsAvailable >= 1
                    obj.zprpUpdateChanLUT(1,val);
                end
            end
        end
        
        function set.chan2LUT(obj,val)
            val = obj.validatePropArg('chan2LUT',val);
            if obj.componentUpdateProperty('chan2LUT',val)
                val = obj.zprvCoerceLutToAdcRange(val);
                obj.chan2LUT = val;
                if obj.hSI.hChannels.channelsAvailable >= 2
                    obj.zprpUpdateChanLUT(2,val);
                end
            end
        end
        
        function set.chan3LUT(obj,val)
            val = obj.validatePropArg('chan3LUT',val);
            if obj.componentUpdateProperty('chan3LUT',val)
                val = obj.zprvCoerceLutToAdcRange(val);
                obj.chan3LUT = val;
                
                if obj.hSI.hChannels.channelsAvailable >= 3
                    obj.zprpUpdateChanLUT(3,val);
                end
            end
        end
        
        function set.chan4LUT(obj,val)
            val = obj.validatePropArg('chan4LUT',val);
            if obj.componentUpdateProperty('chan4LUT',val)
                val = obj.zprvCoerceLutToAdcRange(val);
                obj.chan4LUT = val;
                
                if obj.hSI.hChannels.channelsAvailable >= 4
                    obj.zprpUpdateChanLUT(4,val);
                end
            end
        end
        
        function set.channelsMergeEnable(obj,val)
            val = obj.validatePropArg('channelsMergeEnable',val); %allow during acq
            if obj.componentUpdateProperty('channelsMergeEnable',val)
                obj.channelsMergeEnable = val;
                if val
                    obj.resetDisplayFigs([],true); %Resets merge figure, setting up tiling, etc
                else
                    set(obj.hMergeFigs,'Visible','off');
                end
            end
        end
        
        function set.channelsMergeFocusOnly(obj,val)
            val = obj.validatePropArg('channelsMergeFocusOnly',val);
            if obj.componentUpdateProperty('channelsMergeFocusOnly',val)
                obj.channelsMergeFocusOnly = val;
            end
        end
        
        function val = get.stripeBufferLength(obj)
            val = obj.displayRollingAverageFactor * obj.displayFrameBatchFactor;
        end
        
        function set.roiDisplayEdgeColor(obj,v)
            v = obj.validatePropArg('roiDisplayEdgeColor',v);
            if obj.componentUpdateProperty('roiDisplayEdgeColor',v)
                obj.roiDisplayEdgeColor = v;
                obj.resetActiveDisplayFigs(true);
            end
        end
        
        function set.roiDisplayEdgeAlpha(obj,v)
            v = obj.validatePropArg('roiDisplayEdgeAlpha',v);
            if obj.componentUpdateProperty('roiDisplayEdgeAlpha',v)
                obj.roiDisplayEdgeAlpha = v;
                obj.resetActiveDisplayFigs(true);
            end
        end
        
        function set.roiProjectionDisplayEdgeColor(obj,v)
            v = obj.validatePropArg('roiProjectionDisplayEdgeColor',v);
            if obj.componentUpdateProperty('roiProjectionDisplayEdgeColor',v)
                obj.roiProjectionDisplayEdgeColor = v;
                obj.resetActiveDisplayFigs(true);
            end
        end
        
        function set.roiProjectionDisplayEdgeAlpha(obj,v)
            v = obj.validatePropArg('roiProjectionDisplayEdgeAlpha',v);
            if obj.componentUpdateProperty('roiProjectionDisplayEdgeAlpha',v)
                obj.roiProjectionDisplayEdgeAlpha = v;
                obj.resetActiveDisplayFigs(true);
            end
        end
        
        function set.scanfieldDisplays(obj, v)
            v = obj.validatePropArg('scanfieldDisplays',v);
            if obj.componentUpdateProperty('scanfieldDisplays',v)
                obj.scanfieldDisplays = v;
            end
        end
        
        function v = get.needReset(obj)
            zsChanged = (numel(obj.hSI.hStackManager.zs) ~= numel(obj.displayZs)) || any(obj.hSI.hStackManager.zs ~= obj.displayZs);
            v = obj.needReset || zsChanged;
        end
        
        function data = get.lastFrame(obj)
            data = obj.stripeDataBuffer{obj.stripeDataBufferPointer};
        end
        
        function set.lastFrame(obj, data)
            obj.stripeDataBuffer{obj.stripeDataBufferPointer} = data;
        end
    end
    
    %% USER METHODS
    methods
        function resetActiveDisplayFigs(obj,preserveCameraProps)
            if nargin < 2 || isempty(preserveCameraProps)
                preserveCameraProps = false;
            end
            
            obj.resetDisplayFigs([], [], [], preserveCameraProps);
%             obj.resetScanfieldDisplayFigs();
        end
        
        function resetScanfieldDisplayFigs(obj,figsToReset)
            if nargin < 2 || isempty(figsToReset)
                if isempty(obj.scanfieldDisplays)
                    return;
                else
                    figsToReset = obj.scanfieldDisplays([obj.scanfieldDisplays.visible]);
                end
            elseif ~isa(figsToReset,'scanimage.components.display.scanfieldDisplay')
                figsToReset = obj.scanfieldDisplays(figsToReset);
            end
            
            if ~obj.hSI.mdlInitialized || isempty(figsToReset)
                return;
            end
            
            arrayfun(@(x)x.resetDisplay(), figsToReset, 'UniformOutput', false);
        end
        
        function resetDisplayFigs(obj,chansToReset,resetMergeTF,channelsLUTVal,preserveCameraProps)
            if nargin < 2 || isempty(chansToReset)
                chansToReset = obj.hSI.hChannels.channelDisplay;
            end
            
            if (~obj.hSI.mdlInitialized && ~obj.duringInit) || isempty(chansToReset)
                return;
            end
            
            if nargin < 3 || isempty(resetMergeTF)
                resetMergeTF = obj.channelsMergeEnable;
            end
            
            if obj.hSI.hConfigurationSaver.cfgLoadingInProgress
                obj.needReset = true;
                return
            end
            
            if nargin < 4 || isempty(channelsLUTVal)
                channelsLUTVal = [];
            end
            
            if nargin < 5 || isempty(preserveCameraProps)
                preserveCameraProps = [];
            end
            
            persistent resetInProgress
            if isempty(resetInProgress) || ~resetInProgress
                resetInProgress = true;
                obj.needReset = false;                
                try
                    
                    preserveCameraProps = nargin > 4 && ~isempty(preserveCameraProps) && preserveCameraProps;
                    if preserveCameraProps
                        preserveCameraProps = true;
                        obj.saveCameraProps();
                    end
                    
                    if obj.displayTiling
                        numTiles = length(obj.displayFrameBatchSelection); %Number of tiles to be displayed
                    else
                        numTiles = 1;
                    end
                    
                    % fetch all Z's for the roiGroup.
                    if ~isempty(obj.hSI.hStackManager.zs)
                        zs = obj.hSI.hStackManager.zs;
                    else
                        zs = 0;
                    end
                    
                    
                    %Determine optimal tiling
                    columns = round(sqrt(numTiles));
                    rows = ceil(numTiles/columns);
                    tiling = [columns,rows];
                    
                    for i=1:length(chansToReset)
                        set(obj.hFigs(chansToReset(i)),'Name',obj.hSI.hChannels.channelName{chansToReset(i)});
                        obj.hAxes{chansToReset(i)} = obj.zprvPrepareDisplayAxesImages(obj.hFigs(chansToReset(i)),zs,tiling);
                    end
                    
                    
                    if resetMergeTF
                        obj.hMergeAxes = ...
                            obj.zprvPrepareDisplayAxesImages(obj.hMergeFigs,zs,tiling);
                    end
                    
                    if isempty(channelsLUTVal)
                        obj.chan1LUT = obj.chan1LUT;
                        obj.chan2LUT = obj.chan2LUT;
                        obj.chan3LUT = obj.chan3LUT;
                        obj.chan4LUT = obj.chan4LUT;
                    else
                        channelsLUTValLookup = channelsLUTVal';
                        for i = 1:min(obj.hSI.hScan2D.channelsAvailable,4);
                            obj.(sprintf('chan%dLUT',i)) = channelsLUTValLookup((i*2-1):(i*2));
                        end
                    end
                    
                    %dont restore if you are going to or from 1 plane
                    if preserveCameraProps
                        obj.restoreCameraProps();
                    end
                catch ME
                    resetInProgress = false;
                    ME.rethrow();
                end
                resetInProgress = false;
                obj.displayZs = obj.hSI.hStackManager.zs;
                
                if ~isempty(obj.resetReq)
                    vars = obj.resetReq;
                    obj.resetReq = {};
                    obj.resetDisplayFigs(vars{:})
                end
            else
                obj.resetReq = {chansToReset,resetMergeTF,channelsLUTVal,preserveCameraProps};
            end
        end
                
        function saveCameraProps(obj)
        end
        
        function restoreCameraProps(obj)
        end
        
        %%% Frame Decimation
        function val = zprpLockDisplayRollAvgFactor(obj)
            %Identify (and apply or return) constrained displayRollingAverageFactor value
            val = obj.displayRollingAverageFactor;
            
            constrainedRollAvgFactor = obj.hSI.hScan2D.logAverageFactor;
            if val ~= constrainedRollAvgFactor
                if constrainedRollAvgFactor == round(constrainedRollAvgFactor)
                    val = constrainedRollAvgFactor;
                else
                    val = 1;
                end
            end
            
            if nargout == 0
                obj.displayRollingAverageFactor = val;
            end
        end
    end
    
    %% FRIEND METHODS
    methods (Hidden,Access=?scanimage.interfaces.Class)
        
        function updateFrameBatchProps(obj, redraw)
            if nargin < 2 || isempty(redraw)
                redraw = false;
            end
            
            if obj.hSI.hRoiManager.mroiEnable || obj.displayVolumeLock
                obj.displayRollingAverageFactorLock = false;
                newDisplayFrameBatchFactor = obj.hSI.hStackManager.slicesPerAcq + obj.hSI.hFastZ.numDiscardFlybackFrames;
                SF = obj.hSI.hStackManager.slicesPerAcq;
            else
                if obj.displayFrameBatchFactorLock
                    newDisplayFrameBatchFactor = obj.displayRollingAverageFactor;
                else
                    newDisplayFrameBatchFactor = obj.displayFrameBatchFactor;
                end
                SF = newDisplayFrameBatchFactor;
            end
            
            if obj.displayVolumeLock || obj.displayFrameBatchSelectAll
                obj.displayFrameBatchSelectLast = false;
                newDisplayFrameBatchSelection = 1:SF;
            elseif obj.displayFrameBatchSelectLast
                newDisplayFrameBatchSelection = SF;
            else
                newDisplayFrameBatchSelection = unique(min(obj.displayFrameBatchSelection,SF));
            end
            
            if newDisplayFrameBatchFactor ~= obj.displayFrameBatchFactor
                obj.displayFrameBatchFactor = newDisplayFrameBatchFactor;
            end
            if numel(newDisplayFrameBatchSelection) ~= numel(obj.displayFrameBatchSelection) || any(newDisplayFrameBatchSelection ~= obj.displayFrameBatchSelection)
                obj.displayFrameBatchSelection = newDisplayFrameBatchSelection;
                redraw = false; %above line will cause a redraw
            end
            
            if redraw
                obj.zprvResetBuffers();
                obj.resetActiveDisplayFigs(true);
            end
        end

        function averageStripe(obj,newStripe)
            %**********************************************************
            %Identify current frame within the acquisition frame buffer
            %**********************************************************
            if newStripe.frameNumberAcq == 1
                obj.frameAverageIndex = 0;
                obj.frameCounterForBuffer = 1;
            elseif newStripe.startOfFrame
                obj.frameCounterForBuffer = obj.frameCounterForBuffer + 1;
            end

            if newStripe.startOfFrame
                obj.stripeDataBufferPointer = obj.stripeDataBufferPointer+1;
                if obj.stripeDataBufferPointer > obj.stripeBufferLength
                    obj.stripeDataBufferPointer = 1;
                end
            end
            
            if newStripe.startOfFrame && newStripe.endOfFrame
                % no stripes in this frame
                obj.lastFrame = newStripe;
            else
                newStripe.merge(obj.lastFrame);
                obj.lastFrame = newStripe;
            end
            
            frameBatchIdx = mod(obj.hSI.internalFrameCounter-1,obj.displayFrameBatchFactor) + 1;
            
            if isempty(newStripe.roiData)
                return;
            end
            
            obj.lastFrameNumber = obj.lastFrame.roiData{1}.frameNumberAcq;
            
            if newStripe.startOfFrame && frameBatchIdx == 1
                obj.frameAverageIndex = obj.frameAverageIndex + 1;
            end
            
            if obj.frameAverageIndex > obj.displayRollingAverageFactor
                obj.frameAverageIndex = 1;
            end
            
            if obj.frameAverageIndex < 1
                obj.frameAverageIndex = 1;
            end

            %[displayTF, tileIdx] = ismember(frameBatchIdx,obj.displayFrameBatchSelection);
            % the following code is 7x faster
            tileIdx = find(frameBatchIdx==obj.displayFrameBatchSelection);
%             sfTileIdx = find(frameBatchIdx==obj.scanfieldDisplayFrameSelection);
            sfTileIdx = false;
            displayTF = any(tileIdx) || any(sfTileIdx);
            
            if displayTF
                rollAveFactor = obj.displayRollingAverageFactor;
                if rollAveFactor > 1 %Display averaging enabled
				    rollingBufferDatatype = 'int32';
                    newStripe.castRoiData(rollingBufferDatatype);
                    if isempty(obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData)
                        %if any buffer element is empty it means buffers
                        %were just reset. fill all buffers with current
                        %image data to get the averaging started
                        if newStripe.startOfFrame && newStripe.endOfFrame
                            % no stripes in this frame
                            for i = 2:(obj.displayRollingAverageFactor+1)
                                obj.rollingStripeDataBuffer{frameBatchIdx}{i} = newStripe;
                            end
                            
                            %buffer #1 is the averaged data. stripeData is a
                            %handle class! we need a fresh unique piece of
                            %memory for the averaged data to reside in
                            averagedStripe = copy(newStripe);
                            averagedStripe.multiplyRoiData(rollAveFactor);
                            obj.rollingStripeDataBuffer{frameBatchIdx}{1} = averagedStripe;
                        else
                            % striping
                            newStripe.resetData();
                            newStripeCp = copy(newStripe);
                            newStripeCp.resetDataToZero();
                            for i = 2:(obj.displayRollingAverageFactor+1)
                                obj.rollingStripeDataBuffer{frameBatchIdx}{i} = newStripeCp;
                            end
                            obj.rollingStripeDataBuffer{frameBatchIdx}{2} = newStripeCp;
                            
                            %buffer #1 is the averaged data. stripeData is a
                            %handle class! we need a fresh unique piece of
                            %memory for the averaged data to reside in
                            averagedStripe = copy(newStripe);
                            %averagedStripe.multiplyRoiData(rollAveFactor);
                            obj.rollingStripeDataBuffer{frameBatchIdx}{1} = averagedStripe;                            
                        end
                    else
                        for iterRoiData = 1:length(obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData)
                            stripePos = newStripe.roiData{iterRoiData}.stripePosition;
                            for iterChannel = 1:length(obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.channels)
                                %this if/else structure is here to handle the case where selection factor ~= number of planes and there are flyback frames
                                if isempty(newStripe.roiData)
                                    %no new data to add
                                    if isempty(obj.rollingStripeDataBuffer{frameBatchIdx}{obj.frameAverageIndex+1}.roiData)
                                        %no old data to subtract
                                        %do nothing
                                    else
                                        %old data to subtract
                                        obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2)) = ...
                                            obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2)) - ...
                                            obj.rollingStripeDataBuffer{frameBatchIdx}{obj.frameAverageIndex+1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2));
                                    end
                                else
                                    %new data to add
                                    if isempty(obj.rollingStripeDataBuffer{frameBatchIdx}{obj.frameAverageIndex+1}.roiData)
                                        %no old data to subtract
                                        obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2)) = ...
                                            obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos(1):stripePos(2)) + ...
                                            newStripe.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos(1):stripePos(2));
                                    else
                                        %old data to subtract
                                        %normal case
                                        obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2));
                                        obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2)) = ...
                                            obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2)) + ...
                                            newStripe.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2)) - ...
                                            obj.rollingStripeDataBuffer{frameBatchIdx}{obj.frameAverageIndex+1}.roiData{iterRoiData}.imageData{iterChannel}{1}(:,stripePos{1}(1):stripePos{1}(2));

% hIm.CData = newStripe.roiData{iterRoiData}.imageData{1}{1};%(:,stripePos{1}(1):stripePos{1}(2));
% fprintf('%d,%d  ',newStripe.roiData{1}.stripePosition{1}(1),newStripe.roiData{1}.stripePosition{1}(2));
                                    end
                                end
                            end
                            
                            %update roi data and frame number
                            if ~isempty(newStripe.roiData)
                                obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.hRoi = newStripe.roiData{iterRoiData}.hRoi;
                                obj.rollingStripeDataBuffer{frameBatchIdx}{1}.roiData{iterRoiData}.frameNumberAcq = newStripe.roiData{iterRoiData}.frameNumberAcq;
                            end
                        end
                        obj.rollingStripeDataBuffer{frameBatchIdx}{obj.frameAverageIndex+1} = newStripe;
                    end
                else
                    obj.rollingStripeDataBuffer{frameBatchIdx}{1} = newStripe;
                end
                
                obj.rollingStripeDataBufferDirty{frameBatchIdx} = true(1,obj.hSI.hChannels.channelsAvailable);
            end
        end
        
        function mergeStripe(obj,stripeData)
            if obj.channelsMergeEnable                
                frameBatchIdx = mod(obj.hSI.internalFrameCounter-1,obj.displayFrameBatchFactor) + 1;
                lclStripeData = obj.rollingStripeDataBuffer{frameBatchIdx}{1};
                % Determine which channels to display, and the merge colors
                % (based on user selections in the GUI)
                chansToDisp = obj.hSI.hChannels.channelDisplay;
                mergeColors = obj.hSI.hChannels.channelMergeColor;

                obj.mergeStripeData = copy(lclStripeData);
                
                % Iterate through the number of roiDatas in this
                % stripeData.
                for iter = 1:numel(lclStripeData.roiData)
                    lclRoiData = copy(lclStripeData.roiData{iter});
                    % Get dimensions of this roiData in pixels. Remember
                    % that this varies in MROI mode.
                    [ roiPixelsX, roiPixelsY ] = size(lclRoiData.imageData{1}{1});
                    % Create empty mergeData array, which is sized to the current roiData, and will hold
                    % a 3D RGB array for all channels displaying this ROI.
                    mergeData = zeros(roiPixelsX,roiPixelsY,3,'uint8');
                    % Loop through all channels displaying this roiData.
                    for chanIdx = 1:numel(chansToDisp)
                        channel = chansToDisp(chanIdx);
                        % Get the actual image data in the roiData for this
                        % channel.
                        imageData = lclRoiData.imageData{chanIdx}{1};
                        chanProp = sprintf('chan%dLUT',channel);
                        chanLut = (obj.(chanProp)) * obj.displayRollingAverageFactor; % adjust LUT to perform frame averaging
                        % Iteratively merge all channels displaying this roiData with the
                        % user-selected channel colors and LUTs.
                        mergeData = obj.zprvAddChanDataToMergeData(mergeData,imageData,mergeColors{channel},chanLut);
                    end
                    %Take the merged channel data and write it into
                    %mergeStripeData.
                    lclRoiData.imageData{1}{1} = mergeData;
                    lclRoiData.channels = 1;
                    obj.mergeStripeData.roiData{iter} = lclRoiData;
                end
                % Put merged stripe data into the mergeStripeDataBuffer.
                obj.mergeStripeDataBuffer{frameBatchIdx} = obj.mergeStripeData;
            end
        end

        function displayChannels(obj)
            %if obj.componentExecuteFunction('displayChannels') % remove for performance
                for chanIdx = 1:numel(obj.hSI.hChannels.channelDisplay)
                    % Get the actual channel number.
                    lclChannelNumber = obj.hSI.hChannels.channelDisplay(chanIdx);
                    % Get the display function registered for this channel.
                    lclDisplayFunction = obj.hSI.hChannels.channelDisplayFunction{lclChannelNumber};                    
                    if ~isempty(lclDisplayFunction)
                        % Call the registered display function (if it exists.)
                        obj.(lclDisplayFunction)(lclChannelNumber,[]);
                    else
                        fprintf('Warning: No display function specified for channel %d.\n',chanIdx);
                    end
                end
                if obj.frameRateDisplay && ~isempty(obj.lastDisplayUpdate)
                    obj.frameRateAverageBuffer(end+1) = toc(obj.lastDisplayUpdate);
                    nToDel = max(0,length(obj.frameRateAverageBuffer) - obj.frameRateAverageFactor);
                    obj.frameRateAverageBuffer(1:nToDel) = [];
                    obj.frameRate = 1/mean(obj.frameRateAverageBuffer);
                    fprintf('Frame rate: %.2f fps\n', obj.frameRate);
                end
                obj.lastDisplayUpdate = tic;
            %end
        end
        
        % A display function for displaying a raster image.
        function displayRaster(obj,channelNumber,buffer)
            fprintf('displayRaster...\n');
            if obj.componentExecuteFunction('displayRaster')
                %TODO
            end
        end
            
        % A display function for displaying a single roi data type.
        function displayRoi(obj,channelNumber,buffer)
            if obj.componentExecuteFunction('displayBuffer')
                %TODO
            end
        end
        
        function displayStripe(obj,channelNumber,buffer)            
           % if obj.componentExecuteFunction('displayStripe') % removed for performance
                if isempty(buffer)
                    bufferTemp = obj.rollingStripeDataBuffer;
                else
                    bufferTemp = buffer;                   
                end                
                bufferDirtyFlagsTemp = obj.rollingStripeDataBufferDirty;
                chanAxes = obj.hAxes{channelNumber};
                
                % Update scanfield windows
%                 arrayfun(@(x)x.updateDisplay(), obj.scanfieldDisplays([obj.scanfieldDisplays.visible]), 'UniformOutput', false);
                
                % Update channel windows
                if obj.displayFrameBatchFactor > 1 && (obj.displayTiling || obj.hSI.hRoiManager.mroiEnable)
                    % tiled mode operation.
                    for iter = 1:length(obj.displayFrameBatchSelection)
                        index = obj.displayFrameBatchSelection(iter);
                        if bufferDirtyFlagsTemp{index}(channelNumber)
                            lclRoiData = bufferTemp{index}{1}.roiData;
                            if ~isempty(lclRoiData)
                                if obj.hSI.hRoiManager.mroiEnable
                                    hRoiDisplay = chanAxes{1};
                                else
                                    hRoiDisplay = chanAxes{iter};
                                end
                                hRoiDisplay.drawRoiData(lclRoiData,channelNumber);
                                obj.rollingStripeDataBufferDirty{index}(channelNumber) = false;
                            end
                        end
                        if obj.channelsMergeEnable
                            % TODO: Get rid of this hack and fix the issue where
                            % the mergeStripe has no roiData or the imageData is not 3D.
                            mergeRoiData = obj.mergeStripeDataBuffer{index}.roiData;
                            if iscell(mergeRoiData)
                                if numel(size(mergeRoiData{1}.imageData{1}{1})) == 3
                                    if obj.hSI.hRoiManager.mroiEnable
                                        hMergeDisplay = obj.hMergeAxes{1};
                                    else
                                        hMergeDisplay = obj.hMergeAxes{iter};
                                    end
                                    hMergeDisplay.drawRoiData(mergeRoiData);
                                else
                                    fprintf('obj.mergeStripeData not 3D as expected...\n');
                                end
                            end
                        end
                    end
                else
                    % untiled mode operation.
                    tileIdx = mod(obj.hSI.internalFrameCounter-1,obj.displayFrameBatchFactor)+1;
                    [exists,index] = ismember(tileIdx,obj.displayFrameBatchSelection);
                    if exists
                        if bufferDirtyFlagsTemp{index}(channelNumber)
                            hRoiDisplay = chanAxes{1};
                            lclRoiData = bufferTemp{obj.displayFrameBatchSelection(index)}{1}.roiData;
                            
                            hRoiDisplay.drawRoiData(lclRoiData,channelNumber);
                            obj.rollingStripeDataBufferDirty{index}(channelNumber) = false;
                        end                        
                    end
                    if obj.channelsMergeEnable
                        % TODO: Get rid of this hack and fix the issue where
                        % the mergeStripe has no roiData or the imageData is not 3D.
                        mergeRoiData = obj.mergeStripeDataBuffer{1}.roiData;
                        if iscell(mergeRoiData)
                            if numel(size(mergeRoiData{1}.imageData{1}{1})) == 3
                                obj.hMergeAxes{1}.drawRoiData(mergeRoiData);
                            else
                                fprintf('obj.mergeStripeData not 3D as expected...\n');
                            end
                        end
                    end
                end
           % end
        end
    end
    
    %% INTERNAL METHODS
    methods (Hidden)
        function lclDisplayFigCloseEventHandler(obj,src,evnt) 
            channelToHide = find(obj.hFigs == src);
            if isempty(channelToHide) %this should never occur
                set(src,'Visible','off');
                return
            end
            
            if isempty(find(obj.hSI.hChannels.channelDisplay==channelToHide, 1))
                set(src,'Visible','off'); % if the channel is not actively displayed, the window can be closed during an active acquisition
            else
                if obj.componentExecuteFunction('lclDisplayFigCloseEventHandler',src,evnt)
                    obj.hSI.hChannels.channelDisplay(obj.hSI.hChannels.channelDisplay==channelToHide) = [];
                    if ~ismember(channelToHide,obj.hSI.hChannels.channelDisplay)
                        set(src,'Visible','off');
                    end
                end
            end
        end
    end
    
    methods (Hidden, Access=private)
        function zprpUpdateChanLUT(obj,chanIdx,newVal)
            if chanIdx>length(obj.hAxes)
                return;
            end
            
            for i = 1:length(obj.hAxes{chanIdx})
                hRoiAx = obj.hAxes{chanIdx}{i};
                hRoiAx.CLim = newVal;
                hRoiAx.dataMultiplier = obj.displayRollingAverageFactor; % adjust LUT to perform frame averaging
            end
        end
        
        function mergeData = zprvAddChanDataToMergeData(obj,mergeData,chanData,clr,lut)
                lut = single(lut);
                maxVal = single(255);
                chanDataRescaled = uint8((single(chanData) - lut(1)) .* (maxVal / (lut(2)-lut(1))));
                
                switch clr
                    case 'red'
                        mergeData(:,:,1) = mergeData(:,:,1) + chanDataRescaled;
                    case 'green'
                        mergeData(:,:,2) = mergeData(:,:,2) + chanDataRescaled;
                    case 'blue'
                        mergeData(:,:,3) = mergeData(:,:,3) + chanDataRescaled;
                    case 'gray'
                        mergeData(:,:,:) = mergeData(:,:,:) + repmat(chanDataRescaled,[1 1 3]);
                    case 'none'
                        % no-op
                    otherwise
                        assert(false);
                end
        end        
        
        function numChannelFigs = ziniCreateFigs(obj)
            %Initialize channel figure windows
            numChans = obj.hSI.hChannels.channelsAvailable;

            %Create the figures first.
            if most.idioms.graphics2014b
                obj.hFigs = gobjects(1,obj.MAXIMUM_CHANNELS); % ensure the figures are stored as object handles, not as numeric values
            end
            
            for i=1:obj.MAXIMUM_CHANNELS
                obj.hFigs(i) = most.idioms.figureSquare('Name',sprintf('CH%s',i),'Visible','off',...
                    'ColorMap',gray(255),'NumberTitle','off','Menubar','none','Tag',sprintf('image_channel%d',i),...
                    'CloseRequestFcn',@(src,evnt)obj.lclDisplayFigCloseEventHandler(src,evnt));
            end
            
            numChannelFigs = numChans;
            obj.hMergeFigs = most.idioms.figureSquare('Name','Channel Merge',...
                'Visible','off','NumberTitle','off','Menubar','none',...
                'Tag','channel_merge','CloseRequestFcn',@lclMergeFigCloseEventHandler);
            
            obj.duringInit = true;
            obj.resetDisplayFigs(1:numChans,true,obj.hSI.hChannels.channelLUTInitVal);
            obj.duringInit = false;
            
            % register all channel figs with controller
            assert(numel(obj.hController) <= 1); % for now always have a single controller
            if ~isempty(obj.hController)
                ctrler = obj.hController{1};
                for c = 1:numChans
                    ctrler.registerGUI(obj.hFigs(c));
                end
                ctrler.registerGUI(obj.hMergeFigs);
            end
            
            function lclMergeFigCloseEventHandler(~,~)
                obj.channelsMergeEnable = false;
            end
        end
        
        function zzzSetImageFigureAxesLimits(obj)
            if ~isempty(obj.hSI.hChannels.channelDisplay)
                for i=1:numel(obj.hSI.hChannels.channelDisplay)
                    chan = obj.hSI.hChannels.channelDisplay(i);
                    figure(obj.hFigs(chan));
                    set(obj.hFigs(chan),'HandleVisibility','callback');
                    for tileIdx = 1:numel(obj.hAxes{chan});
                        obj.hAxes{chan}{tileIdx}.resetScanFields();
                    end
                end
            end
        end
        
        function zprvResetBuffers(obj)
            % the frameBuffers hold references to stripeData objects
            % (no need to preallocate the buffers)
            obj.frameCounterForBuffer = 0;
            
            % the frameBuffers hold references to stripeData objects
            % (no need to preallocate the buffers)
            obj.stripeDataBuffer = cell(obj.stripeBufferLength,1);
            for i=1:obj.stripeBufferLength
               obj.stripeDataBuffer{i} = scanimage.interfaces.StripeData();
            end
            obj.stripeDataBufferPointer = obj.stripeBufferLength;
            
            obj.frameCounterForBuffer = 0;
            
            tmpBufDirty = {};
            tmpBuf = {};
            tmpMergeBuffer = {};
            for i=1:obj.displayFrameBatchFactor
                tmpBufDirty{i} = false(1,obj.hSI.hChannels.channelsAvailable);
                for j=1:(obj.displayRollingAverageFactor+1)
                    tmpBuf{i}{j} = scanimage.interfaces.StripeData;
                end
                % The Merge Buffer uses already averaged frame data, and
                % therefore doesn't require a 2D buffer.
                tmpMergeBuffer{i} = scanimage.interfaces.StripeData;
            end
            
            obj.rollingStripeDataBufferDirty = tmpBufDirty;
            obj.rollingStripeDataBuffer = tmpBuf;
            
            obj.mergeStripeData = scanimage.interfaces.StripeData;
            obj.mergeStripeDataBuffer = tmpMergeBuffer;
            
        end
        
        function hRoiAx = zprvPrepareDisplayAxesImages(obj,hFig,zs,tiling)
            clf(hFig);
            set(hFig,'HandleVisibility','callback');
            set(0, 'CurrentFigure', hFig);
            hRoiAx = {};
            
            if ~isempty(zs)
                zs(obj.hSI.hStackManager.slicesPerAcq+1:obj.hSI.hStackManager.slicesPerAcq+obj.hSI.hFastZ.numDiscardFlybackFrames) = NaN;
                zs =repmat(zs,1,ceil(max(obj.displayFrameBatchSelection) / numel(zs)));
                
                if ~mod(obj.displayFrameBatchFactor,obj.hSI.hStackManager.slicesPerAcq + obj.hSI.hFastZ.numDiscardFlybackFrames)
                    obj.zLocked = true;
                else
                    % most.idioms.warn('The frame batch factor is not a multiple of the number of z planes. Frames will not be at consistent z levels.')
                    obj.zLocked = false;
                    zs = nan(size(zs));
                end
                
                numTiles = length(obj.displayFrameBatchSelection);
                
                if obj.forceRoiDisplayTransform
                    displayMode = 'transform';
                else
                    displayMode = 'no_transform';
                end
            end
            
            if ~obj.displayTiling
                numTiles = 1; % override numTiles if displayTiling is not set.
            end

            if numTiles > 1
                %TODO: Make sure that when the user selects frames for display that all the Z levels
                %      corresponding to those z levels are displayed.
                %Configure subplots
                
                hGrid = uigridcontainer('v0','Parent',hFig,'Units','norm','Position',[0 0 1 1],'Margin',2);
                set(hGrid,'GridSize',tiling);
                
                for tileIdx=1:numTiles
                    hRoiAx{tileIdx} = scanimage.mroi.RoiDisplay(hGrid);
                    hRoiAx{tileIdx}.debugEnabled = obj.hSI.debugEnabled;
                    
                    if ~obj.hSI.hRoiManager.mroiEnable || obj.displayTiling
                        hRoiAx{tileIdx}.zs = zs(obj.displayFrameBatchSelection(tileIdx));
                        hRoiAx{tileIdx}.initialize(obj.hSI,zs(obj.displayFrameBatchSelection(tileIdx)),displayMode);
                    end
                end
            else
                hRoiAx{1} = scanimage.mroi.RoiDisplay(hFig);
                hRoiAx{1}.initialize(obj.hSI,zs(obj.displayFrameBatchSelection),displayMode);
                hRoiAx{1}.debugEnabled = obj.hSI.debugEnabled;
            end
        end
        
        function data = zprvChannelDataCurrentDisplay(obj,chanIdx)
            % Return the data matrix for the currently-displayed data for the
            % specified channel. (The currently-displayed data is affected by
            % zooming, etc.)
            
            chanRelIdx = find(obj.hSI.hChannels.channelDisplay==chanIdx);
            if isempty(chanRelIdx)
                disp('WARNING: The desired channel was not found');
            else
                data = obj.lastFrame.stripeData{chanRelIdx};
            end
        end
        
        function val = zprvCoerceLutToAdcRange(obj,val)
            val = cast(val,'int16');
            rangeMax = obj.hSI.hChannels.channelLUTRange(2);
            rangeMin = obj.hSI.hChannels.channelLUTRange(1);
            % ensure that the values are within the ADC range
            val = max(val,rangeMin);
            val = min(val,rangeMax);
            
            % ensure that val(2) > val(1)
            if val(2) == rangeMax
                val(1) = min(val(1),val(2)-1);
            else
                val(2) = max(val(2),val(1)+1);
            end
        end
    end
    
    %%%Abstract method impementations (most.Model)
    methods (Access=protected, Hidden)
        function mdlInitialize(obj)
            %Initialize Channel Figures
            numChannelFigs = obj.ziniCreateFigs();
            obj.numInstances = numChannelFigs;
            
            mdlInitialize@most.Model(obj);
            
            %Reset Buffers for Frame Averaging
            obj.zprvResetBuffers();
        end
    end
    
    %%% Abstract methods realizations (scanimage.interfaces.Component)
    methods (Access = protected, Hidden)
        function componentStart(obj)
            %Set the image figure axes limits
            obj.zzzSetImageFigureAxesLimits();
            
            if obj.needReset
                obj.resetActiveDisplayFigs();
            end
            
%             frames = arrayfun(@(x)x.cacheData(), obj.scanfieldDisplays([obj.scanfieldDisplays.visible]), 'UniformOutput', false);
%             obj.scanfieldDisplayFrameSelection = horzcat(frames{:});
%             keyboard
            
            obj.zprvResetBuffers();
            obj.lastAcqStripeDataBuffer = [];
            obj.lastAcqScannerName = '';
        end
        
        function componentAbort(obj,soft)
            obj.lastDisplayUpdate = [];
            obj.frameRateAverageBuffer = [];
            % save data
            if ~soft
                obj.lastAcqScannerName = obj.hSI.imagingSystem;
                obj.lastAcqStripeDataBuffer = cell(1,obj.displayFrameBatchFactor);
                for i=1:obj.displayFrameBatchFactor
                    obj.lastAcqStripeDataBuffer{i} = obj.rollingStripeDataBuffer{i}{1};
                end
            end
        end
    end
end

%% LOCAL
function s = ziniInitPropAttributes()
s.displayFrameBatchSelectLast = struct('Classes','binaryflex','Attributes','scalar');
s.displayFrameBatchSelectAll = struct('Classes','binaryflex','Attributes','scalar');
s.displayTiling = struct('Classes','binaryflex','Attributes','scalar');
s.displayRollingAverageFactorLock = struct('Classes','binaryflex','Attributes','scalar');
s.displayFrameBatchFactorLock = struct('Classes','binaryflex','Attributes','scalar');
s.displayRollingAverageFactor = struct('Classes','numeric','Attributes',{{'positive' 'integer' 'finite'}});
s.displayFrameBatchFactor = struct('Classes','numeric','Attributes',{{'positive' 'integer' 'finite' 'scalar'}});
s.displayFrameBatchSelection = struct('Classes','numeric','Attributes',{{'vector' 'positive' 'integer' 'finite'}});
s.scanfieldDisplays = struct('Classes','scanimage.components.display.scanfieldDisplay','AllowEmpty',true);

s.roiDisplayEdgeColor           = struct('Options',{{'none','y','yellow','m','magenta','c','cyan','r','red','g','green','b','blue','w','white','k','black'}});
s.roiDisplayEdgeAlpha           = struct('Classes','numeric','Attributes',{{'scalar','>=',0,'<=',1}});
s.roiProjectionDisplayEdgeColor	= struct('Options',{{'none','y','yellow','m','magenta','c','cyan','r','red','g','green','b','blue','w','white','k','black'}});
s.roiProjectionDisplayEdgeAlpha = struct('Classes','numeric','Attributes',{{'scalar','>=',0,'<=',1}});

s.chan1LUT = struct('Attributes',{{'numel', 2, 'finite' 'integer'}});
s.chan2LUT = struct('Attributes',{{'numel', 2, 'finite' 'integer'}});
s.chan3LUT = struct('Attributes',{{'numel', 2, 'finite' 'integer'}});
s.chan4LUT = struct('Attributes',{{'numel', 2, 'finite' 'integer'}});
s.channelsMergeEnable = struct('Classes','binaryflex','Attributes','scalar');
s.channelsMergeFocusOnly = struct('Classes','binaryflex','Attributes','scalar');
s.renderer = struct('Options',{{'auto' 'painters' 'opengl'}});
s.forceRoiDisplayTransform = struct('Classes','binaryflex','Attributes','scalar');
end


%--------------------------------------------------------------------------%
% Display.m                                                                %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
