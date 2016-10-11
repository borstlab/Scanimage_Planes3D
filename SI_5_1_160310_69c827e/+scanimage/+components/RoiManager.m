classdef RoiManager < scanimage.interfaces.Component
    %% USER PROPS
    properties (SetObservable)

        %%% Frame geometry and resolution
        pixelsPerLine = 512;            % [1] defaultROI only: horizontal resolution
        linesPerFrame = 512;            % [1] defaultROI only: vertical resolution
        
        scanZoomFactor = 1;             % [1] defaultROI only: value of zoom. Constraint: zoomFactor >= 1
        scanRotation   = 0;             % [degrees] defaultROI only: rotation counter clockwise about the Z-axis of the scanned area or line
        scanAngleMultiplierSlow = 1;    % [1] defaultROI only: scale slow output
        scanAngleMultiplierFast = 1;    % [1] defaultROI only: scale fast scanner output
        scanAngleShiftSlow = 0;         % [1] defaultROI only: shift slow scanner output (in FOV coordinates)
        scanAngleShiftFast = 0;         % [1] defaultROI only: shift fast scanner output (in FOV coordinates)
        
        forceSquarePixelation = true;   % (logical) defaultROI only: specifies if linesPerFrame is forced to equal pixelsPerLine
        forceSquarePixels = true;       % (logical) defaultROI only: if true scanAngleMultiplierSlow is constrained to match the fraction scanAngleMultiplierFast * linesPerFrame/pixelsPerLine
    end
    
    properties (SetObservable, Dependent)
        %%% Frame timing
        scanFrameRate;                  % [Hz] number of frames per second.
        scanFramePeriod;                % [s] seconds per frame.
        linePeriod;                     % [s] seconds to scan one line
        scanVolumeRate;                 % [Hz] number of volumes per second.
    end
    
    properties (SetObservable, Dependent, Transient)
        currentRoiGroup;                % The currently set roiGroup.
    end
    
    
    %% INTERNAL PROPS
    properties(Hidden, Access = private)
        hRoiGroupDelayedEventListener;
        hCfgLoadingListener;
        abortUpdatePixelRatioProps = false;
        preventLiveUpdate = false;
        cachedScanTimesPerPlane;
    end
    
    properties(SetObservable, Hidden, SetAccess = private)
        roiGroupDefault_;
    end
    
    properties(Hidden, Dependent, SetAccess = private)
        roiGroupDefault;                % The roiGroup used for non-MROI focus/grab/loop modes.
    end
    
    properties(Hidden, Dependent, SetObservable)
        fastZSettling;
    end
    
    properties (SetObservable, Constant, Hidden)
        mroiEnable = false;
        mroiFocusEnable = false;
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = ziniInitPropAttributes();
        mdlHeaderExcludeProps = {'currentRoiGroup'};
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (scanimage.interfaces.Component)
    properties (SetAccess = protected, Hidden)
        numInstances = 1;
    end
    
    properties (Constant, Hidden)
        COMPONENT_NAME = 'RoiGroup';                        % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
        PROP_TRUE_LIVE_UPDATE = {};                         % Cell array of strings specifying properties that can be set while the component is active
        PROP_FOCUS_TRUE_LIVE_UPDATE = {...                  % Cell array of strings specifying properties that can be set while focusing
            'scanZoomFactor','scanRotation','scanAngleMultiplierSlow',...
            'scanAngleMultiplierFast','scanAngleShiftSlow','scanAngleShiftFast'};
        DENY_PROP_LIVE_UPDATE = {};                         % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
        
        FUNC_TRUE_LIVE_EXECUTION = {};                      % Cell array of strings specifying functions that can be executed while the component is active
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {};                % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {};                      % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end
    
    %% LIFECYCLE
    methods
        function obj = RoiManager(hSI)
            obj = obj@scanimage.interfaces.Component(hSI);
        end
        
        function delete(obj)
            most.idioms.safeDeleteObj(obj.roiGroupDefault_);
            most.idioms.safeDeleteObj(obj.hCfgLoadingListener);
        end
    end
    
    methods (Access=protected, Hidden)
        function mdlInitialize(obj)
            mdlInitialize@most.Model(obj);
            
            
            %add listener to know when config file finishes loading to do batch operations on new prop values
            obj.hCfgLoadingListener = addlistener(obj.hSI.hConfigurationSaver,'cfgLoadingInProgress','PostSet',@obj.cfgLoadingChanged);
        end
    end
    
    %% PROP ACCESS
    methods
        
        function set.currentRoiGroup(obj,val)
            obj.mdlDummySetProp(val,'currentRoiGroup');
            obj.updateTimingInformation();
        end
        
        function val = get.currentRoiGroup(obj)
            val = obj.roiGroupDefault;
        end
        
        function set.forceSquarePixelation(obj,val)
            val = obj.validatePropArg('forceSquarePixelation',val);
            if obj.componentUpdateProperty('forceSquarePixelation',val)
                obj.forceSquarePixelation = val;
                obj.updatePixelRatioProps('forceSquarePixelation');
            end
        end
        
        function set.forceSquarePixels(obj,val)
            val = obj.validatePropArg('forceSquarePixels',val);
            if obj.componentUpdateProperty('forceSquarePixels',val)
                obj.forceSquarePixels = val;
                obj.updatePixelRatioProps('forceSquarePixels');
            end
        end
        
        function set.linesPerFrame(obj,val)
            val = obj.validatePropArg('linesPerFrame',val);
            if obj.componentUpdateProperty('linesPerFrame',val)
                obj.linesPerFrame = val;
                obj.updatePixelRatioProps('linesPerFrame');
                if ~obj.mroiEnable
                    obj.clearRoiGroupScannerCoordCache()
                end
            end
        end
        
       
        function set.pixelsPerLine(obj,val)
            val = obj.validatePropArg('pixelsPerLine',val);
            if obj.componentUpdateProperty('pixelsPerLine',val)
                obj.pixelsPerLine = val;
                obj.updatePixelRatioProps('pixelsPerLine');
                if ~obj.mroiEnable
                    obj.clearRoiGroupScannerCoordCache()
                end
            end
        end
        
        function val = get.roiGroupDefault(obj)
            if isempty(obj.roiGroupDefault_)
                obj.roiGroupDefault_ = scanimage.mroi.RoiGroup('Default Imaging ROI Group');
                scanfield = scanimage.mroi.scanfield.fields.RotatedRectangle([0 0 1 1],0,[512, 512]);
                roi = scanimage.mroi.Roi();
                roi.add(0,scanfield);
                obj.roiGroupDefault_.add(roi);
            end
            
            if ~isvalid(obj.hSI.hScan2D)
                val = [];
                return
            end
            
            %Get scanner default roi size
            sz = obj.hSI.hScan2D.defaultRoiFovSize;
            
            rect = [-sz/2 sz];
            rect = rect / obj.scanZoomFactor;
            rect = rect * obj.hSI.hScan2D.fillFractionSpatial;
            rect([1,3]) = rect([1,3]) * obj.scanAngleMultiplierFast;
            rect([2,4]) = rect([2,4]) * obj.scanAngleMultiplierSlow;
            
            rect([1,2]) = rect([1,2]) + 0.5;
            rect(1) = rect(1) + obj.scanAngleShiftFast;
            rect(2) = rect(2) + obj.scanAngleShiftSlow;
            
            %Transform from scanner coords to ref coords
            rect = most.idioms.xformRect(rect,obj.hSI.hScan2D.scannerToRefTransform);
            
            % replace existing roi in default roi group with new roi
            obj.roiGroupDefault_.rois(1).scanfields(1).rect = rect;
            obj.roiGroupDefault_.rois(1).scanfields(1).degrees = obj.scanRotation;
            obj.roiGroupDefault_.rois(1).scanfields(1).pixelResolution = [obj.pixelsPerLine, obj.linesPerFrame];
            
            val = obj.roiGroupDefault_;
        end
        
        function set.scanAngleMultiplierFast(obj,val)
            val = obj.validatePropArg('scanAngleMultiplierFast',val);
            if obj.componentUpdateProperty('scanAngleMultiplierFast',val)
                obj.scanAngleMultiplierFast = val;
                
                %Side effects
                obj.updatePixelRatioProps('scanAngleMultiplierFast');
                if ~obj.mroiEnable
                    obj.clearRoiGroupScannerCoordCache();
                    if ~obj.preventLiveUpdate
                        obj.hSI.hScan2D.updateLiveValues();
                        
                        if obj.hSI.hDisplay.forceRoiDisplayTransform
                            obj.hSI.hDisplay.resetActiveDisplayFigs(true);
                        end
                    end
                end
            end
        end
        
        function set.scanAngleMultiplierSlow(obj,val)
            val = obj.validatePropArg('scanAngleMultiplierSlow',val);
            if obj.componentUpdateProperty('scanAngleMultiplierSlow',val)
                obj.scanAngleMultiplierSlow = val;
                
                %Side effects
                obj.updatePixelRatioProps('scanAngleMultiplierSlow');
                if ~obj.mroiEnable
                    obj.clearRoiGroupScannerCoordCache();
                    if ~obj.preventLiveUpdate
                        if obj.hSI.active
                            obj.hSI.hScan2D.updateLiveValues();
                        end
                        
                        if obj.hSI.hDisplay.forceRoiDisplayTransform
                            obj.hSI.hDisplay.resetActiveDisplayFigs(true);
                        end
                    end
                end
            end
        end
        
        function set.scanAngleShiftSlow(obj,val)
            val = obj.validatePropArg('scanAngleShiftSlow',val);
            if obj.componentUpdateProperty('scanAngleShiftSlow',val)
                
                obj.scanAngleShiftSlow = round(val*1000)/1000;
                
                %Coerce SA shift to acceptable values
                obj.coerceDefaultRoi();
                
                %Side effects
                if ~obj.mroiEnable
                    obj.clearRoiGroupScannerCoordCache();
                    if ~obj.preventLiveUpdate
                        if obj.hSI.active
                            obj.hSI.hScan2D.updateLiveValues();
                        end
                        
                        if obj.hSI.hDisplay.forceRoiDisplayTransform
                            obj.hSI.hDisplay.resetActiveDisplayFigs(true);
                        end
                    end
                end
            end
        end
        
        function set.scanAngleShiftFast(obj,val)
            val = obj.validatePropArg('scanAngleShiftFast',val);
            if obj.componentUpdateProperty('scanAngleShiftFast',val)
                obj.scanAngleShiftFast = round(val*1000)/1000;
                
                %Coerce SA shift to acceptable values
                obj.coerceDefaultRoi();
                
                %Side effects
                if ~obj.mroiEnable
                    obj.clearRoiGroupScannerCoordCache();
                    if ~obj.preventLiveUpdate
                        if obj.hSI.active
                            obj.hSI.hScan2D.updateLiveValues();
                        end
                        
                        if obj.hSI.hDisplay.forceRoiDisplayTransform
                            obj.hSI.hDisplay.resetActiveDisplayFigs(true);
                        end
                    end
                end
            end
        end
        
        function updateTimingInformation(obj,setLinePeriod)
            obj.cacheScanTimesPerPlane;
            obj.scanFramePeriod = NaN; % trigger GUI update for scanFramePeriod
            obj.hSI.hScan2D.scanPixelTimeMean = NaN; % trigger GUI update for scanPixelTimeMean
            
            if nargin < 2 || isempty(setLinePeriod) || setLinePeriod
                obj.linePeriod = NaN;      % trigger GUI update for linePeriod
            end
        end
        
        function cacheScanTimesPerPlane(obj,varargin)
            s = obj.hSI.hScan2D.scannerset;
            rg = obj.hSI.hScan2D.currentRoiGroupScannerCoords;
            N = numel(obj.hSI.hStackManager.zs);
            obj.cachedScanTimesPerPlane(N+1:end) = [];
            
            for idx = N : -1 : 1
                obj.cachedScanTimesPerPlane(idx) = rg.sliceTime(s,obj.hSI.hStackManager.zs(idx));
            end
        end
        
        function set.scanFramePeriod(obj,val)
            obj.mdlDummySetProp(val,'scanFramePeriod');
        end
        
        function val = get.scanFramePeriod(obj)
            if isempty(obj.currentRoiGroup)
                val = NaN;
            else
                if isempty(obj.cachedScanTimesPerPlane)
                    s=obj.hSI.hScan2D.scannerset;
                    scanTimesPerPlane = arrayfun(@(z)obj.hSI.hScan2D.currentRoiGroupScannerCoords.sliceTime(s,z),obj.hSI.hStackManager.zs);
                else
                    scanTimesPerPlane = obj.cachedScanTimesPerPlane;
                end
                val = max(scanTimesPerPlane);
            end
        end
        
        function set.linePeriod(obj,val)
            obj.mdlDummySetProp(val,'linePeriod');
            obj.updateTimingInformation(false);
        end
        
        function val = get.linePeriod(obj)
            % currently this only outputs the scantime for the default roi
            scannerset = obj.hSI.hScan2D.scannerset;
            [lineScanPeriod,~] = scannerset.linePeriod(obj.roiGroupDefault.rois(1).scanfields(1));
            val = lineScanPeriod;
        end
        
        function set.fastZSettling(obj,~)
            obj.updateTimingInformation();
        end
        
        function val = get.fastZSettling(~)
            val = NaN;
        end
        
        function set.scanFrameRate(obj,val)
            obj.mdlDummySetProp(val,'scanFrameRate');
        end
        
        function val = get.scanFrameRate(obj)
            val = 1/obj.scanFramePeriod;
        end

        function set.scanVolumeRate(obj,val)
            obj.mdlDummySetProp(val,'scanFrameRate');
        end
        
        function val = get.scanVolumeRate(obj)
            val = (1 / obj.scanFramePeriod) / (obj.hSI.hStackManager.slicesPerAcq + obj.hSI.hFastZ.numDiscardFlybackFrames);
        end
        
        function set.scanRotation(obj,val)
            val = obj.validatePropArg('scanRotation',val);
            if obj.componentUpdateProperty('scanRotation',val)
                obj.scanRotation = val;
                
                obj.coerceDefaultRoi();
                
                %Side effects
                if ~obj.mroiEnable
                    obj.clearRoiGroupScannerCoordCache();
                    if obj.hSI.active && ~obj.preventLiveUpdate
                        obj.hSI.hScan2D.updateLiveValues();
                        
                        if obj.hSI.hDisplay.forceRoiDisplayTransform
                            obj.hSI.hDisplay.resetActiveDisplayFigs(true);
                        end
                    end
                end
            end
        end
        
        function set.scanZoomFactor(obj,val)
            val = obj.validatePropArg('scanZoomFactor',val);
            if obj.componentUpdateProperty('scanZoomFactor',val)
                obj.scanZoomFactor = round(val*100)/100;
                
                %Coerce SA shift to acceptable values
                obj.coerceDefaultRoi();
                
                %Side effects
                if ~obj.mroiEnable
                    obj.clearRoiGroupScannerCoordCache();
                    if ~obj.preventLiveUpdate
                        obj.hSI.hScan2D.updateLiveValues();
                        
                        if obj.hSI.hDisplay.forceRoiDisplayTransform
                            obj.hSI.hDisplay.resetActiveDisplayFigs(true);
                        end
                    end
                end
            end
        end
    end
    
    %% USER METHODS
    
    %% FRIEND METHODS
    methods (Hidden)
        function clearRoiGroupScannerCoordCache(obj)
            for hScanner = obj.hSI.hScanners
                hScanner{1}.clearRoiGroupScannerCoordCache();
            end
        end
    end
    
    %% INTERNAL METHODS
    methods (Access = protected)
        function componentStart(obj)
            obj.coerceDefaultRoi();
            obj.updateTimingInformation();
            assert(~isempty(obj.hSI.hScan2D.currentRoiGroupScannerCoords.activeRois) && ~isempty(obj.hSI.hScan2D.currentRoiGroupScannerCoords.activeRois(1).scanfields), 'There must be at least one active roi with at least one scanfield within the scanner FOV to start an acquisition.');
        end
        
        function componentAbort(obj)
            % TODO: clear the cache once the acquisition completes. The
            % problem is that currently componentAbort is being called
            % prior to the end of the acquisition, which nullifies the
            % advantages of caching the scantimes per plane.
        end
        
        function roiGroupChanged(obj)
            if obj.mroiEnable
                obj.clearRoiGroupScannerCoordCache();
                obj.updateTimingInformation();
                obj.hSI.hScan2D.updateLiveValues();
                
                if obj.hSI.active
                    obj.hSI.hDisplay.resetActiveDisplayFigs(true);
                else
                    obj.hSI.hDisplay.needReset = true;
                end
            end
        end
        
        function coerceDefaultRoi(obj)
            persistent inp;
            
            if isempty(inp)
                try
                    inp = true;
                    
                    rG = obj.roiGroupDefault;
                    obj.hSI.hScan2D.scannerset.satisfyConstraintsRoiGroup(rG);
                    r = rG.rois.scanfields.rect;
                    
                    sasS = r(2)+r(4)/2 - .5;
                    sasF = r(1)+r(3)/2 - .5;
                    
                    obj.preventLiveUpdate = true;
                    if abs(obj.scanAngleShiftSlow) - sasS > 0.000001
                        obj.scanAngleShiftSlow = sasS;
                    end
                    if abs(obj.scanAngleShiftFast) - sasF > 0.000001
                        obj.scanAngleShiftFast = sasF;
                    end
                    if ~obj.hSI.hScan2D.supportsRoiRotation
                        obj.scanRotation = 0;
                    end
                    
                    obj.preventLiveUpdate = false;
                catch ME
                    inp = [];
                    ME.rethrow;
                end
                inp = [];
            end
        end
    end
    
    methods (Access = private)        
        function updatePixelRatioProps(obj,sourceProp)
            if nargin < 2
                sourceProp = '';
            end
            
            if ~obj.abortUpdatePixelRatioProps && ~obj.hSI.hConfigurationSaver.cfgLoadingInProgress % prevent infinite recursion
                obj.abortUpdatePixelRatioProps = true;
                
                if obj.forceSquarePixelation && obj.linesPerFrame ~= obj.pixelsPerLine
                    obj.linesPerFrame = obj.pixelsPerLine;
                end
                
                if obj.forceSquarePixels
                    if isempty(strfind(sourceProp, 'scanAngleMultiplier'))
                        %changed a pixel value. change SA multipliers appropriately
                        samSlow = obj.scanAngleMultiplierFast * obj.linesPerFrame/obj.pixelsPerLine;
                        if samSlow > 1
                            obj.scanAngleMultiplierSlow = 1;
                            obj.scanAngleMultiplierFast = obj.pixelsPerLine/obj.linesPerFrame;
                        else
                            obj.scanAngleMultiplierSlow = samSlow;
                        end
                    else
                        if obj.forceSquarePixelation
                            %changed an SA multiplier. Since both forceSquarePixels and forceSquarePixelation are on, SA multipliers must be equal
                            if strcmp(sourceProp, 'scanAngleMultiplierSlow')
                                obj.scanAngleMultiplierFast = obj.scanAngleMultiplierSlow;
                            else
                                obj.scanAngleMultiplierSlow = obj.scanAngleMultiplierFast;
                            end
                        else
                            %changed an SA multiplier. change pixel values appropriately
                            obj.linesPerFrame = round(obj.pixelsPerLine * obj.scanAngleMultiplierSlow/obj.scanAngleMultiplierFast);
                        end
                    end
                end
                
                obj.abortUpdatePixelRatioProps = false;
                if ~obj.mroiEnable
                    obj.hSI.hDisplay.resetActiveDisplayFigs(true);
                end
            end
        end
        
        function cfgLoadingChanged(obj, ~, evnt)
            if ~evnt.AffectedObject.cfgLoadingInProgress
                %Just finsihed loading cfg file
                obj.updatePixelRatioProps();
            end
        end
    end   
    
    %% USER EVENTS
    %% FRIEND EVENTS
    %% INTERNAL EVENTS
end

%% LOCAL (after classdef)
function s = ziniInitPropAttributes()
s = struct();

s.mroiEnable = struct('Classes','binaryflex','Attribues',{{'scalar'}});
s.mroiFocusEnable = struct('Classes','binaryflex','Attribues',{{'scalar'}});

%%% Frame geometry and resolution
s.pixelsPerLine             = struct('Classes','numeric','Attributes',{{'integer','positive','finite','scalar'}});
s.linesPerFrame             = struct('Classes','numeric','Attributes',{{'integer','positive','finite','scalar'}});
s.scanZoomFactor            = struct('Classes','numeric','Attributes',{{'scalar','finite','>=',1}});
s.scanRotation              = struct('Classes','numeric','Attributes',{{'scalar','finite'}});
s.scanAngleMultiplierSlow   = struct('Classes','numeric','Attributes',{{'scalar','finite','>=',0,'<=',1}});
s.scanAngleMultiplierFast   = struct('Classes','numeric','Attributes',{{'scalar','finite','>=',0,'<=',1}});
s.forceSquarePixelation     = struct('Classes','binaryflex','Attributes',{{'scalar'}});
s.forceSquarePixels         = struct('Classes','binaryflex','Attributes',{{'scalar'}});
s.scanAngleShiftSlow        = struct('Classes','numeric','Attributes',{{'scalar','finite','>=',-.5,'<=',.5}});
s.scanAngleShiftFast        = struct('Classes','numeric','Attributes',{{'scalar','finite','>=',-.5,'<=',.5}});

%%% Frame timing
s.scanFrameRate   = struct('DependsOn',{{'scanFramePeriod'}});
s.scanVolumeRate  = struct('DependsOn',{{'scanFramePeriod','hSI.hStackManager.slicesPerAcq','hSI.hFastZ.numDiscardFlybackFrames'}});
s.scanFramePeriod = struct('DependsOn',{{'linePeriod','hSI.hStackManager.zs','fastZSettling'}});
s.fastZSettling   = struct('DependsOn',{{'hSI.hFastZ.enable','hSI.hFastZ.waveformType','hSI.hFastZ.settlingTime'}});
s.linePeriod      = struct('DependsOn',{{'hSI.imagingSystem','hSI.hScan2D.scannerset','hSI.hScan2D.pixelBinFactor','hSI.hScan2D.sampleRate','mroiEnable','currentRoiGroup','pixelsPerLine','linesPerFrame'}});
end


%--------------------------------------------------------------------------%
% RoiManager.m                                                             %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
