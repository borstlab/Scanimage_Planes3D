classdef Pmts < scanimage.interfaces.Component
    % Module handling all functionality associated with remote controllable PMT power supplies and amplifiers.
    
    %% USER PROPS
    properties (SetObservable,Transient,SetAccess = private)
        names;          % [string] Cell array of strings containing display names for all PMT channels
    end
    
    % Non-dependent properties are saved to config file
    properties (SetObservable)
        gains;          % [numeric] 1xN array containing power supply gain setting for each PMT
        offsets;        % [numeric] 1xN array containing signal offset setting for each PMT
        bandwidths;     % [numeric] 1xN array containing amplifier bandwidth setting for each PMT
    end
    
    properties (SetObservable,Transient,Dependent)
        powersOn;       % [logical] 1xN array containing indicating power state for each PMT
        tripped;        % [logical] 1xN array containing indicating trip status for each PMT
    end
    
    %% INTERNAL PROPS
    properties (Constant, Hidden)
        PMT_UPDATE_PERIOD = 1;
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (most.Model)
    properties (Hidden, SetAccess=protected)
        mdlPropAttributes = ziniInitPropAttributes();
        mdlHeaderExcludeProps = {'hPmtControllers','hTimer'};                            
    end
    
    %%% ABSTRACT PROPERTY REALIZATION (scanimage.interfaces.Component)
    properties (SetAccess = protected, Hidden)
        numInstances = 0;
    end
    
    properties (Constant, Hidden)
        COMPONENT_NAME = 'PMTs';                                            % [char array] short name describing functionality of component e.g. 'Beams' or 'FastZ'
        PROP_TRUE_LIVE_UPDATE = {'powersOn' 'gains' 'offsets' 'bandwidths'};% Cell array of strings specifying properties that can be set while the component is active
        PROP_FOCUS_TRUE_LIVE_UPDATE = {};                                   % Cell array of strings specifying properties that can be set while focusing
        DENY_PROP_LIVE_UPDATE = {};                                         % Cell array of strings specifying properties for which a live update is denied (during acqState = Focus)
        
        FUNC_TRUE_LIVE_EXECUTION = {'updateStatus','setPmtPower','setPmtGain', ...
            'resetTripStatus','updateStatus','setPmtOffset','setPmtBandwidth'}; % Cell array of strings specifying functions that can be executed while the component is active
        FUNC_FOCUS_TRUE_LIVE_EXECUTION = {};                                % Cell array of strings specifying functions that can be executed while focusing
        DENY_FUNC_LIVE_EXECUTION = {};                                      % Cell array of strings specifying functions for which a live execution is denied (during acqState = Focus)
    end
    
    properties (SetAccess = protected, Hidden)
        numPmts = 0;
        hPmtControllers = {};
        hPmtControllerListeners = {};
        hTimer;
    end
    
    %% USER EVENTS
    events
        statusChanged;          % Event that indicates a change in any pmt status
    end
    
    
    %% LIFECYCLE
    methods (Hidden)
        function obj = Pmts(hModel)
            obj = obj@scanimage.interfaces.Component(hModel);
             
            obj.hTimer = timer('Name','PMT status update timer');
            obj.hTimer.ExecutionMode = 'fixedSpacing';
            obj.hTimer.Period = obj.PMT_UPDATE_PERIOD;
            obj.hTimer.TimerFcn = @obj.updateStatus;
            
            obj.initialize();
        end
        
        
        function initialize(obj)
            for i = 1:numel(obj.hSI.OptionalComponents)
                hComp = obj.hSI.(obj.hSI.OptionalComponents{i});
                if isa(hComp,'scanimage.interfaces.PmtController') && hComp.pmtInitSuccessful
                    obj.hPmtControllers{end+1} = hComp;
                    obj.numPmts = obj.numPmts + hComp.numPmts;
                    
                    obj.hPmtControllerListeners{end+1} = hComp.addlistener('pmtStatusChanged', @(varargin)obj.notify('statusChanged'));
                    
                    %only one supported
                    break
                end
            end
            
            obj.hPmtControllerListeners = [obj.hPmtControllerListeners{:}];
            obj.numInstances = double(obj.numPmts > 0);
            
            %start the update timer
            if obj.numPmts > 0
                start(obj.hTimer);
            end
        end
        
        function delete(obj)
            if most.idioms.isValidObj(obj.hTimer)
                stop(obj.hTimer);
                delete(obj.hTimer);
            end
            
            most.idioms.safeDeleteObj(obj.hPmtControllerListeners);
        end
    end
    
    
    %% PROP ACCESS
    methods
        function val = get.names(obj)
            if isempty(obj.hPmtControllers)
                val = [];
            else
                % Todo: Support multiple pmt controllers
                hCtl = obj.hPmtControllers{1};
                val = hCtl.pmtNames;
            end
        end
        
        function set.powersOn(obj,val)
            %Validation
            val = obj.validatePropArg('powersOn',val);
            assert(numel(val) == obj.numPmts);
            
            if obj.componentUpdateProperty('powersOn',val)
                hCtl = obj.hPmtControllers{1};
                hCtl.pmtsPowerOn = val;
            end
        end
        
        function val = get.powersOn(obj)
            if isempty(obj.hPmtControllers)
                val = [];
            else
                % Todo: Support multiple pmt controllers
                hCtl = obj.hPmtControllers{1};
                val = logical(hCtl.pmtsPowerOn);
            end
        end
        
        function set.offsets(obj,val)
            %Validation
            val = obj.validatePropArg('offsets',val);
            assert(numel(val) == obj.numPmts);
            
            if obj.componentUpdateProperty('offsets',val)
                % Todo: Support multiple pmt controllers
                hCtl = obj.hPmtControllers{1};
                hCtl.pmtsOffsets = val;
            end
        end
        
        function val = get.offsets(obj)
            if isempty(obj.hPmtControllers)
                val = [];
            else
                % Todo: Support multiple pmt controllers
                hCtl = obj.hPmtControllers{1};
                val = hCtl.pmtsOffsets;
            end
        end
        
        function set.bandwidths(obj,val)
            %Validation
            val = obj.validatePropArg('bandwidths',val);
            assert(numel(val) == obj.numPmts);
            
            if obj.componentUpdateProperty('bandwidths',val)
                % Todo: Support multiple pmt controllers
                hCtl = obj.hPmtControllers{1};
                hCtl.pmtsBandwidths = val;
            end
        end
        
        function val = get.bandwidths(obj)
            if isempty(obj.hPmtControllers)
                val = [];
            else
                % Todo: Support multiple pmt controllers
                hCtl = obj.hPmtControllers{1};
                val = hCtl.pmtsBandwidths;
            end
        end
        
        function set.gains(obj,val)
            %Validation
            val = obj.validatePropArg('gains',val);
            assert(numel(val) == obj.numPmts);
            
            %Side-effect
            if obj.componentUpdateProperty('gains',val)
                hCtl = obj.hPmtControllers{1};
                hCtl.pmtsGain = val;
            end
            
        end
        
        function val = get.gains(obj)
            if isempty(obj.hPmtControllers)
                val = [];
            else
                hCtl = obj.hPmtControllers{1};
                val = hCtl.pmtsGain;
            end
        end
        
        function set.tripped(obj,val)
            obj.mdlDummySetProp(val,'tripped');
        end
        
        function val = get.tripped(obj)
            if isempty(obj.hPmtControllers)
                val = [];
            else
                hCtl = obj.hPmtControllers{1};
                val = hCtl.pmtsTripped;
            end
        end
    end
    
    
    %% USER METHODS
    methods
        function setPmtPower(obj, pmtNum, val)
            % hSI.hPmts.setPmtPower(pmtNum, val)
            %
            % Set the power state of an individual PMT.
            %
            % Arguments:
            %  - pmtNum: [Numeric] Index of PMT to set
            %  - val:    [Logical] Desired state
            
            if obj.componentExecuteFunction('setPmtPower',pmtNum,val)
                hCtl = obj.hPmtControllers{1};
                hCtl.setPmtPower(pmtNum, val);
            end
        end
        
        function setPmtGain(obj, pmtNum, val)
            % hSI.hPmts.setPmtGain(pmtNum, val)
            %
            % Set the gain of an individual PMT.
            %
            % Arguments:
            %  - pmtNum: [Numeric] Index of PMT to set
            %  - val:    [Numeric] Desired gain
            
            if obj.componentExecuteFunction('setPmtGain',pmtNum,val)
                hCtl = obj.hPmtControllers{1};
                hCtl.setPmtGain(pmtNum, val);
            end
        end
        
        function resetTripStatus(obj, pmtNum)
            % hSI.hPmts.resetTripStatus(pmtNum)
            %
            % Reset the trip status of an individual PMT.
            %
            % Arguments:
            %  - pmtNum: [Numeric] Index of PMT to reset
            
            if obj.componentExecuteFunction('resetTripStatus',pmtNum)
                hCtl = obj.hPmtControllers{1};
                hCtl.resetPmtTripStatus(pmtNum);
            end
        end
        
        function updateStatus(obj,varargin)
            % hSI.hPmts.updateStatus()
            %
            % Request an asynchronous status update of all PMT statuses
            
            if obj.componentExecuteFunction('updateStatus',varargin{:})
                hCtl = obj.hPmtControllers{1};
                hCtl.updatePmtsStatus();
            end
        end
        
        function setPmtOffset(obj, pmtNum, val)
            % hSI.hPmts.setPmtOffset(pmtNum, val)
            %
            % Set the offset of an individual PMT.
            %
            % Arguments:
            %  - pmtNum: [Numeric] Index of PMT to set
            %  - val:    [Numeric] Desired offset
            
            if obj.componentExecuteFunction('setPmtOffset',pmtNum,val)
                hCtl = obj.hPmtControllers{1};
                hCtl.setPmtOffset(pmtNum, val);
            end
        end
        
        function setPmtBandwidth(obj, pmtNum, val)
            % hSI.hPmts.setPmtBandwidth(pmtNum, val)
            %
            % Set the bandwidth of an individual PMT.
            %
            % Arguments:
            %  - pmtNum: [Numeric] Index of PMT to set
            %  - val:    [Numeric] Desired bandwidth
            if obj.componentExecuteFunction('setPmtBandwidth',pmtNum,val)
                hCtl = obj.hPmtControllers{1};
                hCtl.setPmtBandwidth(pmtNum, val);
            end
        end
        
        function [powerOn, gain, tripped, offsets, bandwidths] = getLastStatus(obj)
            % [powerOn, gain, tripped, offsets, bandwidths] = hSI.hPmts.getLastStatus()
            %
            % Returns the last read pmt status. This does not querry the
            % hardware device for an update. Properties should be querried
            % if a blocking hardware querry is desired.
            
            powerOn = [];
            gain = [];
            tripped = [];
            offsets = [];
            bandwidths = [];
            
            if ~isempty(obj.hPmtControllers)
                hCtl = obj.hPmtControllers{1};
                [powerOn, gain, tripped, offsets, bandwidths] = hCtl.getLastPmtStatus();
            end
        end
    end
    
    %% INTERNAL METHODS
    %%% Abstract method implementation (scanimage.interfaces.Component)
    methods (Access = protected, Hidden)
        function componentStart(obj)
            % No-op
        end
        
        function componentAbort(obj)
            % No-op
        end
    end
end

%% LOCAL
function s = ziniInitPropAttributes()
    s = struct();

    s.powerOn = struct('Classes','binaryflex','Attributes',{{'vector'}},'AllowEmpty',1);
    s.gains   = struct('Classes','numeric','Attributes',{{'vector','finite'}},'AllowEmpty',1);
    s.tripped = struct('Classes','binaryflex','Attributes',{{'vector'}},'AllowEmpty',1);
    s.offsets = struct('Classes','numeric','Attributes',{{'vector'}},'AllowEmpty',1);
    s.bandwidths = struct('Classes','numeric','Attributes',{{'vector'}},'AllowEmpty',1);
end


%--------------------------------------------------------------------------%
% Pmts.m                                                                   %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
