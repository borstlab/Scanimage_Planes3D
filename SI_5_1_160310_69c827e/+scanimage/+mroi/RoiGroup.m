classdef RoiGroup < scanimage.mroi.RoiTree
    
    %% Public properties
    properties(SetObservable)
        name = '';      % [string] description of roi
        coordSys = 'Reference';
    end
    
    %% Properties
    properties(SetAccess = private,SetObservable)
        rois = [];
    end
    
    properties(SetAccess = private,Dependent)
        activeRois;   % subset of rois, where roi.enable  == true
        displayRois;  % subset of rois, where roi.display == true
        zs;           % array containing Z's of Rois in RoiGroup
    end
    
    %% Private properties
    properties(Access = private)
        nextRoiId = 1;
        
        roisListenerMap;
    end
    
    %% Events    
    events
        roiGroupChanged;
    end
    
    %% Lifecycle
    methods
        function obj=RoiGroup(nm)
            %% Makes an empty RoiGroup
            if nargin > 0 && ~isempty(nm)
                obj.name = nm;
            end
        end
        
        function delete(obj)
            delete(obj.roisListenerMap);
        end
        
        function s=saveobj(obj)
            s=struct(...
                 'name',obj.name ...
                ,'rois',arrayfun(@(r) saveobj(r),obj.rois) ...
                );
        end
        
        function copyobj(obj,other)
            obj.name = other.name;
            obj.coordSys = other.coordSys;
            obj.clear();
            arrayfun(@(roi)obj.addKeepId(roi,roi.id),other.rois,'UniformOutput',false);
        end

        function mc=scanfieldMetaclass(obj)
            if(isempty(obj.rois) || isempty(obj.rois(1).scanfields)),
                mc=meta.class.fromName(''); % empty class if no scanfields/not determined
            else
                mc=metaclass(obj.rois(1).scanfields(1));
            end
        end

        function obj=foreachScanfield(obj,f)
            % Applies f (in place) to all scanfields
            %
            % f must be a function mapping a scanfield to a scanfield.
            for r=obj.rois
                r.mapScanfields(f);
            end
        end

        function obj=filterByScanfield(obj,f)
            % Disables some scanfields according to f.
            %
            % f must be a function mapping a scanfield to a boolean
            %   if f returns false, then the entire roi will be disabled.            
            for r=obj.rois
                tf=arrayfun(f,r.scanfields);
                if any(~tf)
                    r.enable=false;
                end
            end

        end
    end
    
methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            %cpObj = copyElement@matlab.mixin.Copyable(obj);
            
            cpObj = scanimage.mroi.RoiGroup(obj.name);
            cpObj.coordSys = obj.coordSys;
            arrayfun(@(roi)cpObj.addKeepId(roi.copy(),roi.id),obj.rois,'UniformOutput',false);
        end
    end
    
    %% Public methods for AO generation
    methods
        
        % public
        function [ao_volts,samplesPerTrigger] = scanStackAO(obj,scannerset,zs,zWaveform,flybackFrames)
            [path_FOV,samplesPerTrigger] = obj.scanStackFOV(scannerset,zs,zWaveform,flybackFrames);
            ao_volts = scannerset.pathFovToAo(path_FOV);
        end
        
        % private
        function [path_FOV,samplesPerTrigger] = scanStackFOV(obj,scannerset,zs,zWaveform,flybackFrames)
            scannerset.satisfyConstraintsRoiGroup(obj);
            
            %++++
            %scanTimesPerSlice = arrayfun(@(z)obj.sliceTime(scannerset,z),zs);
            %disp(['scanTimesPerSlice = ' num2str(numel(zs))]);
            for idx = numel(zs) : -1 : 1
                scanTimesPerSlice(idx) = obj.sliceTime(scannerset,zs(idx));
            end

            scanTimePerFrame = max(scanTimesPerSlice);
            
            if numel(zs) > 1 && ~strcmp(zWaveform, 'step')
                dz = (zs(end)-zs(1))/(numel(zs)-1);
                dzdt = dz/scanTimePerFrame;
            else
                dzdt = 0;
            end
            
            flybackTime = scannerset.scanners{2}.flybackTimeSeconds; % GJ: little risky, assumes that scannerset.scanners{2} is always a galvo...
            frameTime = scanTimePerFrame - flybackTime;
            
            tmpObj = obj;
            %++++
            %outputData = arrayfun(@(z) tmpObj.scanSliceFOV(scannerset,z,dzdt,frameTime,flybackTime),[zs nan(1,flybackFrames)],'UniformOutput',false);
            tmpZs = [zs nan(1,flybackFrames)];
            %disp(['outputData = ' num2str(numel(tmpZs))]);
            for idx = numel(tmpZs) : -1 : 1
                outputData{idx} = tmpObj.scanSliceFOV(scannerset,tmpZs(idx),dzdt,frameTime,flybackTime);
            end

            samplesPerTrigger = scannerset.samplesPerTriggerForAO(outputData);
            
            dataPoints = most.util.vertcatfields([outputData{:}]);
            path_FOV   = scannerset.interpolateTransits(dataPoints);
        end

        % private
        % (used by scanStackFOV and scanStackAO)
        function path_FOV = scanSliceFOV(obj,scannerset,z,dzdt,frameTime,flybackTime)
            %% ao_volts = scan(obj,scannerset,z,dzdt,frameTime,flybackTime)
            %
            %  Generates the full ao for scanning plane z using the 
            %  specified scannerset
            if isnan(z) % nan signals a flyback
                paths{1} = scannerset.zFlybackFrame(frameTime);
                % place holder scanfield. Does not matter. Just need some nan's
                [paths{2}, ~] = scannerset.transitNaN(scanimage.mroi.scanfield.fields.Rectangle([1 1 1 1],1),NaN);
                
                path_FOV = most.util.vertcatfields([paths{:}]);
            else
                paths{1} = obj.rois.scanPathFOV(scannerset,z,z,dzdt);
                paths{2} = scannerset.transitNaN(obj.rois.scanfields,NaN);
                
                path_FOV = most.util.vertcatfields([paths{:}]);
            end
            
            % Padding: 
            if (frameTime + flybackTime) > 0
                path_FOV = scannerset.padFrameAO(path_FOV,frameTime,flybackTime);
            end
        end
        
        % public (but should look at why)
        function scanTime = scanTimes(obj,scannerset,z)
            % Returns array of seconds with scanTime for each scanfield
            % at a particular z
            scanTime=0;
            if ~isa(scannerset,'scanimage.mroi.scannerset.ScannerSet')
                return;                
            end
                
            scanfields  = obj.scanFieldsAtZScannerCoordinates(scannerset,z);
            scanTime    = cellfun(@(scanfield)scannerset.scanTime(scanfield),scanfields);
        end

        % public (but should look at why)
        function [seconds,flybackseconds] = transitTimes(obj,scannerset,z)
            % Returns array of seconds with transitionTime for each scanfield
            % at a particular z
            % seconds includes the transition from park to the first scanfield of the RoiGroup
            % flybackseconds is the flyback transition from last scanfield to park

            seconds=0;
            flybackseconds=0;            
            if ~isa(scannerset,'scanimage.mroi.scannerset.ScannerSet')
                return;                
            end
            
            scanfields = obj.scanFieldsAtZScannerCoordinates(scannerset,z);
            if isempty(scanfields)
                seconds = [];
                flybackseconds = 0;
            else
                scanfields = [{NaN} scanfields {NaN}]; % pre- and ap- pend "park" to the scan field sequence
                
                tp = scanimage.mroi.util.chain(scanfields); % form pair of scanfields for transition
                seconds = cellfun(@(pair) scannerset.transitTime(pair{1},pair{2}),tp);
                
                flybackseconds = seconds(end); % save flybackseconds separately
                seconds(end) = [];
            end
        end
        
        % public
        function seconds = sliceTime(obj,scannerset,z)
            %% Returns the minimum time [seconds] to scan plane z (does not include any padding)
            scantimes = obj.scanTimes(scannerset,z);
            [transitTimes,flybackTime] = obj.transitTimes(scannerset,z);
            seconds = sum(scantimes) + sum(transitTimes) + flybackTime;
        end
        
        % public
        % (depends on scannerset)
        function [scanfields,zrois] = scanFieldsAtZScannerCoordinates(obj,scannerset,z,activeSfsOnly)
            scanfields = {obj.rois.scanfields};
            zrois = num2cell(obj.rois);
        end

        % public
        % (does not depend on scannerset)
        function [scanfields,zrois] = scanFieldsAtZReferenceCoordinates(obj,z,activeSfsOnly)
            scanfields = {obj.rois.scanfields};
            zrois = num2cell(obj.rois);
        end

    end

    %% Public methods for operating on the roi list -- mostly for UI
    methods
        function clear(obj)
            obj.rois = [];
            delete(obj.roisListenerMap);
        end
        
        function roi = getRoiById(obj,id)
            roi = obj.rois;
        end

        function idx = idToIndex(obj,id,silent)
            if nargin < 3 || isempty(silent)
                silent = false;
            end
            
            if ischar(id)
                mask = strcmp(id,{obj.rois.uuid});
                idx=find(mask,1,'first');
            else
                if isempty(id) || id > length(obj.rois) || id < 1
                    idx = [];
                else
                    idx = id;
                end
            end
            
            if isempty(idx) && ~silent
                error('SI:mroi:StimSeriesIndexNotFound Could not find StimSeries with id %s',id);
            end
        end
        
        function obj = add(obj,roi)
            if(~isa(roi,'scanimage.mroi.Roi'))
                error('MROI:TypeError','Expected an object of type scanimage.mroi.Roi');
            end
            
            obj.clear();
            roi.id = 1;
            obj.rois = roi;
            obj.roisListenerMap = addlistener(roi,'roiChanged',@obj.handleRoisChangedEvent);
        end
        
    end % end public methods
    
    %% Private methods
    methods (Hidden, Access = private)
        function obj = addKeepId(obj,roi,id)
            if(~isa(roi,'scanimage.mroi.Roi'))
                error('MROI:TypeError','Expected an object of type scanimage.mroi.Roi');
            end
            
            obj.clear();
            roi.id = id;
            obj.rois = roi;
            obj.roisListenerMap = addlistener(roi,'roiChanged',@obj.handleRoisChangedEvent);
        end
    end
    
    %% Event handling methods
    methods
        function handleRoisChangedEvent(obj,~,evt)
            notify(obj,'roiGroupChanged',evt);
        end
        
        function handlePropEvents(obj,~,evt)
            notify(obj,'roiGroupChanged',evt);
        end
        
        function fireChangedEvent(obj)
            notify(obj,'roiGroupChanged');
        end
    end
    
    %% Property access methods
    methods
        function val = get.activeRois(obj)
            if ~isempty(obj.rois)
                val = obj.rois([obj.rois.enable]);
            else
                val = [];
            end
        end
        
        function val = get.displayRois(obj)
            if ~isempty(obj.rois)
                val = obj.rois([obj.rois.enable] & [obj.rois.display]);
            else
                val = [];
            end
        end
        
        function val = get.zs(obj)
            zs = [];
            for roi = obj.rois();
                zs = horzcat(zs,roi.zs(:)'); %#ok<AGROW>
            end
            val = sort(unique(zs));
        end
        
        function set.rois(obj,val)
            obj.rois = val;
            obj.fireChangedEvent();
        end       
    end
    
    %% Static methods
    methods(Static)
        function obj=loadobj(s)
            obj=scanimage.mroi.RoiGroup;
            obj.name = s.name;
            arrayfun(@(r) obj.add(scanimage.mroi.Roi.loadobj(r)),s.rois,'UniformOutput',false);
        end
    end
end


%--------------------------------------------------------------------------%
% RoiGroup.m                                                               %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
