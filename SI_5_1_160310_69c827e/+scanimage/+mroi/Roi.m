classdef Roi < scanimage.mroi.RoiTree
    %% 3D region of interest formed from 2d scanfields over a z interval.
    % 
    % The 3d shape is determined by interpolating between (z,scanfield)
    % pairs that are added as control points.
    %
    
    %% Public properties
    properties(SetObservable)
        zs = [];                        % the plane for each set of control points
        name = '';                      % [string] description of roi
        enable = true;                  % specifies if the Roi is scanned
        display = true;                 % specifies if the Roi is displayed
        discretePlaneMode = false;      % if true, roi only exists on planes where scanfield is defined
        
        % Roi specific beam settings
        powers = [];                % array containing power values for each beam [mumeric]
        pzAdjust = [];              % #ok<*NBRAK> % array indicating whether power/z adjustment is enabled for each beam [logical]
        Lzs = [];                   % array containing length constant for each beam, to use for power adjustment during Z stacks [numeric]
        interlaceDecimation = [];   % array indicating for each beam that beam should only be on every n'th line
        interlaceOffset = [];       % array indicating for each beam the offset line to start interlace
    end
    
    properties(SetObservable, SetAccess = private)
        scanfields = [] % A collection of scanimage.mroi.Scanfield objects
    end
    
    %% Private / friend properties
    properties(Access = private)
       scanFieldsListenerMap;
    end
    
    properties (SetAccess = ?scanimage.mroi.RoiGroup)
        id;             % unique identifier of roi within RoiGroup
    end
    
    properties (SetAccess = immutable)
        uuid;
    end
    
    %% Events
    events
        roiChanged;
    end

    %% Lifecycle
    methods
        function obj = Roi()
            %% Makes an empty Roi
            obj.uuid = most.util.generateUUID();
            obj.scanFieldsListenerMap = containers.Map();
        end
        
        function delete(obj)
            cellfun(@(lh)delete(lh),obj.scanFieldsListenerMap.values); % delete all scanfield listener handles
        end
        
        function s=saveobj(obj)
            s=struct(...
                'zs',obj.zs,...                
                'scanfields',arrayfun(@(sf)saveobj(sf),obj.scanfields),...
                'id',obj.id,...
                'name',obj.name,...
                'discretePlaneMode',obj.discretePlaneMode);
        end
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            %cpObj = copyElement@matlab.mixin.Copyable(obj);
            
            cpObj = scanimage.mroi.Roi();
            cpObj.id=obj.id;
            cpObj.name = obj.name;
            cpObj.enable = obj.enable;
            cpObj.display = obj.display;
            cpObj.powers = obj.powers;
            cpObj.pzAdjust = obj.pzAdjust;
            cpObj.Lzs = obj.Lzs;
            cpObj.interlaceDecimation = obj.interlaceDecimation;
            cpObj.interlaceOffset = obj.interlaceOffset;
            cpObj.discretePlaneMode = obj.discretePlaneMode;
            
            arrayfun(@(z)cpObj.add(z,obj.get(z).copy()),obj.zs,'UniformOutput',false);
        end
    end
    
    
    %% Public methods
    methods
        function [path_FOV,dt] = scanPathFOV(obj,scannerset,z,actz,dzdt)
            %% returns the scan pattern for the z'th plane
            %  nx,ny are the number of x pixels and y pixels to scan, respectively
            %
            %  If obj.hit(z) is not true, returns [].
            %  otherwise returns a "number of channels" by "number of samples" array.
            %            
            path_FOV=[]; dt=0;
            
            [sf,err] = obj.interpolate(z); % interpolate only returns enabled sfs
            if(err);return; end
                        
            [path_FOV,dt] = scannerset.scanPathFOV(sf,obj,actz,dzdt);
        end        
        
        function idxs = uuidToIndex(obj,uuid,silent)
            if nargin < 3 || isempty(silent)
                silent = false;
            end
            
            mask = strcmp(uuid,{obj.scanfields.uuid});
            idxs=find(mask);
            
            if isempty(idxs) && ~silent
                error('SI:mroi:StimSeriesIndexNotFound Could not find StimSeries with id %s',uuid);
            end
        end
                
        function obj=add(obj,z,scanfield)
            %% obj=add(obj,z,scanfield)
            % Adds a scanfield at a particular z plane.
            % Serves as a control point for interpolation.
            % FIXME: See Note (1)
     
            if(~isa(scanfield,'scanimage.mroi.scanfield.ScanField')),
                error('MROI:TypeError','scanfield must be a kind of scanimage.mroi.scanfield.ScanField');
            end
            if(~isempty(obj.scanfields))
                if(~isa(scanfield,class(obj.scanfields(1)))),
                    error('MROI:TypeError','All the scanfields added to an scanimage.mroi.Roi must be the same type.');
                end
            end
            
            obj.zs = z;
            obj.scanfields = scanfield;
        end
        
        function obj=mapScanfields(obj,f)
            % f should be a function mapping a scanfield to a scanfield
            sfs=arrayfun(f,obj.scanfields,'UniformOutput',false);
            obj.scanfields=[sfs{:}];
        end
        
        function obj=removeByUuid(obj,uuid)
            idxs = obj.uuidToIndex(obj,uuid);
            obj.scanfields(idxs)=[];
            obj.zs(idxs)=[];
            
            if ~any(strcmp(uuid,{obj.scanfields.uuid}))
                lh = obj.scanFieldsListenerMap(uuid);
                delete(lh);
                obj.scanFieldsListenerMap.remove(uuid);
            end
        end
        
        function obj=remove(obj,z)
            rmScanfields = obj.scanfields(z==obj.zs);
            
            obj.scanfields(z==obj.zs)=[];
            obj.zs(z==obj.zs)=[];
            
            for scanfield = rmScanfields
                if ~any(strcmp(scanfield.uuid,{obj.scanfields.uuid}))
                    lh = obj.scanFieldsListenerMap(scanfield.uuid);
                    delete(lh);
                    obj.scanFieldsListenerMap.remove(scanfield.uuid);
                end
            end
        end
        
        function sf = get(obj,z)
            [sf,err] = obj.interpolate(z);
        end
        
        function tf = hit(obj,z)
            %% returns true if this roi is involved in the imaging of plane z
            if(isempty(obj.zs)),   tf = false; return; end
            if obj.discretePlaneMode, tf = any(obj.zs == z); return; end
            if(length(obj.zs)==1), tf = true;  return; end
            tf = min(obj.zs(:))<=z && z<=max(obj.zs(:));
        end
    end
    
    %% Private methods
    methods (Hidden)
        function handleScanfieldsChangedEvent(obj,~,evt)
            notify(obj,'roiChanged',evt);
        end
        
        function handlePropEvents(obj,~,evt)
            notify(obj,'roiChanged',evt);
        end
        
        function fireChangedEvent(obj)
            notify(obj,'roiChanged');
        end
    end

    methods(Access=private)        
        function [sfs,err] = interpolate(obj,z)
            
            if isempty(obj.scanfields)
                sfs = [];
                err = 1;
                return;
            else
                err=0;
                sfs = obj.scanfields;
                return;
            end
            
        end
    end
    
    %% Static methods
    methods(Static)
        function obj=loadobj(s)
            obj=scanimage.mroi.Roi;
            obj.name=s.name;
            obj.id=s.id;
            if isfield(s, 'discretePlaneMode')
                obj.discretePlaneMode = s.discretePlaneMode;
            else
                most.idioms.warn('Older roigroup format detected. Assuming discretePlaneMode is disabled');
            end
            sfs=arrayfun(@(e) scanimage.mroi.scanfield.ScanField.loadobj(e),s.scanfields,'UniformOutput',false);
            for i=1:length(s.zs)
                obj.add(s.zs(i),sfs{i});
            end
        end        
    end
    
    %% Property setter
    methods
        function set.zs(obj,val)
            obj.zs = val;
            obj.fireChangedEvent();
        end
        
        function set.name(obj,val)
            obj.name = val;
            obj.fireChangedEvent();
        end
        
        function set.enable(obj,val)
            obj.enable = val;
            obj.fireChangedEvent();
        end
        
        function set.display(obj,val)
            obj.display = val;
            obj.fireChangedEvent();
        end
        
        function set.powers(obj,val)
            obj.powers = val;
            obj.fireChangedEvent();
        end
        
        function set.pzAdjust(obj,val)
            obj.pzAdjust = val;
            obj.fireChangedEvent();
        end
        
        function set.Lzs(obj,val)
            obj.Lzs = val;
            obj.fireChangedEvent();
        end
        
        function set.interlaceDecimation(obj,val)
            obj.interlaceDecimation = val;
            obj.fireChangedEvent();
        end
        
        function set.interlaceOffset(obj,val)
            obj.interlaceOffset = val;
            obj.fireChangedEvent();
        end
    end
end

%% NOTES
%{

1. The z's should be unique.  Should replace the scanfields from one z
   with another when collisions come in.
%}    


%--------------------------------------------------------------------------%
% Roi.m                                                                    %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
