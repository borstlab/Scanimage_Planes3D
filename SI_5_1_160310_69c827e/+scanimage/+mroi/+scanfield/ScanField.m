classdef ScanField < scanimage.mroi.RoiTree
    %% property definitions
    properties (Abstract, SetAccess = protected)
        shortDescription;   % string, description for GUI
    end
    
    properties (SetObservable)
        name = 'default_name';
        enable = true;
        rect;
    end
    
    properties (SetAccess = protected, Hidden)
       uuid;                % UUID for scanfield, generated on construction of scanfield (not loaded from file)
    end
    
    events
       scanFieldChanged;    % fires PostSet event when any SetObservable property is changed
    end
    
    %% Lifcecycle
    methods
        function obj = ScanField()
            obj.uuid = most.util.generateUUID();
        end
        
        function delete(obj)
            % No-op
        end
    end
    
    methods(Access = protected)
        % Override copyElement method:
        function cpObj = copyElement(obj)
            cpObj = copyElement@matlab.mixin.Copyable(obj);
            cpObj.uuid = most.util.generateUUID();
        end
    end
    
    %% Abstract methods
    methods(Abstract)
        % Returns a struct that includes all properties to be saved to file
        % example. s.rect = [0,0,1,1], s.pixelResolution = [100,100]
        s = getPropertyStructForSaving(obj);
        
        % Generate a new scan field by interpolating between this and other
        % frac should be between 0 and 1.
        out=interpolate(obj,other,frac);
        
        % Returns the bounding rectangle as [x y width height]
        % in field-of-view coordinates: (0,0) is top-left and
        % (1,1) is bottom right.
        rect=boundingbox(obj);
        
        % Legacy
%         % Returns a logical array with the same shape as xs and ys
%         % such that bw(i) is true iff the point [xs(i) ys(i)] is inside
%         % the scanfield.
%         %
%         % xs and ys must be arrays of the same shape and be specified in
%         % field-of-view coordinates: (0,0) is top-left and
%         % (1,1) is bottom right.
%         bw=hit(obj,xs,ys);
        
        % Transforms points from a a unit box to
        % field-of-view coordinates.
        [xs,ys]=transform(obj,xs,ys);
        
        % Affine matrix that transform scanfield coordinates to scanner
        % coordinates
        T = affine(obj);
    end
    
    %% Public methods
    methods
        function s=saveobj(obj)
            mc=metaclass(obj);
            s = obj.getPropertyStructForSaving();
            s.name   = obj.name;
            s.enable = obj.enable;
            
            s.type   = mc.Name;
        end
        
        function tf = isnan(~)
            tf = false;
        end
        
        function set.rect(obj,val)
            oldVal = obj.rect;
            obj.rect = val;
            obj.fireChangedEvent(oldVal,val);
        end
    end
    
    %% static methods
    methods(Static)
        function obj=loadobj(s)
            mc=meta.class.fromName(s.type);
            if(any(mc.SuperclassList==?scanimage.mroi.scanfield.ImagingField))
                obj=scanimage.mroi.scanfield.ImagingField.loadobj(s);
            elseif mc==?scanimage.mroi.scanfield.fields.StimulusField
                obj=scanimage.mroi.scanfield.fields.StimulusField.loadobj(s);
            else
                error('MROI:loadobj','Unrecognized scanfield type.');
            end
            
            obj.name   = s.name;
            obj.enable = s.enable;
        end
    end
    
    %% private / protected methods
    methods (Access = protected)
        function fireChangedEvent(obj,oldVal,newVal)
            if nargin > 1
                changed = ~isequal(oldVal,newVal);
            else
                changed = true;
            end
            
            if changed
                notify(obj,'scanFieldChanged');
            end
        end
    end
    
    %% property setters
    methods
        function set.name(obj,val)
            oldVal = obj.name;
            obj.name = val;
            obj.fireChangedEvent(oldVal,val);
        end
        
        function set.enable(obj,val)
            oldVal = obj.enable;
            obj.enable = val;
            obj.fireChangedEvent(oldVal,val);
        end
    end
    
    %% legacy
%     methods
%         function hax=plot(obj)
%             t=linspace(0,1,64);
%             [xs,ys]=meshgrid(t,t);
%             [xs,ys]=obj.transform(xs,ys);
%             mask=obj.hit(xs,ys);
%             
%             hax1=plot(xs(mask(:)),ys(mask(:)),'g.');
%             tf=ishold;
%             hold on; hax2=plot(xs(~mask(:)),ys(~mask(:)),'r.');
%             if(~tf), hold off; end
%             
%             axis([0 1 0 1]);
%             xlabel('X (Unit Interval)');
%             ylabel('Y (Unit Interval)');
%             title('ScanField: Field of View');
%             
%             hax=[hax1;hax2];
%         end
%     end
end

% CONCEPTS
%{

# Spaces

These functions need to do mapping between a few different coordinate
systems.  It's important to understand what they are.

## Scan-unit space

A unit-square spanning the bounding rectangle of a 2d scan field for a
region of interest.  This space is used to generate the control points for
which analog output will be generated.  (0,0) is the top-left corner of the
scanfield; (1,1) the farthest corner.

## Field-of-view space

This is a unit square.  The origin is at (0,0).  The far corner is at
(1,1).

Conceptually, this represents the two-dimensional field of view that can be
scanned by a set of scanners.  For example: (0,0) would be full negative
deflection on a couple of mirrors and (1,1) would be the full positive
deflection.

%}


%--------------------------------------------------------------------------%
% ScanField.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
