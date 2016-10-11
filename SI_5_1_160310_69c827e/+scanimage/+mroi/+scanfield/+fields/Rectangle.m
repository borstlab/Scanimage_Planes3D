classdef Rectangle < scanimage.mroi.scanfield.ImagingField & handle
    %% Abstract property realization scanimage.mroi.scanfield.ScanField    
    properties (SetAccess = protected)
        shortDescription = 'Rect';
    end
    
    %% Lifecycle
    methods
        function obj=Rectangle(rect,pixelResolution) 
            obj = obj@scanimage.mroi.scanfield.ImagingField(rect,pixelResolution);
        end
    end
    
    %% Abstract method realization scanimage.mroi.scanfield.ScanField    
    methods
        function out=interpolate(obj,other,frac)
            assert(isa(other,class(obj)),'MROI:TypeError','Got an argument of class %s. Expected %s.',class(other),class(obj));
            rect = (other.rect-obj.rect).*frac+obj.rect;
            pixelResolution = (other.pixelResolution - obj.pixelResolution).*frac + obj.pixelResolution;
            pixelResolution(1) = round(pixelResolution(1) / 2) * 2; % only allow even number of pixels 
            pixelResolution(2) = round(pixelResolution(2));
            out=scanimage.mroi.scanfield.fields.Rectangle(rect,pixelResolution);
        end
        
        function rect=boundingbox(obj) 
            % Required by the ScanField interface
            rect=obj.rect;
        end
        
        % Legacy
%         function bw=hit(obj,xs,ys)
%             % Returns a logical array with the same shape as xs and ys
%             % such that bw(i) is true iff the point [xs(i) ys(i)] is inside
%             % the scanfield.
%             %
%             % xs and ys must be arrays of the same shape and be specified in
%             % field-of-view coordinates: (0,0) is top-left and
%             % (1,1) is bottom right.
%             bw=true(size(xs));
%         end

        function [xs,ys]=transform(obj,xs,ys)
            r=num2cell(obj.rect);
            [x0,y0,w,h]=deal(r{:});
            xs=xs.*w+x0;
            ys=ys.*h+y0;
        end
        
        function T=affine(obj)
            % Returns the affine transform from pre-rotation to post-rotation
            
            O=eye(3);  %translation to origin (pre scaling)
            R=eye(3);  %rotation by obj.degrees
            C=eye(3);  %translation to rect center
            
            O(:,3)=[-0.5 -0.5 1];
            
            radians=0;
            c=cos(radians);
            s=sin(radians);
            R(1:2,1:2)=[c -s; s c];

            rc=num2cell(obj.rect);
            [x0,y0,w,h]=deal(rc{:});
            S=diag([w h 1]);                %scaling by w,h
            C(:,3)=[x0+w/2 y0+h/2 1];

            T = C*R*S*O;
        end
    end
    
    %% Static methods
    methods(Static)
        function obj=loadobj(s)
            obj=scanimage.mroi.scanfield.fields.Rectangle(s.rect,s.pixelResolution);
        end
    end   
end


%--------------------------------------------------------------------------%
% Rectangle.m                                                              %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
