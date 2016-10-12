classdef RotatedRectangle < scanimage.mroi.scanfield.ImagingField
    %% Abstract property realization scanimage.mroi.scanfield.ScanField
    properties (SetAccess = protected)
        shortDescription = 'Rotated Rect';
    end
    
    %% Public properties
    properties (SetObservable) 
        degrees   % The obj.rect rectangle will be rotated about it's center by 'degree' degrees
        degreesX
    end
    
    %% Abstract methods realization scanimage.mroi.scanfield.ScanField
    methods
        function obj=RotatedRectangle(rect,degrees,pixelResolution)
            obj = obj@scanimage.mroi.scanfield.ImagingField(rect,pixelResolution);
            validateattributes(degrees,{'numeric'},{'scalar','finite'})
            obj.degrees=degrees;
        end

        function out=interpolate(obj,other,frac)
            assert(isa(other,class(obj)),'MROI:TypeError','Got an argument of class %s. Expected %s.',class(other),class(obj));
            shortest_angle=mod( mod(other.degrees-obj.degrees,360)+540,360) - 180;
            
            angle = shortest_angle.*frac + obj.degrees;
            rect = (other.rect-obj.rect).*frac+obj.rect;
            pixelResolution = (other.pixelResolution - obj.pixelResolution).*frac + obj.pixelResolution;
            pixelResolution(1) = round(pixelResolution(1) / 2) * 2; % only allow even number of pixels
            pixelResolution(2) = round(pixelResolution(2));

            out=scanimage.mroi.scanfield.fields.RotatedRectangle(rect,angle,pixelResolution);
        end
        
        function rect=boundingbox(obj)
            p=obj.points();
            mns=min(p,[],2);
            d=max(p,[],2)-mns;
            rect=[mns(1) mns(2) d(1) d(2)];
        end       

        function [xs,ys]=transform(obj,xs,ys)
            % Transforms points from unit-scan space to fov space
            r=[xs(:)';ys(:)';ones(size(xs(:)'))];
            r=obj.affine()*r;
            xs=reshape(r(1,:),size(xs));
            ys=reshape(r(2,:),size(ys));
        end
    end
    
    methods
        function set.degrees(obj,val)
            oldVal = obj.degrees;
            val = mod(val,360);
            obj.degrees = val;
            obj.fireChangedEvent(oldVal,val);
        end
    end
    
    methods
        function s = getPropertyStructForSaving(obj)
            s=getPropertyStructForSaving@scanimage.mroi.scanfield.ImagingField(obj);
            s.degrees=obj.degrees;
        end

        function T=affine(obj)
            % Returns the affine transform from pre-rotation to post-rotation
            
            O=eye(3);  %translation to origin (pre scaling)
            R=eye(3);  %rotation by obj.degrees
            C=eye(3);  %translation to rect center
            
            O(:,3)=[-0.5 -0.5 1];
            
            radians=obj.degrees*pi/180;
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
    
    %% Private methods
    methods (Access = private)
        function r=points(obj)
            % returns 2 x 4 points.  First row xs, second row ys.
            % Corner points of rect rotated about the center.
            xs=[0 0 1 1];
            ys=[0 1 1 0];
            r=[xs;ys;ones(size(xs))];
            r=obj.affine()*r;
            r=r(1:2,:);
        end

        function rr=inverse(obj,xs,ys)
            % Maps points form field-of-view coordinates back to the unrotated
            % space for hit detection
            r=[xs(:) ys(:) ones(size(xs(:)))]';
            rr=obj.affine()\r;
            rr=rr(1:2,:);
        end
    end
    
    %% Static methods
    methods(Static)
        function obj=loadobj(s)
            obj=scanimage.mroi.scanfield.fields.RotatedRectangle(s.rect,s.degrees,s.pixelResolution);            
        end
    end
end


%--------------------------------------------------------------------------%
% RotatedRectangle.m                                                       %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
