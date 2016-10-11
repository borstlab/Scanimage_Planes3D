classdef ImagingField < scanimage.mroi.scanfield.ScanField
    %% Public properties
    properties(SetObservable)        
        pixelRatio;             % 2 element vector defined by pixelResolution / rect
        defaultPixelResolution; % 2 element vector defining the pixel resolution for scanning the field whel pixelRatio is Inf
    end
    
    properties(SetObservable,Dependent)
        pixelResolution;        % 2 element vector [pixelsPerLine, linesPerFrame] defining the pixel resolution for scanning the field
    end

    %% Lifecycle
    methods
        function obj = ImagingField(rect,pixelResolution)
            obj = obj@scanimage.mroi.scanfield.ScanField();
            
            % validate inputs and set properties
            validateattributes(rect,{'numeric'},{'row','numel',4,'finite'});
            
            %{
            % (ngc) maybe do these as constraints
            
            validateattributes(pixelResolution,{'numeric'},{'row','numel',2,'finite','positive','integer'});
            assert(rem(pixelResolution(1),2) == 0,'ScanField: X-resolution must be even. Requested value: %d',...
                pixelResolution(1));
            %}
            
            % set properties
            obj.rect = rect;
            %obj.pixelResolution = pixelResolution;
            obj.defaultPixelResolution = pixelResolution;
            % set pixel Ratio based on passed in pixelResolution.
            if rect(3:4) == [0 0]
                obj.pixelRatio = [512 512]; %set to some default non-zero value.
            elseif rect(3) == 0
                fprintf('x extent is 0.\n');
                obj.pixelRatio = [ 512 (pixelResolution(2) ./ rect(4)) ];
            elseif rect(4) == 0
                fprintf('y extent is 0.\n');
                obj.pixelRatio = [ (pixelResolution(1) ./ rect(3)) 512 ];
            else
                obj.pixelRatio = pixelResolution ./ rect(3:4);
            end
        end
    end
    
    %% Abstract methods realization scanimage.mroi.scanfield.ScanField
    methods
        function s = getPropertyStructForSaving(obj)
            s=struct(...
                'rect',obj.rect...
                ,'pixelResolution',obj.pixelResolution);
        end
    end
    
    %% Property getter/setter methods
    methods
        
        function set.pixelRatio(obj,val)
            oldVal = obj.pixelRatio;
            obj.pixelRatio = val;
            obj.fireChangedEvent(oldVal,val);
        end
        
        % The following methods were implemented for the Kerlin Rig
        function val = get.pixelResolution(obj)
            pixels = obj.pixelRatio .* obj.rect(3:4);
            pixels(1) = round(pixels(1));
            pixels(2) = 2 * round(pixels(2) / 2);
            val = pixels;
            
            if any(isnan(val))
                val(isnan(val)) = obj.defaultPixelResolution(isnan(val));
            end
            
            if any(isinf(val))
                val(isinf(val)) = obj.defaultPixelResolution(isnan(val));
            end
            
            val = max(1,val); %ensure resolution is non zero
        end
        
        function set.pixelResolution(obj,val)
            %do nothing.
            % set pixel ratio to reflect new pixel resolution.
            oldVal = obj.pixelResolution;
            obj.pixelRatio = val ./ obj.rect(3:4);
            obj.defaultPixelResolution = val;
            obj.fireChangedEvent(oldVal,val);
        end
    end
    
    %% Static methods
    methods(Static)
        function obj=loadobj(s)
            mc=meta.class.fromName(s.type);
            if(mc==?scanimage.mroi.scanfield.fields.Rectangle)
                obj=scanimage.mroi.scanfield.fields.Rectangle.loadobj(s);
            elseif(mc==?scanimage.mroi.scanfield.fields.RotatedRectangle)
                obj=scanimage.mroi.scanfield.fields.RotatedRectangle.loadobj(s);
            else
                error('MROI:loadobj','Unrecognized scanfield type.');
            end
        end
    end
end


%--------------------------------------------------------------------------%
% ImagingField.m                                                           %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
