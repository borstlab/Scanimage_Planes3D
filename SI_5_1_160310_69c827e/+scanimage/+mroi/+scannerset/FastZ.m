classdef FastZ < scanimage.mroi.scannerset.ScannerSet
    
    properties
        scanners={}; % Order is by axis.
        flybackFrames;
        sampleRateHz;
    end
    methods(Static)
        function obj=default()
            %% Construct a default version of this scanner set for testing
            z=scanimage.mroi.scanners.Piezo.default();
            obj=scanimage.mroi.scannerset.FastZ(z,5e5);
        end
    end
    methods
        
        function obj = FastZ(piezo,flybackFrames,sampleRateHz)
            %% ResonantGalvoGalvo(resonantx,galvox,galvoy,sampleRateHz)
            % 
            % Describes a piezo-only FastZ scanner set.
            asserttype(piezo,'scanimage.mroi.scanners.Piezo');
            obj.scanners={piezo};
            obj.sampleRateHz=sampleRateHz;
            obj.flybackFrames=flybackFrames;
        end

        function scanfield = satisfyConstraints(obj,scanfield)
            %% Returns a recomputed scanfield that satisfies any constraints
            %  that need to be fullfilled for the scannerset.
            %
            % In this case the scanfield size is expanded so an even number
            % of pixels are used in x and y.
            scanfield.pixelResolution(1) = scanfield.pixelResolution(1); % No restrictions on Piezo-only FastZ.
            scanfield.pixelResolution(2) = scanfield.pixelResolution(2); % No restrictions on Piezo-only FastZ.
        end     
        
        function ao_volts = scan(obj,scanfield)
            %% Returns 1d array.
            % column is fastz ao voltage channel (piezo)
            % rows are samples
            %
            % Output should look like:
            % 1. one column, piezo only output voltage.
            assert(isa(scanfield,'scanimage.mroi.scanfield.ScanField'));
            seconds=obj.scanTime(scanfield);
            fov=ones(ceil(seconds*obj.sampleRateHz),1);

            % For Piezo scans, the fov for a given scanfield is only
            % important in terms of the total number of elements. The
            % actual voltage value will be determined at a later time.
            fov(:,1)=1;
            
            ao_volts(:,1)=obj.fov2volts(fov(:,1),1);
        end
        
        function count = nsamples(obj,seconds)
            count = seconds*obj.sampleRateHz;
        end
        
        function ao_volts = transit(obj,scanfield_from,scanfield_to)
            %% Returns the analog output for transitioning from one scanfield
            % to another.
            assert(transitArgumentTypeCheck(scanfield_from,scanfield_to));
            dt=obj.transitTime(scanfield_from,scanfield_to);
            ao_fov=ones(ceil(dt*obj.sampleRateHz),3);
            if(isempty(scanfield_to))
                % at this point scanfield_from is gauranteed not to be
                % empty
                x=obj.degrees2fov(2,obj.scanners{2}.parkAngleDegrees);
                y=obj.degrees2fov(3,obj.scanners{3}.parkAngleDegrees);
                rect=scanfield_from.boundingbox(); % use this width
                rect(1:2)=[x-rect(3)/2 y];
            else
                rect=scanfield_to.boundingbox();
            end
            
            ao_volts=zeros(size(ao_fov));
            ao_volts(:,1)=obj.fov2volts(rect(3),1);
        end

        function [pixelSizeX,pixelSizeY] = pixelSizeAngle(obj,scanfield)
            scanfield = obj.satisfyConstraints(scanfield);
            sWidth     = scanfield.rect(3); % width of the scanfield  (0..1)
            sHeight    = scanfield.rect(4); % height of the scanfield (0..1)
            pixelsNumX = scanfield.pixelResolution(1);
            pixelsNumY = scanfield.pixelResolution(2);
            
            fullAngleX = obj.mirrors(1).fullAngleDegrees;
            fullAngleY = obj.mirrors(2).fullAngleDegrees;
            
            pixelSizeX = fullAngleX * sWidth  / pixelsNumX;
            pixelSizeY = fullAngleY * sHeight / pixelsNumY;
        end
        
        function seconds = scanTime(obj,scanfield)
            %% Returns the time required for to scan the scanfield in seconds
            numLines = scanfield.pixelResolution(2);
            seconds = (numLines*obj.scanners{1}.scannerPeriod/2^(obj.scanners{1}.bidirectionalScan)-9); %eg 512 lines / (7920 lines/s)
        end

        function [lineScanPeriod,lineAcquisitionPeriod] = linePeriod(obj,scanfield)
            % Definition of lineScanPeriod:
            %   * scanPeriod is lineAcquisitionPeriod + includes the turnaround time for MROI scanning
            % Definition of lineAcquisitionPeriod:
            %   * lineAcquisitionPeriod is the period that is actually used for the image acquisition
            scanfield = obj.satisfyConstraints(scanfield);
            % These are set to the line scan period of the resonant scanner. Since the resonant scanner handles image
            % formation, these parameters do not have the same importance as in Galvo Galvo scanning.
            lineAcquisitionPeriod = obj.scanners{1}.scannerPeriod;
            lineScanPeriod = obj.scanners{1}.scannerPeriod;
        end
        
        function seconds = transitTime(obj,scanfield_from,scanfield_to)
            %% Returns the estimated time required to position the scanners when
            % moving from scanfield to scanfield.
            % Must be a multiple of the line time
            assert(transitArgumentTypeCheck(scanfield_from,scanfield_to));
            
            % FIXME: compute estimated transit time for reals
            % caller should constraint this to be an integer number of periods
            seconds = obj.scanners{2}.flybackTimeSeconds;
        end
    end
    
    methods(Access=private)        
        function volts=fov2volts(obj,fov,iscanner)
            s=obj.scanners{iscanner};
            %% Converts from fov coordinates to volts
            A = s.fullAngleDegrees;  % amplitude
            V = s.voltsPerDegree;    % conversion factor
            O = 0;                   % offset
            F = 1;                   % flip
            if(isa(s,'scanimage.mroi.scanners.Galvo'))
                O=A/2;
                F=1-2.*s.invertDirection;
            end
            volts = (fov.*A-O).*V.*F;
        end

        function fov=degrees2fov(obj,imirror,degrees)
            m=obj.scanners{imirror};
            A = m.fullAngleDegrees;     % amplitude
            O = A./2;                   % offset (degrees)
            F = 1-2.*m.invertDirection; % flip
            fov=(F.*degrees+O)./A;
        end
        
        function n=numberOfLines(obj,scanfield)
            rect=scanfield.boundingbox();
            n=rect(4)*obj.scanners{2}.pixelCount;
            n=floor(n/2)*2; % force even number of lines
        end
    end
end

function tf=transitArgumentTypeCheck(scanfield_from,scanfield_to)
from = isempty(scanfield_from) || isa(scanfield_from,'scanimage.mroi.scanfield.ScanField');
to   = isempty(scanfield_from) || isa(scanfield_from,'scanimage.mroi.scanfield.ScanField');
bothempty = isempty(scanfield_from) && isempty(scanfield_to);
tf= from && to && ~bothempty;
end

function asserttype(val,typestr)
assert(isa(val,typestr),'MROI:TypeError','Got type %s.  Expected type %s.',class(val),typestr);
end


%--------------------------------------------------------------------------%
% FastZ.m                                                                  %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
