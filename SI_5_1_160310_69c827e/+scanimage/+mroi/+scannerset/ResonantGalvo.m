classdef ResonantGalvo < scanimage.mroi.scannerset.ScannerSet
    
    properties
        scanners={}; % order is resonant, galvo, galvo
        fillFractionSpatial;
    end
    
    properties(Constant)
        CONSTRAINTS = {@scanimage.mroi.constraints.noRotation @scanimage.mroi.constraints.maxWidth @scanimage.mroi.constraints.sameWidth...
                       @scanimage.mroi.constraints.centeredX @scanimage.mroi.constraints.maxHeight @scanimage.mroi.constraints.yCenterInRange...
                       @scanimage.mroi.constraints.samePixelsPerLine};
        BEAM_SCANNER_ID = 3;
    end
    
    methods(Static)
        function obj=default()
            %% Construct a default version of this scanner set for testing
            g=scanimage.mroi.scanners.Galvo.default();
            r=scanimage.mroi.scanners.Resonant.default();
            obj=scanimage.mroi.scannerset.ResonantGalvo(r,g,[]);
        end
    end
    
    methods
        function obj = ResonantGalvo(name,resonantx,galvoy,beams,fillFractionSpatial)
            %% ResonantGalvoGalvo(resonantx,galvoy,beams)
            % 
            % Describes a resonant-galvo scanner set.
            scanimage.mroi.util.asserttype(resonantx,'scanimage.mroi.scanners.Resonant');
            scanimage.mroi.util.asserttype(galvoy,'scanimage.mroi.scanners.Galvo');
            
            obj.name = name;
            obj.scanners={resonantx,galvoy};
            obj.fillFractionSpatial = fillFractionSpatial;
            
            if ~isempty(beams)
                scanimage.mroi.util.asserttype(beams,'scanimage.mroi.scanners.Beams');
                obj.scanners{end+1}=beams;
            end
        end
        
        function fovarray = fov(obj)
            resFov = [ -obj.scanners{1}.fullAngleDegrees/2 obj.scanners{1}.fullAngleDegrees/2 ];
            galvoFov = [ -obj.scanners{2}.fullAngleDegrees/2 obj.scanners{2}.fullAngleDegrees/2 ];
            fovarray = [ resFov galvoFov ];
        end

        function scanfield = satisfyConstraints(obj,scanfield)
            %% Returns a recomputed scanfield that satisfies any constraints
            %  that need to be fullfilled for the scannerset.
            %
            % In this case the scanfield size is expanded so an even number
            % of pixels are used in x and y.
            assert(isa(scanfield,'scanimage.mroi.scanfield.ImagingField'),'Cannot handle scanfield of class %s',class(scanfield));
            scanfield.pixelResolution(1) = ceil(scanfield.pixelResolution(1) / 2) * 2; % only allow even number of pixels in lines
            scanfield.pixelResolution(2) = round(scanfield.pixelResolution(2));
        end        
        
        function [ao_volts, seconds] = scanPathAO(obj,scanfield,roi,actz,dzdt)
            [path_FOV, seconds] = obj.scanPathFOV(scanfield,roi,actz,dzdt);
            ao_volts = obj.pathFovToAo(path_FOV);
        end
        
        function ao_volts = pathFovToAo(obj,path_FOV)
            ao_volts.R = obj.fov2volts(path_FOV.R,1);
            ao_volts.G = obj.fov2volts(path_FOV.G,2);           
            
            if obj.hasBeams
                bIDs = obj.scanners{obj.BEAM_SCANNER_ID}.beamIDs;
                for i = 1:numel(bIDs)
                    ao_volts.B(:,i) = obj.scanners{obj.BEAM_SCANNER_ID}.powerFracToVoltageFunc(bIDs(i),path_FOV.B(:,i));
                    if obj.hasPowerBox
                        ao_volts.Bpb(:,i) = obj.scanners{obj.BEAM_SCANNER_ID}.powerFracToVoltageFunc(bIDs(i),path_FOV.Bpb(:,i));
                    end
                end
            end
        end
            
        function [path_FOV, seconds] = scanPathFOV(obj,scanfield,roi,actz,dzdt)
           %% Returns struct. Each field has ao channel data in collumn vectors
            % 
            % ao_volts.R: resonant amplitude
            % ao_volts.G: galvo
            % ao_volts.B: beams (columns are beam1,beam2,...,beamN)
            %
            % Output should look like:
            % 1.  Resonant_amplitude is constant. Set for width of scanned
            %     field
            % 2.  Galvo is continuously moving down the field.
            assert(isa(scanfield,'scanimage.mroi.scanfield.ScanField'));            
            seconds=obj.scanTime(scanfield);
            rect=scanfield.boundingbox();
            
            rsamples=ceil(seconds*obj.scanners{1}.sampleRateHz);
            rfov=rect(3)*ones(rsamples,1);
            
            gsamples=ceil(seconds*obj.scanners{2}.sampleRateHz);
            gfov=linspace(rect(2),rect(2)+rect(4),gsamples)';
            
            path_FOV.R = round(rfov * 1000000 / obj.fillFractionSpatial) / 1000000;
            path_FOV.G = gfov;
            
            assert(all(path_FOV.R <= 1.0001) && all(path_FOV.R >= -0.0001), 'Attempted to scan outside resonant scanner FOV.');
            assert(all(path_FOV.G <= 1.0001) && all(path_FOV.G >= -0.0001), 'Attempted to scan outside Y galvo scanner FOV.');
            
            %avoid small rounding error
            path_FOV.R(path_FOV.R > 1) = 1;
            path_FOV.R(path_FOV.R < 0) = 0;
            path_FOV.G(path_FOV.G > 1) = 1;
            path_FOV.G(path_FOV.G < 0) = 0;
            
            %% Beams AO
            % determine number of samples
            if obj.hasBeams
                hBm = obj.scanners{obj.BEAM_SCANNER_ID};
                
                [~,lineAcquisitionPeriod] = obj.linePeriod(scanfield);
                bExtendSamples = floor(hBm.beamClockExtend * 1e-6 * hBm.sampleRateHz);
                bSamplesPerLine = ceil(lineAcquisitionPeriod*hBm.sampleRateHz) + 1 + bExtendSamples;
                nlines = scanfield.pixelResolution(2);
                
                % get roi specific beam settings
                [powers, pzAdjust, Lzs, interlaceDecimation, interlaceOffset] = obj.getRoiBeamProps(...
                    roi, 'powers', 'pzAdjust', 'Lzs', 'interlaceDecimation', 'interlaceOffset');
                
                % determine which beams need decimation
                ids = find(interlaceDecimation ~= 1);
                
                % start with nomimal power fraction sample array for single line
                powerFracs = repmat(powers,bSamplesPerLine,1);
                
                % zero last sample of line if blanking flyback. beam decimation requires blanking
                if hBm.flybackBlanking || numel(ids)
                    powerFracs(end,:) = 0;
                end
                % replicate for n lines
                powerFracs = repmat(powerFracs,nlines,1);
                
                % mask off lines if decimated
                for i = 1:numel(ids)
                    lineMask = zeros(1,interlaceDecimation(ids(i)));
                    lineMask(1+interlaceOffset(ids(i))) = 1;
                    lineMask = repmat(lineMask, bSamplesPerLine, ceil(nlines/interlaceDecimation(ids(i))));
                    lineMask(:,nlines+1:end) = [];
                    lineMask = reshape(lineMask,[],1);
                    powerFracs(:,ids(i)) = powerFracs(:,ids(i)) .* lineMask;
                end
                
                % apply power boxes
                powerFracsPb = powerFracs;
                for pb = obj.scanners{obj.BEAM_SCANNER_ID}.powerBoxes
                    rx1 = pb.rect(1);
                    rx2 = pb.rect(1)+pb.rect(3);
                    ry1 = pb.rect(2);
                    ry2 = pb.rect(2)+pb.rect(4);
                    
                    lineSamps = bSamplesPerLine-1;
                    smpStart = ceil(max(1,min(lineSamps,lineSamps*rx1)));
                    smpEnd = ceil(max(1,min(lineSamps,lineSamps*rx2)));
                    lineStart = floor(min(nlines-1,max(0,nlines * ry1)));
                    lineEnd = floor(min(nlines-1,max(0,nlines * ry2)));
                    
                    for ln = lineStart:lineEnd
                        md = mod(ln+1,2)>0;
                        if (md && pb.oddLines) || ((~md) && pb.evenLines)
                            if obj.scanners{1}.bidirectionalScan && mod(ln,2)
                                se = bSamplesPerLine - smpStart;
                                ss = bSamplesPerLine - smpEnd;
                            else
                                ss = smpStart;
                                se = smpEnd;
                            end
                            powerFracsPb(bSamplesPerLine*ln+ss:bSamplesPerLine*ln+se,:) = repmat(pb.powers,se-ss+1,1);
                        end
                    end
                end
                
                if any(pzAdjust)
                    % create array of z position corresponding to each sample
                    if dzdt ~= 0
                        lineSampleTimes = nan(bSamplesPerLine,nlines);
                        lineSampleTimes(1,:) = linspace(0,obj.linePeriod(scanfield)*(nlines-1),nlines);
                        lineSampleTimes(1,:) = lineSampleTimes(1,:) + 0.25*((1-obj.scanners{1}.fillFractionTemporal) * obj.scanners{1}.scannerPeriod);
                        
                        for i = 1:nlines
                            lineSampleTimes(:,i) = linspace(lineSampleTimes(1,i), lineSampleTimes(1,i) + (bSamplesPerLine-1)/hBm.sampleRateHz, bSamplesPerLine);
                        end
                        
                        lineSampleZs = actz + lineSampleTimes * dzdt;
                        lineSampleZs = reshape(lineSampleZs,[],1);
                    else
                        lineSampleZs = actz * ones(size(powerFracs,1),1);
                    end
                    
                    % scale power fracs using Lz
                    adj = find(pzAdjust == true);
                    for i = 1:numel(adj)
                        LzArray = repmat(Lzs(adj(i)), bSamplesPerLine*nlines,1);
                        powerFracs(:,adj(i)) = powerFracs(:,adj(i)) .* exp(lineSampleZs./LzArray);
                        if obj.hasPowerBox
                            powerFracsPb(:,adj(i)) = powerFracsPb(:,adj(i)) .* exp(lineSampleZs./LzArray);
                        end
                    end
                end
                
                % IDs of the beams actually being used in this acq
                bIDs = hBm.beamIDs;
                for i = 1:numel(bIDs)
                    path_FOV.B(:,i) = min(powerFracs(:,bIDs(i)),hBm.powerLimits(bIDs(i))) / 100;
                    if obj.hasPowerBox
                        pFs = powerFracsPb(:,bIDs(i));
                        path_FOV.Bpb(:,i) = min(pFs,hBm.powerLimits(bIDs(i))) / 100;
                        path_FOV.Bpb(isnan(pFs),i) =  path_FOV.B(isnan(pFs),i);
                    end
                end
            end
        end
        
        function count = nsamples(obj,scannerid,seconds)
            count = seconds*obj.scanners{scannerid}.sampleRateHz;
        end
        
        function seconds = nseconds(obj,scannerid,nsamples)
            seconds = nsamples / obj.scanners{scannerid}.sampleRateHz; 
        end
        
        function position_FOV = mirrorsActiveParkFovPosition(obj)            
            position_FOV=zeros(1,2);
            position_FOV(:,1) = NaN; % we can't really calculate that here. the resonant scanner amplitude should not be touched for the flyback. NaN makes sure that nobody accidentally tries to use this value.
            position_FOV(:,2) = obj.degrees2fov(2,obj.scanners{2}.parkAngleDegrees);
        end
        
        function path_FOV = interpolateTransits(obj,path_FOV)
            path_FOV.G = scanimage.mroi.util.interpolateCircularNaNRanges(path_FOV.G,[0,1]);
            
            % beams ao
            if obj.hasBeams
                bIDs = obj.scanners{obj.BEAM_SCANNER_ID}.beamIDs;
                if obj.scanners{obj.BEAM_SCANNER_ID}.flybackBlanking || any(obj.scanners{obj.BEAM_SCANNER_ID}.interlaceDecimation(bIDs) > 1)
                    for ctr = 1:numel(bIDs)
                        path_FOV.B(isnan(path_FOV.B(:,ctr)),ctr) = 0;
                    end
                else
                    for ctr = 1:numel(bIDs)
                        path_FOV.B(:,ctr) = scanimage.mroi.util.expInterpolateCircularNaNRanges(path_FOV.B(:,ctr),obj.scanners{obj.BEAM_SCANNER_ID}.Lzs(bIDs(ctr)));
                        path_FOV.B(end,ctr) = 0;
                    end
                end
                
                if obj.hasPowerBox
                    if obj.scanners{obj.BEAM_SCANNER_ID}.flybackBlanking || any(obj.scanners{obj.BEAM_SCANNER_ID}.interlaceDecimation(bIDs) > 1)
                        for ctr = 1:numel(bIDs)
                            path_FOV.Bpb(isnan(path_FOV.Bpb(:,ctr)),ctr) = 0;
                        end
                    else
                        for ctr = 1:numel(bIDs)
                            path_FOV.Bpb(:,ctr) = scanimage.mroi.util.expInterpolateCircularNaNRanges(path_FOV.Bpb(:,ctr),obj.scanners{obj.BEAM_SCANNER_ID}.Lzs(bIDs(ctr)));
                            path_FOV.Bpb(end,ctr) = 0;
                        end
                    end
                end
            end
        end
        
        function [path_FOV, dt] = transitNaN(obj,scanfield_from,scanfield_to)
            assert(scanimage.mroi.util.transitArgumentTypeCheck(scanfield_from,scanfield_to));
            
            dt = obj.transitTime(scanfield_from,scanfield_to);
            if ~isempty(scanfield_to) && isnan(scanfield_to)
                dt = 0; % flyback time is added in padFrameAO
            end
            
            rsamples = round(dt*obj.scanners{1}.sampleRateHz);
            path_FOV.R = nan(rsamples,1);
            
            gsamples = round(dt*obj.scanners{2}.sampleRateHz);
            path_FOV.G = nan(gsamples,1);
            
            if obj.hasBeams
                hBm = obj.scanners{obj.BEAM_SCANNER_ID};
                [lineScanPeriod,lineAcquisitionPeriod] = obj.linePeriod([]);
                bExtendSamples = floor(hBm.beamClockExtend * 1e-6 * hBm.sampleRateHz);
                bSamplesPerLine = ceil(lineAcquisitionPeriod*hBm.sampleRateHz) + 1 + bExtendSamples;
                nlines = round(dt/lineScanPeriod);
                path_FOV.B = NaN(bSamplesPerLine*nlines,numel(hBm.beamIDs));
                if obj.hasPowerBox
                    path_FOV.Bpb = path_FOV.B;
                end
            end
        end
        
        function path_FOV = zFlybackFrame(obj, frameTime)            
            path_FOV.R = nan(round(obj.nsamples(1,frameTime)),1);
            path_FOV.G = nan(round(obj.nsamples(2,frameTime)),1);
            
            % Beams AO
            if obj.hasBeams
                hBm = obj.scanners{obj.BEAM_SCANNER_ID};
                [lineScanPeriod,lineAcquisitionPeriod] = obj.linePeriod([]);
                bExtendSamples = floor(hBm.beamClockExtend * 1e-6 * hBm.sampleRateHz);
                bSamplesPerLine = ceil(lineAcquisitionPeriod*hBm.sampleRateHz) + 1 + bExtendSamples;
                nlines = round(frameTime/lineScanPeriod);

                if hBm.flybackBlanking
                    path_FOV.B = zeros(bSamplesPerLine*nlines,numel(hBm.beamIDs));
                else
                    path_FOV.B = NaN(bSamplesPerLine*nlines,numel(hBm.beamIDs));
                end
                
                if obj.hasPowerBox
                    path_FOV.Bpb = path_FOV.B;
                end
            end
        end
        
        function path_FOV = padFrameAO(obj, path_FOV, frameTime, flybackTime)
                   
            padSamples = ceil(obj.nsamples(2,frameTime+flybackTime/2)) - size(path_FOV.G,1); % cut off half of the flyback time to leave some breathing room to receive the next frame trigger
            if padSamples > 0
                path_FOV.R(end+1:end+padSamples,:) = NaN;
                path_FOV.G(end+1:end+padSamples,:) = NaN;
            end
            
            % Beams AO
            if obj.hasBeams
                hBm = obj.scanners{obj.BEAM_SCANNER_ID};
                [lineScanPeriod,lineAcquisitionPeriod] = obj.linePeriod([]);
                bExtendSamples = floor(hBm.beamClockExtend * 1e-6 * hBm.sampleRateHz);
                bSamplesPerLine = ceil(lineAcquisitionPeriod*hBm.sampleRateHz) + 1 + bExtendSamples;
                nlines = round(frameTime/lineScanPeriod);
                nTotalSamples = bSamplesPerLine * nlines;
                padSamples = nTotalSamples - size(path_FOV.B,1);
                if padSamples > 0
                    path_FOV.B(end+1:end+padSamples,:) = NaN;
                    if obj.hasPowerBox
                        path_FOV.Bpb(end+1:end+padSamples,:) = NaN;
                    end
                end
            end
        end

        function [pixelSizeX,pixelSizeY] = pixelSizeAngle(obj,scanfield)
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
            %% Returns the time required to scan the scanfield in seconds
            numLines = scanfield.pixelResolution(2);
            seconds = (numLines/2^(obj.scanners{1}.bidirectionalScan)) * obj.scanners{1}.scannerPeriod; %eg 512 lines / (7920 lines/s)
            numSamples = round(seconds * obj.scanners{2}.sampleRateHz);
            seconds = numSamples / obj.scanners{2}.sampleRateHz;
        end

        function [lineScanPeriod,lineAcquisitionPeriod] = linePeriod(obj,scanfield)
            % Definition of lineScanPeriod:
            %   * scanPeriod is lineAcquisitionPeriod + includes the turnaround time for MROI scanning
            % Definition of lineAcquisitionPeriod:
            %   * lineAcquisitionPeriod is the period that is actually used for the image acquisition

            % These are set to the line scan period of the resonant scanner. Since the resonant scanner handles image
            % formation, these parameters do not have the same importance as in Galvo Galvo scanning.
            lineScanPeriod = obj.scanners{1}.scannerPeriod / 2^(obj.scanners{1}.bidirectionalScan);
            lineAcquisitionPeriod = obj.scanners{1}.scannerPeriod / 2 * obj.scanners{1}.fillFractionTemporal;
        end
        
        function [startTimes, endTimes] = acqActiveTimes(obj,scanfield)
            % TODO: implement this
            startTimes = [NaN];
            endTimes   = [NaN];
        end
        
        function seconds = transitTime(obj,scanfield_from,scanfield_to)
            %% Returns the estimated time required to position the scanners when
            % moving from scanfield to scanfield.
            % Must be a multiple of the line time
            assert(scanimage.mroi.util.transitArgumentTypeCheck(scanfield_from,scanfield_to));
            
            % FIXME: compute estimated transit time for reals
            % caller should constraint this to be an integer number of periods
            if isnan(scanfield_from)
                seconds = 0; % do not scan first flyto in plane
                return
            end
            
            if isnan(scanfield_to)
                seconds = obj.scanners{2}.flybackTimeSeconds;
            else
                seconds = obj.scanners{2}.flytoTimeSeconds;
            end
                
            samples = round(seconds * obj.scanners{2}.sampleRateHz);
            seconds = samples / obj.scanners{2}.sampleRateHz;
        end
        
        function samplesPerTrigger = samplesPerTriggerForAO(obj,outputData)
            % input: unconcatenated output for the stack
            samplesPerTrigger.G = max( cellfun(@(frameAO)size(frameAO.G,1),outputData) );
            
            if obj.hasBeams
                hBm = obj.scanners{obj.BEAM_SCANNER_ID};
                [~,lineAcquisitionPeriod] = obj.linePeriod([]);
                bExtendSamples = floor(hBm.beamClockExtend * 1e-6 * hBm.sampleRateHz);
                samplesPerTrigger.B = ceil( lineAcquisitionPeriod * hBm.sampleRateHz ) + 1 + bExtendSamples;
            end
        end
        
        function cfg = beamsTriggerCfg(obj)
            cfg = struct();
            if obj.hasBeams
                cfg.triggerType = 'lineClk';
                cfg.requiresReferenceClk = false;
            else
                cfg.triggerType = '';
                cfg.requiresReferenceClk = [];
            end
        end
        
        function v = resonantScanFov(obj, roiGroup)
            % returns the resonant fov that will be used to scan the
            % roiGroup. Assumes all rois will have the same x fov
            if ~isempty(roiGroup.activeRois) && ~isempty(roiGroup.activeRois(1).scanfields)
                rect=roiGroup.activeRois(1).scanfields(1).boundingbox();
                v = rect(3) / obj.fillFractionSpatial;
            else
                v = 0;
            end
        end
        
        function v = resonantScanVoltage(obj, roiGroup)
            % returns the resonant voltage that will be used to scan the
            % roiGroup. Assumes all rois will have the same x fov
            if ~isempty(roiGroup.activeRois) && ~isempty(roiGroup.activeRois(1).scanfields)
                rect=roiGroup.activeRois(1).scanfields(1).boundingbox();
                v = obj.fov2volts(rect(3) / obj.fillFractionSpatial,1);
            else
                v = 0;
            end
        end
    end
    
    methods(Access=private)
        function volts=fov2volts(obj,fov,iscanner)
            %% Converts from fov coordinates to volts
            s=obj.scanners{iscanner};
            if isa(s,'scanimage.mroi.scanners.Resonant')
                u_fov = unique(fov);
                u_fov(isnan(u_fov)) = [];
                volts = fov; %preallocate, copy nan's
                for i = 1:numel(u_fov)
                    volts(fov == u_fov(i)) = s.fov2VoltageFunc(u_fov(i));
                end
            else
                A = s.fullAngleDegrees;  % amplitude
                V = s.voltsPerDegree;    % conversion factor
                O = 0;                   % offset
                if isa(s,'scanimage.mroi.scanners.Galvo')
                    O=A/2;
                end
                volts = (fov.*A-O).*V;
            end
        end

        function fov=degrees2fov(obj,imirror,degrees)
            m=obj.scanners{imirror};
            A = m.fullAngleDegrees;     % amplitude
            O = A./2;                   % offset (degrees)
            fov=(degrees+O)./A;
        end
        
        function n=numberOfLines(obj,scanfield)
            rect=scanfield.boundingbox();
            n=rect(4)*obj.scanners{2}.pixelCount;
            n=floor(n/2)*2; % force even number of lines
        end
    end
end


%--------------------------------------------------------------------------%
% ResonantGalvo.m                                                          %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
