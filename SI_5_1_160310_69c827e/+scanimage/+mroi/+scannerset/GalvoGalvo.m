classdef GalvoGalvo < scanimage.mroi.scannerset.ScannerSet
    properties
        scanners={};
        fillFractionSpatial;  % fillFractionSpatial and fillFractionTemporal are equal for pure galvo galvo scanning
        settleTimeFraction;
        pixelTime;
        bidirectional;
        stepY = false;
    end
    
    properties (Constant)
        CONSTRAINTS = {@scanimage.mroi.constraints.evenPixelsPerLine @scanimage.mroi.constraints.maxWidth @scanimage.mroi.constraints.xCenterInRange...
                       @scanimage.mroi.constraints.maxHeight @scanimage.mroi.constraints.yCenterInRange};        
        BEAM_SCANNER_ID = 4;
    end
    
    methods(Static)
        function obj=default
            g=scanimage.mroi.scanners.Galvo.default;
            b=scanimage.mroi.scanners.Beams.default;
            obj=scanimage.mroi.scannerset.GalvoGalvo('Default GG set',g,g,g,b,.7,.001,true,true,0);
        end
    end
    
    methods
        function obj = GalvoGalvo(name,galvox,galvoy,galvoz,beams,fillFractionSpatial,pixelTime,bidirectional,stepY,settleTimeFraction)
            scanimage.mroi.util.asserttype(galvox,'scanimage.mroi.scanners.Galvo');
            scanimage.mroi.util.asserttype(galvoy,'scanimage.mroi.scanners.Galvo');
            scanimage.mroi.util.asserttype(galvoz,'scanimage.mroi.scanners.Galvo');
            
            obj.name = name;
            obj.scanners={galvox,galvoy,galvoz};
            obj.fillFractionSpatial = fillFractionSpatial;
            obj.pixelTime = pixelTime;
            obj.bidirectional = bidirectional;
            obj.stepY = stepY;
            obj.settleTimeFraction = settleTimeFraction;
            
            if ~isempty(beams)
                scanimage.mroi.util.asserttype(beams,'scanimage.mroi.scanners.Beams');
                obj.scanners{end+1}=beams;
            end
        end
        
        function fovarray = fov(obj)
            galvoRangeX = obj.scanners{1}.fullAngleDegrees;
            galvoRangeY = obj.scanners{2}.fullAngleDegrees;
            galvoRangeZ = obj.scanners{3}.fullAngleDegrees;
            fovarray = [ -galvoRangeX/2 , galvoRangeX/2 , -galvoRangeY/2 , galvoRangeY/2,  -galvoRangeZ/2 , galvoRangeZ/2 ];
        end
        
        function scanfield = satisfyConstraints(obj,scanfield)
            %% Returns a recomputed scanfield that satisfies any constraints
            %  that need to be fullfilled for the scannerset.
            %
            % In this case the scanfield size is expanded so an even number
            % of pixels are used in x and y.
            
            scanfield.pixelResolution(1) = ceil(scanfield.pixelResolution(1) / 2) * 2; % only allow even number of pixels in lines
            scanfield.pixelResolution(2) = round(scanfield.pixelResolution(2));
        end
        
        function [ao_volts,seconds] = scanPathAO(obj,scanfield,roi,actz,dzdt)
            [path_FOV, seconds] = obj.scanPathFOV(scanfield,roi,actz,dzdt);
            ao_volts = obj.pathFovToAo(path_FOV);
        end
        
        function path_FOV = pathAoToFov(obj,ao_volts)
            path_FOV.G(:,1) = obj.volts2fov(ao_volts(:,1),1);
            path_FOV.G(:,2) = obj.volts2fov(ao_volts(:,2),2);
            % AS prob. need one more line here at some point, not needed for FOCUS or GRAB
        end
        
        function ao_volts = pathFovToAo(obj,path_FOV)
            tolerance = 1e-4;
            assert(isempty(path_FOV.G) || (min(path_FOV.G(:,1) >= 0-tolerance) && max(path_FOV.G(:,1) <= 1+tolerance)), 'Attempted to scan outside X galvo scanner FOV.');
            assert(isempty(path_FOV.G) || (min(path_FOV.G(:,2) >= 0-tolerance) && max(path_FOV.G(:,2) <= 1+tolerance)), 'Attempted to scan outside Y galvo scanner FOV.');
            assert(isempty(path_FOV.G) || (min(path_FOV.G(:,3) >= 0-tolerance) && max(path_FOV.G(:,3) <= 1+tolerance)), 'Attempted to scan outside Z galvo scanner FOV.');
            
            %avoid small rounding error
            path_FOV.G(path_FOV.G > 1) = 1;
            path_FOV.G(path_FOV.G < 0) = 0;
            
            ao_volts = struct();
            ao_volts.G(:,1) = obj.fov2volts(path_FOV.G(:,1),1);
            ao_volts.G(:,2) = obj.fov2volts(path_FOV.G(:,2),2);
            ao_volts.G(:,3) = obj.fov2volts(path_FOV.G(:,3),3);
            
            if obj.hasBeams
                bIDs = obj.scanners{obj.BEAM_SCANNER_ID}.beamIDs;
                ao_volts.B = zeros(size(path_FOV.B));
                for i = 1:numel(bIDs)
                    ao_volts.B(:,i) = obj.scanners{obj.BEAM_SCANNER_ID}.powerFracToVoltageFunc(bIDs(i),path_FOV.B(:,i));
                    
                    if obj.hasPowerBox
                        ao_volts.Bpb(:,i) = obj.scanners{obj.BEAM_SCANNER_ID}.powerFracToVoltageFunc(bIDs(i),path_FOV.Bpb(:,i));
                    end
                end
            end
        end
        
        function [path_FOV, seconds] = scanPathFOV(obj,scanfield,roi,actz,dzdt)
            assert(isa(scanfield,'scanimage.mroi.scanfield.ScanField'));
            obj.checkScannerSampleRateRatios();
            
            if isa(scanfield,'scanimage.mroi.scanfield.ImagingField')
                [path_FOV, seconds] = obj.scanPathImagingFOV(scanfield,roi,actz,dzdt);
            elseif isa(scanfield,'scanimage.mroi.scanfield.fields.StimulusField')
                [path_FOV, seconds] = obj.scanPathStimulusFOV(scanfield,roi,actz,dzdt);
            else
                error('function scanPathFOV is undefined for class of type %s',class(scanfield));
            end
        end
        
        function [success,imageData,stripePosition] = formImage(obj,scanfieldParams,sampleBuffer,fieldSamples,channelsActive,linePhaseSamples)
            [dataBuffer,bufferStartSample,bufferEndSample] = sampleBuffer.getData();
            datatypeAi = class(dataBuffer);
            
            % apply line phase
            dataBuffer = circshift(dataBuffer, [-linePhaseSamples 0]);
            placeholdervalue = intmin(datatypeAi);
            dataBuffer(1:-linePhaseSamples,:) = placeholdervalue; % we don't want the circshift to roll over
            dataBuffer(end-linePhaseSamples+1:end,:) = placeholdervalue;
            
            xPixels = scanfieldParams.pixelResolution(1);
            yPixels = scanfieldParams.pixelResolution(2);
            
            fieldStartSample = fieldSamples(1);
            fieldEndSample   = fieldSamples(2);
            
            stripeStartSample = round( fieldStartSample + floor( ( bufferStartSample - fieldStartSample + 1 ) / scanfieldParams.lineScanSamples ) * scanfieldParams.lineScanSamples );
            stripeStartSample = max(fieldStartSample,stripeStartSample);
            stripeEndSample   = round( fieldStartSample + floor( (  bufferEndSample  - fieldStartSample + 1 ) / scanfieldParams.lineScanSamples ) * scanfieldParams.lineScanSamples - 1);
            stripeEndSample   = min(stripeEndSample,fieldEndSample);
            
            stripePosition(1) = round( (stripeStartSample - fieldStartSample)/scanfieldParams.lineScanSamples + 1 );
            stripePosition(2) = round( (stripeEndSample - fieldStartSample + 1)/scanfieldParams.lineScanSamples );
            
            if stripePosition(1) < 1 || stripePosition(2) > yPixels || stripePosition(1) > stripePosition(2)
                success = false;
                imageData = {};
                stripePosition = [];
                return
            end
            
            numLines = diff(stripePosition) + 1;
            numChans = length(channelsActive);
            
            imageData = {};
            for idx = 1:numChans
                chan = channelsActive(idx);
                chanAi = dataBuffer(stripeStartSample:stripeEndSample,chan);
                chanAi = reshape(chanAi,scanfieldParams.lineScanSamples,numLines); % image is transposed at this point
                
                % crop 'overscan'
                overScanSamples = (scanfieldParams.lineScanSamples-scanfieldParams.lineAcqSamples)/2;
                chanAi(1:overScanSamples,:) = [];
                chanAi(end-overScanSamples+1:end,:) = [];
                
                % flip lines for bidirectional scanning
                if obj.bidirectional
                    flipevenlines = mod(stripePosition(1),2)>0;
                    chanAi(:,2^flipevenlines:2:end) = flipud(chanAi(:,2^flipevenlines:2:end)); % mirror every second line of the image
                end
                
                pixelBinFactor = scanfieldParams.lineAcqSamples/xPixels;
                assert(mod(pixelBinFactor,1) == 0)
                
                if pixelBinFactor > 1
                    chanAi = reshape(mean(reshape(chanAi,pixelBinFactor,[]),1),xPixels,numLines);
                    chanAi = cast(chanAi,datatypeAi);
                end
                imageData{idx} = chanAi; % imageData is transposed at this point
            end
            success = true;
        end
        
        function count = nsamples(obj,iscanner,seconds)
            count = round(seconds*obj.scanners{iscanner}.sampleRateHz);
        end
        
        function seconds = nseconds(obj,iscanner,nsamples)
            seconds = nsamples / obj.scanners{iscanner}.sampleRateHz;
        end
        
        function [seconds,durationPerRepetitionInt,durationPerRepetitionFrac] = scanTime(obj,scanfield)
            if isa(scanfield,'scanimage.mroi.scanfield.ImagingField')
                lineScanPeriod = obj.linePeriod(scanfield);
                numLines = scanfield.pixelResolution(2);
                seconds = lineScanPeriod * numLines;
                durationPerRepetitionInt = [];
                durationPerRepetitionFrac = [];
            elseif isa(scanfield,'scanimage.mroi.scanfield.fields.StimulusField')
                slowestRate = obj.slowestScannerSampleRate(); % normalize period to integer number of samples of slowest output
                repetitionsInteger        = fix(scanfield.repetitions);
                durationPerRepetitionInt  = round(slowestRate * scanfield.duration) / slowestRate;
                durationPerRepetitionFrac = round(slowestRate * scanfield.duration * (scanfield.repetitions-repetitionsInteger) ) / slowestRate;
                seconds = durationPerRepetitionInt * repetitionsInteger + durationPerRepetitionFrac;
            else
                error('Function scanTime is undefined for class of type %s',class(scanfield));
            end
        end
        
        function [lineScanPeriod,lineAcquisitionPeriod] = linePeriod(obj,scanfield)
            assert(isempty(scanfield)||isa(scanfield,'scanimage.mroi.scanfield.ImagingField'),...
                'Function linePeriod undefined for class of type %s',class(scanfield));
            pixelsX = scanfield.pixelResolution(1);
            
            slowestRate = obj.slowestScannerSampleRate(); % normalize line period to integer number of samples of slowest output
                        
            lineAcquisitionPeriod = pixelsX * obj.pixelTime;
            samplesAcq = lineAcquisitionPeriod * slowestRate;
            samplesTurnaroundHalf = ceil(((samplesAcq / obj.fillFractionSpatial) - samplesAcq)/2); % making sure this is an integer number
            
            samplesScan = samplesAcq + 2*samplesTurnaroundHalf;
            lineScanPeriod = samplesScan / slowestRate;
        end
        
        function [startTimes, endTimes] = acqActiveTimes(obj,scanfield)
            assert(isa(scanfield,'scanimage.mroi.scanfield.ImagingField'),'Function acqActiveTimes undefined for class of type %s',class(scanfield));
            lines   = scanfield.pixelResolution(2);
            [lineScanPeriod,lineAcquisitionPeriod] = obj.linePeriod(scanfield);
            
            padTime = (lineScanPeriod-lineAcquisitionPeriod)/2;
            startTimes = linspace(padTime,padTime + lineScanPeriod*(lines-1),lines)';
            endTimes = startTimes + lineAcquisitionPeriod; 
        end
        
        function seconds = transitTime(obj,scanfield_from,scanfield_to) %#ok<INUSL>
            if isa(scanfield_from,'scanimage.mroi.scanfield.fields.StimulusField') ||...
                isa(scanfield_to,'scanimage.mroi.scanfield.fields.StimulusField')
                seconds = 0;
                return                
            end
            
            assert(scanimage.mroi.util.transitArgumentTypeCheck(scanfield_from,scanfield_to));
                        
            if isnan(scanfield_from)
                seconds = 0; % do not scan first flyto in plane
                return
            end
            
            if isnan(scanfield_to)
                seconds = obj.scanners{2}.flybackTimeSeconds;
            else
                seconds = obj.scanners{2}.flytoTimeSeconds;
            end
                
            seconds = obj.nseconds(1,obj.nsamples(1,seconds)); % round to closest multiple of sample time
        end
        
        function position_FOV = mirrorsActiveParkFovPosition(obj)
            x=obj.degrees2fov(1,obj.scanners{1}.parkAngleDegrees);
            y=obj.degrees2fov(2,obj.scanners{2}.parkAngleDegrees);
            position_FOV = [x,y];
        end
        
        function [path_FOV, dt] = transitNaN(obj,scanfield_from,scanfield_to)
            assert(scanimage.mroi.util.transitArgumentTypeCheck(scanfield_from,scanfield_to));
            
            dt=obj.transitTime(scanfield_from,scanfield_to);
            
            gsamples = obj.nsamples(1,dt);
            path_FOV.G = nan(gsamples,3);
            
            if obj.hasBeams
                bsamples = obj.nsamples(obj.BEAM_SCANNER_ID,dt);
                bIDs = obj.scanners{obj.BEAM_SCANNER_ID}.beamIDs;
                
                if isnan(scanfield_to) && obj.scanners{obj.BEAM_SCANNER_ID}.flybackBlanking
                    path_FOV.B = zeros(bsamples,numel(bIDs));
                else
                    path_FOV.B = nan(bsamples,numel(bIDs));
                end
                
                if obj.hasPowerBox
                    path_FOV.Bpb = path_FOV.B;
                end
            end
        end
        
        function path_FOV = interpolateTransits(obj,path_FOV)
            path_FOV.G(:,1) = scanimage.mroi.util.interpolateCircularNaNRanges(path_FOV.G(:,1),[0,1]);
            path_FOV.G(:,2) = scanimage.mroi.util.interpolateCircularNaNRanges(path_FOV.G(:,2),[0,1]);
            
            % beams output FOV
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
                            path_FOV.Bpb(:,ctr) = scanimage.mroi.util.expInterpolateCircularNaNRanges(path_FOV.Bp(:,ctr),obj.scanners{obj.BEAM_SCANNER_ID}.Lzs(bIDs(ctr)));
                            path_FOV.Bpb(end,ctr) = 0;
                        end
                    end
                end
            end
        end
        
        function [pixelSizeX,pixelSizeY] = pixelSizeAngle(obj,scanfield)
            assert(isa(scanfield,'scanimage.mroi.scanfield.ImagingField'),'Function pixelSizeAngle undefined for class of type %s',class(scanfield));
            sWidth     = scanfield.rect(3); % width of the scanfield  (0..1)
            sHeight    = scanfield.rect(4); % height of the scanfield (0..1)
            pixelsNumX = scanfield.pixelResolution(1);
            pixelsNumY = scanfield.pixelResolution(2);
            
            fullAngleX = obj.scanners{1}.fullAngleDegrees;
            fullAngleY = obj.scanners{2}.fullAngleDegrees;
            
            pixelSizeX = fullAngleX * sWidth  / pixelsNumX;
            pixelSizeY = fullAngleY * sHeight / pixelsNumY;
        end
        
        function samplesPerTrigger = samplesPerTriggerForAO(obj,outputData)
            % input: unconcatenated output for the stack
            samplesPerTrigger.G = max( cellfun(@(frameAO)size(frameAO.G,1),outputData) );
            
            if obj.hasBeams
                samplesPerTrigger.B = max( cellfun(@(frameAO)size(frameAO.B,1),outputData) );
            end
        end
        
        function cfg = beamsTriggerCfg(obj)
            cfg = struct();
            if obj.hasBeams
                cfg.triggerType = 'frameClk';
                cfg.requiresReferenceClk = true;
            else
                cfg.triggerType = '';
                cfg.requiresReferenceClk = [];
            end
        end
        
        function path_FOV = padFrameAO(obj, path_FOV, frameTime, flybackTime)
            %TODO: Not sure yet what to do with this
            padSamplesG = obj.nsamples(1,frameTime+flybackTime) - size(path_FOV.G,1);
            if padSamplesG > 0
                path_FOV.G(end+1:end+padSamplesG,:) = NaN;
            end
            
            % Beams AO
            if obj.hasBeams
                padSamplesB = obj.nsamples(obj.BEAM_SCANNER_ID,frameTime+flybackTime) - size(path_FOV.B,1);
                if padSamplesB > 0
                    path_FOV.B(end+1:end+padSamplesB,:) = NaN;
                    if obj.hasPowerBox
                        path_FOV.Bpb(end+1:end+padSamplesB,:) = NaN;
                    end
                end
            end
        end
        
        function path_FOV = zFlybackFrame(obj, frameTime)
            position_FOV = obj.mirrorsActiveParkFovPosition();
            path_FOV.G = repmat(position_FOV(1:2),obj.nsamples(2,frameTime),1);
            
            if obj.hasBeams
                bSamples = obj.nsamples(obj.BEAM_SCANNER_ID,frameTime);
                bIDs = obj.scanners{obj.BEAM_SCANNER_ID}.beamIDs;
                
                if obj.scanners{obj.BEAM_SCANNER_ID}.flybackBlanking
                    path_FOV.B = zeros(bSamples,numel(bIDs));
                else
                    path_FOV.B = NaN(bSamples,numel(bIDs));
                end
                
                if obj.hasPowerBox
                    path_FOV.Bpb = path_FOV.B;
                end
            end
        end
    end
    
    methods(Hidden)
        function checkScannerSampleRateRatios(obj)
            assert( obj.scanners{1}.sampleRateHz == obj.scanners{2}.sampleRateHz );
            if obj.hasBeams
                galvosSampleRate = obj.scanners{1}.sampleRateHz;
                beamsSampleRate  = obj.scanners{obj.BEAM_SCANNER_ID}.sampleRateHz;
                
                sampleRateRatio = galvosSampleRate / beamsSampleRate;
                assert(log2(sampleRateRatio) == nextpow2(sampleRateRatio),...
                    'The galvo output sample rate has to be 2^x times the beams output rate');
            end
        end
        
        function val = slowestScannerSampleRate(obj)
            if obj.hasBeams
                val = min( cellfun(@(scanner)scanner.sampleRateHz,obj.scanners) );
            else
                val = min( cellfun(@(scanner)scanner.sampleRateHz,obj.scanners(1:2)) );
            end
        end
        
        function [path_FOV, seconds] = scanPathImagingFOV(obj,scanfield,roi,actz,dzdt)
            %% Returns struct. Each field has ao channel data in column vectors
            % 
            % path_FOV.G: galvo (columns are X,Y)
            % path_FOV.B: beams (columns are beam1,beam2,...,beamN)
            %
            assert(isa(scanfield,'scanimage.mroi.scanfield.ScanField'));
            
            path_FOV = struct();
            [path_FOV,seconds] = generateGalvoPathImaging(path_FOV);
            
            if obj.hasBeams
                path_FOV = generateBeamsPathImaging(path_FOV);
            end
            
            %%% nested functions
            function [path_FOV,seconds] = generateGalvoPathImaging(path_FOV)
                % generate grid       
                [lineScanPeriod,lineAcquisitionPeriod] = obj.linePeriod(scanfield);
                nxAcq = obj.nsamples(1,lineAcquisitionPeriod); % number of active acquisition samples per line
                nx = obj.nsamples(1,lineScanPeriod);           % total number of scan samples per line
                ny = scanfield.pixelResolution(2);             % number of lines
                nTurn = nx - nxAcq;
                
                assert(obj.settleTimeFraction>=0 && obj.settleTimeFraction<=1,'settleTimeFraction must be in interval [0,1]. Current value: %f',obj.settleTimeFraction);
                nSettle = min(round(nTurn*obj.settleTimeFraction),nTurn);
                
                assert(rem(nTurn,2)==0); % sanity check: at the moment we only support even number of samples per line
                
                xxFillfrac = NaN(1,nTurn/2); % placeholder will be replaced below
                xxLine = [xxFillfrac linspace(0,1,nxAcq) xxFillfrac];
                xx = repmat(xxLine(:),1,ny);
                
                if obj.stepY
                    [yy,~]=meshgrid(linspace(0,ny,ny),linspace(0,nx,nx));
                    yy=yy./ny;
                else
                    yy = linspace(0,1,(nx*ny-nTurn))';
                    yy = [zeros(nTurn/2,1);yy;ones(nTurn/2,1)];
                end
                
                zz = zeros(size(yy),'like',yy);

                if obj.bidirectional
                    xx(:,2:2:end)=flipud(xx(:,2:2:end)); % flip every second line
                    
                    % compute turnaround
                    slopeX = 1/nxAcq;
                    
                    splineInterp  = spline([0 nTurn-nSettle+1],[slopeX,0,nSettle*slopeX,-slopeX],1:(nTurn-nSettle));
                    settleInterp = linspace(nSettle*slopeX,slopeX,nSettle);
                    interpTurnAround = [splineInterp settleInterp];
                    
                    turnXOdd  = 1 + interpTurnAround;
                    turnXEven = - interpTurnAround;
                else
                    % compute turnaround
                    slopeX = 1/nxAcq;
                    splineInterp = spline([0 nTurn-nSettle+1],[slopeX,1,-nSettle*slopeX,slopeX],1:(nTurn-nSettle));
                    settleInterp = linspace(-nSettle*slopeX,-slopeX,nSettle);
                    
                    turnXOdd  = [splineInterp settleInterp];
                    turnXEven = turnXOdd;
                end
                
                slopeY = 0;
                turnY = spline([0 nTurn+1],[slopeY,0,1/(ny-1),slopeY],1:nTurn);
                
                % transform meshgrid into column vectors
                xx = reshape(xx,[],1);
                yy = reshape(yy,[],1);
                zz = reshape(zz,[],1);

                for line = 1:(ny-1)
                    startIdx = nTurn/2 + line*nx - nTurn + 1;
                    endIdx   = nTurn/2 + line*nx;
                    
                    if mod(line,2) == 0 % line is even
                        xx(startIdx:endIdx) = turnXEven;
                    else
                        xx(startIdx:endIdx) = turnXOdd;
                    end
                    
                    if obj.stepY
                        yy(startIdx:endIdx) = turnY + (line-1)/(ny-1);
                    end
                end
                [xx,yy,zz]=scanfield.transform(xx,yy,zz);
                
                path_FOV.G(:,1) = xx;
                path_FOV.G(:,2) = yy;
                path_FOV.G(:,3) = zz;
                samples = size(path_FOV.G,1);
                seconds = obj.nseconds(1,samples);
            end
            
            function path_FOV = generateBeamsPathImaging(path_FOV)               
                % get roi specific beam settings
                [powers, pzAdjust, Lzs, interlaceDecimation, interlaceOffset] = obj.getRoiBeamProps(...
                    roi, 'powers', 'pzAdjust', 'Lzs', 'interlaceDecimation', 'interlaceOffset');                
                
                % determine number of samples
                [lineScanPeriod,lineAcquisitionPeriod] = obj.linePeriod(scanfield);
                nxAcq = obj.nsamples(obj.BEAM_SCANNER_ID,lineAcquisitionPeriod); % number of active acquisition samples per line
                nx = obj.nsamples(obj.BEAM_SCANNER_ID,lineScanPeriod);           % total number of scan samples per line, guaranteed to be an integer due to implementation of obj.linePeriod
                ny = scanfield.pixelResolution(2);
                nBlank = (nx - nxAcq) / 2;
                assert(rem(nBlank,1)==0); % sanity check, nBlank needs to be an integer number

                totalSamples = nx * ny;
                sampleTime = obj.nseconds(obj.BEAM_SCANNER_ID,1);
                
                % start with nomimal power fraction sample array for single line
                powerFracs = repmat(powers(:)',totalSamples,1);
                               
                % determine which beams need decimation
                ids = find(interlaceDecimation ~= 1);
                for id = ids
                    grid = nan(ny,nx);
                    grid(interlaceOffset(id)+1:interlaceDecimation(id):end,:) = 1;
                    mask = reshape(grid',[],1);
                    powerFracs(:,id) = powerFracs(:,id) .* mask;
                end
                
                % mask turnaround samples
                if obj.scanners{obj.BEAM_SCANNER_ID}.flybackBlanking || numel(ids)
                    grid = nan(ny,nx);
                    grid(:,(nBlank+1):(nBlank+nxAcq)) = 1;
                    mask = reshape(grid',[],1);
                    for iter = 1:size(powerFracs,2)
                        powerFracs(:,iter) = powerFracs(:,iter) .* mask;
                    end
                end
                
                % apply power box samples
                powerFracsPb = powerFracs;
                for pb = obj.scanners{obj.BEAM_SCANNER_ID}.powerBoxes
                    rx1 = pb.rect(1);
                    rx2 = pb.rect(1)+pb.rect(3);
                    ry1 = pb.rect(2);
                    ry2 = pb.rect(2)+pb.rect(4);
                    
                    xStart = floor(min(nxAcq-1,max(0,(nxAcq-1) * rx1))+1);
                    xEnd   = floor(min(nxAcq-1,max(0,(nxAcq-1) * rx2))+1);
                    yStart = floor(min(ny-1,max(0,(ny-1) * ry1))+1);
                    yEnd   = floor(min(ny-1,max(0,(ny-1) * ry2))+1);
                    
                    lines = yStart:yEnd;
                    inds = logical(mod(lines,2));
                    fwdLines = lines(inds);
                    revLines = lines(~inds);
                    xrStart = nxAcq - xEnd + 1;
                    xrEnd = nxAcq - xStart + 1;
                    
                    for iter = 1:size(powerFracs,2)
                        grid = nan(ny,nx);
                        if obj.bidirectional
                            grid(fwdLines,(nBlank+xStart):(nBlank+xEnd)) = pb.powers(iter);
                            grid(revLines,(nBlank+xrStart):(nBlank+xrEnd)) = pb.powers(iter);
                        else
                            grid(lines,(nBlank+xStart):(nBlank+xEnd)) = pb.powers(iter);
                        end
                        mask = reshape(grid',[],1);
                        powerFracsPb(~isnan(mask),iter) = mask(~isnan(mask));
                    end
                end
                
                % adjust for line phase and beamClockDelay
                shiftSamples = -obj.nsamples(1,obj.scanners{obj.BEAM_SCANNER_ID}.linePhase) + obj.scanners{obj.BEAM_SCANNER_ID}.beamClockDelay; % beamClockDelay is measured in ticks
                powerFracs = circshift(powerFracs, [-shiftSamples 0]);
                powerFracs(1:-shiftSamples,:) = NaN;
                powerFracs(end-shiftSamples+1:end,:) = NaN;
                
                %same for the power box version
                powerFracsPb = circshift(powerFracsPb, [-shiftSamples 0]);
                powerFracsPb(1:-shiftSamples,:) = NaN;
                powerFracsPb(end-shiftSamples+1:end,:) = NaN;
                
                if any(pzAdjust)
                    % create array of z position corresponding to each sample
                    if dzdt ~= 0
                        sampleTimes = cumsum(repmat(sampleTime,totalSamples,1)) - sampleTime;
                        sampleZs = actz + sampleTimes * dzdt;
                    else
                        sampleZs = repmat(actz,totalSamples,1);
                    end

                    % scale power fracs using Lz
                    adjs = find(pzAdjust == true);
                    for adj = adjs
                        powerFracs(:,adj) = powerFracs(:,adj) .* exp(sampleZs./Lzs(adj));
                        powerFracsPb(:,adj) = powerFracsPb(:,adj) .* exp(sampleZs./Lzs(adj));
                    end
                end
                
                % replace NaNs with zeros
                powerFracs(isnan(powerFracs)) = 0;
                powerFracsPb(isnan(powerFracsPb)) = 0;

                % convert fracs to voltage
                bIDs = obj.scanners{obj.BEAM_SCANNER_ID}.beamIDs;
                for i = 1:numel(bIDs)
                    path_FOV.B(:,i) = min(powerFracs(:,bIDs(i)),obj.scanners{obj.BEAM_SCANNER_ID}.powerLimits(bIDs(i))) / 100;
                    if obj.hasPowerBox
                        pFs = powerFracsPb(:,bIDs(i));
                        path_FOV.Bpb(:,i) = min(pFs,obj.scanners{obj.BEAM_SCANNER_ID}.powerLimits(bIDs(i))) / 100;
                        path_FOV.Bpb(isnan(pFs),i) =  path_FOV.B(isnan(pFs),i);
                    end
                end
            end
        end
        
        function [path_FOV, seconds] = scanPathStimulusFOV(obj,scanfield,roi,actz,dzdt,transform,scanBeams,maxPoints)
            if nargin < 6 || isempty(transform)
                transform = true;
            end
            
            if nargin < 7 || isempty(scanBeams)
                scanBeams = true;
            end
            
            if nargin < 8 || isempty(maxPoints)
                maxPoints = inf;
            end
            
            parkfunctiondetected = false;
            
            assert(isa(scanfield,'scanimage.mroi.scanfield.ScanField'));
            
            % the implementation of scanTime ensures that the galvo task
            % and beams task stay in sync
            [totalduration,durationPerRepetitionInt,durationPerRepetitionFrac] = obj.scanTime(scanfield);
            repetitionsInteger = fix(scanfield.repetitions);
            seconds = totalduration;
            
            % get all scanfield properties to stimulus generation
            scanfieldprops = scanfield.getPropertyStructForSaving();
            propfieldnames = fieldnames(scanfieldprops);
            propfieldvals  = struct2cell(scanfieldprops);
            scanfieldprops = reshape({propfieldnames{:};propfieldvals{:}},1,[]);
            
            path_FOV = struct();
            path_FOV = generateGalvoPathStimulus(path_FOV);
            
            if obj.hasBeams && scanBeams
                path_FOV = generateBeamsPathStimulus(path_FOV);
            end
            
            %%% nested functions
            function path_FOV = generateGalvoPathStimulus(path_FOV)
                %TODO: make sure galvo and beams stay in sync here
                numsamples = obj.nsamples(1,durationPerRepetitionInt);
                tt = linspace(0,obj.nseconds(1,numsamples-1),min(numsamples,maxPoints));
                numsamples = length(tt); % recalculate in case maxPoints < numsamples
                
                [xx,yy] = scanfield.stimfcnhdl(tt,scanfield.stimparams{:},'actualDuration',durationPerRepetitionInt,scanfieldprops{:});
                assert(length(xx) == numsamples && length(yy) == numsamples,...
                    ['Stimulus generation function ''%s'' returned incorrect number of samples:',...
                    'Expected: %d Returned: x:%d, y:%d'],...
                    func2str(scanfield.stimfcnhdl),numsamples,length(xx),length(yy));
                
                % convert to column vector
                xx = xx(:);
                yy = yy(:);
                
                if transform
                    if any(isinf(abs(xx))) || any(isinf(abs(yy)))
                        % replace inf values from the park stimulus
                        % function with the appropriate park values
                        parkFov = obj.mirrorsActiveParkFovPosition();
                        xx(isinf(xx)) = parkFov(1);
                        yy(isinf(yy)) = parkFov(2);
                        parkfunctiondetected = true;
                    else
                        [xx,yy] = scanfield.transform(xx,yy);
                    end
                else
                    repetitionsInteger = 1;
                    durationPerRepetitionFrac = 0;
                end
                
                path_FOV.G(:,1) = repmat(xx,repetitionsInteger,1);
                path_FOV.G(:,2) = repmat(yy,repetitionsInteger,1);
                
                % fractional repetitions
                numsamples = obj.nsamples(1,durationPerRepetitionFrac);
                path_FOV.G(end+1:end+numsamples,:) = [xx(1:numsamples),yy(1:numsamples)];
            end
            
            function path_FOV = generateBeamsPathStimulus(path_FOV)
                if isempty(scanfield.powers)
                    powers = obj.scanners{obj.BEAM_SCANNER_ID}.powers;
                elseif numel(scanfield.powers) == 1;
                    powers = repmat(scanfield.powers,1,numel(obj.scanners{obj.BEAM_SCANNER_ID}.powers));
                else
                    assert(numel(scanfield.powers) == numel(obj.scanners{obj.BEAM_SCANNER_ID}.powers),...
                        'Number of defined beam powers for stimulus %s does not match number of beams',...
                        scanfield.name);
                    powers = scanfield.powers(:)';
                end
                                
                if all(all(isnan(path_FOV.G))) || parkfunctiondetected
                    % detected output from the pause/park stimulus function. set
                    % beam powers to zero
                    numsamples = obj.nsamples(obj.BEAM_SCANNER_ID,totalduration);
                    allRepetitions = zeros(numsamples,numel(powers));
                else
                    powerFracs = powers ./ 100; % scale from percent to powerfraction
                    
                    numsamples = obj.nsamples(obj.BEAM_SCANNER_ID,durationPerRepetitionInt);
                    tt = linspace(0,obj.nseconds(1,numsamples-1),min(numsamples,maxPoints));
                    numsamples = length(tt); % recalculate in case maxPoints < numsamples
                    
                    powers = scanfield.beamsfcnhdl(tt,powerFracs,'actualDuration',durationPerRepetitionInt,scanfieldprops{:});
                    assert(size(powers,1) == numsamples,...
                        ['Beams generation function ''%s'' returned incorrect number of samples:',...
                        'Expected: %d Returned: %d'],...
                        func2str(scanfield.beamsfcnhdl),numsamples,size(powers,1));
                    assert(size(powers,2) == numel(obj.scanners{obj.BEAM_SCANNER_ID}.powers),...
                        ['Beams generation function ''%s'' returned incorrect number of beams:',...
                        'Expected: %d Returned: %d'],...
                        func2str(scanfield.beamsfcnhdl),numel(obj.scanners{obj.BEAM_SCANNER_ID}.powers),size(powers,2));
                    
                    % enforce power Limits
                    for iter = 1:numel(obj.scanners{obj.BEAM_SCANNER_ID}.powers)
                        powers(iter) = min(powers(iter),obj.scanners{obj.BEAM_SCANNER_ID}.powerLimits(iter)./100);
                    end
                    
                    % apply repetitions
                    allRepetitions = repmat(powers,repetitionsInteger,1);
                    
                    % fractional repetitions
                    numsamples = obj.nsamples(obj.BEAM_SCANNER_ID,durationPerRepetitionFrac);
                    allRepetitions(end+1:end+numsamples,:) = powers(1:numsamples,:);
                end
                path_FOV.B = allRepetitions(:,obj.scanners{obj.BEAM_SCANNER_ID}.beamIDs);
                path_FOV.B(end,:) = 0;
            end
        end
        
        function ao_volts=fov2volts(obj,path_FOV,iscanner)
            %% ao_volts=fov2volts(obj,path_FOV)
            %  path_FOV: Nx2 field-of-view coordinates.  Rows are for x and y
            %          respectively.
            if isempty(iscanner)
                assert(size(path_FOV,2)==2);
                iscanner = [1,2,3];
            end
            
            A = cellfun(@(m) m.fullAngleDegrees,obj.scanners(1:3));     % amplitude
            O = A./2;                                              % offset (degrees)
            V = cellfun(@(m) m.voltsPerDegree,obj.scanners(1:3));       % degrees to volts conversion
            
            % convert to volts
            ao_volts=zeros(size(path_FOV));
            
            iter = 1;
            for scanner = iscanner
                ao_volts(:,iter)=V(scanner)*(path_FOV(:,iter)*A(scanner)-O(scanner));
                iter = iter + 1;
            end
        end
        
        function fov=degrees2fov(obj,imirror,degrees)
            m=obj.scanners{imirror};
            A = m.fullAngleDegrees;     % amplitude
            O = A./2;                   % offset (degrees)
            fov=(degrees+O)./A;
        end
        
        function path_FOV = volts2fov(obj,ao_volts,iscanner)
            if isempty(iscanner)
                assert(size(ao_volts,2)==2);
                iscanner = [1,2];
            end
            
            A = cellfun(@(m) m.fullAngleDegrees,obj.scanners(1:2));     % amplitude
            O = A./2;                                                   % offset (degrees)
            V = cellfun(@(m) m.voltsPerDegree,obj.scanners(1:2));       % degrees to volts conversion
            
            % convert to volts
            path_FOV=zeros(size(ao_volts));
            
            iter = 1;
            for scanner = iscanner
                path_FOV(:,iter)=(ao_volts(:,iter)/V(scanner)+O(scanner))/A(scanner);
                iter = iter + 1;
            end
        end
    end
end

%% NOTES
%{

(Some note numbers may be missing.  Those notes were deleted.)

3. The scannerset somehow determines the constraints on rois.
   Not sure how to manage this.

4. FIXME
   Need to check/correct the scan pattern in practice.
   Current scan pattern calculation is just a guess.

%}

%% TODO
%{
    [ ] - incorporate internalLineSettlingTime
    [ ] - bidi v non-bidi (cycloid waveform)
%}



%--------------------------------------------------------------------------%
% GalvoGalvo.m                                                             %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
