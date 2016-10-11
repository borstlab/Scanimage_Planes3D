classdef ScannerSet < handle
    properties(Abstract)
        scanners;
    end

    properties
        name = 'scanner';
        refToScannerTransform=eye(3);
        scannerToRefTransform=eye(3);
    end

    properties (Hidden, SetAccess=private)
        internalSet = false;
    end
    
    properties(Abstract, Constant)
        CONSTRAINTS;      % Cell array of scanimage.mroi.constraints function handles
        BEAM_SCANNER_ID;  % ID of the beam scanner in the obj.scanner cell array
    end
    
    methods(Abstract)
        % Returns an array of the form: [xmin xmax ymin ymax],
        % where xmin, xmax, ymin, and ymax describe the maximum angular
        % displacement of each axis in degrees. This is done in scannerset
        % instead of roiManager, because roiManager would otherwise have
        % a switch statement to define the different types of scannersets.
        fovArray = fov(obj);
         
        % Returns 2d array.
        % columns are ao channels
        % rows are samples
        ao_volts = scanPathAO(obj,scanfield,z,dz);
        
        % converts a path for all scanners (resonant, galvos, beams)
        % from field of view coordinates to output volts
        ao_volts = pathFovToAo(obj,path_FOV)
        
        % generates scan path for all scanners (resonant, galvos, beams)
        % in field of view coordinates
        [path_FOV, seconds] = scanPathFOV(obj,scanfield,actz,dzdt)

        % Returns the number of output samples to fill an interval 
        % of 'seconds'
        count = nsamples(obj,scannerid,seconds);
        
        % Returns the number of seconds to scan n samples
        seconds = nseconds(obj,scannerid,nsamples);
        
        % Returns the time required to scan the scanfield in seconds
        % scanfield must be a scanimage.mroi.scanfield.ScanField object.
        seconds  = scanTime(obj,scanfield);
        
        % Returns the total time required to scan a line (including scanner
        % turnaround time) and the time during which the acquisition takes
        % place
        [scanPeriod,acquisitionPeriod] = linePeriod(obj,scanfield);
        
        % Returns the active acquisition time for each line of the
        % scanfield (for precalculation of beams output)
        [startTime,endTime] = acqActiveTimes(obj,scanfield)

        % Returns array of mirror positions in FOV coordinates for parking the mirror
        % during an active scan
        position_FOV = mirrorsActiveParkFovPosition(obj);
        
        % Returns the estimated time required to position the scanners when
        % moving from scanfield to scanfield.
        %
        % The park position is represented by NaN
        % Either scanfield_from or scanfield_to may be NaN
        seconds  = transitTime(obj,scanfield_from,scanfield_to);
        
        % Returns the path in field of view coordinates for transitioning from one scanfield
        % to another. the transti path is represented as NaN and can be filled
        % in subsequently with the function interpolateTransits
        %
        % The park position is represented by NaN
        % Either scanfield_from or scanfield_to may be NaN
        % transits samples will be filled in with NaN
        path_FOV = transitNaN(obj,scanfieldFrom,scanfieldTo);
        
        % interpolates over NaN ranges of a path in FOV coordinates 
        path_FOV = interpolateTransits(obj,path_FOV);
        
        % generates path in field of view coordinates for a flyback frame
        path_FOV = zFlybackFrame(obj, frameTime);
        
        % pads all channels to the specified time duration with NaN's
        path_FOV = padFrameAO(obj, path_FOV, frameTime, flybackTime);
        
        % returns a recomputed scanfield that satisfies any constraints
        % that need to be fullfilled for the scannerset.
        scanfield = satisfyConstraints(obj,scanfield);
        
        % returns the angular pixels size of a scanfield
        [pixelSizeX,pixelSizeY] = pixelSizeAngle(obj,scanfield);
        
        % returns number of samples per trigger for each scanner
        samplesPerTrigger = samplesPerTriggerForAO(obj,outputData)
        
        % returns triggerType (lineClk/frameClk) and configuration of
        % refernce clock
        cfg = beamsTriggerCfg(obj);
    end
    
    
    methods
        % Returns t/f indication if the scannerset includes beams
        function tf = hasBeams(obj)
            tf = numel(obj.scanners) >= obj.BEAM_SCANNER_ID...
                && (numel(obj.scanners{obj.BEAM_SCANNER_ID}.beamIDs) > 0);
        end
        
        function tf = hasPowerBox(obj)
            tf = ~isempty(obj.scanners{obj.BEAM_SCANNER_ID}.powerBoxes);
        end
        
        function scanner = getBeamScanner(obj)
            tf = obj.hasBeams();
            if tf
                scanner = obj.scanners{obj.BEAM_SCANNER_ID};
            else
                scanner = [];
            end
        end
        
        function varargout = getRoiBeamProps(obj, hRoi, varargin)
            assert((nargin-2) == nargout, 'Incorrect use of getRoiBeamProps. Number of inputs must equal outputs.');
            
            assert(obj.hasBeams)
                        
            for i = 1:nargout
                varargout{i} = hRoi.(varargin{i});
                if isempty(varargout{i})
                    varargout{i} = obj.scanners{obj.BEAM_SCANNER_ID}.(varargin{i});
                else
                    varargout{i}(isnan(varargout{i})) = obj.scanners{obj.BEAM_SCANNER_ID}.(varargin{i})(isnan(varargout{i}));
                end
            end
        end


        % returns a recomputed scanfield that satisfies any constraints
        % that need to be fullfilled for the scannerset.
        function roigroup = satisfyConstraintsRoiGroup(obj,roigroup,scanfield)
            if nargin < 3 || isempty(scanfield)
                scanfield = [];
            end
            cellfun(@(constraint) constraint(roigroup,obj,scanfield),obj.CONSTRAINTS);
        end
    end % public methods

end


%--------------------------------------------------------------------------%
% ScannerSet.m                                                             %
% Copyright © 2016 Vidrio Technologies, LLC                                %
%                                                                          %
% ScanImage 2015 is premium software to be used under the purchased terms  %
% Code may be modified, but not redistributed without the permission       %
% of Vidrio Technologies, LLC                                              %
%--------------------------------------------------------------------------%
