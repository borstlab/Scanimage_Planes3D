function  out = dummyEOMCalibrate(beamNum,forceNonCalibrate)
%% function  out = dummyEOMCalibrate(beamNum)
%DUMMYEOMCALIBRATE Handles special cases where Pockels Cell calibration is either a) not needed, or b) not possible because of absence of a photodiode
%
%% SYNTAX
%   out = dummyEOMCalibrate(beamNum)
%       beamNum: Integer value specifying number of beam to calibrate
%       forceNonCalibrate: Optional logical flag, TRUE if non-calibration should be forced. FALSE is assumed. 
%       out: Logical value, 1 if a dummy calibration was done, 0 if not
%% NOTES
%   DEPRECATED - Logic now returned to calibrateEom -- Vijay Iyer 3/26/09
%
%   Calling functions should use 'out' value to determine whether or not a true calibration should proceed
%
%   Could consider implementing a default sin^2 function for the naive calibration, which would be more likely to match. (Pitfall is that
%       the half-wave voltage is not knowable a priori)
%
%% MODIFICATIONS
%   VI041808A Vijay Iyer 4/18/08 -- Get beam-specific pockels voltage range
%   VI070808A Vijay Iyer 7/08/08 -- Fill in other beam-specific variables determined during calibration process
%   VI103108A Vijay Iyer 10/31/08 -- Handle special case where calibration is not required (incl. case where beam is not used). Also, add argument to allow such non-calibration to be forced.
%   VI032609A Vijay Iyer 3/26/09 -- Determine if beam requires calibration via si_isBeamActive; if not, any previous calibration is actually deleted
%   VI032609B Vijay Iyer 3/26/09 -- Make beam calibration messages more uniform in appearance (all have 'Beam #' at beginning)
%   VI032609C Vijay Iyer 3/26/09 -- Make message for case of inactive beam more specific.
%   VI010810A Vijay Iyer 1/8/10 -- Warning message should reference Beams... dialog, not Laser Function Panel anymore
%
%% **************************************************
global state

out = 0;

if nargin < 2
    forceNonCalibrate = false;
end

if forceNonCalibrate || ~si_isBeamActive(beamNum) %VI032609A
    out = 1;

    if ~forceNonCalibrate %VI032609C
        fprintf(1,['Beam #' num2str(beamNum) ' not active at this time (see Beams... dialog). Calibration was thus skipped.\n']); %VI010809A %VI032609B, VI032609C
    end
    
    state.init.eom.lut(beamNum,:) = zeros(1,100);
    state.init.eom.min(beamNum) = 1; %1 is as low as we can go
    state.init.eom.maxPhotodiodeVoltage(beamNum) = 0;
    
elseif isempty(state.init.eom.(['photodiodeInputBoardID' num2str(beamNum)]))
    out = 1;
    state.init.eom.lut(beamNum,:) = linspace(0,getfield(state.init.eom,['pockelsVoltageRange' num2str(beamNum)]),100);  %VI041808A
    %%%%% (VI070808A)
    state.init.eom.min(beamNum) = 1;
    calibFactor = state.init.eom.(['powerConversion' num2str(beamNum)]);
    state.init.eom.maxPhotodiodeVoltage(beamNum) = 100/calibFactor; %Max photodiode voltage forced to correspond to 100mW of light power, so mW is effectively a percentage as well
    %%%%%%%%%%%%%%%%
    fprintf(1,['Beam #' num2str(beamNum) ' has no photodiode and is thus uncalibrated. A naive linear scale is employed instead.' sprintf('\n')]); %VI032609B
end

