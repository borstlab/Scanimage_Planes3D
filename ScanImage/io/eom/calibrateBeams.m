%% function calibrateBeams(startup)
%  A 'macro' that calibrates the Pockels Cells for all the beams which require it
%
%% SYNTAX
%   calibrateBeams()
%   calibrateBeams(startup)
%       startup: Optional boolean flag indicating TRUE when called during startup (i.e. open USR file for first time), FALSE otherwise. FALSE is assumed.
%
%% NOTES
%   This function is called at startup, upon loading a USR file, and upon closing the Laser Function Panel
%
%% CHANGES
%   VI011609A: Changed state.init.pockelsOn to state.init.eom.pockelsOn -- Vijay Iyer 1/16/09
%   VI032609A: Handle case where beam is not active in LaserFucntionPanel -- Vijay Iyer 3/26/09
%   VI032609B: Handle case where beam has been previously calibrated -- Vijay Iyer 3/26/09
%   VI032311A: Avoid hard error in case where poor calibration causes avgDev=nan  -- Vijay Ieyr 3/23/11
%
%% CREDITS
%   Created 10/31/08 by Vijay Iyer
%% **************************************************************

function calibrateBeams(startup)
global state;

if ~nargin
    startup = false;
end

if state.init.eom.pockelsOn %VI011609A
    disp('*** Calibrating Pockels Cells ************');
    for i = 1:state.init.eom.numberOfBeams        
        % Only do calibration if it's not been done already
        if ~si_isBeamActive(i) %VI032609A
            dummyEOMCalibrate(i); %Will handle this case correctly
        elseif  isempty(state.init.eom.lut) || size(state.init.eom.lut,1) < i || all(state.init.eom.lut(i,:) == 0) %VI032609B
            if ~startup || state.internal.eom.calibrateOnStartup
                [eom_min, eom_max, avgDev] = calibrateEom(i); %will defer to non/naive dummy calibrations, as needed
                if isnan(avgDev) || avgDev ~= 0  %VI032311A %a non-elegant way to test that an actual calibration was done
                    fprintf(1,['Beam #' num2str(i) ' calibrated.\n']);
                end
            else
                dummyEOMCalibrate(i,true); %Force non-calibration
            end
        else
            fprintf(1,['Beam #' num2str(i) ' has been previously calibrated.\n']); %VI032609B
        end
    end
    disp('******************************************');    
end

