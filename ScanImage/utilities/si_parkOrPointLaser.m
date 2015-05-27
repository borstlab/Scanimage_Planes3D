function si_parkOrPointLaser(varargin)       %VI05208B, VI05208C
%% function si_parkOrPointLaser(varargin)
% Function to park or point the laser beam(s), at either standard or user-specified angular position
%% USAGE
%   si_parkOrPointLaser(): parks laser at standard.ini defined park location (vars state.acq.parkAngleX & state.acq.parkAngleY); closes shutter and turns off beam with Pockels Cell
%   si_parkOrPointLaser(xy): parks laser at user defined location xy, a 2 element vector of optical degree values
%   si_parkOrPointLaser(...,'soft'): 'soft' flag causes function to blank beam with Pockels, but leave shutter open
%   si_parkOrPointLaser(...,'transmit'): 'transmit' flag signals to transmit beam with Pockels and leave shutter open
%
%% NOTES
%   When parking at the standard.ini location, the Pockels Cell is set to transmit the minimum possible vlaue.
%
%   Note that X&Y correspond to channels as per the X/YMirrorChannelID settings in the INI file
%   When xy is passed, note that 1) scanOffsetAngleX/Y is NOT added, and 2) value is converted to voltage via voltsPerOpticalDegree value in INI file
%
%   The 'soft' option is intended for 'quick' parking, avoiding frequent open/close of the shutter
%   With the 'transmit' option, the Pockels Cell(s) is (are) set to transmit the value currently specified by the Power Control slider (per-beam)
%
%% MODIFICATIONS
%   VI052008A Vijay Iyer 5/20/08 -- Moved actual park laser functionality from makeAndPutDataPark() to this function
%   VI052008B Vijay Iyer 5/20/08 -- Renamed to scim_parkLaser(), making this available on the commaand line
%   VI052008C Vijay Iyer 5/20/08 -- Handle Pockels Cell voltage differently, for the two separate cases
%   VI052708A Vijay Iyer 5/27/08 -- Add shutter opening/closing, for the two cases
%   VI061908A Vijay Iyer 6/19/08 -- Added 'soft' park mode
%   VI061908B Vijay Iyer 6/19/08 -- Blank /all/ the laser beams
%   VI011609A Vijay Iyer 1/16/09 -- Changed state.init.pockelsOn to state.init.eom.pockelsOn
%   VI082909A Vijay Iyer 8/29/09 -- Changes to use new DAQmx interface
%   VI090209A Vijay Iyer 9/2/092 -- Handle X/Y channel transposition correctly; leave range error reporting to DAQmx
%   VI122309A Vijay Iyer 12/23/09 -- Handle newly separated park Tasks for each Pockels beam
%   VI062410A Vijay Iyer 6/24/10 -- Handle case where calibration for particular Pockels beam has not yet been (successfully) completed
%   VI062510A Vijay Iyer 6/25/10 -- Unreserve timed AO Tasks to park mirrors and beam (if needed)
%   VI092010A Vijay Iyer 9/20/10 -- This function works with X&Y voltage commands directly, so no need to process state.acq.fastScanningX; 
%   VI100510A Vijay Iyer 10/05/10 -- Correct implementation of VI062410A to work for both xy empty/not-empty cases; not sure why 'min', rather than 'lut' is used as test for calibration completion -- Vijay Iyer 10/5/10
%   VI101510A Vijay Iyer 10/15/10 -- Added 'transmit' flag; changed function logic/behavior accordingly; fixes issues which appeared following VI092010A fix -- Vijay Iyer 10/15/10
%   VI110210A Vijay Iyer 11/2/10 -- Apply scaling by state.init.voltsPerOpticalDegree to X&Y values
%   VI110310A Vijay Iyer 11/3/10 -- Rename parkAmplitudeX/Y to parkAngleX/Y
%   VI033011A Vijay Iyer 3/30/11 -- Park at specified absolute park angle on side closest to start of ramps for next scan
%   VI063011A Vijay Iyer 6/30/11 -- Handle parking in case where point-scanning is enabled and scanAngleMultiplierXXX=0
%
%% CREDITS
%   Created 7/15/11, by Vijay Iyer 
%   Based on prior scim_parkLaser()
%% ******************************************************************************************

if isempty(whos('global','state')) || isempty(whos('global','gh'))
    disp('ScanImage is not running or not properly running. Cannot park laser.');
    return;
end
global state

% start(state.init.aoPark);
% 
% while strcmp(state.init.aoPark.Running, 'On')
% end

% makeAndPutDataPark;

%%%Handle flag options (VI101510A/VI061908A)%%%
soft = false;
transmit = false; %VI101510A
flagNames = {'soft' 'transmit'};
for i=1:length(varargin)
    if ischar(varargin{i}) && isvector(varargin{i})
        [flagFound,flagIdx] = ismember(lower(varargin{i}),flagNames);      
        
        if flagFound
            varargin(i) = [];
            
            switch flagNames{flagIdx}
                case 'soft'
                    soft = true;
                case 'transmit'
                    transmit = true;
            end
        else
            error('Unrecognized flag argument ''%s'' was detected', varargin{1});
        end
    end
end

%Ensure only 1 flag is set
if soft && transmit
    error('Only one option flag, ''soft'' or ''transmit'', can be set at at time');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Ensure number of arguments
assert(length(varargin)<=1, 'Invalid number of arguments detected');

%%%%%%%%(052008A) Added from makeAndPutDataPark() -- with minor mods %%%%%%%%%%%%%%%%%%%%%%%%%%
if isempty(varargin) || isempty(varargin{1}) %VI101510A
    xy = [state.init.parkAngleX state.init.parkAngleY]; %VI110310A %VI092010A
    
    %%%VI033011A%%%
    if state.init.parkAngleAllowInvert %Specifies that angle value is an absolute value, which can be inverted if advantageous
        if state.acq.fastScanningX
            signMultiplier = [-sign(state.acq.scanAngleMultiplierFast * state.init.scanAngularRangeReferenceFast), -sign(state.acq.scanAngleMultiplierSlow * state.init.scanAngularRangeReferenceSlow)];
        else
            signMultiplier = [-sign(state.acq.scanAngleMultiplierSlow * state.init.scanAngularRangeReferenceSlow), -sign(state.acq.scanAngleMultiplierFast * state.init.scanAngularRangeReferenceFast)];
        end
        
        signMultiplier(signMultiplier==0) = 1; %VI063011A: Handle scanAngleMultiplierXXX=0 case
        xy = abs(xy) .* signMultiplier;        
    end
    %%%%%%%%%%%%%%%%%
    
    %%%VI092010A: Removed %%%%%%%%
    %     if state.acq.fastScanningX %VI090209A
    %         state.internal.finalParkedLaserDataOutput= [state.init.parkAmplitudeX state.init.parkAmplitudeY];
    %     else
    %         state.internal.finalParkedLaserDataOutput= [state.init.parkAmplitudeY state.init.parkAmplitudeX]; %VI090209A
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else
    xy = varargin{1};
	if length(xy)~=2 || ~isnumeric(xy) 
        error('The ''xy'' argument must be a 2-element numeric vector containing X & Y park voltages');
        
        %%%VI090209A: Removed -- DAQmx will report range error reasonably well %%%%%%%
        %     elseif min(xy(1)) < min(get(state.init.XMirrorChannelPark,'OutputRange')) || max(xy(1)) > max(get(state.init.XMirrorChannelPark,'OutputRange')) ... %VI052708A
        %             min(xy(2)) < min(get(state.init.YMirrorChannelPark,'OutputRange')) || max(xy(2)) > max(get(state.init.YMirrorChannelPark,'OutputRange'))
        %         error('Specified park voltages are outside of the allowed range.');
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %%%VI092010A: Removed%%%%%%
        %     else
        %         if state.acq.fastScanningX %VI090209A
        %             state.internal.finalParkedLaserDataOutput= [xy(1) xy(2)];
        %         else
        %             state.internal.finalParkedLaserDataOutput= [xy(2) xy(1)]; %VI090209A
        %         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end      

end

%putsample(state.init.aoPark, state.internal.finalParkedLaserDataOutput); %VI082809A: Removed % Queues Data to engine for Board 2 (Mirrors)
state.init.hAO.control('DAQmx_Val_Task_Unreserve'); %VI062510A
state.init.hAOPark.writeAnalogData(xy * state.init.voltsPerOpticalDegree,.2,true); %VI110210A %VI092010A %VI082809A 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%xy = [];
%TPMODPockels

if state.init.eom.pockelsOn == 1        
    %voltageLevels = zeros(1,state.init.eom.numberOfBeams); %VI101510A: Removed %VI082909A
   
    if ~isempty(state.init.eom.lut) %Don't set anything
        
        state.init.eom.hAO.control('DAQmx_Val_Task_Unreserve'); %VI062510A
                
        for i=1:state.init.eom.numberOfBeams %VI061908B
            powerLevel = []; %VI062410A
            
            %Size of state.init.eom.lut used as test of whether beam has
            %been calibrated
            if size(state.init.eom.lut,1) >= i  %VI100510A %VI062410A
                if transmit %VI101510A
                    powerLevel = state.init.eom.maxPower(i);
                else
                    powerLevel = state.init.eom.min(i);                
                end
            end

            if ~isempty(powerLevel)  %VI062410A
                %setPockelsVoltage(i,state.init.eom.lut(i,powerLevel)); %VI082909A: Removed
                voltageLevel = state.init.eom.lut(i,powerLevel);                
                state.init.eom.(['hAOPark' num2str(i)]).writeAnalogData(voltageLevel,1,true);  %VI122309A %VI082909A
            end            
        end
    end
end

%%%V101510A%%%
if transmit || soft
    openShutter();
else
    closeShutter();
end
%%%%%%%%%%%%%%

%%%VI101510A: Removed %%%%%%%
% if ~soft %VI061908A
%     %Open/close shutter %VI052708A
%     if useParkAmplitude 
%         closeShutter;
%     else
%         openShutter;
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
