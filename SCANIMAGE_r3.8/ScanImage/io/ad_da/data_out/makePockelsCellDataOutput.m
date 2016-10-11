function pockelsDataOutput = makePockelsCellDataOutput(beamArrayOrList, flybackOnly, offBeamValue)
%% function pockelsDataOutput = makePockelsCellDataOutput(beamArrayOrList, flybackOnly, offBeamValue)
%  Function that constructs the Pockels Cell Data Output
%
%% SYNTAX
%    function pockelsDataOutput = makePockelsCellDataOutput(beamArrayOrList)
%    function pockelsDataOutput = makePockelsCellDataOutput(beamArrayOrList, flybackOnly)
%    function pockelsDataOutput = makePockelsCellDataOutput(beamArrayOrList, flybackOnly, offBeamValue)
%       beamArrayOrList: Integer array or list of Channel names, indicating which beams are active. Inactive beams will have 'minimum' power value
%       flybackOnly: Logical indicating, when true, to compute flyback blanking (if enabled) rather than 'special feature' waveform. Default value is false.
%       offBeamValue: One of {'min', 'max'} specifying whether 'off' beams should be set to their minimum or currently set 'maximum' level. If empty/omitted, 'min' is assumed.
%
%       pockelsDataOutput: A matrix of size MxN containing Pockels output data, where N is the number of beams specified in INI file, and M is the number of samples.
%
%% NOTES
%   This version was rewritten from scratch. To see earlier versions of this function, see makePockelsCellDataOutput.mold -- Vijay Iyer 1/27/09
%
%   The function now outputs a matrix whose number of columns is given by state.init.eom.numberOfBeams -- i.e. data is output for all available beams, rather than specified per beam (per Channel), as with @daqmanager
%
%   This function produces the flyback blanking signal -- a square wave which is 'on' during the acquisition window, and 'off' otherwise.
%   If any 'special feature' is used -- i.e. the powerBox-- the function implementPockelsCellTiming() is used instead.
%
%   TODO: Consider whether to use floor/ceil, rather than round, for flyback blanking. Avoiding floor/ceil makes sense for PowerBox feature, but not necessarily here -- Vijay Iyer 10/6/10
%
%% CHANGES
%   VI011609A Vijay Iyer 1/16/09 -- Changed state.init.pockelsOn to state.init.eom.pockelsOn
%   VI090209A Vijay Iyer 9/02/09 -- Construct output array for all beams
%   VI102209A Vijay Iyer 10/22/09 -- Handle slow dimension flyback options
%   VI010610A Vijay Iyer 1/6/10 -- Allow for power scaling associated with Power vs Z feature. Note this only applies for standard Pockels buffers, not for any 'special features' (which are not at this time supported with stacks).
%   VI010810A Vijay Iyer 1/08/10 -- Abide the per-beam max power limit in determining the 'scaled' power
%   VI010910A Vijay Iyer 1/09/10 -- BUGFIX: Expand number of columns only, not both columns and rows
%   VI011210A Vijay Iyer 1/12/10 -- When using P vs z, determine Pockels output level to use by interpolating the beam's LUT; however, display rounded version of scaled max power
%
%% CREDITS
%  Created 1/27/09, by Vijay Iyer
%  Based heavily on earlier version by Tom Pologruto and Tim O'Connor
%% ************************************************************************
global state

%%%VI090209A: Assemble each beam's data into matrix
%Parse input
if isnumeric(beamArrayOrList) %an array of beam #s
    beamList = cell(length(beamArrayOrList),1);
    for i=1:length(beamArrayOrList)
        beamList{i} = ['PockelsCell-' num2str(beamArrayOrList(i))];
    end
elseif ischar(beamArrayOrList) %a comma-delimited string of beam names
    beamList = delimitedList(state.init.eom.focusLaserList, ','); %convert to cell array
else
    error('First argument must be an array of beam numbers or a comma-delimited list of beam names');
end

if nargin < 2
    flybackOnly = 0;
end

if nargin < 3
    offBeamValue = 'min';
end

%Determine which beams are on
beamOnMask = ismember(state.init.eom.pockelsCellNames, beamList);
onBeams = find(beamOnMask);
offBeams = find(~beamOnMask);

%Tim O'Connor TO092004a - Look out for the total disabling of lasers for a given function. - 9/20/04
if length(offBeams) == state.init.eom.numberOfBeams
    errordlg('A beam must be enabled in the LaserFunctionPanel for EOM feature to work properly.');
end

%Allocate based on first beam
pockelsDataOutput = repmat(makeBeamOutput(onBeams(1)), 1, state.init.eom.numberOfBeams);

%Fill in other 'on' beams
for i=2:length(onBeams)
    pockelsDataOutput(:,onBeams(i)) = makeBeamOutput(onBeams(i));
end

%Fill in 'off' beams with minimum value for each beam
for i=1:length(offBeams)
    beamIdx = offBeams(i);
    switch offBeamValue
        case 'min'
            pockelsDataOutput(:,beamIdx) = state.init.eom.lut(beamIdx,state.init.eom.min(beamIdx));
        case 'max'
            pockelsDataOutput(:,beamIdx) = state.init.eom.lut(beam, state.init.eom.maxPower(beamIdx));
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%VI090209A: The original per-beam functionality
    function beamOutput = makeBeamOutput(beam)
        
        sampleShift = 0; %Variable for case that acquisition window 'leaks' beyond line period
        
        % warning(state.init.eom.pockelsCellNames{beam});
        if ~state.init.eom.pockelsOn %VI011609A
            error('Pockels cell disabled.');
        end
        
        if isempty(state.init.eom.lut)
            return;
        end
        
        %Identify if a special feature is being used
        specialFeature = ~flybackOnly && (state.init.eom.usePowerArray || any(state.init.eom.showBoxArray) || any(state.init.eom.uncagingMapper.enabled)); %VI030708A %VI041808A %VI041808B
        
        state.init.eom.min = round(state.init.eom.min);
        
        if state.init.eom.min(beam) > 100
            fprintf(2, 'WARNING: Minimum power for beam %s is over 100%%. Forcing it to 99%%...\n', num2str(beam));
            state.init.eom.min(beam) = 99;
        elseif state.init.eom.min(beam) < 1
            fprintf(2, 'WARNING: Minimum power for beam %s is below 1%%. Forcing it to 1%%...\n', num2str(beam));
            state.init.eom.min(beam) = 1;
        end
        
        if ~specialFeature
            %Pre load array with minimum value
            beamOutput = state.init.eom.lut(beam, state.init.eom.min(beam)) + zeros(state.internal.lengthOfXData, 1);
            
            if state.init.eom.powerVsZActive && state.init.eom.powerVsZEnable && ~isinf(state.init.eom.powerLzStoredArray(beam))
                %maxPowerValue = max(min(round(state.init.eom.maxPower(beam) * state.init.eom.stackPowerScaling(beam)),state.init.eom.maxLimit(beam)),1); %VI010810B %VI010810A %VI010610A
                maxPowerValue = computeScaledMaxPower(beam);
                %maxValue = state.init.eom.lut(beam, maxPowerValue); %VI010610A
                maxValue = interp1(1:100,state.init.eom.lut(beam, :),maxPowerValue); %VI011210A %VI010610A
                updateMaxPowerDisplay(beam,round(maxPowerValue)); %VI101209A
            else %VI011210A: Avoid interpolation if not needed
                maxValue = state.init.eom.lut(beam, state.init.eom.maxPower(beam)); 
            end
            
            if state.acq.pockelsClosedOnFlyback
                startGoodPockelsDataBase = (state.acq.scanDelay + state.acq.acqDelay - state.acq.pockelsFillFracAdjust/2) * state.acq.outputRate;
                startGoodPockelsData = round(startGoodPockelsDataBase) + 1;
                endGoodPockelsData = round(startGoodPockelsDataBase + state.internal.lengthOfXData*state.acq.fillFraction + (state.acq.pockelsFillFracAdjust/2) * state.acq.outputRate);
                
                %%%Handle case where acq window is shifted if startGoodPockelsData < 1
                if startGoodPockelsData < 1
                    sampleShift = 1 - startGoodPockelsData;
                    startGoodPockelsData = 1;
                    beamOutput(end-sampleShift+1:end) = maxValue; %VI010610A;
                end
                
                %%%Handle case where acquisition window leaks beyond edge of line period
                if endGoodPockelsData > state.internal.lengthOfXData
                    sampleShift = max(sampleShift,endGoodPockelsData - state.internal.lengthOfXData);
                    endGoodPockelsData = state.internal.lengthOfXData;
                    beamOutput(1:sampleShift) = maxValue; %VI010610A
                end
                
            else
                startGoodPockelsData = 1;
                endGoodPockelsData = state.internal.lengthOfXData;
            end
            
            %Fill in the 'on' portion of the command waveform
            beamOutput(startGoodPockelsData:endGoodPockelsData) = maxValue; %VI010610A;
            
            %Repeat Pockels Data to create for one frame; this will be the repeated unit
            beamOutput = repmat(beamOutput, [state.acq.linesPerFrame 1]);
            
            
            %%%VI102209A: Handle slow dimension flyback case %%%%%%%
            if state.acq.slowDimFlybackFinalLine && state.acq.pockelsClosedOnFlyback
                beamOutput(end-state.internal.lengthOfXData+1:end) = state.init.eom.lut(beam, state.init.eom.min(beam));
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        else %Handle case where a special feature is used
            beamOutput = implementPockelsCellTiming(beam);
        end
    end

end