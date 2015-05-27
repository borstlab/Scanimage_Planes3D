function finalMirrorDataOutput = makeMirrorDataOutput()
%% function finalMirrorDataOutput = makeMirrorDataOutput()
% Function that assembles the data matrix sent to the DAQ Analog output Engine for controlling the laser scanning mirrors
%% SYNTAX
%   finalMirrorDataOutput = makeMirrorDataOutput()
%       finalMirrorDataOutput: A Nx2 matrix containing X and Y mirror signals (in that order), in volts, following all linear transformations (i.e. zoom, rotation, shift)
%
%% NOTES
% This version was rewritten from scratch. To see earlier versions of this function, see makeMirrorDataOutput.mold -- Vijay Iyer 1/23/09
%
% The primary purpose of this function is the computation of state.acq.mirrorDataOutputOrg, which is the 'original'
%   mirror data output prior to any linear transformation (i.e. shift, rotation, zoom) AND prior to Fast/Slow to X/Y mapping. 
%
% The function then calls through to linTransformMirrorData() which 1) applies the linear transformation and
% 2) maps the Fast & Slow axes to the X & Y scanner channels.
%
% For reference:
%   state.acq.mirrorDataOutputOrg is organized as [Fast Slow]
%   state.acq.mirrorDataOutput is organized as [X Y]

%% CHANGES
%   VI090109A: Handle changes to use new DAQmx interface -- Vijay Iyer 9/1/09
%   VI102609A: Use state.internal.scanAmplitudeX/Y in lieu of state.acq.scanAmplitudeX/Y, as the internal value is now used to represent the actual command voltage --  Vijay Iyer 10/26/09
%   VI091910A: The scan amplitudes are now specified directly as fast/slow -- Vijay Iyer 9/19/10
%   VI092010A: Moved the makeSawtoothX/Y and makeMirrorDataX functions (with new names, e.g. now referred to as Line/Frame, Fast/Slow) into this file, as helper functions -- Vijay Iyer 9/20/10
%   VI092010B: Defer to linTransformMirrorData() the mapping of fast/slow axes to X/Y scanner -- Vijay Iyer 9/20/10
%   VI092610A: ScanAmplitudeX/Y is now ScanAmplitudeFast/Slow -- Vijay Iyer 9/26/10
%   VI110210A: BUGFIX - linescan operation was broken by VI091910A -- Vijay Iyer 11/2/10
%   VI032911A: BUGFIX - Error in flyback slope calculation for slow dimension. Was not manifesting on most scanners b/c of slew-rate/error limiting, but does manifest on systems without such limiting -- Vijay Iyer 3/29/11
%
%% CREDITS
% Created 5/5/09 by Vijay Iyer
% Derived from earlier version by Tom Pologruto
%% ***********************************
global state 

updateZoom; %This ensures FF/line period are correctly specified for the current zoom
%sampleRate = get(state.init.ao2, 'SampleRate'); %VI090109A
sampleRate = state.init.hAO.sampClkRate;

samplesPerLine = round(sampleRate * state.acq.msPerLine * 1e-3);  %Will be integer-valued naturally, but this ensures no numerical error 

%%%VI091910A: Removed%%%%%
% if state.acq.fastScanningX
%     scanAmplitudeFast = state.internal.scanAmplitudeX; %VI102609A
%     scanAmplitudeSlow = state.internal.scanAmplitudeY; %VI102609A
% else
%     scanAmplitudeFast = state.internal.scanAmplitudeY; %VI102609A
%     scanAmplitudeSlow = state.internal.scanAmplitudeX; %VI102609A
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%

% DEQ20110427
% if state.acq.linescan
%     scanAmplitudeSlow = 0;
% else
%     scanAmplitudeSlow = state.internal.scanAmplitudeSlow; %VI110210A
% end

fast = makeLineCommandFast(linspace(0, state.acq.msPerLine * 1e-3, samplesPerLine), 0, state.internal.scanAmplitudeFast); 
fast = makeFrameCommandFast(fast); %VI091708A

slow = makeFrameCommandSlow(linspace(0,(state.acq.linesPerFrame * state.acq.msPerLine * 1e-3), ... 
    (samplesPerLine * state.acq.linesPerFrame)), 0, state.internal.scanAmplitudeSlow); %VI110210A

%%%VI092010B: Removed%%%%%
% if state.acq.fastScanningX
%     finalMirrorDataOutput = [fast slow]; 
% else
%     finalMirrorDataOutput = [slow fast];
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%
finalMirrorDataOutput = [fast slow]; %VI092010B

%Cache original waveform (at base zoom), and the scan parameters used to make this waveform
state.acq.mirrorDataOutputOrg = finalMirrorDataOutput; 
state.internal.mirrorDataOutputMsPerLine = state.acq.msPerLine; 
state.internal.mirrorDataOutputScanDelay = state.acq.scanDelay; 
state.internal.mirrorDataOutputAcqDelay = state.acq.acqDelay; 

%Now transform data (zoom, rotation,etc) /and/ map fast/slow axes to X/Y scanner channels
linTransformMirrorData(); %VI120908A
finalMirrorDataOutput = state.acq.mirrorDataOutput; 

return;


function fast = makeLineCommandFast(t, scanOffsetFast, scanAmplitudeFast)
% Construct fast scan dimension command for one line 
% For sawtooth/unidirectional scans, a cycloid waveform is used for the flyback
% For bidirectional scans, a simple triangle wave is used for the command
%
% NOTE - The scanOffsetFast parameter passed in is, in current usage, always 0. Offset/shift is applied at last step during linTransformMirrorData().

global state

state.internal.lengthOfXData = length(t); 

% determine the type of scan and modulate behavior accordingly
scanType = state.hSI.computeROIType(state.acq.scanAngleMultiplierFast, state.acq.scanAngleMultiplierSlow);

if strcmp(scanType,'point')
	fast(1:state.internal.lengthOfXData,1) = state.acq.scanShiftFast;
	
%elseif strcmp(scanType,'line') % TODO: does this need its own special case?
	
elseif ~state.acq.bidirectionalScan   

    %Key parameters
    rampTime = state.acq.fillFraction * (1e-3 * state.acq.msPerLine); %The 'true' ramp period, not including the settlingTime 'extension'
    settlingTime = state.acq.scanDelay; %Time added to ramp portion of the waveform, extending the ramp amplitude and hence compensating for scan attenuation
    flybackTime = (1e-3 * state.acq.msPerLine) - rampTime - settlingTime; % The period of the cycloid portion of the waveform  
    
    fast = zeros(state.internal.lengthOfXData,1);
    
    %Ramp waveform portion
    slope1 = 2 * scanAmplitudeFast/(1e-3 * state.acq.msPerLine * state.acq.fillFraction); %VI092610A              
    intercept1 = scanOffsetFast -scanAmplitudeFast - slope1 * settlingTime; %VI092610A
    numRampPoints = round(state.internal.lengthOfXData * (rampTime + settlingTime) / (1e-3 * state.acq.msPerLine));
    
    fast(1:numRampPoints) = slope1*t(1:numRampPoints) + intercept1;
    
    %Cycloid waveform portion
    cycloidVelocity = 2*pi/flybackTime;
    cycloidAmplitude = abs(2*scanAmplitudeFast) + abs(slope1) * (settlingTime + flybackTime); %VI092610A %Amplitude adjustments to account for added ramp time and ongoing ramp waveform req'd for initial conditon matching   
      
    t2 = t(numRampPoints+1:end) - t(numRampPoints);
    fast(numRampPoints+1:end) = fast(numRampPoints) + (-sign(scanAmplitudeFast)) *(cycloidAmplitude/(2*pi)) * (cycloidVelocity*t2 - sin(cycloidVelocity * t2)) + slope1*t2;   
    
else         
    slope1 = (2*scanAmplitudeFast)/(1e-3*state.acq.msPerLine*state.acq.fillFraction); %VI092610A 
    slope2 = -slope1;

    intercept1 = scanOffsetFast - (scanAmplitudeFast/state.acq.fillFraction); %VI092610A
    intercept2 = scanOffsetFast + (scanAmplitudeFast/state.acq.fillFraction); %VI092610A
    
    fast1 = slope1*t + intercept1;
    fast2 = slope2*t + intercept2;

    fast = [fast1'; fast2'];    
end

return;

function fast = makeFrameCommandFast(fast)
% Convert single line command for fast scan dimension into full frame command

global state

% determine the type of scan and modulate behavior accordingly
scanType = state.hSI.computeROIType(state.acq.scanAngleMultiplierFast, state.acq.scanAngleMultiplierSlow);

if ~state.acq.bidirectionalScan || strcmp(scanType,'point')
    fast = repmat(fast, [state.acq.linesPerFrame 1]); % Constructs an array of sawtoothx functions for each line acquired% Makes the column vector for one line of data
else
    if ~mod(state.acq.linesPerFrame,2)
        fast = repmat(fast,[state.acq.linesPerFrame/2 1]); %Only need half the repeats--each repeated element contains 2 lines
    elseif state.acq.slowDimFlybackFinalLine %If odd and using final line as flyback -- skip scanning the last line. All frames start with same-direction slope this way.
        fast = repmat(fast,[(state.acq.linesPerFrame-1)/2 1]);
        fast(end+1:end+length(fast)/2) = fast(1); %Pad with first value of next line
    else
        assert(false); 
    end
end
    
function slow = makeFrameCommandSlow(t, scanOffset, scanAmplitude)
% Construct slow scan dimension command for one frame 

global state

% determine the type of scan and modulate behavior accordingly
scanType = state.hSI.computeROIType(state.acq.scanAngleMultiplierFast, state.acq.scanAngleMultiplierSlow);

if strcmp(scanType,'point')
	slow(1:length(t),1) = state.acq.scanShiftSlow;
else
	%Slow dimension flyback command can either be explicitly added, at start of the final line (i.e. if last line is blanked/skipped/ignored) 
	%Or, by default, no flyback command is given and the flyback practically/physically occurs at start of first line (of next frame)
	if state.acq.slowDimFlybackFinalLine
		rampLinesPerFrame = state.acq.linesPerFrame - 1; 
	else
		rampLinesPerFrame = state.acq.linesPerFrame;
	end    

	slope1 = 2 * scanAmplitude/(1e-3 * state.acq.msPerLine*rampLinesPerFrame); %VI102209A   
	intercept1 = scanOffset - scanAmplitude; 

    if state.acq.slowDimFlybackFinalLine
        slope2 = -2*scanAmplitude/(1e-3 * state.acq.msPerLine); %VI032911A %flyback in the time it takes for one line
        intercept2 = scanOffset + scanAmplitude;
    end

	numberOfPositiveSlopePoints = rampLinesPerFrame*state.internal.lengthOfXData; 

	slow1 = slope1*t + intercept1;
	slow2 = []; 
	if ~state.acq.bidirectionalScan 
		if state.acq.slowDimFlybackFinalLine 
			slow2 = slope2*(t-(1e-3*state.acq.msPerLine*rampLinesPerFrame)) + intercept2; 
		end        
	else
		if state.acq.staircaseSlowDim
			slow1    = zeros(1,numberOfPositiveSlopePoints);

			stepVals = scanOffset + linspace(-scanAmplitude, scanAmplitude, rampLinesPerFrame);
			for i=1:length(stepVals)
				slow1(((i-1)*state.internal.lengthOfXData+1):(i*state.internal.lengthOfXData)) = stepVals(i);
			end

			%Account for data shift that can arise with long acqDelay values...
			overage = round((state.acq.acqDelay + state.acq.scanDelay + state.acq.fillFraction * state.acq.msPerLine * 1e-3) ...
				* state.acq.outputRate) + 1 - state.internal.lengthOfXData;
			if overage > 0
				slow1 = circshift(slow1,overage);
			end
		end

		if state.acq.slowDimFlybackFinalLine 
			slow2 = slope2*(t-(1e-3*state.acq.msPerLine*rampLinesPerFrame)) + intercept2; 
		end
	end

	slow = [slow1(1:numberOfPositiveSlopePoints)'; slow2(numberOfPositiveSlopePoints+1:end)']; 
end
return;


