
function linTransformMirrorData()
%% function linTransformMirrorData()
% This function is responsible for all the linear transformations to mirror data -- scaling, rotation, and shift (in that order)
% In addition, as final steps, this function handles the mapping of fast/slow scanners to X/Y channels, and applies the X/Y offset
%
%% NOTES
%  This function was based on rotateAndShiftMirrorData(), but then modified it to be a state-transformer and added the handling of scaling -- Vijay Iyer 9/28/08
%
%  In the case that a change in the msPerLine value has been detected, both the AO and AI buffers must be recomputed
%
%% CHANGES
%   VI022308A Vijay Iyer 2/23/08 - No longer declare lengthofframedata as a global -- not seemingly used anywhere else
%   VI091208A: Add scanOffsetX/Y in addition to the scaleX/YShift
%   VI092208A: Compute state.internal.scanAmplitudeX/Y here...not very elegant, but this is the endpoint at which the scaling is found
%   VI092808A: Add scaling here and make this a state variable processing function (rather than an input/output processor)
%   VI120908A: Correctly use state.internal.baseZoomFactor vs state.acq.baseZoomFactor
%   VI010809A: Recompute base mirror data if timing parameters have changed -- Vijay Iyer 1/08/09
%   VI011509A: (Refactoring) Remove explicit call to setupAOData(), as this is now called as part of setupDAQDevices_ConfigSpecific() -- Vijay Iyer 1/15/09
%   VI012809A: Recompute base mirror data if scan delay has changed -- Vijay Iyer 1/28/09
%   VI013109A: Recompute if acq delay has changed also -- Vijay Iyer 1/31/09
%   VI013109B: Remove differential scaling for above/below base zoom factor -- Vijay Iyer 1/31/09
%   VI030409A: Don't recompute scan following acq delay change if the change is not 'significant' (i.e. if it doesn't change Pockels or Y command signals) -- Vijay Iyer 3/4/09
%   VI052009A: (REFACTOR) All calls to setupDaqDevices_ConfigSpecifig() also call preallocateMemory() -- Vijay Iyer 5/21/09
%   VI102609A: State variable state.internal.scanAmplitudeX/Y is now used to represent the full scan amplitude in the scanner units (i.e. volts). It is no longer (and had not been) used for representing the value at each zoom level. -- Vijay Iyer 10/26/09
%   VI060710A: Handle case where computed scan command amplitude exceeds output voltage range (e.g. +/-10V). For now just clip and issue a command-line warning -- Vijay Iyer 6/7/10
%   VI092010A: This function now handles mapping of fast/slow scanners to X/Y channels. ScanOffsetX/Y is accordingly now handled as last step. -- Vijay Iyer 9/20/10
%   VI110210A: ScanOffsetX/Y renamed to ScanOffsetAngleX/Y -- Vijay Iyer 11/2/10
%   VI110310A: Convert ScanOffsetAngleX/Y & scanShiftFast/Slow to voltages before adding to mirrorDataOutput (which is in volts as well) -- Vijay Iyer 11/3/10
%   VI110310B: Do check for voltage range clamping /after/ adding offset -- Vijay Iyer 11/3/10
%   VI110310C: Add offsets one column at a time; avoiding repmat() speeds up sum operation > 10x. -- Vijay Iyer 11/3/10
%   VI052011A: Implement scanAngleMultiplierFast/Slow transformation here as well -- Vijay Iyer 5/20/11
%
%% CREDITS
%   Created 9/28/08, by Vijay Iyer
%   Based on rotateMirrorData(), written by Tom Pologruto, 9/29/00
%% ********************************************

global state 

%%%VI030409A%%%%%%
sigAcqDelayChange = (state.acq.acqDelay ~= state.internal.mirrorDataOutputAcqDelay) && ...
    ((state.init.eom.pockelsOn && state.acq.pockelsClosedOnFlyback) || state.acq.staircaseSlowDim); 
%%%%%%%%%%%%%%%%%%

if state.acq.msPerLine ~= state.internal.mirrorDataOutputMsPerLine || ... %VI010809A, VI012809A, VI013109A, VI030409A
        state.acq.scanDelay ~= state.internal.mirrorDataOutputScanDelay || sigAcqDelayChange 
        %state.acq.acqDelay ~= state.internal.mirrorDataOutputAcqDelay  %% && state.init.pockelsOn && state.acq.pockelsClosedOnFlyback)  [This is a nice idea, but doesn't really save much time in the end]
    %%%VI010809A%%%%%
    setStatusString('Recomputing Scan');
    setupDAQDevices_ConfigSpecific;    
	%preallocateMemory; %VI052109A
    %setupAOData();  %VI011509A
    return; %linTransformMirrorData() will get called    
    %%%%%%%%%%%%%%%%%%
end

%Scale the base mirror data 
scaledMirrorDataOutput = state.acq.mirrorDataOutputOrg / state.acq.zoomFactor; %VI013109B
scaledMirrorDataOutput = scaledMirrorDataOutput .* ... %VI052011A
    repmat([state.acq.scanAngleMultiplierFast state.acq.scanAngleMultiplierSlow],length(scaledMirrorDataOutput),1);
scaledMirrorDataOutput(isinf(scaledMirrorDataOutput)) = 0;
%%%VI013109B: Removed %%%%%%%%%
% if state.acq.zoomFactor < state.acq.baseZoomFactor
%     scaledMirrorDataOutput = state.acq.mirrorDataOutputOrg / state.acq.zoomFactor;
% else
%     scaledMirrorDataOutput= state.acq.mirrorDataOutputOrg/(state.acq.zoomFactor/state.acq.baseZoomFactor); %VI092808A, VI120908A
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%VI102609A: Removed %%%%%%%%%
%%%VI092208A
% state.internal.scanAmplitudeX = state.acq.scanAmplitudeX/state.acq.zoomFactor;
% state.internal.scanAmplitudeY = state.acq.scanAmplitudeY/state.acq.zoomFactor;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lengthofframedata = size(scaledMirrorDataOutput,1); %VI092808A
%lengthofframedata = lengthofframedata(1,1);

c = cos(state.acq.scanRotation*pi/180);
s = sin(state.acq.scanRotation*pi/180);

%a = 1:lengthofframedata;
% finalMirrorDataOutput(a,1)=finalMirrorDataOutput(a,1);
% finalMirrorDataOutput(a,2)=finalMirrorDataOutput(a,2);
state.acq.mirrorDataOutput = zeros(lengthofframedata,2); %VI010809A
state.acq.mirrorDataOutput(:,1) = (c*scaledMirrorDataOutput(:,1) + s*scaledMirrorDataOutput(:,2)) + (state.acq.scanShiftFast * state.init.voltsPerOpticalDegree); %VI110310A %VI092010A %VI091208A
state.acq.mirrorDataOutput(:,2) = (c*scaledMirrorDataOutput(:,2) - s*scaledMirrorDataOutput(:,1)) + (state.acq.scanShiftSlow * state.init.voltsPerOpticalDegree); %VI110310A %VI092010A %VI091208A

%%%VI092010A%%%%%%
%Input order is [fast slow], output order is [x y]
if ~state.acq.fastScanningX
    state.acq.mirrorDataOutput = fliplr(state.acq.mirrorDataOutput); %Input order is [fast slow], output order is [x y]
end
%state.acq.mirrorDataOutput = state.acq.mirrorDataOutput + repmat([state.init.scanOffsetAngleX state.init.scanOffsetAngleY] * state.init.voltsPerOpticalDegree,length(state.acq.mirrorDataOutput),1); %VI110310B %VI110310A %VI110210A
state.acq.mirrorDataOutput(:,1) = state.acq.mirrorDataOutput(:,1) + state.init.scanOffsetAngleX * state.init.voltsPerOpticalDegree; %VI110310C %VI110310A
state.acq.mirrorDataOutput(:,2) = state.acq.mirrorDataOutput(:,2) + state.init.scanOffsetAngleY * state.init.voltsPerOpticalDegree; %VI110310C %VI110310A
%%%%%%%%%%%%%%%%%

%%%VI110310B/VI060710A%%%%%%
for i=1:size(state.acq.mirrorDataOutput,2)
    
    warnMsg = false;
    minLimit = -state.init.outputVoltageRange;
    maxLimit = state.init.outputVoltageRange;

    if min(state.acq.mirrorDataOutput(:,i)) < minLimit
       warnMsg = true;
       state.acq.mirrorDataOutput(state.acq.mirrorDataOutput(:,i) < minLimit,i) = minLimit;
    end
    
    if max(state.acq.mirrorDataOutput(:,i)) > maxLimit
       warnMsg = true;
       state.acq.mirrorDataOutput(state.acq.mirrorDataOutput(:,i) > maxLimit,i) = maxLimit;
    end
       
       
    if warnMsg
        fprintf(2,'WARNING: Computed mirror output signal exceeds configured output range of Analog Output board. Command signal has been clipped.\n');
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%


%finalMirrorDataOutput = rotatedImage;
