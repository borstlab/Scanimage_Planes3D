function stopAndRestartFocus
%% function stopAndRestartFocus
%   Handle allowable configuration changes that occur during FOCUS acquisitions
%
%% NOTES
%   This function had been causing a segmentation violation, when zoom or scan parameter GUI updates are made that do not change any scan parameters.
%   This is resolved by forcing recompute of scan in all cases, but this seems to be a timing/race issue, rather than a real logical issue -- Vijay Iyer 9/15/09
%
%% CHANGES
%   11/24/03 Tim O'Connor - Start using the daqmanager object.
%   VI052008A - Used scim_parkLaser instead of parkLaser, for consistency -- Vijay Iyer 5/20/08
%   VI061908A - Call scim_parkLaser in a way that avoids shutter closing and limits mirror motion to the edge of the FOV-- Vijay Iyer 6/19/08
%   VI061908B - Removed redundant code -- Vijay Iyer 6/19/08
%   VI06208A - Use new maxOffsetX/Y and maxAmplitudeX/Y vars to determine 'fast' park location -- Vijay Iyer 6/25/08
%   VI121908A - Use scanAmplitudeX/Y rather than maxAmplitudeX/Y for 'fast' parking. maxAmplitudeX/Y vars have been eliminated.  -- Vijay Iyer 12/19/08
%   VI010809A - Use linTransformMirrorData instead of rotateAndShiftMirrorData() -- Vijay Iyer 1/08/09
%   VI090209A - Signal that stopFocus() call is an 'abort' operation -- Vijay Iyer 09/02/09
%   VI090209B - Remove superfluous 'flush' of AI data. stopFocus() call is sufficient. -- Vijay Iyer 09/02/09
%   VI090409A - Use new stopActionFunctions field to prevent excess callbacks -- Vijay Iyer 9/4/09
%   VI091509B - Force recompute of scan (and all configuration processing) every time, as this resolves segmentation violation issue -- Vijay Iyer 9/15/09
%   VI102609A - Handle state.acq.scanAmplitudeX/Y and state.init.scanOffsetAngleX/Y being specified in optical degrees. For former, using state.internal.scanAmplitudeX/Y suffices; latter is converted as needed. -- Vijay Iyer 10/26/09
%   VI092010A - BUGFIX: Was doing 'fast park' at wrong amplitude, due to parentheses error -- Vijay Iyer 9/20/10
%   VI092010B - Use new scanAmplitudeFast/Slow variables; handle X/Y->Fast/Slow mapping specified by state.acq.fastScanningX  -- Vijay Iyer 9/20/10
%   VI092610A: ScanAmplitude Fast/Slow are now full peak-peak amplitude values -- Vijay Iyer 9/26/10
%   VI110210A: ScanOffsetX/Y renamed to ScanOffsetAngleX/Y; scanAmplitudeX/Y renamed to scanAngularRangeX/Y; scim_parkLaser() now handles conversion from degrees to volts internally, so no longer do it here -- Vijay Iyer 11/2/10
%   VI111010A: Call scim_parkLaser() with 'soft' flag; this is now required to avoid closing shutter -- Vijay Iyer 11/10/10
%   VI111110A: Maintain flag during stopAndRestartFocus() behavior -- Vijay Iyer 11/11/10
%
%% ************************************************************

global state

state.internal.fastRestart = 1; %VI111110A

try  %VI111110A
    %Stop everything that is happening now....
    state.internal.abortActionFunctions = 1;
    stopFocus(true); %VI090209A: Signal this is an 'abort' operation
    
    %%%VI061908B: Removed%%%%%%%
    % if state.init.pockelsOn == 1
    %     deviceList=[state.init.aiF state.init.ao2F];
    % else
    %     deviceList=[state.init.aiF];
    % end
    % stop(deviceList);
    %
    % while ~any(strcmp(get(deviceList, 'Running'), repmat('Off', length(deviceList), 1)))
    %     pause(0.001);
    % end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%VI092010B%%%%%%
    if state.acq.fastScanningX
        X = ((state.init.scanAngularRangeReferenceFast * state.acq.scanAngleMultiplierFast)/2) + state.init.scanOffsetAngleX; %VI110210A %VI092610A
        Y = ((state.init.scanAngularRangeReferenceSlow * state.acq.scanAngleMultiplierSlow)/2) + state.init.scanOffsetAngleY; %VI110210A %VI092610A
    else
        X = ((state.init.scanAngularRangeReferenceSlow * state.acq.scanAngleMultiplierSlow)/2) + state.init.scanOffsetAngleX; %VI110210A %VI092610A
        Y = ((state.init.scanAngularRangeReferenceFast * state.acq.scanAngleMultiplierFast)/2) + state.init.scanOffsetAngleY; %VI110210A %VI092610A
    end
    scim_parkLaser([X Y],'soft'); %VI111010A
    %scim_parkLaser([(state.internal.scanAmplitudeX + state.init.scanOffsetAngleX) * state.init.voltsPerOpticalDegree (state.internal.scanAmplitudeY + state.init.scanOffsetAngleY) * state.init.opticalDegreesConversion]); %VI092010B: Removed %VI092010A %VI102609A %VI061908A %VI062508A %VI121908A
    %%%%%%%%%%%%%%%%%%
    
    %state.acq.mirrorDataOutput = rotateAndShiftMirrorData(1 / state.acq.zoomFactor * state.acq.mirrorDataOutputOrg); %VI010809A
    %linTransformMirrorData(); %VI091509A: Removed   %VI010809A
    setupDAQDevices_ConfigSpecific(); %VI091509A
    %flushAOData; %VI091509: Redundant
    
    resetCounters;
    openShutter; %Note that it's not clear why this was done--the shutter wasn't originally closed at all
    %%%VI090209B: Removed %%%%%%%%%%%
    % if get(state.init.aiF, 'SamplesAvailable') > 0
    %     try
    %         flushdata(state.init.aiF);
    %     end
    % end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    startFocus;
    state.internal.stripeCounter = 0;
    state.internal.stripeCounter2 = 0;
    state.internal.forceFirst = 1;
    state.internal.abortActionFunctions = 0;
    state.internal.stopActionFunctions = 0; %VI090409A
    dioTrigger;
    
    state.internal.fastRestart = 0;  %VI111110A
catch ME %VI111110A
    state.internal.fastRestart = 0;  %VI111110A
    ME.rethrow();
end
