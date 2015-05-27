function doSnapShot
%% function doSnapShot
% Handle 
%
%% NOTES
%   Not sure why file is called doSnapShot, when it really pertains to the end of snapshot -- Vijay Iyer 9/9/09
%
%% CHANGES
%   Fixed putDataGrab bug when going from Snap to Grab - T. O'Connor 12/30/03
%   TPMOD for SnapShot Mode....6/2/03
%   VI052909A: Reset stack power scaling preemptively in all cases if this function is called - Vijay Iyer 5/29/09
%   VI090909A: Changes associated with use of new DAQmx interface -- Vijay Iyer 9/9/09
%   VI092210A: Don't do anything special with state.acq.acquiredData during snapshots. Just another GRAB. -- Vijay Iyer 9/22/10
%   VI081011A: TurnOnMenus() before preallocateMemory(), so that saved imaged data still appears in display figures (though it's lost for saving) -- Vijay Iyer 8/10/11
%
%% CREDITS
%   Created by Tom Pologruto, 2003
%% *********************************************

global state gh


state.init.eom.stackPowerScaling = ones(state.init.eom.numberOfBeams,1); %VI052909A
if state.internal.snapping 
    state.files.autoSave=state.files.autoSaveBSnap;	
    state.acq.numberOfZSlices=state.acq.numberOfZSlicesBSnap;
    state.acq.numberOfFrames=state.acq.numberOfFramesBSnap;
    state.acq.numAvgFramesDisplay=state.acq.numAvgFramesDisplayBSnap;
    state.acq.averagingDisplay=state.acq.averagingDisplayBSnap;
    state.acq.lockAvgFrames=state.acq.lockAvgFramesBSnap;
    
    state.internal.snapping=0;

    %snappedData=state.acq.acquiredData; %VI092210A
    setImagesToWhole();
    turnOnMenus; %VI081011A
    preallocateMemory; 
    %%%VI092210A%%%
    %     for cc=1:length(snappedData)
    %         state.acq.acquiredData{cc}(:,:,1)=snappedData{cc};
    %     end
    %%%%%%%%%%%%%%%
    alterDAQ_NewNumberOfFrames;
    %putDataGrab; %VI090909A: Removed    %12/30/03 - Tim O'Connor
    flushAOData(); %VI090909A: Added instead of deprecated putDataGrab(). It's not clear that it's actually required though, as SNAP does not seem to affect the output control signals. But it does no harm.
    setStatusString('Ending Grab...');
    set(gh.mainControls.focusButton, 'Visible', 'On');
    set(gh.mainControls.startLoopButton, 'Visible', 'On');
    set(gh.mainControls.grabOneButton, 'String', 'GRAB');
    set(gh.mainControls.grabOneButton, 'Visible', 'On');
    %turnOnMenus; %VI081011A: Relocated
    setStatusString('');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%