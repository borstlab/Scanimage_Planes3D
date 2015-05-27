function setupAOData
global state
% setupAOData.m******
% Function that create and store the output data for the scanning mirrors and the
% pockel's cell.
%
%******************************************************************************
% Mirror Data Output
% Uses the PCI 6110E Board
% Setting up analog output for the NI Board and adding 2 channels to it.  
% Setting up Mirror controls.
% Constructing appropriate output data.
%% CHANGES
%   VI022108A: Support for infinite focus moves set of AO 'RepeatOutput' property to startFocus() -- Vijay Iyer 2/21/08
%   VI011509A: (Refactoring) Moved in AO object property sets, from setupDAQDevices_ConfigSpecific(); always call flushAOData() after completing other setup steps -- Vijay Iyer 1/15/09
%   VI011609A: Changed state.init.pockelsOn to state.init.eom.pockelsOn -- Vijay Iyer 1/16/09
%   VI082809A: Handle change to new DAQmx interface -- Vijay Iyer 8/28/09
%
%% CREDITS
% Written by Thomas Pologruto  
% Cold Spring Harbor Labs
% February 7, 2000
%% ***********************************************

%%%VI011509A: Refactored In %%%%%%%%%
% GRAB output: set number of frames in GRAB output object to drive mirrors
%set(state.init.ao2, 'RepeatOutput', (state.acq.numberOfFrames -1)); %VI082809A: The # of repeats can now only be set after the buffer is configured/reconfigured 

% FOCUS output: set number of frames in FOCUS output object to drive mirrors
%set(state.init.ao2F, 'RepeatOutput', (state.internal.numberOfFocusFrames -1)); %VI022108A

% 	if state.init.pockelsOn == 1			% and pockel cell, if on
% 		set(getfield(state.init,['ao'  num2str(state.init.eom.scanLaserBeam) 'F']), 'RepeatOutput', (state.internal.numberOfFocusFrames -1));
% 	end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Data Queing and AO execution
%DEQ20101222state.acq.mirrorDataOutput = makeMirrorDataOutput; 			% Defines the data matrix sent to the mirrors
state.acq.mirrorDataOutput = feval(state.hSI.hMakeMirrorDataOutput); 			% Defines the data matrix sent to the mirrors
%TPMODPockels
if state.init.eom.pockelsOn == 1 %VI011609A
	state.init.eom.changed(:) = 1;
end

%VI011509A: Always flush AO data after setting it up
flushAOData();

