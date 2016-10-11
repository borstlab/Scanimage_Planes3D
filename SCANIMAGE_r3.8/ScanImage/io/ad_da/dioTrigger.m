function dioTrigger
%% function dioTrigger
% Function that ouputs a 0,1,0 to the dio device, providing an edge trigger to all DAQ devices
%
%% NOTES
% This function is used to "start" the DAQ session. 
%
% This function also opens the shutter prior to acquisition.
%
%% CHANGES
%   VI022708A Vijay Iyer 2/27/2008 - Modifications to make triggering more reliable. This solves the problem where the first focus following a configuration update would fail.
%   VI070109A Vijay Iyer 7/1/2009 - Rename dioTriggerTime to softTriggerTime. Store softTriggerTimeString value as well.
%   VI082909A Vijay Iyer 8/29/2009 - Handle change(s) to new DAQmx interface
%   VI091509A Vijay Iyer 9/15/2009 -- Simplify trigger call to one line (effectively reverting VI022708A). Adding pauses to the trigger call does not seem to have any benefit anymore
%
%% CREDITS
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% February 7, 2001
%% **************************************************************

global state

state.internal.softTriggerTime = clock; %VI0710109A %NOTE: This should probably be done after the putvalue() calls -- Vijay Iyer 7/1/09
%%%VI070109A%%%%%%%%
state.internal.softTriggerTimeString = clockToString(state.internal.softTriggerTime);
updateHeaderString('state.internal.softTriggerTimeString');
%%%%%%%%%%%%%%%%%%%%

%TPMODPockels
% if state.init.pockelsOn == 1
% 	% Pockels Board Trigger
% 	putvalue(state.init.pockelsLine, 1);
% 	putvalue(state.init.pockelsLine, 0);
% end

% Acquisition Board Trigger
%%%VI082909A: Removed %%%%%%%%%%%%
% putvalue(state.init.triggerLine, 0);			% VI022708A (for good measure)
% pause(.1);                                      % VI022708A
% putvalue(state.init.triggerLine, 1);			% Places an 'on' signal on the line initially
% pause(.1);                                      % VI022708A
% putvalue(state.init.triggerLine, 0); 			% Digital Trigger: Places a go signal (1 to 0 transition; FallingEdge) to 
% 												% the line to trigger the ao1, ao2, & ai.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                

%%%VI082909A%%%%%%%%%%%%%
%TODO: Use writeDigitalSample() option instead ( to be developed)
state.init.hTrigger.writeDigitalData([0;1;0],.2); %VI091509A
%%%%%%%%%%%%%%%%%%%%%%%%%%

