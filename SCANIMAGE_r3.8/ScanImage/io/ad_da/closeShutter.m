function closeShutter
%% function closeShutter
% Function that sends the close signal defined in the state global variable to the shutter.
%
%% NOTES
% Must be executed after the setupDAQDevices.m function.
%
%% CHANGES
%   VI031109A: Handle renamed state variable -- Vijay Iyer 3/11/09 
%   VI082909A: Change(s) to use new DAQmx interface -- Vijay Iyer 8/29/09
%
%% CREDITS
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% December 5, 2000
%% ******************************

global state

state.shutter.shutterOpen=0;
if state.shutter.shutterOn %VI031109A
    %putvalue(state.shutter.shutterLine, state.shutter.closed); %VO082909A: Removed
    state.shutter.hDO.writeDigitalData(~state.shutter.open,.2); %VI082909A %TODO: Perhaps use a new 'writeDigitalSample' method, if created
end
