function openShutter(applyShutterDelay)
%% function openShutter
% Function that sends the open signal defined in the state global variable to the shutter.
% 
%% SYNTAX
%   applyShutterDelay: <OPTIONAL - Default=false> Logical indicating whether to wait state.shutter.shutterDelay value (if supplied) for shutter to physically open following electrically command -- Vijay Iyer 4/5/11
%
%% NOTES
% Must be executed after the setupDAQDevices.m function.
%
%% CHANGES
%   VI031109A: Handle renamed state variable -- Vijay Iyer 3/11/09 
%   VI082909A: Change(s) to use new DAQmx interface -- Vijay Iyer 8/29/09
%   VI040511A: Wait for shutter to open after sending electrical command -- Vijay Iyer 4/5/11
%
%% CREDITS
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% December 5, 2000
%% ******************************

global state
            
state.shutter.shutterOpen=1;
if state.shutter.shutterOn %VI031109A
    %putvalue(state.shutter.shutterLine, state.shutter.open); %VI082909A: Removed
    state.shutter.hDO.writeDigitalData(state.shutter.open,.2); %VI082909A %TODO: Perhaps use a new 'writeDigitalSample' method, if created
end

%%%VI040511A%%%%
if nargin < 1
    applyShutterDelay = false;
end

if applyShutterDelay && isfield(state.shutter, 'shutterOpenTime')
    most.idioms.pauseTight(state.shutter.shutterOpenTime * 1e-6);    
end
%%%%%%%%%%%%%%%%
    
