% Function that checks the strings of FOCUS, GRAB, and LOOP and 
% aborts if they are running.
% This function is used in genericKeyPressFunction for the letter 'a'.
%% SYNTAX
%   abortCurrent()
%   abortCurrent(verbose)
%       verbose: Logical value indicating, if true, to display status info during function operation. If omitted, value assumed to be true.
%
%
%% CHANGES
%   VI100608A: Ensure MP285 state using updateMotorPosition(), rather than MP285Config() -- Vijay Iyer 10/06/08
%   VI101008A: Signal abort completion to the user -- Vijay Iyer 10/10/08
%   VI101208A: Correct Loop stop string from 'STOP' to 'ABORT' -- Vijay Iyer 10/12/08
%   VI111708A: Dont' generate error if this is invoked when there's nothing to abort...just ignore. -- Vijay Iyer 11/17/08
%   VI120108A: Allow this function to operate 'silently' if desired -- Vijay Iyer 12/01/08
%   VI031310A/VI032010A: Make 'silent' an abort when no acquisition was occurring -- Vijay Iyer 3/20/10
%% CREDITS
% Written By: Thomas Pologruto and Bernardo Sabatini
% Cold Spring Harbor Labs
% January 30, 2001
%% *****************************************

function abortCurrent(verbose)
global state gh

%%%VI120108A%%%
if nargin < 1 || isempty(verbose)
    verbose = true;
end
%%%%%%%%%%%%%%%


if strcmp(get(gh.mainControls.focusButton, 'String'), 'ABORT')
	executeFocusCallback(gh.mainControls.focusButton);
    modeStr='Focus'; %VI101008A
elseif strcmp(get(gh.mainControls.grabOneButton, 'String'), 'ABORT')
	executeGrabOneCallback(gh.mainControls.grabOneButton);
    modeStr='Grab'; %VI101008A
elseif strcmp(get(gh.mainControls.startLoopButton, 'String'), 'ABORT') %VI101208A
	executeStartLoopCallback(gh.mainControls.startLoopButton);
    modeStr='Loop'; %VI101008A
else
    return; %VI032010A
    %%%VI031310A: Removed%%%%%%
    % else
    %     if verbose %VI120108A
    %         setStatusString('Nothing to Abort');
    %     end
    %     return; %VI111708A
    %     %error('Abort attempted while in non-action state'); %VI111708A
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end  

%%%VI100608A%%%%%%%%%%%%%
if ~motorErrorPending() %VI032210A - prevents error/warning on
    motorGetPosition();
end
%%%%%%%%%%%%%%%%%%%%%%%%%%

if verbose %VI120108A
    setStatusString(['Aborted ' modeStr]); %VI101008A
end

