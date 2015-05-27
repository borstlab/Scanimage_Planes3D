%% function motorSetPositionRelative(newPos,displayMode)
% Function that updates the MP285 motor position with the specified newPos, specified in relative coordinates
%% SYNTAX
%  motorSetPositionRelative()
%  motorSetPositionRelative(newPos)
%       newPos: <OPTIONAL> 1x3 array of absolute X/Y/Z positions to which motor should go. If empty, the absX/Y/ZPosition state variable is used. 
%       displayMode: <OPTIONAL;DEFAULT='none'> One of {'assume' 'verify' 'none'}.  Determines now newly set position should be displayed in ScanImage:
%                       'assume': The ScanImage relative/absolute X/Y/Z positions (those displayed) are updated to match the specified position at start of move
%                       'verify': Same as 'assume', but motorGetPosition() is called at end of move which updates ScanImage stored/displayed position values to match that read from device.
%                       'none': Do not set the Scanimage stored/displayed relative/absolute X/Y/Z positions and do not explicitly retrieve the final position from the motor controller
%           
%% NOTES
%   Main action is now deferred to use motorSetPosition() generic move function -- Vijay Iyer 4/13/10
%
%% CHANGES
%   VI052010A: Added displayMode argument, now use varargin -- Vijay Iyer 5/20/10
%   
%% CREDITS
%   Created 3/20/10, by Vijay Iyer
%% *********************

function motorSetPositionRelative(varargin)

motorSetPosition('relative',varargin{:}); %VI052010A


% global state
% 
% %If position is not supplied, then use state.motor.relX/Y/ZPosition variable
% if nargin<1 || isempty(newPos)
%     newPos(1) = state.motor.relXPosition;		
%     newPos(2) = state.motor.relYPosition;		
%     newPos(3) = state.motor.relZPosition;		
% end
% relOrigin = get(state.motor.hMotor,'relativeOrigin');
% motorSetPosition(newPos + relOrigin); %Set motor position in absolute coordinates
% 
% 
