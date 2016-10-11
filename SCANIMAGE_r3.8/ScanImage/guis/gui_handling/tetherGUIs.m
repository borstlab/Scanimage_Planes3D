function tetherGUIs(parentGUI,childGUI,relPosn)
%% function tetherGUIs(parent,child,relPosn)
% Tethers specified child GUI to specified parent GUI, according to relPosn
%
%% SYNTAX
%   tetherGUIs(parent,child,relPosn)
%       parentGUI,childGUI: Valid GUI names of parent and child GUIs
%       relPosn: String from set {'righttop' 'rightcenter' 'bottom'} indicating desired location of child GUI relative to parent GUI
%% CHANGES
%   VI043009A: For now, remove memory of whether GUI has been 'de-tethered'. Always force tethering. -- Vijay Iyer 4/30/09
%% CREDITS
%   Created 2/10/09, by Vijay Iyer
%% ***************************************************

global gh state

%Only tether if it hasn't been previously tethered (or otherwise had position defined)
parPosn = get(gh.(parentGUI).figure1,'OuterPosition');
childPosn = get(gh.(childGUI).figure1,'OuterPosition');

switch relPosn
    case 'righttop'
        childPosn(1) = sum(parPosn([1 3]));
        childPosn(2) = sum(parPosn([2 4])) - childPosn(4);
    case 'rightcenter'
        childPosn(1) = sum(parPosn([1 3]));
        childPosn(2) = parPosn(2) + parPosn(4)/2 - childPosn(4)/2;
    case 'bottom'
        childPosn(1) = parPosn(1) + parPosn(3)/2 - childPosn(3)/2;
        childPosn(2) = parPosn(2) - childPosn(4);
end

set(gh.(childGUI).figure1,'OuterPosition',childPosn);

seeGUI(['gh.' childGUI '.figure1']);
       
%%%VI043009A: Removed %%%%%%%%%%%%%%
% %Flag position, which can be used to prevent tethering from happening again
% if isfield(state.internal,[childGUI 'Left'])
%     childPosn = get(gh.(childGUI).figure1,'Position');
%     state.internal.([childGUI 'Left']) = childPosn(1);
%     state.internal.([childGUI 'Bottom']) = childPosn(2);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
