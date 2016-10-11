%% function updateStackEndpoints(relOrigin)
% Function that updates display of stack endpoints
%
%% SYNTAX
%   relOrigin: (OPTIONAL) Supply updated relative origin, previously obtained, to use in computing new stack endpoints. If not, it will be retrieved from motor controller.
%
%% CHANGES
%   VI111108A: Handle case where update is a clearing of stack start or stop; enable pbGrabOneStack button only if stack start & stop are defined
%   VI112309A: Allow pbGrabOneStack button on motorControls to appear if only start is defined, if the 'stackEndpointsDominate' flag is false  -- Vijay Iyer 11/23/09
%   VI112309B: Disable End controls when 'stackEndpointsDominate' is false, as End has no effect -- Vijay Iyer 11/23/09
%   VI032310A: Use relative origin from LinearStageController class, via motorGetRelativeOrigin(), instead of offsetX/Y/Z vars -- Vijay Iyer 3/23/10
%   VI032410A: Only retrieve relative origin from LinearStageController class if it's not provided -- Vijay Iyer 3/24/10
%   VI040210A: The etStackEnd control should be enabled as 'inactive', not 'on', when the endControls are enabled -- Vijay Iyer 4/2/10
%   VI051211A: Support secondary Z motor controller cases -- Vijay Iyer 5/12/11
%
%% CREDITS
%   Created 10/09/08 by Vijay Iyer
%% **************************

function updateStackEndpoints(relOrigin)

global state gh

%out=[];

if state.motor.motorOn
    if (nargin < 1 || isempty(relOrigin)) && (~isempty(state.motor.stackStart) || ~isempty(state.motor.stackStop)) %VI032410A
        relOrigin =motorGetRelativeOrigin(); %VI032310A
    end
    
    if ~isempty(state.motor.stackStart)
        if state.motor.dimensionsXYZZ && state.motor.motorZEnable  %VI051211A
            set(gh.motorControls.etStackStart,'String', num2str(state.motor.stackStart(4) - relOrigin(4))); 
        else
            set(gh.motorControls.etStackStart,'String', num2str(state.motor.stackStart(3) - relOrigin(3))); %VI032310A
        end
    else
        set(gh.motorControls.etStackStart,'String', ''); %VI111108A
    end
    
    if ~isempty(state.motor.stackStop)
        if state.motor.dimensionsXYZZ && state.motor.motorZEnable  %VI051211A
            set(gh.motorControls.etStackEnd,'String', num2str(state.motor.stackStop(4) - relOrigin(4))); 
        else
            set(gh.motorControls.etStackEnd,'String', num2str(state.motor.stackStop(3) - relOrigin(3))); %VI032310A
        end
    else
        set(gh.motorControls.etStackEnd,'String', ''); %VI111108A
    end
    
    %%%VI111108A%%%%
    if ~isempty(state.motor.stackStart) && (~isempty(state.motor.stackStop) || ~state.motor.stackEndpointsDominate) %VI112309A
        set(gh.motorControls.pbGrabOneStack,'Enable','on');
    else
        set(gh.motorControls.pbGrabOneStack,'Enable','off');
    end
    %%%%%%%%%%%%%%%%%
    
    %%%VI112309B%%%%
    if state.motor.stackEndpointsDominate
        set(gh.motorControls.pbSetEnd,'Enable','on');        
        set(gh.motorControls.etStackEnd,'Enable','inactive'); %VI040210A
    else
        set([gh.motorControls.pbSetEnd gh.motorControls.etStackEnd],'Enable','off'); %VI040210A
    end
    %%%%%%%%%%%%%%%%%
        
end
    

