function updateImageBox(handle)
%% function updateImageBox(handle)
% Callback function that handles update to the ImageBox (crosshair) checkbox
%
%% NOTES
%   Function is an INI-file callback, so it is invoked either upon adjusting the GUI control or loading a CFG file
%
%   Activating crosshairs requires that EraseMode of all image display figures be set to Normal.
%
%   Function is a cut-and-paste from original callback in userPreferenceGui.m, plus handling of EraseMode
%% CHANGES
%   VI022311A: Use state variable instead of handle object's value -- Vijay Iyer 2/23/11
%
%% CREDITS
%   Created 9/10/09, by Vijay Iyer
%% ******************************************************************

global state
%val=get(handle,'Value'); %VI022311A: Removed

if isfield(state.internal,'axis') %Ignore calls before image when this is called from openini(). Wait till openusr().
    if state.acq.imageBox %VI022311A
        setAxisGrids(state.internal.axis, 2);
        set(state.internal.imagehandle,'EraseMode','normal'); %EraseMode must be normal if the crosshair is to display
    else
        %setImagesToWhole;
        set(state.internal.axis, 'XGrid', 'off', 'YGrid', 'off', 'XColor', 'b', 'YColor', 'b', 'GridLineStyle', 'none','Layer','Bottom');
        set(state.internal.imagehandle,'EraseMode', 'none'); %This is fastest
    end
    rearrangeAxes(state.internal.axis);
end

        
        