function [axis,volts_per_pixelFast,volts_per_pixelSlow,sizeImage]=genericFigSelectionFcn(handle)
%This is the generic function called when the user wants to change the 
% scan params interactively with the mouse selection.
% This includes ROI selection, Center selection, and 
% linescan selection.
% Calculates which axis is selected, the pixles per volts in Fast and Slow dimensions, as
% well as the image size...
%% CHANGES
%   VI110308A: Handle negative scanAmplitude case for Y dimension -- Vijay Iyer 11/03/08
%   VI021909A: Check if the supplied handle belongs to any of the valid image figures, including the merge image -- Vijay Iyer 2/19/09
%   VI021909B: Compute volts/pixel correctly, accounting for fact that FOV now always corresponds to full amplitude -- Vijay Iyer 2/19/09
%   VI102609A: Use state.internal.scanAmplitudeX/Y in lieu of state.acq.scanAmplitudeX/Y, as the internal value is now used to represent the actual command voltage --  Vijay Iyer 10/26/09
%   VI092010A: Refer to X/Y now as Fast/Slow -- Vijay Iyer 9/20/10
%
%% ***************************************
global state gh
setImagesToWhole;
if nargin<1
    axis=state.internal.axis(logical(state.acq.imagingChannel));
    image=state.internal.imagehandle(logical(state.acq.imagingChannel));
    axis=axis(1);
    image=image(1);
elseif ishandle(handle)
    
    %%%VI021909A%%%%%%%%%%%%%%%%%%
    imageHandles = [state.internal.imagehandle(:); state.internal.mergeimage];
    axHandles = zeros(length(imageHandles),1);
    for i=1:length(imageHandles)
        axHandles = [axHandles; ancestor(imageHandles(i),'axes')];
    end
    
    if ~ismember(handle,axHandles)
        return;
    else
        axis = handle;
    end
    %%%%%%%%%%%%%%%%%%%%%%%
        
    %%%VI021909A:Removed %%%%%%%%%%    
    %     ind=find(handle==state.internal.axis);
    %     if isempty(ind)
    %         return
    %     end
    %     axis=handle;
    %     image=state.internal.imagehandle(ind);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
else
    return
end
%fractionUsedXDirection=state.acq.fillFraction; %VI021909B
x=get(axis,'XLim');
y=get(axis,'YLim');
sizeImage=[y(2) round(state.acq.roiCalibrationFactor*x(2))]; %Note that roiCalibrationFactor is now always 1 -- Vijay Iyer 2/19/09

%volts_per_pixelX=((1/state.acq.zoomFactor)*2*fractionUsedXDirection*abs(state.acq.scanAmplitudeX))/sizeImage(2); %VI091508A, VI021909B
volts_per_pixelFast=((1/state.acq.zoomFactor)*2*abs(state.internal.scanAmplitudeFast))/sizeImage(2); %VI102609A %VI091508A, VI021909B
volts_per_pixelSlow=((1/state.acq.zoomFactor)*2*abs(state.internal.scanAmplitudeSlow))/sizeImage(1); %VI102609A %VI110308A
