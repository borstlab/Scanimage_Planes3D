function updateCurrentROI
%This function updates the current ROI 
% without changing the others...
global state gh
mockROI=[state.acq.scanShiftFastBase state.acq.scanShiftSlowBase state.acq.scanRotationBase state.acq.zoomFactorBase];
now=[state.acq.scanShiftFast state.acq.scanShiftSlow state.acq.scanRotation state.acq.zoomFactor];

fractionUsedXDirection=state.acq.fillFraction;
scanX=1/state.acq.zoomFactorBase*2*state.acq.roiCalibrationFactor*abs(state.internal.scanAmplitudeFast); %VI102609A %VI091508A
scanY=1/state.acq.zoomFactorBase*2*abs(state.internal.scanAmplitudeSlow); %VI102609A %VI091508A
if scanY == 0
    return
end
amplitude=mockROI(4)/now(4);

diffX=(now(1)-mockROI(1))/(scanX) - amplitude/2;
diffY=(now(2)-mockROI(2))/(scanY) - amplitude/2;
xdata=[diffX diffX diffX+amplitude diffX+amplitude]-state.acq.roiPhaseCorrection*state.internal.lineDelay;
ydata=[diffY diffY+amplitude diffY+amplitude diffY];

% update object....
obj=findobj(state.internal.roiaxis,'type','patch','Tag','currentROI');
if isempty(obj)
    obj=patch(xdata,ydata,[1 0 0],'EdgeColor','blue',...
        'FaceColor','none','LineWidth',2,'Tag','currentROI','HitTest','off','Parent',state.internal.roiaxis);
else
    set(obj,'XData',xdata,'YData',ydata);
end
angle=mockROI(3)-now(3);
if angle~=0
    rotate(obj,[0 0 1],angle);
end
set(obj,'ZData',[0 0 0 0]);
children=get(state.internal.roiaxis,'Children');
index=find(children==state.internal.roiimage);
index2=find(children==obj);
children([index index2])=[];
set(state.internal.roiaxis,'Children',[obj children' state.internal.roiimage]);
if amplitude < 1
    axis(state.internal.roiaxis,'tight');
else
    xrange=.05*(max(xdata)-min(xdata));
    yrange=.05*(max(ydata)-min(ydata));
    try
        set(state.internal.roiaxis,'XLim',[min(xdata)-xrange max(xdata)+xrange],'YLim',...
            [min(ydata)-yrange max(ydata)+yrange]);
    end
end