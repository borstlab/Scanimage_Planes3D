function updateShutterDelay
%Updates the vector witht he frame nad strip after which to open the shutter...

global state

ns=state.internal.numberOfStripes;
lpf=state.acq.linesPerFrame;
mspl=state.acq.msPerLine; %VI012109A
nf=state.acq.numberOfFrames;

mspstripe=mspl*lpf/ns;
stripeNumber=state.shutter.shutterDelay/mspstripe;
if ns == 1 %VI113009A: Use round() instead of floor(), if numberOfStripes==1
    frameNumber=round(stripeNumber/ns)+1; %VI113009A: Use round() instead of floor()
else
    frameNumber=floor(stripeNumber/ns)+1;
end
    
%Correct for inherent 2 stripe delay at moment that makeFrameByStripes() reads these values. 2 stripe delay is because function is not generally reached first time until 2 buffers-full have been collected.
if ns == 1 %VI113009B:Handle stripe-less case separately
    stripeInFrame = 0;
    frameNumber = max(frameNumber-2,0);
else
    stripeInFrame=round(stripeNumber - (frameNumber-1)*ns); %VI113009A: Subtract off stripes used to reach current frame, rather than rem(stripeNumber,ns); use round() instead of ceil(); handle case where value rounds to zero with max()

    if stripeInFrame > 1  && frameNumber > 1 %VI113009B: Use stripeInFrame > 1, not > 2
        stripeInFrame=stripeInFrame-2;
    elseif stripeInFrame == 1 && frameNumber > 1 %VI113009B: Use stripeInFrame == 1 , not < 2
        stripeInFrame=ns-1;
        frameNumber=frameNumber-1;
    %%%VI11309B%%%%
    elseif stripeInFrame == 0 && frameNumber > 1
        stripeInFrame=ns-2;
        frameNumber=frameNumber-1;
    elseif frameNumber == 1
        stripeInFrame = max(stripeInFrame-2,0);
    end
    %%%VI113009B: Removed%%%%%
    % elseif stripeInFrame < 2 & frameNumber == 1
    %     stripeInFrame=stripeInFrame+1;
    % end
    %%%%%%%%%%%%%%%%%%%%%%%%%%
end

state.shutter.shutterDelayVector=([frameNumber stripeInFrame]);
