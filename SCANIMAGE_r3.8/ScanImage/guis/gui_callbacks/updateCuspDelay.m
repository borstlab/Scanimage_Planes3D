function updateCuspDelay(handle)
%% function updateCuspDelay(handle)
% Callback function for changes to cusp (Servo) delay parameter
%
%% NOTES
%   As of 12/11/08, this function is UNUSED -- servo delay is 'clamped' passively in the makeStrip/makeFrameByStripes functions
%
%   For now, we maintain the restriction that the acquired data for a line must fall within its line period, limiting the maximum allowable 
%   cusp delay
%   
%   Following no longer applies as of VI121008A:
%   This function flags two thresholds that are crossed as the servo delay is increased. At the first threshold, the image is guaranteed to be shifted
%   relative to the specified offset and there will be 'reflections' on one side of the image (i.e. it is impossible to eliminate, so don't keep trying so hard).
%   At the second threshold, the acquisition window exceeds the line period. This is not presently allowed, so the message is fsdafdsafds
%% CHANGES
%   VI121008A: Change function to clamp cusp delay at max allowable value
%% CREDITS
%   Created 9/21/08 by Vijay Iyer
%% ***************************

global state gh

%%%VI121008A%%%%%%%%%
maxCusp = calcMaxServoDelay();
        
if state.acq.cuspDelay > maxCusp
    state.acq.cuspDelay = maxCusp;
    updateGUIByGlobal('state.acq.cuspDelay');
end
%%%%%%%%%%%%%%%%%%%%%

% %Update any text controls bound to cusp delay to color-code the current state
% handles = getGuiOfGlobal('state.acq.cuspDelay');
% for i=1:length(handles)
%     handle = eval(handles{i});
%     if strcmpi(get(handle,'Style'),'edit')
%         if ~state.acq.bidirectionalScan
%             if state.acq.cuspDelay <= 1 - state.acq.fillFraction - 2*state.internal.lineDelay %all good--white
%                 set(handle,'BackgroundColor',[1 1 1]);
%             elseif state.acq.cuspDelay  > 1 - state.acq.fillFraction - state.internal.lineDelay %red zone
%                 set(handle,'BackgroundColor',[1 0 0]);
%             else %warning: yellow
%                 set(handle,'BackgroundColor',[1 1 0]);
%             end
%         else
%             if state.acq.cuspDelay <= state.internal.lineDelay %all good--white
%                 set(handle,'BackgroundColor',[1 1 1]);
%             else 
%                 set(handle,'BackgroundColor',[1 0 0]);
%             end
%         end
%     end
% end
