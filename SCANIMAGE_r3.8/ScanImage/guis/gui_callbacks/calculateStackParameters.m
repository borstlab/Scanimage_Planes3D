function calculateStackParameters(lockNumSlices)
global state gh

%TODO: Determine if stack boundary mode can be used with Cycle mode on?
% if state.cycle.cycleOn
%     disp('*** Stack boundry mode only works with Standard Mode ***');
%     return;
% end

if nargin < 1
    lockNumSlices = false;
end

step=abs(state.acq.zStepSize); 
if ~isempty(state.motor.stackStart) 
    if state.motor.dimensionsXYZZ && state.motor.motorZEnable  %VI051211A 
        zStart = state.motor.stackStart(4);
    else
        zStart=state.motor.stackStart(3);
    end
else
    disp('*** Stack starting position not defined.');
    return
end

if ~isempty(state.motor.stackStop)
    if state.motor.dimensionsXYZZ && state.motor.motorZEnable  %VI051211A
        zStop=state.motor.stackStop(4);
    else
        zStop=state.motor.stackStop(3);
    end
else
    disp('*** Stack ending position not defined.');
    return
end

if zStart < zStop
    newStepVal = step; %VI010611A
else
    newStepVal = -step; %VI010611A
end
%state.acq.zStepSize=state.acq.zStepPerSlice; %VI010611A
%updateGUIByGlobal('state.internal.zStepPerSlice'); %VI010611A

if newStepVal ~= state.acq.zStepSize
    state.acq.zStepSize = newStepVal;
    updateGUIByGlobal('state.acq.zStepSize');
    updateHeaderString('state.acq.zStepSize');
end

if lockNumSlices
    state.acq.zStepSize = sign(state.acq.zStepSize) * abs(zStop-zStart) / state.acq.numberOfZSlices;
    updateGUIByGlobal('state.acq.zStepSize');    
else
    state.acq.numberOfZSlices=max(ceil(abs(zStop-zStart)/step),1); %VI010611A
    %DEQ20110105state.acq.numberOfZSlices=state.internal.numberOfZSlices;
    %updateGUIByGlobal('state.internal.numberOfZSlices'); %VI010611A
    updateGUIByGlobal('state.acq.numberOfZSlices');
end

updateAcquisitionSize(lockNumSlices); %VI07010A
%preallocateMemory; %VI070110A: Removed -- this is called from updateAcquisitionSize()




