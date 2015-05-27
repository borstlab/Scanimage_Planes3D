function abortROICycle(reset)
global state gh

%TO022508A - Make this tolerant to missing ini file field(s).
if isfield(state.init, 'roiCycle')
    if state.init.roiCycle.zombieMode
        state.init.roiCycle.zombieMode = 0;
        lsps2p('mapButton_Callback', gh.lsps2p.mapButton, [], guidata(gh.lsps2p.mapButton));
        return;
    end
end

stopGrab(true); %VI090309A: Identify as abort operation
if nargin < 1
    reset=1;
end
state.internal.roiCycleExecuting=0;
h=gh.mainControls.grabOneButton;
state.internal.abortActionFunctions=1;
closeShutter;
setStatusString('Aborting...');
set(h, 'String', 'GRAB');
turnOnMenus;
set([gh.mainControls.focusButton gh.mainControls.startLoopButton], 'Visible', 'On');
setStatusString('');
set(gh.roiCycleGUI.startROICycle, 'String','GO','ForegroundColor',[0 .6 0]);
state.roiCycle.firstTimeThroughLoop=1;
if reset
    roiCycleGUI('resetROICycle_Callback',gh.roiCycleGUI.resetROICycle);
end
setStatusString('Ready...');
flushAOData;