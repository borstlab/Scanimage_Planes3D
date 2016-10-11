function scim_PFileRefresh()
%% function scim_PFileRefresh()
% Creates pcode for all functions for which this is useful/needed (i.e. in callbacks)

functionList = {'makeFrameByStripes' 'endAcquisition' 'abortCurrent' 'setStatusString' 'abortInActionFunction' 'closeShutter' 'openShutter' 'stopGrab' 'writeData' 'motorAction'};
functionList = [functionList {'motorStartMove' 'motorFinishMove' 'motorGoHome'}];

for i=1:length(functionList)
    pcode(which(functionList{i}),'-inplace');       
end



end

