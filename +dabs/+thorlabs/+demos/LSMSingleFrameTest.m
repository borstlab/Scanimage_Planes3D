function LSMSingleFrameTest

global LSMTest;
            
[status lastFrameIdx] = LSMTest.hLSM.statusAcquisitionEx();

disp(['lastFrameIdx = ' num2str(lastFrameIdx)]);

figH = figure(1);
axis(LSMTest.axes1Hnd, 'off');
colormap(LSMTest.axes1Hnd, 'gray');
figH = figure(2);
axis(LSMTest.axes2Hnd, 'off');
colormap(LSMTest.axes2Hnd, 'gray');


LSMTest.hLSM.triggerMode =  'SW_SINGLE_FRAME';  %'SW_FREE_RUN_MODE', 'SW_MULTI_FRAME'
%LSMTest.hLSM.frameAcquiredEventFcn = [];
LSMTest.hLSM.frameAcquiredEventFcn = @dabs.thorlabs.Demos.frameAcquiredCallback;
%LSMTest.rawLogEnable = true;
%LSMTest.hLSM.setupAcquisition();
tic;
LSMTest.hLSM.arm();
LSMTest.hLSM.start();


end


