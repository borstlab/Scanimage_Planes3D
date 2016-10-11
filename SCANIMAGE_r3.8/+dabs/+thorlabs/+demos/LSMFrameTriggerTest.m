function hLSM = LSMFrameTriggerTest()
%LSMFRAMETRIGGERTEST Summary of this function goes here
%   Detailed explanation goes here

mlock();

persistent hLSMStore

if isempty(hLSMStore) || ~isvalid(hLSMStore)
    hLSMStore = dabs.thorlabs.LSM();
end

hLSM = hLSMStore;


%Constants
numTrials = 5;
interTrialInterval = 0;

hLSM.triggerMode = 'SW_SINGLE_FRAME';
hLSM.pixelsPerDim = 512;


hLSM.arm(); %calls preflight acquisition & setupAcquisition
for i=1:numTrials
    if i==1
        hLSM.start(); %calls startAcquisiton()
    else
        hLSM.resume(); %calls startAcquisition()
    end
    
    while true
        if hLSM.frameCount >= 1
            break;
        end
        pause(0.01);
    end    
    
    hLSM.pause();
    pause(interTrialInterval);
end

hLSM.stop();




end

