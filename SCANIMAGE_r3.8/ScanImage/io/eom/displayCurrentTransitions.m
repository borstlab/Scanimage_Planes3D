function displayCurrentTransitions
% This wil graphically display the data
% being sent by the eom if using transitions
%% CHANGES
%   VI110708A: Cache Pockels calibration figure handles, so they can be deleted programmatically if needed -- Vijay Iyer 11/07/08
%   VI012109A: msPerLine is now actually in milliseconds -- Vijay Iyer 1/21/09
%% **********************************

global state

if isempty(state.init.eom.powerTransitions.time) | isempty(state.init.eom.powerTransitions.power) | ...
        max(state.init.eom.powerTransitions.time(state.init.eom.powerTransitions.beamMenu,:))<0 |...
        max(state.init.eom.powerTransitions.power(state.init.eom.powerTransitions.beamMenu,:))<0
    beep;
    disp('No Transitions to display');
    return
end

currentTimes=state.init.eom.powerTransitions.time(state.init.eom.powerTransitions.beamMenu,:);
currentTimes=currentTimes(currentTimes>=0);
currentTimes(currentTimes==0)=1;
[currentTimes,indices]=sort(currentTimes);

currentPowers=state.init.eom.powerTransitions.power(state.init.eom.powerTransitions.beamMenu,:);
currentPowers=currentPowers(currentPowers>0);
currentPowers=currentPowers(indices);

maxTime=max(state.acq.msPerLine*state.acq.linesPerFrame*state.acq.numberOfFrames,... %VI012109A
    max(currentTimes));
timeData=0:maxTime;
currentTimes=[currentTimes maxTime+1];
powerData=state.init.eom.maxPower(state.init.eom.powerTransitions.beamMenu)*ones(1,length(timeData));
binaryBit=0;
for transCounter=1:length(currentTimes)-1
    if state.init.eom.powerTransitions.useBinaryTransitions
        if binaryBit
            currentpower=state.init.eom.maxPower(state.init.eom.powerTransitions.beamMenu);
        else
            currentpower=state.init.eom.min(state.init.eom.powerTransitions.beamMenu);
        end
        binaryBit=1-binaryBit;
        powerData(currentTimes(transCounter):currentTimes(transCounter+1))=currentpower;
    else
        powerData(currentTimes(transCounter):currentTimes(transCounter+1))=currentPowers(transCounter);
    end
end
f=figure('NumberTitle','off','DoubleBuffer','On','Name',['Transition Output for Beam #' num2str(state.init.eom.powerTransitions.beamMenu)],'Color','White');
a=axes('Parent',f);
plot(timeData,powerData, 'Marker', 'o','MarkerSize',2,'LineStyle','none','Parent',a,'MarkerFaceColor',[0 0 0],'color',[0 0 0]);
title(['Transition Output for Beam #' num2str(state.init.eom.powerTransitions.beamMenu)], 'FontSize', 12, 'FontWeight', 'Bold','Parent',a);
ylabel(sprintf('Percent of Maximum Power (%s mW max)', num2str(round( ...
    getfield(state.init.eom,['powerConversion' num2str(state.init.eom.powerTransitions.beamMenu)]) * state.init.eom.maxPhotodiodeVoltage(state.init.eom.powerTransitions.beamMenu) ...
    ))),'Parent',a,'FontWeight','bold');
xlabel('Time [msec]','Parent',a,'FontWeight','bold');
xlim(a,[0 maxTime]);

state.internal.figHandles = [f state.internal.figHandles]; %VI110708A

