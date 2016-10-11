function [mirrorData,aiFrames,pockelsData]=makeAODataFromCycle(cycle)
% This function will configure the DAQ engine in 
% ScanImage to output the correct data
% for a cycle in one scan form.
global state

mirrorData=[];
pockelsData=[];
numberOfPositions=size(cycle,1);
aiFrames=sum(cycle(:,4));

for posCounter=1:numberOfPositions
    thisPosition=cycle(posCounter,:);
    roi=thisPosition(3);
    frames=thisPosition(4);
    power=thisPosition(5);
    mirrorData=[mirrorData; makeMirrorDataLocal(frames,roi)];
    if state.init.eom.pockelsOn %VI011609A
        pockelsData=[pockelsData; makePockelsDataLocal(frames,power)];
    end
end

function out=makeMirrorDataLocal(frames,roi)
% Makes AO data based on the roi and the number of frames.

function out=makePockelsDataLocal(frames,power)
% Makes Pockels data based on the roi and the number of frames.