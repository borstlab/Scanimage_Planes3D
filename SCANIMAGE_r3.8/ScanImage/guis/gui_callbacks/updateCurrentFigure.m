function updateCurrentFigure
% updateCurrentFigure.m****
% Function that records position of image figures
%
%% CHANGES
%   VI030609A: Store the merge figure now as well -- Vijay Iyer 3/6/09
%
%% CREDITS
% Written By: Thomas Pologruto
% Cold Spring Harbor Labs
% March 2, 2001
%% *****************************************************

global state

for channelCounter = 1:state.init.maximumNumberOfInputChannels
    position = get(state.internal.GraphFigure(channelCounter), 'Position');
    eval(['state.internal.figurePositionX' num2str(channelCounter) ' = position(1,1);']);
    eval(['state.internal.figurePositionY' num2str(channelCounter) '= position(1,2);']);
    eval(['state.internal.figureWidth' num2str(channelCounter) ' = position(1,3);']);
    eval(['state.internal.figureHeight' num2str(channelCounter) ' = position(1,4);']);
    position = get(state.internal.MaxFigure(channelCounter), 'Position');
    eval(['state.internal.maxfigurePositionX' num2str(channelCounter) ' = position(1,1);']);
    eval(['state.internal.maxfigurePositionY' num2str(channelCounter) '= position(1,2);']);
    eval(['state.internal.maxfigureWidth' num2str(channelCounter) ' = position(1,3);']);
    eval(['state.internal.maxfigureHeight' num2str(channelCounter) ' = position(1,4);']);
end

%%%VI030609A%%%%%%%%%%
position = get(ancestor(state.internal.mergeimage,'figure'),'Position');
eval(['state.internal.mergefigurePositionX' '= position(1,1);']);
eval(['state.internal.mergefigurePositionY' '= position(1,2);']);
eval(['state.internal.mergefigureWidth' '= position(1,3);']);
eval(['state.internal.mergefigureHeight' ' = position(1,4);']);
%%%%%%%%%%%%%%%%%%%%%%

% %TPMODPockels
% roipos=[state.internal.roifigurePositionX state.internal.roifigurePositionY state.internal.roifigureWidth state.internal.roifigureHeight];
% pos=get(state.internal.roifigure,'Position');
% state.internal.roifigureVisible=get(state.internal.roifigure,'Visible');
% state.internal.roifigurePositionX=pos(1,1);
% state.internal.roifigurePositionY=pos(1,2);
% state.internal.roifigureWidth=pos(1,3);
% state.internal.roifigureHeight=pos(1,4);

%Handle 'new' ROI figure
% roiNewpos=[state.internal.roifigureNewPositionX state.internal.roifigureNewPositionY state.internal.roifigureNewWidth state.internal.roifigureNewHeight];
% pos=get(state.internal.roifigureNew,'Position');
% state.internal.roifigureNewVisible=get(state.internal.roifigureNew,'Visible');
% state.internal.roifigureNewPositionX=pos(1,1);
% state.internal.roifigureNewPositionY=pos(1,2);
% state.internal.roifigureNewWidth=pos(1,3);
% state.internal.roifigureNewHeight=pos(1,4);
