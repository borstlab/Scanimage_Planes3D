%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Resize/move powerbox graphic.
%%
%%  Changed:
%%    Added in the constrain to line feature.
%%    Updated to force it to overlap only 1 line on the image. -- Tim O'Connor 6/16/04 TO061604a
%%    Added in a half pixel correction (positive in X, negative in Y), to get the box to directly overlap the signal. -- TimO'Connor 6/16/04 - TO061604b
%%    VI012109A: msPerLine is now actually in milliseconds -- Vijay Iyer 1/21/09
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function powerBoxButtonDownFcn
global state gh
%TPMOD 8/6/03
% TPMODBox
if ~strcmp(get(gco,'Type'),'rectangle')
    return
else
    rect=gco;
    beam=get(gco,'UserData');
end

pos=get(rect,'Position');   % current position....

upperLeft=[pos(1) pos(2)];   % position of upper left corner...
bufferSize=.1;     % fraction away from corner that function is executed....
axpoint=get(gca,'CurrentPoint');    % current axes point....
axpoint=axpoint(1,1:2);           % in 2D only ....

% Did they grab the corect corner...
if sqrt(sum((axpoint-upperLeft).^2)) > bufferSize*max(pos(3),pos(4))
    return
end

% See if we should stretch it or shift it...
doShift=0;
doStretch=0;
if strcmp(get(gcf,'SelectionType'),'normal')
    doShift=1;
else
    doStretch=1;
end

figpoint = get(gcf,'CurrentPoint'); % current FIGURE point....
figpoint=figpoint(1,1:2);           % in 2D only ....
offset=abs(figpoint-axpoint);
% correct for figure scaling and such...
figsize=get(gcf,'Position');
imsize=[state.acq.pixelsPerLine  state.acq.linesPerFrame];
xmag=figsize(3)./imsize(1);
ymag=figsize(4)./imsize(2);
figFractionX=figpoint(1)/figsize(3);
figFractionY=figpoint(2)/figsize(4);

if doShift
    finalRect = dragrect([figpoint(1) figpoint(2)-ymag*pos(4) xmag*pos(3) ymag*pos(4)]);
else    
    finalRect = rbbox([figpoint(1) figpoint(2)-ymag*pos(4) xmag*pos(3) ymag*pos(4)]);
end

%Constrain to a single line, for uncaging.
if state.init.eom.constrainBoxToLine(state.init.eom.beamMenu) & abs(pos(4)) ~= 1
    if finalRect(4) < 0
        finalRect(4) = -.5;
    elseif finalRect(4) > 0
        finalRect(4) = .5;
    end
    
    %%Updated to force it to overlap only 1 line on the image. -- Tim O'Connor 6/16/04 TO061604a
    finalRect = round(finalRect);
end

% return figure units
newpoint = get(gca,'CurrentPoint');
xpoint=newpoint(1,1);
ypoint=newpoint(1,2);

%Updated to force it to overlap only 1 line on the image. -- Tim O'Connor 6/16/04 TO061604a
if state.init.eom.constrainBoxToLine(state.init.eom.beamMenu)
    xpoint = round(xpoint);
    ypoint = round(ypoint);
end

if xpoint < 1 | xpoint > state.acq.pixelsPerLine | ypoint < 1 | ...
        ypoint > state.acq.linesPerFrame
    return
else
    width=finalRect(3)./xmag;
    %Constrain the box to be a horizontal line.
    if state.init.eom.constrainBoxToLine(state.init.eom.beamMenu)
        height = 1;
    else
        height=finalRect(4)./ymag;
    end
    state.init.eom.powerBoxNormCoords(beam,:)=[xpoint ypoint width height];
    state.init.eom.powerBoxNormCoords(beam,[1 3])=state.init.eom.powerBoxNormCoords(beam,[1 3])./imsize(1);
    state.init.eom.powerBoxNormCoords(beam,[2 4])=state.init.eom.powerBoxNormCoords(beam,[2 4])./imsize(2);
    %Makes sure edges are ok....
    state.init.eom.powerBoxNormCoords(beam,1)=max(state.init.eom.powerBoxNormCoords(beam,1),.001);
    state.init.eom.powerBoxNormCoords(beam,3)=min(state.init.eom.powerBoxNormCoords(beam,3),1-state.init.eom.powerBoxNormCoords(beam,1));
    state.init.eom.powerBoxNormCoords(beam,2)=max(state.init.eom.powerBoxNormCoords(beam,2),.001);
    state.init.eom.powerBoxNormCoords(beam,4)=min(state.init.eom.powerBoxNormCoords(beam,4),1-state.init.eom.powerBoxNormCoords(beam,2));
    
    %Added in a half pixel correction (positive in X, negative in Y), to get the box to directly overlap the signal. -- TimO'Connor 6/16/04 - TO061604b
    set(state.init.eom.boxHandles(beam,:),'Position', [state.init.eom.powerBoxNormCoords(beam,1).*imsize(1)+.5 state.init.eom.powerBoxNormCoords(beam,2).*imsize(2)-.5...
            state.init.eom.powerBoxNormCoords(beam,3).*imsize(1) state.init.eom.powerBoxNormCoords(beam,4).*imsize(2)]);
    end

state.init.eom.powerBoxNormCoordsString=mat2str(state.init.eom.powerBoxNormCoords);
state.init.eom.changed(beam)=1;

updateGUIByGlobal('state.init.eom.boxWidth', 'Value', round(100 * state.init.eom.powerBoxNormCoords(beam, 3) * imsize(1) * state.acq.msPerLine / state.acq.pixelsPerLine ) / 100, 'Callback', 0); %VI012109A