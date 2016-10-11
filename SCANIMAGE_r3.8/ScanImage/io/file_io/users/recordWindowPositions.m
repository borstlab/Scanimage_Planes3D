function recordWindowPositions
global state gh
wins=state.internal.guinames;
for winCount=1:length(wins)
    winName=wins{winCount};
    h=getfield(getfield(gh, winName), 'figure1');
    if ishandle(h)
        pos=get(h, 'Position');
        %VI020609A: Store dialog box GUI visibilities as 'off'
        if strcmpi(get(h,'WindowStyle'),'modal')
            vis = 'off';
        else
            vis=get(h,'Visible');
        end            
        state.internal=setfield(state.internal, [winName 'Bottom'], pos(2));
        state.internal=setfield(state.internal, [winName 'Left'], pos(1));
        state.internal=setfield(state.internal, [winName 'Visible'], vis);
    end
end
updateCurrentFigure;



