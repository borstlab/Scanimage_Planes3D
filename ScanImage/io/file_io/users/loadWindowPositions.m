function loadWindowPositions
global state gh

wins=fieldnames(gh);

for winCount=1:length(wins)
    winName=wins{winCount};
    if isfield(state.internal, [winName 'Bottom']) && isfield(state.internal, [winName 'Left'])
        pos=get(getfield(getfield(gh, winName), 'figure1'), 'Position');
        if ~isempty(getfield(state.internal, [winName 'Left'])) %TPMOD
            pos(1)=getfield(state.internal, [winName 'Left']);
            pos(2)=getfield(state.internal, [winName 'Bottom']);
            set(getfield(getfield(gh, winName), 'figure1'), 'Position', pos);
        end
        if isfield(state.internal, [winName 'Visible']) && ~isempty(state.internal.(sprintf('%sVisible',winName)))
             set(getfield(getfield(gh, winName), 'figure1'), 'Visible', getfield(state.internal, [winName 'Visible']));
        end
    end
end

