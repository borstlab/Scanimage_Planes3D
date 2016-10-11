function updateNumberOfZSlices(hObject)
%UPDATENUMBEROFZSLICES Shared handler for updates to # of Z slices

global state

if ~isempty(state.motor.stackStart) && ~isempty(state.motor.stackStop)
    if state.motor.stackEndpointsDominate %VI111108A
        calculateStackParameters(true);       
    end
end

updateExternallyTriggered();

end

