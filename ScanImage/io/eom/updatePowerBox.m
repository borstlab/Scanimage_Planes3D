%Changed - 2/26/04 Tim O'Connor TO22604a: Allow loading of powerboxes from config.
%          3/5/04 Tim O'Connor TO3504a: Something funky's happening, try catch for now...
%          10/22/09 Vijay Iyer VI102209A: Use state.internal.storedLinesPerFrame where appropriate
function updatePowerBox
% will make sure pockels cell boxes are correctly displayed regardless of 
% configuration....
global state gh

if isempty(state.init.eom.boxHandles)
    return
elseif isempty(state.init.eom.boxHandles(ishandle(state.init.eom.boxHandles)))
    return
end

%Just clear the thing.
state.init.eom.boxHandles = state.init.eom.boxHandles(find(state.init.eom.boxHandles ~= 0));
for i = 1 : length(state.init.eom.boxHandles)
    if ishandle(state.init.eom.boxHandles)
        delete(state.init.eom.boxHandles);
    end
end

return;

try
recth=state.init.eom.boxHandles(ishandle(state.init.eom.boxHandles));
%TO22604a
if ~ishandle(recth) | isempty(recth)
    fprintf(1, 'WARNING: Attempt to update powerbox, when no powerbox handle exists. Creating handle...\n');

    %Go over all beams.
    for i = 1 : state.init.eom.numberOfBeams
        %If there should be a powerbox, create one.
        if state.init.eom.showBoxArray(i)
            %Do it for each input channel.
            for j = 1 : state.init.maximumNumberOfInputChannels
                state.init.eom.boxHandles(i, j) = rectangle('Position', pos, 'FaceColor', 'none', ...    
                    'EdgeColor', state.init.eom.boxcolors(i, :), 'LineWidth', 3, 'Parent', state.internal.axis(j), ...
                    'ButtonDownFcn', 'powerBoxButtonDownFcn', 'UserData', i, ...
                    'Tag', sprintf('PowerBox%s', num2str(i)));
            end
        end
    end
    
    recth=state.init.eom.boxHandles(ishandle(state.init.eom.boxHandles));
end

imsize=[state.acq.pixelsPerLine  state.internal.storedLinesPerFrame]; %VI102209A
xcoords=state.init.eom.powerBoxNormCoords([1 3]).*imsize(1);
ycoords=state.init.eom.powerBoxNormCoords([2 4]).*imsize(2);
rectPos=[xcoords(1) ycoords(1) xcoords(2) ycoords(2)];
set(recth,'Position',rectPos);
catch
    %As a quick-fix, try-catch this, for now. Tim O'Connor 3/5/04 (TO3504a)
    state.init.eom.boxHandles = [];
    state.init.eom.showBoxArray(:) = 0;
    updateGUIByGlobal('state.init.eom.showBox', 'Value', 0, 'Callback', 1);
    lasterr
end