%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  Sets up the uncagingPulseImporter tool. Mainly just sizes the window
%%  for the correct number of beams and finds default values.
%%
%%  Created - Tim O'Connor 12/18/03
%%
%%  CHANGES
%       VI012109A: msPerLine is now actually in milliseconds -- Vijay Iyer 1/21/09
%
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function initializeUncagingPulseImporter
global state gh;

framePos = get(gh.uncagingPulseImporter.cyclerFrame, 'Position');
syncBoxPos = get(gh.uncagingPulseImporter.syncToPhysiologyCheckbox, 'Position');

%Resize things, hide/unhide, and enable/disable gui elements.
if state.init.eom.numberOfBeams == 1
    %Reposition.
    framePos(3) = 28.4;
    syncBoxPos(1) = 51.8;

    %Hide/show elements.
    set(gh.uncagingPulseImporter.beamLeftSlider1, 'Visible', 'Off');
    set(gh.uncagingPulseImporter.beamLeftSlider1, 'Enable', 'Off');

    set(gh.uncagingPulseImporter.beamRightSlider1, 'Enable', 'Off');
    set(gh.uncagingPulseImporter.beamRightSlider1, 'Visible', 'Off');

    set(gh.uncagingPulseImporter.beamLeftSlider2, 'Visible', 'Off');
    set(gh.uncagingPulseImporter.beamLeftSlider2, 'Enable', 'Off');

    set(gh.uncagingPulseImporter.beamRightSlider2, 'Visible', 'Off');
    set(gh.uncagingPulseImporter.beamRightSlider2, 'Enable', 'Off');

    set(gh.uncagingPulseImporter.beamText2, 'Visible', 'Off');
    set(gh.uncagingPulseImporter.beamText2, 'Enable', 'Off');
    
    set(gh.uncagingPulseImporter.beamLabel2, 'Visible', 'Off')

    set(gh.uncagingPulseImporter.cycleValueText2, 'Visible', 'Off');
    set(gh.uncagingPulseImporter.cycleValueText2, 'Enable', 'Off');
else
    %Reposition.
    framePos(3) = 37.4;
    syncBoxPos(1) = 64.4;
    
    %Hide/show elements.
    if state.init.eom.numberOfBeams > 2
        set(gh.uncagingPulseImporter.beamLeftSlider1, 'Visible', 'On');
        set(gh.uncagingPulseImporter.beamLeftSlider1, 'Enable', 'On');
        
        set(gh.uncagingPulseImporter.beamRightSlider1, 'Enable', 'Off');
        set(gh.uncagingPulseImporter.beamRightSlider1, 'Visible', 'Off');
        
        set(gh.uncagingPulseImporter.beamLeftSlider2, 'Visible', 'Off');
        set(gh.uncagingPulseImporter.beamLeftSlider2, 'Enable', 'Off');
        
        set(gh.uncagingPulseImporter.beamRightSlider2, 'Visible', 'On');
        set(gh.uncagingPulseImporter.beamRightSlider2, 'Enable', 'On');
    end

    set(gh.uncagingPulseImporter.beamText2, 'Visible', 'On');
    set(gh.uncagingPulseImporter.beamText2, 'Enable', 'On');

    set(gh.uncagingPulseImporter.beamLabel2, 'Visible', 'On')
    
    set(gh.uncagingPulseImporter.cycleValueText2, 'Visible', 'On');
    set(gh.uncagingPulseImporter.cycleValueText2, 'Enable', 'On');
end

%Reposition.
set(gh.uncagingPulseImporter.cyclerFrame, 'Position', framePos);
set(gh.uncagingPulseImporter.syncToPhysiologyCheckbox, 'Position', syncBoxPos);

%Load the default array.
% if ~isempty(state.init.eom.uncagingPulseImporter.cycleArrayString)
%     state.init.eom.uncagingPulseImporter.cycleArray = str2num(state.init.eom.uncagingPulseImporter.cycleArrayString);
% else
if length(state.init.eom.uncagingPulseImporter.cycleArray) < state.init.eom.numberOfBeams %VI013009A
    state.init.eom.uncagingPulseImporter.cycleArray = zeros(state.init.eom.numberOfBeams, 1);
end

%Pick up the default values for the conversion factors.
state.init.eom.uncagingPulseImporter.powerConversionFactor = 1;
state.init.eom.uncagingPulseImporter.lineConversionFactor =  state.acq.msPerLine; %VI012109A

updateGUIByGlobal('state.init.eom.uncagingPulseImporter.powerConversionFactor');
updateGUIByGlobal('state.init.eom.uncagingPulseImporter.lineConversionFactor');

%Keep this unavailable until a pulse set is chosen.
set(gh.uncagingPulseImporter.expandWindowButton, 'Enable', 'Off');

%Enable disable a few buttons.
uncagingPulseImporter('pathnameText_Callback', gh.uncagingPulseImporter.pathnameText);
    
%Set text color and print warning, if necessary.
uncagingPulseImporter('lineConversionFactorText_Callback', gh.uncagingPulseImporter.lineConversionFactorText);