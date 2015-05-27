function showCalibrationCurve(varargin)
%% CHANGES
%   VI110708A: Cache Pockels calibration figure handles, so they can be deleted programmatically if needed -- Vijay Iyer 11/07/08
%% ****************************************

global gh state;

if length(varargin) < 1
    beam = get(gh.powerControl.beamMenu, 'Value');
elseif length(varargin) > 1
    for i=1:length(varargin)
        showCalibrationCurve(varargin{i});
    end
else
    beam = varargin{1};
end

chart_title = sprintf('Look Up Table (Pockels Cell %s)', num2str(beam));

f=figure('NumberTitle', 'off', 'DoubleBuffer', 'On', 'Name', chart_title, 'Color', 'White');
a=axes('Parent',f);

plot(state.init.eom.lut(beam, 1:100), 1:100, 'Marker', 'o', 'MarkerSize', 2, 'LineStyle', 'none', 'Parent', a, 'MarkerFaceColor', [0 0 0], 'color', [0 0 0]);

title(chart_title, 'FontSize', 12, 'FontWeight', 'Bold','Parent',a);

ylabel(sprintf('Percent of Maximum Power (%s mW max)', num2str(round( ...
    getfield(state.init.eom,['powerConversion' num2str(beam)]) * state.init.eom.maxPhotodiodeVoltage(beam) ...
    ))), 'Parent', a, 'FontWeight', 'bold');
xlabel('Modulation Voltage [V]', 'Parent', a, 'FontWeight', 'bold');

state.internal.figHandles = [f state.internal.figHandles]; %VI110708A

return;