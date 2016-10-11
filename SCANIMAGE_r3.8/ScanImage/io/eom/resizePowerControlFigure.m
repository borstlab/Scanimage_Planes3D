function resizePowerControlFigure
%Hide/show toggle for the power box portion of the PowerControl window.global gh;
%% CHANGES
%   VI111108A: Toggle the 'Show Power Box' button every time this is accessed (via the menubar Show Power Box item)
%% **************************
global gh;

pwrCtrl = gh.powerControl;
%pos = get(get(gh.powerControl,'Parent'),'position');
pos = get(powerControl, 'position');

children = get(gh.powerControl.Settings, 'Children');
index = getPullDownMenuIndex(gh.powerControl.Settings, 'Show Power Box');
checked = get(children(index), 'Checked');

if strcmpi(checked, 'On')
    pos(3) = 45.4;
    set(powerControl, 'position', pos);
    set(children(index), 'Checked', 'Off');
    set(gh.powerControl.tbShowPowerBox,'Value',0, 'String','Power Box >>' ); %VI111108A
else
%     pos(3) = 70.2;
    pos(3) = 77.60000000000002;
    set(powerControl, 'position', pos);
    set(children(index), 'Checked', 'On');
    set(gh.powerControl.tbShowPowerBox,'Value',1,'String','Hide <<'); %VI111108A
end

return;