function toggleCheckMark(menuItem)

if strcmpi(get(menuItem, 'Checked'), 'On')
    set(menuItem, 'Checked', 'Off');
else
    set(menuItem, 'Checked', 'On');
end

return;