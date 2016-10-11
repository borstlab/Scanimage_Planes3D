function logicalVal = si_isAcquiring()
%SI_ISACQUIRING Returns true if currently acquiring (FOCUS/GRAB/LOOP/SNAPSHOT)

global gh

controls = [gh.mainControls.focusButton gh.mainControls.grabOneButton gh.mainControls.startLoopButton];
logicalVal = any(arrayfun(@(control)strcmpi(get(control,'String'),'Abort'),controls));