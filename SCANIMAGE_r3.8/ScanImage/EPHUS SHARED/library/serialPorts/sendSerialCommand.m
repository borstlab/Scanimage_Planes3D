% ------------------------------------------------------------------
function sendSerialCommand(serialObj, command)

serialControlStruct = get(serialObj, 'UserData');
if isempty(serialControlStruct)
    serialControlStruct = makeSerialControlStruct;
end

serialControlStruct.lastCommandTime = now;
serialControlStruct.lastCommand = command;

fprintf(serialObj, command);

%No timeout has occurred, yet.
serialControlStruct.timeOutOccurred = logical(0);

return;