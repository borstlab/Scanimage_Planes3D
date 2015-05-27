% ------------------------------------------------------------------
function response = getSerialResponse(serialObj)

serialControlStruct = get(serialObj, 'UserData');
if isempty(serialControlStruct)
    serialControlStruct = makeSerialControlStruct;
end

response = fscanf(serialObj, '%c');

%Trim the response, discard previous responses that my have arrived in the
%buffer late.
if ~isempty(response)
    terminators = get(serialObj, 'Terminator');
    lineEnds = find(response == terminators{1});
    
    if length(lineEnds) > 1
        response = response(lineEnds(end - 1) + 1 : lineEnds(end) - 1);
    end
    
    response = deblank(response);
end

if serialControlStruct.timeOutOccurred | isempty(response)
    %Issue a warning.
    fprintf(1, '%s - maiTaiController/getResponse - Warning: timeout occurred for command ''%s''.\n', ...
        datestr(datevec(serialControlStruct.timeOutOccurred), 0), serialControlStruct.lastCommand));
    
    %Return an empty string.
    response = '';
    
    %Clear the buffer.
    fscanf(serialObj, '%c')
end

%Update the display.
serialControlStruct.lastResponse = response;

%No timeout has occurred, yet.
serialControlStruct.timeOutOccurred = logical(0);

return;