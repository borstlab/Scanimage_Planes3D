% @daqmanager/captureDaq - Write daqdevice information to a file handle.
%
% SYNTAX
%  captureDaq(f, daq)
%   f - A valid, writable file handle.
%   daq - A daqdevice object.
%
% USAGE
%
% NOTES
%  Moved from a subfunction in startChannel to a private function in the flass.
%
% CHANGES
%  TO080606A: See @daqmanager/startChannel for changes prior to refactoring. -- Tim O'Connor 8/6/06
%
% Created 8/6/06 Tim O'Connor
% Copyright - Cold Spring Harbor Laboratories/Howard Hughes Medical Institute 2006
function captureDaq(f, daq)

for i = 1 : length(daq)
    fprintf('@daqdevice:\n');
    fnames = fieldnames(daq(i));
    for j = 1 : length(fnames)
        fprintf(f, '  %s: ', fnames{j});
        captureValue(f, get(daq(i), fnames{j}));        
    end
    
    for j = 1 : length(daq(i).Channel)
        fprintf(f, '\r\n CHANNEL:\r\n');
        fnames = fieldnames(daq(i).Channel);
        for k = 1 : length(fnames)
            fprintf(f, '    %s: ', fnames{k});
            captureValue(f, get(daq(i).Channel, fnames{k}));
        end
    end
    fprintf('\n');
end

return;