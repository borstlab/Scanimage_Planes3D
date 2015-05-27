if strcmpi(input('Would you like to clear all variables and guis in the workspace? (y/N): ', 's'), 'y')
    if ~isempty(which('daqjob'))
        fprintf(1, 'Clearing known daqjob instances (''acquisition'', ''scope'').\n');
        delete(daqjob('acquisition'));
        delete(daqjob('scope'));
    end
    if ~isempty(which('getUserFcnCBM'))
        fprintf(1, 'Clearing userFunction callbackManager.\n');
        delete(getUserFcnCBM);
    end
    if ~isempty(which('daqfind'))
        daqobjs = daqfind;
        if ~isempty(daqobjs)
            try
                fprintf(1, '\nStopping all data acquisition objects...\n');
                stop(daqobjs);
            catch
                fprintf(1, 'Failed to properly stop all data acquisition objects.\n');
            end
            try
                fprintf(1, 'Deleting all data acquisition objects...\n');
                delete(daqobjs);
            catch
                fprintf(1, 'Failed to properly delete all data acquisition objects.\n');
            end
        end
    end
    if ~isempty(which('instrfind'))
        instrobjs = instrfind;
        if ~isempty(instrobjs)
            try
                fprintf(1, '\nClosing all serial objects...\n');
                fclose(instrobjs);
            catch
                fprintf(1, 'Failed to properly close all serial objects.\n');
            end
            try
                fprintf(1, 'Deleting all serial objects...\n');
                delete(instrobjs);
            catch
                fprintf(1, 'Failed to properly delete all serial objects.\n');
            end
        end
    end
%     fprintf(1, 'Clearing ''all''...\n');
%     clear all;
%     fprintf(1, 'Clearing ''global''...\n');
%     clear global;
%     fprintf(1, 'Clearing ''mex''...\n');
%     clear mex;
    fprintf(1, 'Deleting all children of handle 0...\n');
    delete(allchild(0));
    fprintf(1, 'Clearing ''classes''...\n');
    clear classes;
%     fprintf(1, 'Re-clearing ''all''...\n');
%     clear all;
%     fprintf(1, 'Re-clearing ''classes''...\n');
%     clear classes;
    %fprintf(1, 'Defragmenting heap (via `pack`)...\n');
    %pack('clearWorkspace.heap_defragmentation');
    fprintf(1, 'Workspace cleanup completed.\n\n\n\n\n');    
end