function outVal = waitbarUpdate(fraction, wb,message)
%Used to update waitbar figure's fraction and display string during a multi-step operation, and to determine whether the waitbar is cancelled
%
% USAGE
%    outval = waitbarUpdate(fraction,wb,message)
%       fraction: number (from 0-1) indicating fraction complete at point of this call
%       wb: handle of waitbar object (created with waitbar())
%       message: string indicating message to display at point of this call
%       outval: 1 if cancelled, 0 otherwise
%
% CHANGES
%  TO091010A - Hacked a workaround for Matlab illegally relying on the waitbar's userdata (only in R2008b). -- Tim O'Connor 9/10/10
%
% Created
%  Vijay Iyer - ??/??/08
%
% Copyright
%  Cold Spring Harbor Laboratories/Howard Hughes Medical Institute/Janelia Farm Research Center 2008
try
    if ishandle(wb)
        waitbar(fraction, wb, message);
        figure(wb);
        outVal = 0;
        %     if isWaitbarCancelled(wb)
        %         delete(wb);
        %         evalin('caller','return');
        %     end
    else
        outVal = 1;
    end
catch
    %TO091010A
    if startsWith(version, '7.7') ...
            && startsWithIgnoreCase(lasterr, 'Error using ==> waitbar at 249') ...
            && endsWithIgnoreCase(lasterr, 'Improper arguments for waitbar')
        outVal = 0;
        fprintf(1, 'Bypassing known Matlab bug in R2008b''s waitbar...\n');
    else
        outVal = 1;
        fprintf(2, 'Call to waitbarUpdate failed: %s\n', getLastErrorStack);
    end
end