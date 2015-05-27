function [varargout] = parseStructString(struct_string)
% PARSESTRUCTSTRING   - Parse structure into parts based on periods.
%   PARSESTRUCTSTRING will take the input string s which is a structure name
%   with subfileds indexed using the '.' notation (ex.
%   'state.acq.numberOfFrames' and output all the fields starting from 
%   the top most field, for as many outputs as are desired.  
%   
%   All outputs of which are strings suitable for use with the
%   dynamic fieldnames referencing in MATLAB 6.5+.
%
%   Ex:  [bottom,middle,top] = PARSESTRUCTSTRING('state.acq.numberOfFrames')
%
%   bottom = 'numberOfFrames'
%
%   middle = 'acq'
% 
%   top = 'state'
%
%   See also FIELDNAMES, ORDERFIELDS, STRUCTNAMEPARTS
%
% Changes:
%   TO022706D: Optimization(s). No need to deblank at all, as tokenize will handle that. No need to fliplr, as indexing can do that. -- Tim O'Connor 2/27/06

%TO022706D: Complete rewrite. - Temporarily reverted to original implementation to solve some backwards compatibility issues. (Tim O'Connor 2/28/06)
struct_string=deblank(struct_string);
struct_string(struct_string=='.')=' ';
struct_string=fliplr(tokenize(struct_string));

if length(struct_string) >= nargout
    varargout(1:nargout)=struct_string(1:nargout);
elseif length(struct_string) < nargout
    difference=nargout-length(struct_string);
    varargout(1:nargout)=[struct_string cell(1,difference)];
end

% % parsed = tokenize(strrep(struct_string, '.', ' '));
% parsed = tokenize(struct_string, '.');
% 
% if length(parsed) >= nargout
%     varargout(1:nargout) = parsed(nargout : -1 : 1);
% elseif length(parsed) < nargout
%     varargout = [cell(1, nargout - length(parsed)), parsed(end : -1 : 1)];
% end

return;