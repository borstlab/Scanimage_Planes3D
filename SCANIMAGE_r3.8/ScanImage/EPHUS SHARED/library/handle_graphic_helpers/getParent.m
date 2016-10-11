function parent = getParent(handle,level)
%GETPARENT   - Returns parent of handle passed to it.
%  GETPARENT without inputs will return the parent of the current object.  If
%  the current object (gotten by calling gco) is a figure, it returns the
%  handle to that figure (same as gcf).
%
% GETPARENT(handle) will return the parent of the graphics object handle.
%
% GETPARENT(handle,level) will return the parent of the graphics object
% handle and apply the same algorithm recursively until the level supplied 
% is reached.  level is a valid handle graphic 'Type' such as
% 'figure','axes','uimenu',... By default, the level is set to 'figure'.  
%
% By default, the parent of the handle is returned unless a level is
% specified.
%
% See also 
%
% Written By: Thomas Pologruto  2/24/04
%
% These are the valid levels, since these are the only objects allowed to
% have children.
%
% CHANGES
%  TO022706D: Optimization. -- Tim O'Connor 2/27/06

%TO022706D: With the check against this removed, this array is now unnecessary. -- Tim O'Connor 2/27/06
% validLevels={'root','figure','axes','uimenu','uicontrol','uicontextmenu'};

% Parse the input arguments.
if nargin == 0
    handle=gco;
    level='figure';
elseif nargin == 1
    level='figure';
%TO022706D: This check is taking a lot of time (hundreds of milliseconds on a fast machine), since it's called very often. -- Tim O'Connor 2/27/06
% elseif nargin == 2
%     if ~ismember(lower(level),validLevels)
%         error(['getParent: invalid level specified.']);
%     end
elseif nargin > 2
    error(['getParent: too many inputs.']);
end

%Check integrity of the inputs.
if ~ishandle(handle)
    error(['getParent: 1st input must be a valid handle.']);
end

% Check to see if the handle is empty or is the root (0)
if isempty(handle)  % Empty handle returns empty
    parent=[];
    return
elseif handle==0     % Root always returns root.
    parent=handle;
    return
end

% Check current type of input.
input_level=get(handle,'Type');

% If the input level is the same as the desired output level, return it.
if strcmpi(input_level,level)
    parent=handle;
    return
else
    parent=get(handle,'Parent');
    currentlevel=get(parent,'Type');
end

% If the user supplied a level, then check for that parent.
if nargin == 2
    max_iteration=10;   % Only do it 10 times at most.tes
    counter=1;
    currentlevel=get(parent,'Type');
    while ~strcmpi(level,currentlevel) & counter < max_iteration
        parent=get(parent,'Parent');
        %TO022706D: Because the level check was removed in the beginning, check here for hitting the top level. -- Tim O'Connor 2/27/06
        if parent == 0
            error('Invalid gui level, parent of type ''%s'' not found.', level);
        end
        currentlevel=get(parent,'Type');
        counter=counter+1;
    end
end

