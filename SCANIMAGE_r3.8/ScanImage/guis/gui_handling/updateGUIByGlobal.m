%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%  The same as the "classic" 'updateGuiByGlobal' function,
%%  except this also executes any GUI callbacks.
%%
%%  If the variable is tied to multiple GUIs, there is no gaurantee
%%  of the order in which the callbacks will get executed.
%%
%%  updateGuiByGlobalWithCallback(globalName [, 'Value', VALUE] [, 'Callback', BOOL])
%%
%%  VALUE - If the 'Value': VALUE pair is passed, the variable is updated, before any callbacks are executed.
%%  CALLBACK - If the 'Callback': BOOL pair is passed, and BOOL evalues to TRUE, all callbacks tied to the 
%%             variable are executed.
%%
%%  The original form of the call (passing only globalName) works the same as always. Just updating the GUIs, 
%%  not changing values or executing callbacks.
%%
%%  Changed:
%%   1/6/04 TnT - Completely rewritten, now executes callbacks and sets values.
%%   3/2/04 Tim O'Connor - Update the header for every call, regardless of GUI existence.
%    VI022410: Handle function-handle callback arguments; important for GUIDE GUIs created since 2008a.  -- Vijay Iyer 2/24/10
%%
%%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor Laboratories 2003
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function updateGUIByGlobal(globalName, varargin)
global gh;

%A little argument checking.
if ~ismember(length(varargin), [0 2 4])
    error('Wrong number of arguments in call to updateGuiByGlobal.');
end

exCallback = 0;

for i = 1 : 2 : length(varargin)

    if ~strcmpi(class(varargin{i}), 'char')
        
        %Make sure it's a string that we're examining.
        error(sprintf('Expected argument of type ''char'' for updateGuiByGlobal, found: ''%s''', class(varargin{i})));
        
    elseif strcmpi(varargin{i}, 'Value') | strcmpi(varargin{i}, 'Val')
    
        %Wrestle with Matlab's workspace partitioning...
        val = varargin{i + 1};
        dots = find(globalName == '.');

        %Update the variable.
        eval(sprintf('global %s, %s = val;', globalName(1 : dots(1) - 1), globalName));
        
    elseif strcmpi(varargin{i}, 'Callback')
        
        if varargin{i + 1}
            %Execute the callback.
            exCallback = 1;
        end
        
    else
        
        %What'chou talkin' 'bout, Willis?!?
        error(sprintf('Unrecognized argument to ''updateGuiByGlobal'' function: ''%s''', varargin{i}));
        
    end
    
end

%Given the name of a global variable, find and update the GUI that contains it.
guiLoc = getGuiOfGlobal(globalName);

if length(guiLoc) ~= 0
    %Update the GUI
    updateGUIByName(guiLoc);
end

%Update header for those variables that do not have GUIs 
%This was taken out of the above `if` statement. Tim O'Connor 3/2/04 TO3204c
updateHeaderString(globalName);

if exCallback
    %Iterate over all bound GUIs, in arbitrary order.
    for guiname = guiLoc

        %Get the GUI handle.
        g = eval(eval('guiname{1}'));

        %Get the callback function.
        cllbk = get(g, 'Callback');
        
        %Execute the callback.
        if isa(cllbk,'function_handle') %VI022410
            feval(cllbk,g,[]); %VI022410
        else            
            evalin('base', strrep(cllbk, 'gcbo', guiname{1}));
        end
    end
end

return;