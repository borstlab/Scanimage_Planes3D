function obj_out=progmanager(object, varargin)
%PROGMANAGER   - Constructor for programmanger class.
%   PROGMANAGER is An object that manages different custom built applications written in
%   MATLAB.  The prgram manager object is unique, and makes reference to
%   the programmanager structure in memory (a global variable called
%   'programmanager').  The progmanager object contains by default an internal
%   (called 'internal') field which lists various attibutes of the current program manager
%   object. 
%
%   The first call to progmanager creates a variable whose name is 
%   specified by the global variable defaultglobalname in the workspace.  
%   
%   Subsequent calls to progmanager simply pass out a reference to the
%   progmanager object, whihc is stored in progmanagerglobal.internal.obj.
%
%   Fields found in the programmanager.internal structure:
%
% 		version	[numeric]                   - The current progmanager version
% 		
% 		daqmanager [char]                   - The name of the current daqmanager (empty if none exist)
% 		
% 		obj [progmanager object]            - Reference to current progmanager object.
%           
% 		deleteFigsOnClose [bool]            - Delete figure handles stoed in
%                                            program manager on closeProgram call.
%
%       deleteObjectsOnClose [bool]         - Delete objects stored in program manager on closeProgram call.
%
%       stopObjectsOnClose [bool]           - Stop objects, which support a `stop` method, on closeProgram call.
% 		
% 		GUIDE_opts [struct]                 - Current GUIDE options. See GUIDEOPTS for details.
% 		
% 		MaxConfigBits [double]              - Sets the number of bits to be uised when parsing the CongigFlag 
% 									          for variables stored in thr program manager. See PARSEGLOBALCELLARRAY
%                                             for details.
% 		
% 		ConfigBitForSaving [double]         - Sets the bit to use for saving variables in the PROGRAM
%                                             objects. See PARSEGLOBALCELLARRAY for details.
% 		
% 		ConfigBitForHeader [double]         - Sets the bit to use for saving variables in a generic header.
%                                             See PARSEGLOBALCELLARRAY for details.
% 		
% 		ProgmanagerDisplayOn [bool]         -  1 if Progmanagerdisplay is currently active, and 0 otherwise.
% 		
% 		editable_fields [cell]              -   A cell array of strings of parameters user can set.
% 		
% 		min_val [cell]                      -   A cell array of minimum values for the editable parameters.  
%                                               Strings are ignored, but need to be here for place keeping.
%            
% 		max_val [cell]                      -   A cell array of maximum values for the editable parameters.  
%                                               Strings are ignored, but need to be here for place keeping.
%
% 		internal_editable_fields [cell]     -   A cell array of strings of internal parameters.
% 		
% 		internal_min_val [cell]              -   A cell array of minimum values for the internal parameters.  
%                                               Strings are ignored, but need to be here for place keeping.
%            
% 		internal_max_val [cell]              -   A cell array of maximum values for the internal parameters.  
%                                               Strings are ignored, but need to be
%                                               here for place keeping.
%
% 		filename [char]                     -   Filename where default program manager props are stored.
%
%   See also ADDPROGRAM, STARTPROGRAM, GUIDEOPTS, PARSEGLOBALCELLARRAY,
%   PROGMANAGERDISP

%  Created - Tom Pologruto 2/4/04
%
%  Changed:
%           Tim O'Connor 3/4/04 (TO030404a) - Removed 'version' field from editable properties.
%           TnT 3/5/04 (TT030504A) - Changed default 'programs' field to an empty struct.
%           Tom Pologruto 3/5/04 (TP030504a): udpated edittable fields. 
%           Tom Pologruto 3/5/04 (TP030504b): added internally settable fields. 
%           Tim O'Connor 4/6/04 (TO040604b): Added functionality to delete objects on close.
%           Tim O'Connor 4/6/04 (TO040604d): Added functionality to stop objects on close.
%           TnT 6/16/04 (TnT061604a): NOTE: Must have an equal size for min_val and max_val as for editable_fields.
%           Tim O'Connor 12/9/05 (TO120905M): Changed default of stopObjectsOnClose to 0.
%           TO060208J - Added a non-program oriented header, for assorted user data. -- Tim O'Connor 6/1/08
%
%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor
%  Laboratories 2004

% Configure program manager global values

defaultglobalname='progmanagerglobal';

if evalin('base', ['exist(''' defaultglobalname ''');'])
    obj_out=evalin('base', [defaultglobalname '.internal.obj']);
else
    % create pass by reference object
    pmObject.name=defaultglobalname;
    obj_out=class(pmObject,'progmanager');   % Passes out object as correct class.
    
    % Assemble the global program manager structure.
    
    % These internal properties allow the user to change parameters after
    % program begins.  
    %TO030404a: Removed 'version' from this list, it should be purely hardcoded. - Tim O'Connor 3/4/03
    %TP030504a: removed extra minmax fields - Tom Pologruto 3/5/04
    %TO040604b: Added 'deleteObjectsOnClose' field.
    %TO040604d - see TO040604b.
    editable_fields={'deleteFigsOnClose','GUIDE_opts.singleton', 'deleteObjectsOnClose', 'stopObjectsOnClose'};
    %TnT061604a - NOTE: Must have an equal size for min_val and max_val as for editable_fields. - 6/16/04
    min_val={0,0, 0, 0};
    max_val={1,1, 1, 1};
    
    %TP030504b: added internally settable fields. - Tom Pologruto 3/5/04
    internal_editable_fields={'daqmanagername','ProgmanagerDisplayOn'};
    internal_min_val={'',0};
    internal_max_val={'',1};
    
    % Initialize editable params.
    version=1.0;
    daqmanagername='gdm';
    deleteFigsOnClose=1;
    deleteObjectsOnClose = logical(1);%TO040604b
    stopObjectsOnClose = logical(0);%TO040604d, TO120905M
    
    GUIDE_opts.singleton = 0;             % Force single copy of GUI per session?

    % Load the default file if one exists.
    filename_ending='progmanagerdefaults.mat';
    pathtodefaultfile=fileparts(which(mfilename));
    filename=fullfile(pathtodefaultfile,filename_ending);
    
    if exist(filename)==2
        load(filename);               % load program manager saved defaults if file exists.
    end
    
    % Initialize non editable parameters.
    % Configure Default GUI options.  See GUIDEOPTS for more details.
    GUIDE_opts.access      = 'callback';   % Access to GUI through callbacks only.
    GUIDE_opts.syscolorfig = 1;             % Use system color for objects.
    GUIDE_opts.resize      = 'none';        % Resize Option.
    GUIDE_opts.mfile       = 1;             % Generate Callback m file.
    GUIDE_opts.callbacks   = 1;             % Generate Callback subfunctions.

    % Set the properties in the global structure.
    eval([defaultglobalname '.internal.version=version;']);
    eval([defaultglobalname '.internal.daqmanagername=daqmanagername;']);
    eval([defaultglobalname '.internal.deleteFigsOnClose=deleteFigsOnClose;']);
    eval([defaultglobalname '.internal.deleteObjectsOnClose = deleteObjectsOnClose;']);%TO040604b
    eval([defaultglobalname '.internal.stopObjectsOnClose = stopObjectsOnClose;']);%TO040604d
    eval([defaultglobalname '.internal.GUIDE_opts=GUIDE_opts;']);
    eval([defaultglobalname '.internal.filename=filename;']);
    eval([defaultglobalname '.internal.editable_fields=editable_fields;']);
    eval([defaultglobalname '.internal.min_val=min_val;']);
    eval([defaultglobalname '.internal.max_val=max_val;']);
    eval([defaultglobalname '.internal.internal_editable_fields=internal_editable_fields;']);
    eval([defaultglobalname '.internal.internal_min_val=internal_min_val;']);
    eval([defaultglobalname '.internal.internal_max_val=internal_max_val;']);    
    eval([defaultglobalname '.internal.obj=obj_out;']);
    eval([defaultglobalname '.internal.ProgmanagerDisplayOn=0;']);
    eval([defaultglobalname '.internal.MaxConfigBits=2;']);
    eval([defaultglobalname '.internal.ConfigBitForSaving=1;']);
    eval([defaultglobalname '.internal.ConfigBitForHeader=2;']);
    eval([defaultglobalname '.internal.userDataHeaders = [];']);%TO060208J
    
    % Initialize the programs field for the program manager.
    %Changed from an empty double array to an empty struct. -- TnT 3/5/04 TT030504A
    eval([defaultglobalname '.programs=struct;']);
    
    % Declare progmanager as a global and publish to workspace.
    evalin('base', ['global ' defaultglobalname]);
    assignin('base',defaultglobalname,eval(defaultglobalname));
    
    %Set root properties for session with prgoram manager
    set(0,'DefaultFigureIntegerHandle','off','DefaultFigureDoubleBuffer','on','DefaultFigureNumberTitle','off');
end