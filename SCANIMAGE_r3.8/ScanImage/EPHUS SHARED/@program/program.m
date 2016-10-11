function obj_out=program(varargin)
%PROGRAM   - Constructor for program class.
%   PROGMANAGER is an object that manages programs for use with the progrma
%   manager class (@PROGMANAGER).  Each Program strores program specific
%   information, and the object and be saved and loaded to load program
%   defaults into the program manager.
%
%   Unlike the program manager and @progmanager class, which is by default
%   tied to a global variable and is unique, the program class is a bona
%   fide MATLAB class, and obeys the constructor class definition used by
%   MATLAB.
%
% Object.
% 	program_name:       [char]      Name of the Program.
% 	main_gui_name:      [char]      Name (alias) of Program's Main GUI.
% 	aliases:            [struct]    Names of GUIs contained in Program.
%   version:            [double]    Version number from Main GUI of Program.
%   progmanager_version [double]    Version of program manaegr when saved.
%   filename            [char]      Name of file where program is stored.
% 	editable_fields     [cell]      A cell array of strings of parameters user can set.
% 	min_val             [cell]      A cell array of minimum values for the editable parameters.  
%                                   Strings are ignored, but need to be here for place keeping.
% 	max_val             [cell]      A cell array of maximum values for the editable parameters.  
%                                   Strings are ignored, but need to be here for place keeping.
%
% The aliases structure has one field for each GUI alais contained in the program:
% 	GUI_ALIAS_1:        [struct]    Details of GUI called GUI_ALIAS_1.
% 	GUI_ALIAS_2:        [struct]    Details of GUI called GUI_ALIAS_2.
% 
% Each of ther GUI structures has the following fields:
% 	m_filename:         [char]      Name of m-file/ fig file to construct GUI with.
%   fig_props           [struct]    Structure containing figure properties.
% 	    Position:           [array]     4 element position of figure [left bottom width height]
% 	    Visible:            [char]      Either 'on' or 'off'
% 	variables:          [struct]    Structure of variables and values for this GUI.     
%
% See also PROGMANAGER
%
% NOTES
%  Since it is not clear from any of the documentation, and is a constant source of problems...
%  The typical usage for a program with no supporting GUIs is: program(Name, Name, mFileName) - Tim O'Connor 3/28/05
%
%  Created - Tom Pologruto 2/23/04
%
% MODIFICATIONS
%   VI053108A Vijay Iyer 5/31/08 -- Support a reduced syntax for program creation
%
%
%  Copyright - Howard Hughes Medical Institute/Cold Spring Harbor
%  Laboratories 2004

% Default value of object.
obj_out.editable_fields={'program_name','main_gui_name','aliases'};
obj_out.min_val={'','',''};
obj_out.max_val={'','',''};
obj_out.program_name='default';
obj_out.main_gui_name='';
obj_out.aliases=[];
obj_out.version=NaN;
obj_out.progmanager_version=NaN;
obj_out.filename='';
    
%Ceate new program object instance.
% the following is a MATLAB convention...
if nargin >= 1   % 1 input: if is object, return it.  if it is a character, make a new instance.   
    if ischar(varargin{1})
        obj_out.program_name = varargin{1};
    elseif isa(varargin{1},'program')
        obj_out=varargin{1};
        return
    else
        error('@program/program: 1st input must be a string (Program Name) or a program object.');
    end

    %%%%%%%%%%%% COMMENTED OUT (VI053008A)%%%%%%%%%%%%%%
    % elseif nargin >= 2 %2 or more inputs: 1st input is program name, and the rest are param/value pairs of Alias/MFileNames.
    %     if isa(varargin{1},'program')
    %         obj_out=varargin{1};
    %         return
    %     elseif ~ischar(varargin{1})
    %         error('@program/program: 1st input must be a string (Program Name) or a program object.');
    %     end
    %     get program name.
    %     obj_out.program_name = varargin{1};
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if nargin >= 2
        % now remove program name from list to parse.
        varargin=varargin(2:end);
        % if they only input 2 things, repeat the last input to make it the
        % correct size.
        if length(varargin) < 2
            varargin{end+1}=varargin{end};
        end
    else %(VI053108A)--Handle the most common case
        %re-use varargin{1} as the mainGui alias
        varargin{2} = varargin{1}; %assume that the mainGui mfilename is the same as the alias and program name
    end

    % Parse the param/value pairs and assign them to the object.
    for param_counter=1:2:length(varargin)
        % Main GUI is first one by default.
        if param_counter==1
            obj_out.main_gui_name = varargin{param_counter};
        end
        obj_out.aliases.(varargin{param_counter}).m_filename=varargin{param_counter+1};
        obj_out.aliases.(varargin{param_counter}).fig_props.Position=[];
        obj_out.aliases.(varargin{param_counter}).fig_props.Visible='';
        obj_out.aliases.(varargin{param_counter}).variables=[];
    end
end

% Declare output structure to be a program object.
obj_out=class(obj_out,'program');

