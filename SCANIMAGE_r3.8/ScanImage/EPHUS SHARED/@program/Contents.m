% Svoboda Lab Program Class Definition
% Program Construction Tools for interface with the Program Manager
% 
% The @PROGRAM class will rememeber the Program specific details of a
% program used in the Program manager (@PROGMANAGER class).  It stroes the
% following:
% 
% Object.
% 	program_name:       [char]      Name of the Program.
% 	main_gui_name:      [char]      Name (alias) of Program's Main GUI.
% 	aliases:            [struct]    Names of GUIs contained in Program.
%   version:            [double]    Version number from Main GUI of Program.
%   progmanager_version [double]    Version of program manaegr when saved.
%
% The aliases structure has one field for each GUI alais contained in the program:
% 	GUI_ALIAS_1:    [struct]    Details of GUI called GUI_ALIAS_1.
% 	GUI_ALIAS_2:    [struct]    Details of GUI called GUI_ALIAS_2.
% 
% Each of ther GUI structures has the following fields:
% 	m_filename:     [char]      Name of m-file/ fig file to construct GUI with.
%   fig_props       [struct]    Structure containing figure properties.
% 	    Position:       [array]     4 element position of figure [left bottom width height]
% 	    Visible:        [char]      Either 'on' or 'off'
% 	variables:      [struct]    Structure of variables and values for this GUI.     
%
% Functions
%
% Construction and Setting Properties of the Program Manager object
% 	PROGRAM   - Constructor for program class.                                     
%
% File I/O Methods
% 	LOADOBJ   - @program save overloaded function.                                 
% 	LOADPROGRAM   - @program load method.                                          
% 	SAVEOBJ   - @program save overloaded function.                                 
% 	SAVEPROGRAM   - @program save method.                                          
%
% Setting and Getting object properties
% 	GET   - Overloaed method for @program class.                                   
% 	SET   - Overloaed method for @program class.   
%  
% Get and Set Specific Program Properties.
% 	GETALIASES   - @program method for outputting GUI aliases/mfiles as cell array.
% 	GETFIGPROPS   - @program method for outputting GUI figure properties.          
% 	GETVARIABLES   - @program method for outputting GUI variables.                 

