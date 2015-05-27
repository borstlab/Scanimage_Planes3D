function program_obj=test(obj)
% TEST   - @progmanager class test function.
% 	TEST(prog_obj) will run test suite on the program manaager class
%   
%   See also

% updated to include program class 2/23/03 (TP)
% updated to include openprogram class 2/23/03 (TP)

Program_Name='Test_Program';
program_obj=program(Program_Name,'Image_Handle_Browser','climGUI','Overlay_GUI','overlayGUI_sa','Overlay_GUI_2','overlayGUI_sa');
openprogram(obj,program_obj);

% Program_Name = 'Image_Browser';
% GUI_Alias = 'Image_Handle_Browser';
% M_Filename = 'climGUI';
% program_obj=program(Program_Name,GUI_Alias,M_Filename);
% openprogram(progmanager,program_obj);
