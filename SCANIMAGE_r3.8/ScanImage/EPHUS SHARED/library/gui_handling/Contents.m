% Svoboda Lab Library - GUI Handling Functions
% 
% These functions are used for writing callbacks and menu actions for GUI figures.
% They are sorted by the type of GUI object (popupmenu, uimenu, etc...)
% that they apply.
%
% UIMenus
%     ADDMENUTOFIGURE   - Helper function for creating UIMenus.      
%     GETPULLDOWNMENUINDEX   - UIMenu name to index converter.       
%     TURNOFFPULLDOWNMENU   - Disable selected menu by label.        
%     TURNONPULLDOWNMENU   - Enable selected menu by label.        
%
% Popupmenu and Listbox
%     FINDMENUINDEX   - Listbox/Popupmenu Name to index converter.   
%     GETMENUENTRY   - Returns label in listbox/popupmenu from index.
%
% PushButtons and Toggle Buttons
% 	SWITCHSTRING   - Exchanges string names of a GUI's uicontrol object.
