% Svoboda Lab Program Manager Class Definition
% Graphical User Interface (GUI) Construction/Management Tools
% 
% These functions are for programmers interested in making GUIs or programs using the progmanager 
% object.  This is the code used by the class to construct and handle GUI
% interfaces.  
%
% The Program manager object is unique.  A single copy is made, and
% subsequent calls to the constructor PROGMANAGER will return a copy of
% that object instance.
%
% When the PROGMANAGER consructor is called, a structure called
% progmanagerglobal is created in the workspace.  This variable should not
% be touched by the user, but rather is interacted with through the
% program manger using the methods described below.
%
% A program is defined as a collection of Graphical User interfaces (GUIs)
% like those made in GUIDE.  
% 
% 	Construction and Setting Properties of the Program Manager object
% 		GETMENUHANDLES  - Returns handles to graphical menu objects from specified program.                 
% 		GETPROGMANAGERDEFAULTS   - @progmanager method returns the default settings for the program manager.    
% 		GETPROGRAMVERSION   -  @progmanager method for outputing the program manager and program object versions.              
% 		SETPROGMANAGERDEFAULTS   - @progmanager method sets the default settings for the program manager.                     
% 		PROGMANAGER   - Constructor for programmanger class.                                                                   
% 		SET - Overloaed method for @progmanager class.                                                                         
% 		GET   - Overloaed method for @progmanager class.   
% 
% 	GUI Handling        
% 		UPDATEVARIABLEFROMGUI   - Updates progam manager variable tied to GUI.       
% 		GETGUINAME   -  @progmanager method for outputing the GUI name from figure.     
% 		GETHANDLEFROMNAME   -  @progmanager method for outputing the GUI figure handle from GUI instance's gui_name.           
% 		GETGUIVALUE   - Gets data in GUI handle specified, as well as its type.     
% 		SETGUIVALUE   - Sets data in GUI handle to value.   
% 		SETGUIPROPS   - @progmanager method for setting GUI properties from MainGUI Handle.
% 		SETLOCALGH   - @progmanager method that sets properties of handle with name TAG handle_tag.                       
%
% 	Logical Operations
% 		ISGUIINPROGRAM   - @progmanager logical method for checking if GUI is in program.                                      
% 		ISPROGRAM   - @progmanager logical method for checking program name.                                                   
% 		ISSTARTED   - @progmanager logical method for checking program name.                                                   
% 		PARSEGLOBALCELLARRAY   - Program Manager Variable Parser.     
% 
% 	Program handling  
% 		ADDPROGRAM   - @progmanager method for adding a program to a progmanager object.     
% 		SHOWPROGRAMS   - @progmanager method for displaying the program names currently added.      
% 		STARTPROGRAM   - @progmanager method for starting a program.                                                           
% 		CLOSEPROGRAM   - @progmanager method for closing a program.   
% 		SHOWGUISOFPROGRAM   - @progmanager method for displaying the program names currently added.                            
% 		GETPROGRAMNAME   -  @progmanager method for outputing the program name from figure.  
% 		GETPROGRAMPROP   -  @progmanager method for outputing the program level properties.         
% 		SETPROGRAMPROP   -  @progmanager method for outputing the program level properties.                                    
% 		SETPROGRAM   - @progmanager method that sets value of a variable in specified gui_name in same program as hobject.
%
% 	Variable handling
% 		SETMAIN   - @progmanager method that sets the value of a variable tied to the main GUI of a program.                   
% 		SETGLOBAL   - @progmanager method that sets value of a variable with reference.              
% 		SETGLOBALGH   - @progmanager method that sets properties for handles in GUI object with name TAG handle_tag.
% 		SETLOCAL   - @progmanager method that sets value of a variable tied to an object.                  
% 		GETMAIN   - @progmanager method that gets the value of a variable tied to the main GUI of a program.    
% 		GETCONFIGFLAG   - @progmanager method that gets value of a variable's Config Flag.                                    
% 		GETVARWITHCONFIGFLAG   - @progmanager method that gets varibles with a specified config flag.                         
% 		GETPROGRAM   - @progmanager method that gets the value of a variable in specified gui_name in same program as hobject.
% 		GETGLOBAL   - @progmanager method that gets value of a variable with reference.                                       
% 		GETGLOBALGH   - @progmanager method that gets handle to GUI object with name TAG handle_tag.                          
% 		GETLOCAL   - @progmanager method that gets the value of a variable tied to an object.                                 
% 		GETLOCALGH   - @progmanager method that gets handle to GUI object with name TAG handle_tag from parent object hobject.    
% 		GETSTRUCTURE   - Accesses structure or variable from global through object.
%
% 	Variable manipulations
% 		ADDLOCAL   - @progmanager method that changes variable tied to an object by adding value to it.                       
% 		CHANGELOCAL   - @progmanager method that changes value of a variable tied to an object by applying fcn_handle to it.  
% 		DIVIDELOCAL   - @progmanager method that changes variable tied to an object by dividing it by value.                  
% 		TIMESLOCAL   - @progmanager method that changes variable tied to an object by multiplying it by value.                
%
%   File I/O                                              
% 		LOADPROGRAM   - @progmanager method for loading a program from a program object stored on disk.                        
% 		OPENPROGRAM   - @progmanager method for opening a program from a program object.                                       
% 		RELOADPROGRAM   - @progmanager method for reloading a program already running from a program PMP file.                 
% 		SAVEPROGRAM   - @progmanager save program method.       
%
% 	Testing 
% 		TEST   - @progmanager class test function.  
%
%   Browsing
% 		PROGMANAGERDISP   - @progmanager method for displaying and editing progmanager options and programs.


