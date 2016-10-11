%% FastZ
%FastZ hardware used for fast axial motion, supporting fast stacks and/or volume imaging
%fastZControllerType must be specified to enable this feature. 
%Specifying fastZControllerType='useMotor2' indicates that motor2 ControllerType/StageType/COMPort/etc will be used.
fastZControllerType = '';           % If supplied, one of {'useMotor2', 'pi.e709', 'pi.e753', 'pi.e665', 'pi.e816', 'npoint.lc40x', 'analog'}. 
fastZCOMPort = [];                  % Integer identifying COM port for controller, if using serial communication
fastZBaudRate = [];                 % Value identifying baud rate of serial communication. If empty, default value for controller used.

%Some FastZ hardware requires or benefits from use of an analog output used to control sweep/step profiles
%If analog control is used, then an analog sensor (input channel) must also be configured
fastZDeviceName = '';               % String specifying device name used for FastZ control
frameClockIn = '';                  % One of {PFI0..15, ''} to which external frame trigger is connected. Leave empty for automatic routing via PXI/RTSI bus
fastZAOChanID = [];                 % Scalar integer indicating AO channel used for FastZ control
fastZAIChanID = [];                 % Scalar integer indicating AI channel used for FastZ sensor
