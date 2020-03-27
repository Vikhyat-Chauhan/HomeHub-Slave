# HomeHub-Slave
HomeHub Slave device Firmware and Resources.

V1.1 added all button features and tasks
v1.2 added features to remove redundant reprinting of device json{} when no changes have occurred in the devices, added codes to manage the command json sending too(have to make it more advance in later sections)
V1.3 Added Handshake feature to initially boot, then after completing handshake the sync functions normally transfer serial data on internal changes. 
V1.4 Added Temperature Sensor functions and put it at position 0 strcuture 
V1.5 changed structure to enable not resending of data to master after master changed something, also removed the issue of json empty brackets putting default values into devices structures.
V1.6 Added stability with increasing the baud rate of 115200 and the master processing interrupt to 20ms
V1.7 Added Button data to initial Handshake response, and added new commands to input and output Command reading list. Made some changes to button long press booleans.
#NOTE# Light functioning task remains and Sleep functioning task remains [ Implement them with hardware]
