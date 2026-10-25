clear
close all
clc

addpath FUNCTIONS\

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, ...
        'dynamixel_sdk.h', 'addheader', 'port_handler.h', ...
        'addheader', 'packet_handler.h');
end

ADDR_MX_TORQUE_ENABLE = 24;
ADDR_MX_CW_COMPLIANCE_MARGIN = 26;
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27;
ADDR_MX_CW_COMPLIANCE_SLOPE = 28;
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29;
ADDR_MX_GOAL_POSITION = 30;
ADDR_MX_MOVING_SPEED = 32;
ADDR_MX_PRESENT_POSITION = 36;
ADDR_MX_PUNCH = 48;
PROTOCOL_VER = 1.0;
DXL_IDS = [1,2,3,4];
DEVICENAME = 'COM6';
BAUDRATE = 1000000;
TORQUE_ENABLE = 1;
TORQUE_DISABLE = 0;
port_num = portHandler(DEVICENAME);
packetHandler();
openPort(port_num)
setBaudRate(port_num, BAUDRATE)

for DXL_ID=DXL_IDS
write1ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
write2ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
write2ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
write1ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
write1ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
write2ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_MOVING_SPEED, 100)
end

COMM_SUCCESS = 0;
COMM_TX_FAIL = -1001;

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    return;
end


% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VER);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VER);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VER, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VER, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

for DXL_ID=DXL_IDS
    read2ByteTxRx(port_num, PROTOCOL_VER, DXL_ID, ADDR_MX_PRESENT_POSITION)
end