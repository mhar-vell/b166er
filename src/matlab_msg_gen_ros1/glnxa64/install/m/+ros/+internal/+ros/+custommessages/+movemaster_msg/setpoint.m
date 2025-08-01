function [data, info] = setpoint
%setpoint gives an empty data for movemaster_msg/setpoint
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'movemaster_msg/setpoint';
[data.Set1, info.Set1] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Set2, info.Set2] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Set3, info.Set3] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Set4, info.Set4] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Set5, info.Set5] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SetGRIP, info.SetGRIP] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.EmergencyStop, info.EmergencyStop] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.GoHome, info.GoHome] = ros.internal.ros.messages.ros.default_type('int8',1);
info.MessageType = 'movemaster_msg/setpoint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'set_1';
info.MatPath{2} = 'set_2';
info.MatPath{3} = 'set_3';
info.MatPath{4} = 'set_4';
info.MatPath{5} = 'set_5';
info.MatPath{6} = 'set_GRIP';
info.MatPath{7} = 'emergency_stop';
info.MatPath{8} = 'GoHome';
