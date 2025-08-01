function [data, info] = status
%status gives an empty data for movemaster_msg/status
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'movemaster_msg/status';
[data.Joint, info.Joint] = ros.internal.ros.messages.ros.char('string',0);
[data.Setpoint, info.Setpoint] = ros.internal.ros.messages.ros.default_type('single',1);
[data.PulseCount, info.PulseCount] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Error, info.Error] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Output, info.Output] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ControlLoop, info.ControlLoop] = ros.internal.ros.messages.ros.default_type('single',1);
[data.IsDone, info.IsDone] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'movemaster_msg/status';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'joint';
info.MatPath{2} = 'setpoint';
info.MatPath{3} = 'pulse_count';
info.MatPath{4} = 'error';
info.MatPath{5} = 'output';
info.MatPath{6} = 'control_loop';
info.MatPath{7} = 'IsDone';
