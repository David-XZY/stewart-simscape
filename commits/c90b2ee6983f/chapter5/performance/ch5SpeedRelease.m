function ch5SpeedRelease()
% Keep initialized models/physics in the persistent worker; release run data.
for name=["ch5Records","ch5Predictions","ch5Plans","ch5Audit","ch5State", ...
        "ch5SpeedWarm","ch5SpeedExecution","ch5SpeedIterate"]
    if isappdata(0,name),rmappdata(0,name);end
end
fprintf('Speed probe completed; MATLAB %d retained for reuse.\n',feature('getpid'));
end
