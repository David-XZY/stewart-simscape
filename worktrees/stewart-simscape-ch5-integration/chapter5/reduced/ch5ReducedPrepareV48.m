function r=ch5ReducedPrepareV48(index,mode)
% Restore the full prediction horizon after the isolated short-horizon pilot.
[c,~]=ch5Init();r=ch5ReducedPrepareV45(index,mode);
r.c.H=c.H;r.c.version=[c.version,'-speed-reduced-v1'];
assert(r.c.H==60&&abs(r.c.Ts-.02)<1e-12,'ch5:BlockingContract','Use the original physical horizon and execution step.');
end
