function cp=bdSeed(task,id,c)
cp=bdBaseline(task,c.controlCount); u=linspace(0,1,c.controlCount-6); w=sin(pi*(u+.05)/1.1).^2;
switch id
    case 1
    case 2, cp(6,4:end-3)=cp(6,4:end-3)+deg2rad(35)*w;
    case 3, cp(6,4:end-3)=cp(6,4:end-3)-deg2rad(35)*w;
    case 4, cp(3,4:end-3)=cp(3,4:end-3)+.20*w;
    otherwise
        phase=(id-5)*pi/2;
        cp(2,4:end-3)=cp(2,4:end-3)+.12*cos(phase)*w;
        cp(6,4:end-3)=cp(6,4:end-3)+deg2rad(35)*sin(phase)*w.*(1+.5*(u-.5));
end
end
