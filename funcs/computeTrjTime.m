function tf = computeTrjTime(ip,fp,prcnt,trj_profile,velLim,accLim)
    
    v_prcnt = prcnt(1);
    a_prcnt = prcnt(2);

    kv = v_prcnt*velLim(:,2)';
    ka = a_prcnt*accLim(:,2)';

    qi = ip;
    qf = fp;

    D = qf-qi;

    switch trj_profile
        case 1
            tf = max(abs(D)./kv);
%             vMax = abs(D)/tf;
%             aMax = 0;
        case 2
            tf = max(max(3*abs(D)./(2*kv)),max(sqrt(6*abs(D)./ka)));
%             vMax = 3*abs(D)/(2*tf);
%             aMax = 6*abs(D)/tf^2;
        case 3
            tf = max(max(15*abs(D)./(8*kv)),max(sqrt(10*abs(D)./(sqrt(3)*ka))));
%             vMax = 15*abs(D)/(8*tf);
%             aMax = 10*abs(D)/(sqrt(3)*tf^2);
        case 4
            tf = max(max(2*abs(D)./kv),max(sqrt(4*abs(D)./ka)));
%             vMax = 2*abs(D)/tf;
%             aMax = 4*abs(D)/tf^2;clc

        otherwise
            tf = 0;
    end

end