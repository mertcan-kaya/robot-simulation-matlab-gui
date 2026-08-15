classdef QuinticPolynomialPlanner < robotics.trajectory.TrajectoryPlanner
    methods
        function tf = computeTime(obj, qi, qf, prcnt, velLim, accLim)
            v_prcnt = prcnt(1);
            a_prcnt = prcnt(2);
            kv = v_prcnt * velLim(:,2);
            ka = a_prcnt * accLim(:,2);
            D = qf - qi;
            tf = max(max(15*abs(D)./(8*kv)), max(sqrt(10*abs(D)./(sqrt(3)*ka))));
        end
        
        function [s_pos, s_vel, s_acc] = p2pTrj(obj, t, tf)
            s_pos = 10*(t/tf)^3 - 15*(t/tf)^4 + 6*(t/tf)^5;
            s_vel = 30*(t^2/tf^3) - 60*(t^3/tf^4) + 30*(t^4/tf^5);
            s_acc = 60*(t/tf^3) - 180*(t^2/tf^4) + 120*(t^3/tf^5);
        end
    end
end
