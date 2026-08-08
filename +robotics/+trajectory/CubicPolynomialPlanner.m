classdef CubicPolynomialPlanner < robotics.trajectory.TrajectoryPlanner
    methods
        function tf = computeTime(obj, qi, qf, prcnt, velLim, accLim)
            v_prcnt = prcnt(1);
            a_prcnt = prcnt(2);
            kv = v_prcnt * velLim(:,2);
            ka = a_prcnt * accLim(:,2);
            D = qf - qi;
            tf = max(max(3*abs(D)./(2*kv)), max(sqrt(6*abs(D)./ka)));
        end
        
        function [s_pos, s_vel, s_acc] = p2pTrj(obj, t, tf)
            s_pos = 3*(t/tf)^2 - 2*(t/tf)^3;
            s_vel = 6*(t/tf^2) - 6*(t^2/tf^3);
            s_acc = 6/tf^2 - 12*(t/tf^3);
        end
    end
end

