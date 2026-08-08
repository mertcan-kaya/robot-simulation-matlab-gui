classdef LinearPlanner < robotics.trajectory.TrajectoryPlanner
    methods
        function tf = computeTime(obj, qi, qf, prcnt, velLim, accLim)
            v_prcnt = prcnt(1);
            kv = v_prcnt * velLim(:,2);
            D = qf - qi;
            tf = max(abs(D)./kv);
        end
        
        function [s_pos, s_vel, s_acc] = p2pTrj(obj, t, tf)
            s_pos = t/tf;
            s_vel = 1/tf;
            s_acc = 0;
        end
    end
end

