classdef LinearPlanner < robotics.trajectory.TrajectoryPlanner
    methods
        function tf = computeTime(obj, qi, qf, prcnt, velLim, accLim)
            if isscalar(prcnt)
                if prcnt > 1, prcnt = prcnt / 100; end
                v_prcnt = prcnt;
            else
                if prcnt(1) > 1, prcnt(1) = prcnt(1) / 100; end
                v_prcnt = prcnt(1);
            end
            kv = v_prcnt * velLim(:,2);
            D = qf - qi;
            tf = max(abs(D)./kv);
        end
        
        function [s_pos, s_vel, s_acc] = p2pTrj(obj, t, tf)
            if tf <= 0
                s_pos = 1; s_vel = 0; s_acc = 0;
                return;
            end
            s_pos = t / tf;
            s_vel = 1 / tf;
            s_acc = 0;
        end
    end
end
