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
                s_pos = ones(size(t)); s_vel = zeros(size(t)); s_acc = zeros(size(t));
                return;
            end
            tau = min(max(t / tf, 0), 1);
            s_pos = tau;
            s_vel = ones(size(tau)) / tf;
            s_acc = zeros(size(tau));
        end
    end
end
