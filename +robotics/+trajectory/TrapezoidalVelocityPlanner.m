classdef TrapezoidalVelocityPlanner < robotics.trajectory.TrajectoryPlanner
    methods
        function tf = computeTime(obj, qi, qf, prcnt, velLim, accLim)
            if isscalar(prcnt)
                if prcnt > 1, prcnt = prcnt / 100; end
                v_prcnt = prcnt;
                a_prcnt = prcnt;
            else
                if prcnt(1) > 1, prcnt(1) = prcnt(1) / 100; end
                if prcnt(2) > 1, prcnt(2) = prcnt(2) / 100; end
                v_prcnt = prcnt(1);
                a_prcnt = prcnt(2);
            end
            kv = v_prcnt * velLim(:,2);
            ka = a_prcnt * accLim(:,2);
            D = qf - qi;
            tf = max(max(2*abs(D)./kv), max(sqrt(4*abs(D)./ka)));
        end
        
        function [s_pos, s_vel, s_acc] = p2pTrj(obj, t, tf)
            if tf <= 0
                s_pos = 1; s_vel = 0; s_acc = 0;
                return;
            end
            tau = t / tf;
            if tau <= 0.5
                s_pos = 2 * tau * tau;
                s_vel = (4 * tau) / tf;
                s_acc = 4 / (tf * tf);
            else
                s_pos = -1 + 4*tau - 2*tau*tau;
                s_vel = (4 - 4*tau) / tf;
                s_acc = -4 / (tf * tf);
            end
        end
    end
end
