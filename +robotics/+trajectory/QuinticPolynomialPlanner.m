classdef QuinticPolynomialPlanner < robotics.trajectory.TrajectoryPlanner
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
            tf = max(max(15*abs(D)./(8*kv)), max(sqrt(10*abs(D)./(sqrt(3)*ka))));
        end
        
        function [s_pos, s_vel, s_acc] = p2pTrj(obj, t, tf)
            if tf <= 0
                s_pos = 1; s_vel = 0; s_acc = 0;
                return;
            end
            tau = t / tf;
            tau2 = tau * tau;
            tau3 = tau2 * tau;
            tau4 = tau3 * tau;
            s_pos = 10*tau3 - 15*tau4 + 6*(tau4*tau);
            s_vel = (30*tau2 - 60*tau3 + 30*tau4) / tf;
            s_acc = (60*tau - 180*tau2 + 120*tau3) / (tf*tf);
        end
    end
end
