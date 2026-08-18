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
            if tf <= 0
                s_pos = 1; s_vel = 0; s_acc = 0;
                return;
            end
            tau = t / tf;
            tau2 = tau * tau;
            s_pos = 3*tau2 - 2*(tau2*tau);
            s_vel = (6*tau - 6*tau2) / tf;
            s_acc = (6 - 12*tau) / (tf*tf);
        end
    end
end
