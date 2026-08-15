classdef TrapezoidalVelocityPlanner < robotics.trajectory.TrajectoryPlanner
    methods
        function tf = computeTime(obj, qi, qf, prcnt, velLim, accLim)
            v_prcnt = prcnt(1);
            a_prcnt = prcnt(2);
            kv = v_prcnt * velLim(:,2);
            ka = a_prcnt * accLim(:,2);
            D = qf - qi;
            tf = max(max(2*abs(D)./kv), max(sqrt(4*abs(D)./ka)));
        end
        
        function [s_pos, s_vel, s_acc] = p2pTrj(obj, t, tf)
            if t <= tf/2
                s_pos = 2*(t/tf)^2;
                s_vel = 4*(t/tf^2);
                s_acc = 4/tf^2;
            else
                s_pos = -1 + 4*(t/tf) - 2*(t/tf)^2;
                s_vel = 4/tf - 4*(t/tf^2);
                s_acc = -4/tf^2;
            end
        end
    end
end
