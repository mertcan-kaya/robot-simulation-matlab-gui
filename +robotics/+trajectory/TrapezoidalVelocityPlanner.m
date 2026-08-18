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
                s_pos = ones(size(t)); s_vel = zeros(size(t)); s_acc = zeros(size(t));
                return;
            end
            tau = min(max(t / tf, 0), 1);
            s_pos = zeros(size(tau));
            s_vel = zeros(size(tau));
            s_acc = zeros(size(tau));
            
            mask1 = (tau <= 0.5);
            mask2 = ~mask1;
            
            % Phase 1: Acceleration (tau <= 0.5)
            s_pos(mask1) = 2 * tau(mask1).^2;
            s_vel(mask1) = (4 * tau(mask1)) / tf;
            s_acc(mask1) = 4 / (tf * tf);
            
            % Phase 2: Deceleration (tau > 0.5)
            s_pos(mask2) = -1 + 4*tau(mask2) - 2*tau(mask2).^2;
            s_vel(mask2) = (4 - 4*tau(mask2)) / tf;
            s_acc(mask2) = -4 / (tf * tf);
        end
    end
end
