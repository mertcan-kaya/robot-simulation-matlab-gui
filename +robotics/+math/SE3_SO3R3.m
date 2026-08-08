function [R,r] = SE3_SO3R3(T)
    R = T(1:3,1:3);
    r = T(1:3,4);
end