function T = SO3R3_SE3(R,r)
    T = [R,r;zeros(1,3),1];
end