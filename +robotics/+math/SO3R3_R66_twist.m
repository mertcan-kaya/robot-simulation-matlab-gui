function X = SO3R3_R66_twist(R,r)
    X = [R,robotics.math.SkewSym(r)*R;zeros(3),R];
end