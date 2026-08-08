function ri_j = getri_j_vec(alpha,a,d)
    ri_j = [a;-d*sin(alpha);d*cos(alpha)];
end