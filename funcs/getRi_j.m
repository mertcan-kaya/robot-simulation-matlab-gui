function Ri_j = getRi_j(alpha,theta)
    Ri_j = rot_x(alpha)*rot_z(theta);

    function output = rot_x(input)
        output = [	1	0           0
                    0	cos(input)  -sin(input)
                    0	sin(input)	cos(input)];
    end
    
    function output = rot_z(input)
        output = [  cos(input)	-sin(input)	0
                    sin(input)	 cos(input)	0
                    0            0          1];
    end

end