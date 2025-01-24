function kin = kinematicParameters(robot_model)

    switch robot_model
        case 5
            DH = [ 0     0     0.550 0
                   0.150 -pi/2 0     -pi/2
                   0.825 0     0     pi/2
                   0     pi/2  0.925 0
                   0     -pi/2 0     0
                   0     pi/2  0     0
                   0     0     0.110 0    ];

            kin.j_type = [1;1;1;1;1;1];
        case 4
            DH = [ 0     0     0.550 0
                   0.150 -pi/2 0     -pi/2
                   0.825 0     0     pi/2
                   0     pi/2  0.625 0
                   0     -pi/2 0     0
                   0     pi/2  0     0
                   0     0     0.110 0    ];

            kin.j_type = [1;1;1;1;1;1];
        case 3
            DH = [ 0        0       0.1035      0
                   0        -pi/2   0           pi
                   0.350    0       0           pi-0.2563
                   0.2253   0       0           0.2563
                   0.072    pi/2    0           pi/2
                   0        pi/2    0           0
                   0        0       0.0982      0];
    
            kin.j_type = [1;1;1;1;1;1];
        case 2
            DH = [ 0       0        0.1519      0
                   0       -pi/2    0           0
                   0.24365 0        0           0
                   0.21325 0        0.11235     0
                   0       pi/2     0.08535     0
                   0       -pi/2    0           0
                   0       0        0.0819      0];
    
            kin.j_type = [1;1;1;1;1;1];
        otherwise
            DH = [ 0       0     0.333 0
                   0       -pi/2 0     0
                   0       pi/2  0.316 0
                   0.0825  pi/2  0     0
                   -0.0825 -pi/2 0.384 0
                   0       pi/2  0     0
                   0.088   pi/2  0     0
                   0       0     0.107 0    ];
    
            kin.j_type = [1;1;1;1;1;1;1];
    end

    kin.a_j = DH(:,1);
    kin.alpha_j = DH(:,2);
    kin.d_j = DH(:,3);
    kin.theta_O_j = DH(:,4);
    kin.n = size(DH,1)-1;

    kin.r1 = 1; % first revolute joint
    kin.r2 = 2; % subsequent revolute joint whose axis is not parallel to the axis of joint r1

    [kin.q_posLim,kin.q_posSafeLim,kin.q_velLim,kin.q_velSafeLim,kin.q_accLim] = jointLimits(robot_model);
    kin.zj_j = [0;0;1];

end