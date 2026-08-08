function R_mat = getRotMatfromEA(EA_vec,EA_set)
    
	CPHI = cos(EA_vec(1));
	CTHT = cos(EA_vec(2));
	CPSI = cos(EA_vec(3));

	SPHI = sin(EA_vec(1));
	STHT = sin(EA_vec(2));
	SPSI = sin(EA_vec(3));

    switch EA_set
        case 1 % ZYZ
            R_mat = [   CTHT*CPHI*CPSI-SPHI*SPSI, -CPSI*SPHI-CTHT*CPHI*SPSI , CPHI*STHT
                        CPHI*SPSI+CTHT*CPSI*SPHI, CPHI*CPSI-CTHT*SPHI*SPSI  , STHT*SPHI
                        -CPSI*STHT              , STHT*SPSI                 , CTHT      ];
        case 2 % ZYX
            R_mat = [   CTHT*CPHI   , CPHI*SPSI*STHT-CPSI*SPHI  , SPSI*SPHI+CPSI*CPHI*STHT
                        CTHT*SPHI   , CPSI*CPHI+SPSI*STHT*SPHI  , CPSI*STHT*SPHI-CPHI*SPSI
                        -STHT   	, CTHT*SPSI                 , CPSI*CTHT             ];
        case 3 % XYZ
            R_mat = [   CTHT*CPSI               , -CTHT*SPSI                , STHT
                        CPHI*SPSI+CPSI*SPHI*STHT, CPHI*CPSI-SPHI*STHT*SPSI  , -CTHT*SPHI
                        SPHI*SPSI-CPHI*CPSI*STHT, CPSI*SPHI+CPHI*STHT*SPSI  , CPHI*CTHT ];
        otherwise % ZXZ
            R_mat = [   CPHI*CPSI-CTHT*SPHI*SPSI, -CPHI*SPSI-CTHT*CPSI*SPHI , STHT*SPHI
                        CPSI*SPHI+CTHT*CPHI*SPSI, CTHT*CPHI*CPSI-SPHI*SPSI  , -CPHI*STHT
                        STHT*SPSI               , CPSI*STHT                 , CTHT      ];
    end
    
end