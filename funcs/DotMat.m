function D = DotMat(r)
    D = [   r(1)	, r(2)  , r(3)  , 0 	, 0     , 0
            0       , r(1)  , 0     , r(2)  , r(3)  , 0
            0       , 0     , r(1)  , 0     , r(2)  , r(3)  ];
end