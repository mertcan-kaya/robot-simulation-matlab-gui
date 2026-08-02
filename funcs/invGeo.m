function qDes = invGeo(app,RGoal,tGoal,q0)
    % invGeo is now a wrapper that extracts parameters from the app 
    % and passes them to the decoupled invGeoCore mathematical solver.

    % Pack the configuration required by Inverse Kinematics
    invGeoConfig.inv_geo_type = app.inv_geo_type;
    invGeoConfig.robot_model = app.robot_model;
    invGeoConfig.TI_0 = app.TI_0;
    invGeoConfig.inv_geo_trn = app.inv_geo_trn;
    invGeoConfig.kp_inv = app.kp_inv;
    invGeoConfig.kr_inv = app.kr_inv;
    invGeoConfig.kp_trn = app.kp_trn;
    invGeoConfig.kr_trn = app.kr_trn;

    % Call the decoupled core function
    qDes = invGeoCore(invGeoConfig, app.kin, RGoal, tGoal, q0);

end