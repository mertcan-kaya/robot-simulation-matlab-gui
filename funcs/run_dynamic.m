function run_dynamic(app)

    d = round(app.ctr.tcyc/app.tstp);

    app.ctr.errsum = zeros(app.kin.n,1);

    app.act.q_pos = app.ini.q_pos;
    app.act.q_vel = zeros(app.kin.n,1);
    app.act.q_acc = zeros(app.kin.n,1);

    sim_time = 0;
    tic
    for k = 0:app.tfin/app.tstp

        if app.running_flag == 0
            break;
        end

        if rem(k, d) == 0
            % feedback
            app.fbk.q_vel = app.act.q_vel;
            app.fbk.q_pos = app.act.q_pos;

            % Trajectory generation
            [app.des.q_pos,app.des.q_vel,app.des.q_acc] = trjGeneration(app,k);
            
            % Control algorithm
            tau = controlAlgo(app.ctr,app.kin,app.dyn,app.des,app.fbk);
        end

        % Computation of robot motion
        app.act.q_acc = robotAlgo(app.robot_model,app.kin,app.dyn,tau,app.act.q_pos,app.act.q_vel);
        app.act.q_vel = app.act.q_vel + app.act.q_acc*app.tstp;
        app.act.q_pos = app.act.q_pos + app.act.q_vel*app.tstp;
        for i = 1:app.kin.n
            if app.act.q_pos(i) < app.kin.q_posLim(i,1)
                app.act.q_pos(i) = app.kin.q_posLim(i,1);
                app.act.q_vel(i) = 0;
                app.act.q_acc(i) = 0;
            elseif app.act.q_pos(i) > app.kin.q_posLim(i,2)
                app.act.q_pos(i) = app.kin.q_posLim(i,2);
                app.act.q_vel(i) = 0;
                app.act.q_acc(i) = 0;
            end
        end

        sim_time = sim_time + app.tstp;
        real_time = toc;

        if sim_time > real_time*app.time_const
            plotAxesOutApp(app)
            app.secLabel.Text = append(num2str(k*app.tstp,'%.2f'), ' sec');
            qact_text = 'qact=[';
            for j = 1:app.kin.n-1
                qact_text = append(qact_text,num2str(rad2deg(app.act.q_pos(j,1)),'%4.0f'),',');
            end
            qact_text = append(qact_text,num2str(rad2deg(app.act.q_pos(app.kin.n,1)),'%4.0f'),']deg');
            app.q_actq1q2q3q4q5q6q7degLabel.Text = qact_text;
        end

    end

end