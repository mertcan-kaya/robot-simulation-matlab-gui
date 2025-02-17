function run_kinematic(app)

    app.des.q_pos = app.fin.q_pos;

    sim_time = 0;
    tic
    for k = 0:app.tfin_trj/app.tstp

        if app.running_flag == 0
            break;
        end

        % Trajectory generation
        [app.act.q_pos,app.act.q_vel,app.act.q_acc] = trjGeneration(app,k);

        sim_time = sim_time + app.tstp;
        real_time = toc;
        
        if sim_time > real_time
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