function follower = CollisionAvoid(Loc,radius,v,F_obs,D_obs_x,D_obs_y,U_obs)

count = 1;
d = 10;
x_d = D_obs_x(end,:) - D_obs_x(end-1,:);
% dynamic obs traj function
g1 = inline('60 + 5.*sin(0.2 .* x1)'); % should be changed once the dynamic function changes
g2 = inline('80 + 5.*sin(0.2 .* x2)');
sp_flag = 0;
for t = 1:size(Loc,1)
    if sp_flag == 0
        follower(count,:) = Loc(t,:);
    end
    % Find whether agent is out of the area
    if follower(count,1) >= Loc(end,1) && follower(count,2) >= Loc(end,2) 
        break
    end
    
    % Whether time is not enough for dynamic obs
    if count > size(Loc,1)
        new = count - 1;
        D_obs_x(new,:) = D_obs_x(new-1,:) + x_d(:);
        D_obs_y(new,:) = [g1(D_obs_x(new,1)),g1(D_obs_x(new,2))];
    end
    
    D_obs = [D_obs_x(count,:); 
        D_obs_y(count,:)];
    % Collision detection
    collision(count,:) = detection(follower(count,:), radius,F_obs,D_obs,U_obs);
    detect = find(collision(count,:) == 1); 
    
    if isempty(detect)
        count = count + 1;
        sp_flag = 0;
    else
        t_g = 1/v;
        F_obs_traj = [];
        D_obs_traj = [];
        U_obs_traj = [];
        Num_F = size(F_obs,2);
        Num_D = size(D_obs,2);
        Num_U = size(U_obs,2);
        
        for k = 1:length(detect)
            if detect(k) <= Num_F % Fixed Obs collision 
                F_obs_traj = [F_obs_traj,F_obs(:,detect(k))];
            end
            if detect(k) <= (Num_F + Num_D) && detect(k) > Num_F % Dynamic Obs collision
                renum = detect(k) - Num_F;
                D_obs_traj = [D_obs_traj,D_obs(:,renum)];
            end
            if detect(k) <= (Num_F + Num_D + Num_U) && detect(k) > (Num_F + Num_D) % Unknown Obs collision
                renum = detect(k) - (Num_F + Num_D);
                U_obs_traj = [U_obs_traj,U_obs(:,renum)];
            end
        end
%         if sp_flag >= 1
%             bias_x = sp_flag * 1.01 * (follower(count-1,1) - Loc(t-1,1));
%             bias_y = sp_flag * 1.01 * (follower(count-1,2) - Loc(t-1,2));
%             target_pt = [Loc(t,1)+bias_x,Loc(t,2)+bias_y];
%         else
%             target_pt = Loc(t,:);
%         end
%          
%         [avoid_path,dt] = HJ_avoid_obs(detect,follower(count-1,:),target_pt,3,t_g,v,radius,F_obs_traj,...
%             Num_F,D_obs_traj,Num_D,U_obs_traj,Num_U); 
%         if isempty(avoid_path)
%             loop_num = 1;
%             per = 0.9;
%             while isempty(avoid_path)
%                 s_a = atand((Loc(t+1,2) - Loc(t,2))/(Loc(t+1,1) - Loc(t,1)));
%                 
%                 target_pt = per * [t_g * v * cosd(s_a)+ follower(count-1,1),t_g * v * sind(s_a)+ follower(count-1,2)];
%                 [avoid_path,dt,value] = HJ_avoid_obs(detect,follower(count-1,:),target_pt,3,t_g,v,radius,F_obs_traj,...
%                         Num_F,D_obs_traj,Num_D,U_obs_traj,Num_U); 
%                 
%                 per = 0.9 * per;
%                 store_v(loop_num) = value;
%                 loop_num = loop_num + 1;
%                 if loop_num > 2 
%                     error('?')
%                 end
%             end
                
%         end
        step = 1/dt;
        time = 0:1:t_g;

    follower(count,:) = avoid_path(1:2,end);
    collision2(count,:) = detection(follower(count,:), radius,F_obs,D_obs,U_obs);
    if ~isempty(find(collision2 == 1,1))
%         error('The HJ does not work!')
        while ~isempty(find(collision2 == 1,1))
            sp_flag = sp_flag + 1; % next target point should be changed
        if sp_flag >= 1
            obs_loc = find(collision2 == 1);
            for k = 1:length(obs_loc )
                if obs_loc (k) <= Num_F % Fixed Obs collision 
                    F_obs_traj = [F_obs_traj,F_obs(:,obs_loc (k))];
                end
                if obs_loc (k) <= (Num_F + Num_D) && obs_loc (k) > Num_F % Dynamic Obs collision
                    renum = obs_loc (k) - Num_F;
                    D_obs_traj = [D_obs_traj,D_obs(:,renum)];
                end
                if obs_loc (k) <= (Num_F + Num_D + Num_U) && obs_loc (k) > (Num_F + Num_D) % Unknown Obs collision
                    renum = obs_loc (k) - (Num_F + Num_D);
                    U_obs_traj = [U_obs_traj,U_obs(:,renum)];
                end
            end
%             bias_x = sp_flag * 1.01 * (-follower(count,1) + Loc(t-1,1));
%             bias_y = sp_flag * 1.01 * (-follower(count,2) + Loc(t-1,2));
%             target_pt = [Loc(t,1)+bias_x,Loc(t,2)+bias_y];
            r(sp_flag,:) = target_pt;
        else
            target_pt = Loc(t,:);
        end
         
        [avoid_path,dt,value] = HJ_avoid_obs(detect,follower(count-1,:),target_pt,3,t_g,v,radius,F_obs_traj,...
            Num_F,D_obs_traj,Num_D,U_obs_traj,Num_U); 
        follower(count,:) = avoid_path(1:2,end);
        collision2(count,:) = detection(follower(count,:), radius,F_obs,D_obs,U_obs);
        end
    else
        count = count + t_g;
        sp_flag = 0;
    end
    
    end
    
end


end