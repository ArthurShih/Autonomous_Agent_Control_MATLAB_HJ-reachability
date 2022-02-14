function [follower,collision,collision2] = CollisionAvoid(Loc,radius,v,F_obs,D_obs_x,D_obs_y,U_obs)

count = 1;
d = 10;
x_d = D_obs_x(end,:) - D_obs_x(end-1,:);
% dynamic obs traj function
g1 = inline('60 + 5.*sin(0.2 .* x1)'); % should be changed once the dynamic function changes
g2 = inline('80 + 5.*sin(0.2 .* x2)');

for t = 1:size(Loc,1)
    follower(count,:) = Loc(t,:);
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
    else
        t_g = 2/v;
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
        dis_p2p = norm(follower(count-1,1:2)-Loc(t,1:2));
        if dis_p2p >= (v*t_g)
            per = (v*t_g)/dis_p2p; 
            target_pt = [(Loc(t,1) - follower(count-1,1))* per + follower(count-1,1),...
                (Loc(t,2) - follower(count-1,2))* per + follower(count-1,2)];
        else
            target_pt = Loc(t,:);
        end
        [avoid_path,dt] = HJ_avoid_obs(detect,follower(count-1,:),target_pt,3,t_g,v,radius/2,F_obs_traj,...
            Num_F,D_obs_traj,Num_D,U_obs_traj,Num_U);       
        step = 1/dt;
        time = 0:1:t_g;
%         for n =  1:length(time)
%             follower(count+time(n),:) = avoid_path(1+step*time(n),1:2);
%         end
        collision2(count,:) = detection(follower(count,:), radius,F_obs,D_obs,U_obs);

    follower(count,:) = avoid_path(1:2,end);
    count = count + t_g;
    end
    
end


end