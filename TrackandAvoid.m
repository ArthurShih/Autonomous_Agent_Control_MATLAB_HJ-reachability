function [follower,angle_change] = TrackandAvoid(Loc,v,radius,F_obs,D_obs_x,D_obs_y,U_obs,dt)
follower = [Loc(1,1:2)];
pointing_angle = pi/4;
angle_change = [pointing_angle];
U = [];
t_s = 1;
k = 1;
while follower(end,1) <= Loc(end,1) && follower(end,2) <= Loc(end,2)
    init = follower(end,:);
    if k + 1 > length(Loc)
        target = Loc(end,:);
    else
        target = Loc(k + 1,:);
    end
    [x_re,y_re,th_re,u,t_b,collision_loc,flag] = TrackController(init,target,dt,pointing_angle,...
        radius,F_obs,D_obs_x,D_obs_y,U_obs,t_s);
    
    % Updating position and pointing angle
    pointing_angle = th_re(end);
    U = [U;u];
    angle_change = [angle_change;th_re(2:end)];
    follower = [follower;x_re(2:end),y_re(2:end)];
    
    if flag == 1
        init =  follower(end,:);
        interval = -1;
        m = 0;
        while  interval < 0
            m = m + 1;
            if m >= (length(Loc)-k)
                m = length(Loc)-k;
                p(m) = norm(Loc(end,:) - init(:));
                break
            end
            p(m) = norm(Loc(k+m,:) - init(:));
            interval = p(m) - 2 * radius;
        end
        
        D_obs_b = [D_obs_x(t_b,:);D_obs_y(t_b,:)];
        neighbor_b = detection(init,p(m),F_obs,D_obs_b,U_obs);
        loc_b = find(neighbor_b == 1);
        Real_loc = [collision_loc';loc_b];
        Real_loc = unique(Real_loc,'rows');
        [new_path] = ObstacleAvoidance(init,Loc(k+m,1:2),m,th_re(end),radius,Real_loc,...
            v,F_obs,D_obs_b,U_obs);


        follower = [follower;new_path(1,2:end)',new_path(2,2:end)'];
        angle_change = [angle_change;new_path(3,2:end)'];
        if k + 2 * m > length(Loc) 
            range = m;
        else
            range = 2 * m;
        end                  
        for b = 1:range
            ang(b) = atan2(Loc(k + b,2) -  Loc(k + b - 1,2),Loc(k + b,1) -  Loc(k + b - 1,1));
        end
        [diff_v,back2traj] = min(abs(angle_change(end) - ang(:)));  
        ang = [];
        k = k + back2traj;
        t_s = length(follower);
    else
        t_s = length(follower);
        k = k + 1;
    end
    

end
% toc
% 
% figure(1)
%     xlim([0,100])
%     ylim([0,100])
% %     xlim([0,20])
% %     ylim([0,20])
% ObstaclePlot(xlim,ylim,radius,F_obs,U_obs)
% hold on
% plot(Loc(:,1),Loc(:,2),'o')
% for n = 1:length(follower)
%     plot(follower(n,1),follower(n,2),'ro')
%     pause(0.01)
% end
% plot(path_RRT(:,1),path_RRT(:,2))
% hold off

end