clear all
close all
% ensure waypoint data structure is already loaded
           % x  y   theta  gear_piston  t concurrent   flywheel  t   not_used  t  vertical_conveyor  t  vision
waypoints = [0,    0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0; ...
             0,   9.5,  0,  1, 2000,  1,  0,  0,  0,  0,  0,  0,  0,  0; ...
             ];
          %   7-22/2/12,   20,   0; ...
          %  5,   20,  0; ...
          %  5,    1,  0];


% define constants
time_step = 1/1000; %s; (1ms timestep)
frequency = 1/time_step; %Hz
total_time = 15; %s

w = 27.5/2/12; %f; half the width of wheelbase
l = 23.125/2/12; %f; half the length of wheelbase

theta_dot_min = 20; %degrees/s
theta_dot_max = 860; %degrees/s
theta_double_dot_max = 180; %degrees/s^2

x_dot_max = sqrt(45); %f/s (70)

x_double_dot_max = 9; %f/s^2 (20)

wheel_circumference = pi * 4 / 12; %f

t = 0;

gear_piston = zeros (total_time/time_step,1);
flywheel = zeros (total_time/time_step,1);
not_used = zeros (total_time/time_step,1);
vertical_conveyor = zeros (total_time/time_step,1);
vision = zeros (total_time/time_step,1);

% generate derivative profiles
for i=1:length(waypoints(:,1))-1
    t_waypoint_start = t;
    t_ramp_x = x_dot_max/x_double_dot_max;
    sign_theta = sign(waypoints(i+1,3) - waypoints(i,3));
    distance_ramp_x = x_dot_max.^2 / (2*x_double_dot_max);
    dist_to_travel = sqrt((waypoints(i+1,1) - waypoints(i,1)).^2 + (waypoints(i+1,2) - waypoints(i,2)).^2);
    t_ss_x = (dist_to_travel - 2*distance_ramp_x)/(x_dot_max);
    if(t_ss_x<0)
        %x_dot_max = sqrt(distance_ramp_x * 2 * x_double_dot_max)
        %distance_ramp_x = dist_to_travel/2;
        t_ramp_x = sqrt(dist_to_travel * x_double_dot_max)/x_double_dot_max;
        t_ss_x = 0;
    end
    if(dist_to_travel == 0)
        t_ramp_x = 0;
        t_ss_x = 0;
    end
    %old math to sync rotation, no longer used
    %theta_dot_max = (waypoints(i+1,3) - waypoints(i,3) - (theta_dot_max ^ 2 / (theta_double_dot_max))) / t_ss_x;
    %theta_dot_max^2 / (theta_double_dot_max * t_ss_x) + theta_dot_max - (waypoints(i+1,3) - waypoints(i,3)) / t_ss_x = 0;
    %quadratic formula based on the above 2 lines:
    
    t_ramp_theta = theta_dot_max/theta_double_dot_max;
    rotation_ramp_theta = theta_dot_max.^2 / (2*theta_double_dot_max);
    rotation_to_travel = abs(waypoints(i+1,3) - waypoints(i,3));
    t_ss_theta = (rotation_to_travel - 2*rotation_ramp_theta)/(theta_dot_max);
    if(t_ss_theta<0)
        t_ramp_theta = sqrt(rotation_to_travel * theta_double_dot_max)/theta_double_dot_max;
        t_ss_theta=0;
    end
    if(rotation_to_travel == 0)
        t_ramp_theta = 0;
        t_ss_theta = 0;
    end
    
    waypoint_time = max(2*t_ramp_x+t_ss_x, 2*t_ramp_theta+t_ss_theta);
    
    time(round(t/time_step)+1,:) = t;
    x_dot(round(t/time_step)+1,:) = 0;
    theta_dot(round(t/time_step)+1,:) = 0;
    target_heading(round(t/time_step)+1)=0;
    t = t + time_step;
    
    
    
    while (t<=(t_waypoint_start + waypoint_time))
        last_xdot = x_dot(round(t/time_step));
        last_thetadot = theta_dot(round(t/time_step));
        if(t<= t_waypoint_start + t_ramp_x)
            xdot = last_xdot + x_double_dot_max * time_step;
        elseif(t<=t_waypoint_start + t_ramp_x + t_ss_x)
            xdot = x_dot_max;
        elseif(t<t_waypoint_start + 2*t_ramp_x + t_ss_x)
            xdot = last_xdot - x_double_dot_max * time_step;
        else
            xdot = 0;
        end
        if(t<= t_waypoint_start + t_ramp_theta)
            thetadot = last_thetadot + sign_theta * theta_double_dot_max * time_step;
        elseif(t<=t_waypoint_start + t_ramp_theta + t_ss_theta)
            thetadot = sign_theta * theta_dot_max;
        elseif(t<t_waypoint_start + 2*t_ramp_theta + t_ss_theta)
            thetadot = last_thetadot - sign_theta * theta_double_dot_max * time_step;
        else
            thetadot = 0;
        end
        if (waypoints(i+1,4) == 1 && waypoints(i+1,6) == 1 && t<=(t_waypoint_start + waypoints(i+1,5)))
            gear_piston(round(t/time_step)+1) = 1;
        end
        % derivative_profiles(round(t/time_step)+1,:) = [t,xdot,thetadot];
        time(round(t/time_step)+1,1) = t;
        x_dot(round(t/time_step)+1,1) = xdot;
        theta_dot(round(t/time_step)+1,1) = thetadot;
        target_heading(round(t/time_step)+1,1)=atan2((waypoints(i+1,2) - waypoints(i,2)),(waypoints(i+1,1) - waypoints(i,1))) * 180/pi;
        t = t + time_step;
    end
    
    if (waypoints(i+1,4) == 1 && waypoints(i+1,6) ~= 1)
        gear_piston(round(t/time_step)+1:round(t/time_step)+1+waypoints(i+1,5)) = 1;
    end
    if (waypoints(i+1,7) == 1)
        flywheel(round(t/time_step)+1:round(t/time_step)+1+waypoints(i+1,8)) = 1;
    end
    if (waypoints(i+1,9) == 1)
        not_used(round(t/time_step)+1:round(t/time_step)+1+waypoints(i+1,10)) = 1;
    end
    if (waypoints(i+1,11) == 1)
        vertical_conveyor(round(t/time_step)+1:round(t/time_step)+1+waypoints(i+1,12)) = 1;
    end
    if (waypoints(i+1,13) == 1)
        vision(round(t/time_step)+1:round(t/time_step)+1+waypoints(i+1,14)) = 1;
    end
    t = t + max([waypoints(i+1,5), waypoints(i+1,7), waypoints(i+1,9), waypoints(i+1,11), waypoints(i+1,13)])*time_step;
end

theta = numerical_integrate(theta_dot, time_step, 0);

for i=1:length(time)
    robot_heading = theta(i);
    target_vector_heading = target_heading(i);
    net_vector_heading(i) = target_vector_heading - robot_heading;
    if(net_vector_heading(i) < -180)
        net_vector_heading(i) = net_vector_heading(i) + 360;
    elseif(net_vector_heading(i) > 180)
        net_vector_heading(i) = net_vector_heading(i) - 360;
    end
    if mod(abs(net_vector_heading(i))/90,2)==1
        forward_speed(i,1) = x_dot(i);
        %if (net_vector_heading(i) > 90 && net_vector_heading(i) <= 180)|| (net_vector_heading(i) < -90 && net_vector_heading(i) >= -180)
        if net_vector_heading(i)<0
            forward_speed(i,1) = -1 * forward_speed(i,1);
        end
        kicker_speed(i,1) = 0;
    elseif mod(abs(net_vector_heading(i))/90,2)==0
        kicker_speed(i,1) = x_dot(i);
        if (net_vector_heading(i) > 90 && net_vector_heading(i) <= 180)|| (net_vector_heading(i) < -90 && net_vector_heading(i) >= -180)
        %if net_vector_heading(i)<0
            kicker_speed(i,1) = -1 * kicker_speed(i,1);
        end
        forward_speed(i,1) = 0;
    else
        forward_to_kicker_ratio = tand(net_vector_heading(i));
        % x2 + y2 = v2
        % y/x = ratio
        % y/sqrt(v2-y2) = ratio
        % y2/(v2-y2) = ratio2
        % y2 = ratio2v2 - ratio2y2
        % y2(ratio2+1) = ratio2v2
        % y = sqrt(ratio2v2/(ratio2+1))
        % use above math to find forward and kicker speeds
        forward_speed(i,1) = sqrt(forward_to_kicker_ratio^2 * x_dot(i)^2 / (forward_to_kicker_ratio^2 + 1));
        kicker_speed(i,1) = forward_speed(i)/forward_to_kicker_ratio;
        if net_vector_heading(i) < 0
            if(forward_speed(i,1) > 0)
                forward_speed(i,1) = -1 * forward_speed(i,1);
            end
        end        
        if (net_vector_heading(i) > 90 && net_vector_heading(i) <= 180)|| (net_vector_heading(i) < -90 && net_vector_heading(i) >= -180)
            if(kicker_speed(i,1) > 0)
                kicker_speed(i,1) = -1 .* kicker_speed(i,1);
            end
        end
    end
end

for i=1:length(time)
    % vr-vl = d*thetadot
    % vnom = (vl + vr)/2
    % vnom = (vl + d * thetadot + vl)/2
    % 2 * vnom - d * thetadot = 2vl
    % vl = vnom - d * thetadot / 2
    % vr = vnom + d * thetadot / 2
    
    theta__dot = theta_dot(i) * pi/180;
    left_speed(i) = forward_speed(i) - w * theta__dot / 2;
    right_speed(i) = forward_speed(i) + w * theta__dot / 2;
end

left_position = numerical_integrate(left_speed, time_step, 0);
right_position = numerical_integrate(right_speed, time_step, 0);
kicker_position = numerical_integrate(kicker_speed, time_step, 0);

left_speed = left_speed';
right_speed = right_speed';

%make everything 15s long
theta(length(theta)+1:(total_time/time_step)) = ones((total_time/time_step)-length(theta),1)*theta(length(theta));
left_position(length(left_position)+1:(total_time/time_step)) = ones((total_time/time_step)-length(left_position),1)*left_position(length(left_position));
right_position(length(right_position)+1:(total_time/time_step)) = ones((total_time/time_step)-length(right_position),1)*right_position(length(right_position));
kicker_position(length(kicker_position)+1:(total_time/time_step)) = ones((total_time/time_step)-length(kicker_position),1)*kicker_position(length(kicker_position));
theta_dot(length(theta_dot)+1:(total_time/time_step)) = ones((total_time/time_step)-length(theta_dot),1)*theta_dot(length(theta_dot));
left_speed(length(left_speed)+1:(total_time/time_step)) = ones((total_time/time_step)-length(left_speed),1)*left_speed(length(left_speed));
right_speed(length(right_speed)+1:(total_time/time_step)) = ones((total_time/time_step)-length(right_speed),1)*right_speed(length(right_speed));
kicker_speed(length(kicker_speed)+1:(total_time/time_step)) = ones((total_time/time_step)-length(kicker_speed),1)*kicker_speed(length(kicker_speed));

run('draw_motion.m')