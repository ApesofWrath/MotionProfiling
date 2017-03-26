%theta    left_pos   right_pos   kicker_pos   yaw_rate   left_v   right_v   kicker_v
%gear_piston   flywheel   not_used   vertical_conveyor   vision
csv_mat = [theta(1:10:end), left_position(1:10:end), right_position(1:10:end), kicker_position(1:10:end), theta_dot(1:10:end)*pi/180, left_speed(1:10:end), right_speed(1:10:end), kicker_speed(1:10:end), gear_piston(1:10:end), flywheel(1:10:end), not_used(1:10:end), vertical_conveyor(1:10:end), vision(1:10:end), zeros(ceil(length(kicker_speed)/10),1)];
csvwrite('motion_profile.csv',csv_mat);