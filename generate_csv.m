%left_pos   right_pos   kicker_pos   yaw_rate   left_v   right_v   kicker_v

csv_mat = [left_position(1:10:end), right_position(1:10:end), kicker_position(1:10:end), theta_dot(1:10:end)*pi/180, left_speed(1:10:end)', right_speed(1:10:end)', kicker_speed(1:10:end), zeros(ceil(length(kicker_speed)/10),1), zeros(ceil(length(kicker_speed)/10),1),zeros(ceil(length(kicker_speed)/10),1),zeros(ceil(length(kicker_speed)/10),1),zeros(ceil(length(kicker_speed)/10),1),zeros(ceil(length(kicker_speed)/10),1)];
csvwrite('motion_profile.csv',csv_mat);