center_rotation(1) = 0;

starting_x = 20;
starting_y = 0 + l;

velocity_vector_field_centric(:,1) = [0; 0];

robot_corners_fixed = [w, -w, -w, w, w; ...  %x
                       l,  l, -l,-l, l];    %y

for i=2:length(time)
    center_rotation(i) = (right_position(i) - left_position(i))/w * 180/pi;
    velocity_vector_field_centric(:,i) = [cosd(center_rotation(i)),  -sind(center_rotation(i));...
                                          sind(center_rotation(i)), cosd(center_rotation(i))] * ...
                                         [kicker_speed(i); forward_speed(i)];
                                     
end
center_x = numerical_integrate(velocity_vector_field_centric(1,:), time_step, starting_x)';
center_y = numerical_integrate(velocity_vector_field_centric(2,:), time_step, starting_y)';

for i=1:length(time)
    robot_corners((2*i - 1):2*i,:) = [cosd(center_rotation(i)), -sind(center_rotation(i)); ...
                                      sind(center_rotation(i)),  cosd(center_rotation(i))] * ...
                                      robot_corners_fixed + ...
                                      [center_x(i), center_x(i), center_x(i), center_x(i), center_x(i); ...
                                      center_y(i), center_y(i), center_y(i), center_y(i), center_y(i)];
end

skip_step = 10;
subplot(3,1,1);
plot([0 27 27 0 0], [0 0 54 54 0]);
axis equal

robot_x = robot_corners(1,:);
robot_y = robot_corners(2,:);

hold on
field = plot(robot_x, robot_y);
hold off

subplot(3,1,2);
hold on
left_line = animatedline('Color','r');
right_line = animatedline('Color','g');
kicker_line = animatedline('Color','b');
axis([0, max(time), min(min([left_position,right_position,kicker_position])), max(max([left_position,right_position,kicker_position]))])
addpoints(left_line,time(1),left_position(1));
addpoints(right_line,time(1),right_position(1));
addpoints(kicker_line,time(1),kicker_position(1));
legend('Left wheel displacement', 'Right wheel displacement', 'Kicker wheel displacement');
hold off

subplot(3,1,3);
hold on
left_line_v = animatedline('Color','r');
right_line_v = animatedline('Color','g');
kicker_line_v = animatedline('Color','b');
axis([0, max(time), min(min([left_speed',right_speed',kicker_speed])), max(max([left_speed',right_speed',kicker_speed]))])
addpoints(left_line_v,time(1),left_speed(1));
addpoints(right_line_v,time(1),right_speed(1));
addpoints(kicker_line_v,time(1),kicker_speed(1));
legend('Left wheel velocity', 'Right wheel velocity', 'Kicker wheel velocity');
hold off

for i=3:2 * skip_step:length(robot_corners)
    %subplot(1,1);
    robot_x = robot_corners(i,:);
    robot_y = robot_corners(i+1,:);
    field.XData = robot_x;
    field.YData = robot_y;
    %subplot(2,1);
    addpoints(left_line,time(round((i-1)/2)+1),left_position(round((i-1)/2)+1));
    addpoints(right_line,time(round((i-1)/2)+1),right_position(round((i-1)/2)+1));
    addpoints(kicker_line,time(round((i-1)/2)+1),kicker_position(round((i-1)/2)+1));
    
    addpoints(left_line_v,time(round((i-1)/2)+1),left_speed(round((i-1)/2)+1));
    addpoints(right_line_v,time(round((i-1)/2)+1),right_speed(round((i-1)/2)+1));
    addpoints(kicker_line_v,time(round((i-1)/2)+1),kicker_speed(round((i-1)/2)+1));
    
    drawnow
    pause(time_step*skip_step)
end
    

%plot(center_x,center_y);