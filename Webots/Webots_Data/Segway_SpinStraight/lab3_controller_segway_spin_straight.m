% MATLAB controller for Webots
% File:          lab3_controller.m
% Date:
% Description:
% Author:
% Modifications:


%% Define Type of Task
%task = 'right_turn';
task = 'spin_straight';




% Get simulation time step
TIME_STEP = wb_robot_get_basic_time_step(); % change under Worldinfo, basicTimeStep

% Define motors and sensors
motor_R = wb_robot_get_device('motor_R');
motor_L = wb_robot_get_device('motor_L');
compass = wb_robot_get_device('compass');
gyro = wb_robot_get_device('gyro');
lidar_F = wb_robot_get_device('lidar_F');
lidar_R = wb_robot_get_device('lidar_R');
robot = wb_supervisor_node_get_from_def('robot'); % ground truth access
rotation = wb_supervisor_node_get_field(robot,'rotation');

% Initialize sensors
wb_compass_enable(compass, TIME_STEP);
wb_gyro_enable(gyro, TIME_STEP);
wb_distance_sensor_enable(lidar_F, TIME_STEP);
wb_distance_sensor_enable(lidar_R, TIME_STEP);

% Initialize motors as non-position control
wb_motor_set_position(motor_R, inf);
wb_motor_set_position(motor_L, inf);

% Input max time
max_timestep = ceil(20000/TIME_STEP); % divides total time in ms by time step

% Define movement arrays
compass_library = zeros([2,max_timestep]);

gyro_library =  zeros([1,max_timestep]);
distance_F_library =  zeros([1,max_timestep]);
distance_R_library =  zeros([1,max_timestep]);

position_library = zeros([2,max_timestep]);

angle_library = zeros([1,max_timestep]);
angular_velocity_library =  zeros([1,max_timestep]);

% Input velocities
switch task
  case 'right_turn'
    right_wheel_velocity = -1.75;
    left_wheel_velocity = -2;
  case 'spin_straight'
    load('spin_straight_data.mat');
    right_wheel_velocity = right;
    left_wheel_velocity = left;
end
    

% Initialize loop steps
step = 1;

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1
  % Set motor velocities
  switch task
    case 'right_turn'
      wb_motor_set_velocity(motor_R, right_wheel_velocity);
      wb_motor_set_velocity(motor_L, left_wheel_velocity);
    case 'spin_straight'
      wb_motor_set_velocity(motor_R, right_wheel_velocity(step));
      wb_motor_set_velocity(motor_L, left_wheel_velocity(step));
   end
  
  % Read sensor data
  compass_data = wb_compass_get_values(compass);
  gyro_data = wb_gyro_get_values(gyro);
  distance_F_data = wb_distance_sensor_get_value(lidar_F);
  distance_R_data = wb_distance_sensor_get_value(lidar_R);
  
  % output sensory data to arrays
  compass_library(1,step) = compass_data(1);
  compass_library(2,step) = compass_data(3);
  gyro_library(step) = gyro_data(2); % Y-axis angular velocity
  distance_F_library(step) = distance_F_data; % Record front distance sensor 
  distance_R_library(step) = distance_R_data; % Record right distance sensor
  
  position = wb_supervisor_node_get_position(robot);
  angle = wb_supervisor_field_get_sf_rotation(rotation);
  velocity = wb_supervisor_node_get_velocity(robot);
  position_library(1,step) = position(1);
  position_library(2,step) = position(3);
  angle_library(step) = angle(4);
  angular_velocity_library(step) = velocity(5); % Y-axis angular velocity
  

  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
  
  % step to next timestep, exit after total time expired
  
  if step >= max_timestep
    break;
  end
  step = step + 1;
end

% cleanup code goes here: write data to files, etc.
wb_robot_cleanup();


desktop;
keyboard;

% time (in units of timestep)
x = 1:max_timestep;

% plots of various data
figure(1) % Large right turn ONLY
plot(x, distance_F_library*10/4096);
hold on
plot(x, distance_R_library*10/4096);
hold off
xlabel('timestep')
ylabel('sensor distance (m)')
legend('Front sensor distance', 'Right sensor distance');

figure(2)
plot(x, compass_library(1,:));
hold on
plot(x, compass_library(2,:));
plot(x, gyro_library);
plot(x, angular_velocity_library);
plot(x, angle_library);
hold off

figure(3)
plot(x, position_library(1,:));
hold on
plot(x, position_library(2,:));
