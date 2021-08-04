clear
close all
clc

%% TODO BRYAN
% ) Where to place this block of text?
% we log data for the same task (= # uavs, = static world, = task initilization and reference/goals)
% variations for each task consist out of ~= disturbances (e.g. wind,
% time-varying obstacles), simulations (.sim) or physical experiments (.exp), 
% methods (D-ERG, D-RG, D-CG, D-MPC) and parameters (e.g. method gains, settings)
% taskRunId = 1 % id number of task between 1 and inf
% uavId = 1 % id number of uav as a 1 or 2 digit number ebtween 1 and 99
% data.taskRun{taskRunId,1}.uav{uavId,1}
% maybe ebtter to represent attitude as quaternions xyzw (think about
% heading) . See example code kelly
% ) custom_dt, --> TODO: save time in code
% ) allUavIds seems to be unused. maybe just store as information
% add manually published desired angular rate of controller in all controllers
% Check https://ctu-mrs.github.io/docs/system/uav_ros_interface.html for a lsit of moslty used topics
% ---------------- check here what's sueful to add ------------------------
%trajectory_generation/path 
% gain_manager/diagnostics
% rostopic echo /uav1/odometry/odom_gps
% /uav1/odometry/rtk_local_odom
% rostopic echo /uav1/control_manager/control_error
% rostopic echo /uav1/mavros/setpoint_raw/attitude -> .orientation (todo),
% .body_rate (OK) , .thrust (OK)
% /uav1/odometry/altitude
% rostopic echo /uav1/odometry/height
% rostopic echo /uav1/odometry/max_altitude
% /uav1/odometry/imu_untilted
% rostopic echo /uav1/odometry/lkf_states_x
% rostopic echo /uav1/odometry/lkf_states_y
% rostopic echo /uav1/odometry/odom_main (OK), todo: attitude
% /uav1/odometry/heading_state_out
% rostopic echo /uav1/odometry/odom_main_innovation
% /uav1/odometry/rtk_local
% /uav1/odometry/rtk_out
% rostopic echo /uav1/odometry/diagnostics (which estimator is active)
% rtk stuff?
% rostopic echo /uav1/constraint_manager/diagnostics
% rostopic echo /uav1/control_manager/cmd_twist_out (better understand tis
% use)
% rostopic echo /uav1/control_manager/current_constraints
% rostopic echo /uav1/control_manager/heading
% /uav1/control_manager/mpc_tracker/current_trajectory_point
% /uav1/control_manager/mpc_tracker/diagnostics
% /uav1/control_manager/mpc_tracker/mpc_reference_debugging
% /uav1/control_manager/mpc_tracker/parameter_descriptions
% /uav1/control_manager/mpc_tracker/parameter_updates
% /uav1/control_manager/mpc_tracker/predicted_trajectory (also do this for
% the derg_tracker)
% /uav1/control_manager/mpc_tracker/predicted_trajectory_debugging
% /uav1/control_manager/mpc_tracker/trajectory_processed/markers
% /uav1/control_manager/mpc_tracker/trajectory_processed/poses
% rostopic echo /uav1/control_manager/mass_estimate
% rostopic echo /uav1/control_manager/reference (not published, why ????)
% /uav1/control_manager/se3_controller/parameter_descriptions
% /uav1/control_manager/se3_controller/parameter_updates
% rostopic echo /uav1/control_manager/tilt_error (important)
% % rostopic echo /uav1/control_manager/attitude_cmd (lots of useful info: disred acceleration, attitude, disturbances)
% rostopic echo /uav1/mavros/local_position/velocity_local
% rostopic echo /uav1/control_manager/cmd_odom (lots of useful info)
% rostopic echo /uav1/mavros/imu/data  .orientation TODO
% add the following maybe useful topics?
% rqt_graph
% rostopic echo 
% /uav1/mavros/battery
% /uav1/mavros/global_position/global . latitude . longitude . altitude
% rostopic echo /uav1/mavros/global_position/local
% rostopic echo /uav1/mavros/local_position/pose
% rostopic echo /uav1/garmin/range
% rostopic echo /uav1/odometry/gps_local_odom
% rostopic echo /uav1/odometry/hdg_pixhawk_out
% rostopic echo /uav1/control_manager/position_cmd lot of useful info of (important, thrust = 0 , why? maybe a feedforward = 0) .position, .acceleration .jerk (for mpc tracker all nonzero, for bypasstracekr, dergtracker only position command given)

%% USERS READ ME
%% What does it do? 
% This script converts single or multiple logged .bag files to a single
% .mat file. The .mat file can be used for plotting data of simulations and
% hardware experiments. The bag file should contain topics named in a
% specific way such that they can be interpreted independent of the type of
% controller or tracker (see convention). We also allow the use of ctu
% trackers and controllers, but one needs to make a copy of these .cpp
% files and add the topic publishers required to make graphs.

%% Topic naming convention:
% Each topic starts with /uav#/ where # is a value between 0 and 99
% Each tracker and controller should publish topics under the same
% names. Please take a look below, ctrl + f "_controller/" or "_tracker/".
% This means the controller and tracker names end with _controller and
% _tracker.

%% Other assumptions made:
% We assume that the logged data of the controller and tracker of a
% specific uav only consist of a single controller / tracker type. If
% multiple controllers / trackers where activated and logged during a single
% experiment, only the last logged controller / tracker will be saved in
% the .mat file.

%% How to use this script?
% ) only change the sections starting with [USER INPUT].
% ) do not copy this file to other locations than the default. Instead use
% this file.
% ) contact bryan.convens@gmail.com if think you need you need to change
% this code. This can be e.g. to add custom topics not yet present as default.

%% [USER INPUT] Bag file per uav (columns) and this for multiple task runs (rows)
% 1) example with 1 uav:
% allbagfilenames = {'bagfile_uav1'};  
% 2) example for 2 uavs:
% allbagfilenames = {'bagfile_uav1', 'bagfile_uav2'};
% 3) example for 2 uavs with multiple task runs:
% allbagfilenames = {'bagfile0_uav1', 'bagfile0_uav2';...
%                  'bagfile1_uav1', 'bagfile1_uav2';...
%                  'bagfile2_uav1', 'bagfile2_uav2';...
%                  'bagfile3_uav1', 'bagfile3_uav2'};

%% add your bagfile names here:
% allbagfilenames = {'bagfile_uav1'};
% allbagfilenames = {'bagfile1_uav1';
%                    'bagfile2_uav1'};

% allbagfilenames = {'bagfile1_uav1', 'bagfile1_uav2';
%                    'bagfile2_uav1', 'bagfile2_uav2'}

% allbagfilenames = {'bagfile_dergOn_uav1';
%                    'bagfile_dergOff_dsm10_uav1';
%                    'bagfile_dergOff_dsm20_uav1'};

% allbagfilenames = {'bagfile_reducedKappa25_crash_uav1';
%                    'bagfile_reducedKappa25_oscillations_uav1';
%                    'bagfile_reducedKappa25_smooth_uav1'};

% allbagfilenames = {'bag-uav1-simulation-f450-rtk-bypass_tracker-se3_copy_controller'};  

% allbagfilenames = {'bag-1-uav1-simulation-f450-rtk-dergbryan_tracker-se3_brubotics_controller';
%                    'bag-2-uav1-simulation-f450-rtk-dergbryan_tracker-se3_brubotics_controller'};


%allbagfilenames = {'bag-uav1-simulation-f450-rtk-dergbryan_tracker-se3_brubotics_controller'};
allbagfilenames = {'bag-9-drone-DERG-obstacle-avoidance-uav1-simulation-f450-rtk-dergbryan_tracker-Se3BruboticsController'}; %, 'bag-1-uav2-simulation-f450-rtk-dergbryan_tracker-se3_brubotics_controller';
                   %'bag-2-uav1-simulation-f450-rtk-dergbryan_tracker-se3_brubotics_controller', 'bag-2-uav2-simulation-f450-rtk-dergbryan_tracker-se3_brubotics_controller'};


%% [USER INPUT] change these settings if you like to
matfilename = 'bag-9-drone-DERG-obstacle-avoidance-uav1-simulation-f450-rtk-dergbryan_tracker-Se3BruboticsController';
custom_dt = 0.010; % sample time of controller used in trajectory prediciton %% [TODO bring in via rosbag log so user does not have to define it]

%% start the for loops
numAllbagfilenames = size(allbagfilenames,1)*size(allbagfilenames,2);
bagfilecounter = 0; % initilize
for taskRunId = 1: size(allbagfilenames,1)
    for allbagfilenames_col = 1: size(allbagfilenames,2)
        bagfilecounter = bagfilecounter +1;
        %% load the .bag file
        filename = allbagfilenames(taskRunId, allbagfilenames_col); %'bagfile_uav2';
        bagname = strcat(filename,'.bag'); bagname = bagname{1,1};
        X = sprintf('bagfile: %d / %d, current bagname: %s.',bagfilecounter,numAllbagfilenames,bagname);
        disp(' '); disp(X);
        bag = rosbag(bagname); 
        bagInfo = rosbag('info',bagname);
        allTopicsTable = bag.AvailableTopics;
        allTopicsNames = allTopicsTable.Properties.RowNames;
        numAllTopics = length(allTopicsNames);
        % we assume uavs are numbered as: uav1, uav2, uav7,... , uav10,
        % uav88. No uav100 allowed.
        topicName = allTopicsNames(1); topicName = topicName{1,1}; % for whatever topicName, same uavId is used
        if ~isnan(str2double(topicName(6))) % [2 digit uavId] 6 corresponds to posible second digit e.g. /uav10 to /uav99
            uavId = str2double(topicName(5:6));
        else % [only 1 digit uavId] 5 corresponds to first digit e.g. /uav0 to /uav9
            uavId = str2double(topicName(5));
        end
        
        for topicId = 1: numAllTopics
            topicName = allTopicsNames(topicId); topicName = topicName{1,1};
            X = sprintf('topic: %d / %d, current topic: %s.',topicId,numAllTopics,topicName);
            disp(X);
            %% [USER INPUT OPTIONAL]
            % users, if new topics need to be plot, add them here like the
            % code section below. The more specific the topic, the more you
            % place it at the end. Keep it structured and organized.
            
            %% /odometry/odom_main
            pat = '/odometry/odom_main';
            if contains(topicName,pat)
                bag_odom_main = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.time = bag_odom_main.MessageList.Time;
                msgs_odom_main = readMessages(bag_odom_main,'DataFormat','struct');
                % preallocation
                taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.Pose.Pose.Position = zeros(length(msgs_odom_main),3);
                taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.Twist.Twist.Linear = zeros(length(msgs_odom_main),3);
                for t = 1 : length(msgs_odom_main)
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.Pose.Pose.Position(t,1)=msgs_odom_main{t}.Pose.Pose.Position.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.Pose.Pose.Position(t,2)=msgs_odom_main{t}.Pose.Pose.Position.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.Pose.Pose.Position(t,3)=msgs_odom_main{t}.Pose.Pose.Position.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.Twist.Twist.Linear(t,1)=msgs_odom_main{t}.Twist.Twist.Linear.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.Twist.Twist.Linear(t,2)=msgs_odom_main{t}.Twist.Twist.Linear.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_main.Twist.Twist.Linear(t,3)=msgs_odom_main{t}.Twist.Twist.Linear.Z;
                %  [yaw, pitch, roll] = quat2angle([msgs_odom_main{t}.Pose.Pose.Orientation.W msgs_odom_main{t}.Pose.Pose.Orientation.X msgs_odom_main{t}.Pose.Pose.Orientation.Y msgs_odom_main{t}.Pose.Pose.Orientation.Z]);
                %     attitude(t,1) = roll;
                %     attitude(t,2) = pitch;
                %     attitude(t,3) = yaw;
                end
                clear bag_odom_main msgs_odom_main
            end
            %% /odometry/uav_state
            pat = '/odometry/uav_state';
            if contains(topicName,pat)
                bag_uav_state = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.time = bag_uav_state.MessageList.Time;
                msgs_uav_state = readMessages(bag_uav_state,'DataFormat','struct');
                for t = 1 : length(msgs_uav_state)
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Pose.Position(t,1) = msgs_uav_state{t}.Pose.Position.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Pose.Position(t,2) = msgs_uav_state{t}.Pose.Position.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Pose.Position(t,3) = msgs_uav_state{t}.Pose.Position.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Velocity.Linear(t,1) = msgs_uav_state{t}.Velocity.Linear.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Velocity.Linear(t,2) = msgs_uav_state{t}.Velocity.Linear.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Velocity.Linear(t,3) = msgs_uav_state{t}.Velocity.Linear.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Velocity.Angular(t,1) = msgs_uav_state{t}.Velocity.Angular.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Velocity.Angular(t,2) = msgs_uav_state{t}.Velocity.Angular.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Velocity.Angular(t,3) = msgs_uav_state{t}.Velocity.Angular.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Acceleration.Linear(t,1) = msgs_uav_state{t}.Acceleration.Linear.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Acceleration.Linear(t,2) = msgs_uav_state{t}.Acceleration.Linear.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.uav_state.Acceleration.Linear(t,3) = msgs_uav_state{t}.Acceleration.Linear.Z;
        
                end
                clear bag_uav_state msgs_uav_state
            end
            
           %% /odometry/odom_mavros
            pat = '/odometry/odom_mavros';
            if contains(topicName,pat)
                bag_this = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.time = bag_this.MessageList.Time;
                msgs_this = readMessages(bag_this,'DataFormat','struct');
                for t = 1 : length(msgs_this)
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Pose.Pose.Position(t,1) = msgs_this{t}.Pose.Pose.Position.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Pose.Pose.Position(t,2) = msgs_this{t}.Pose.Pose.Position.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Pose.Pose.Position(t,3) = msgs_this{t}.Pose.Pose.Position.Z;   
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Twist.Twist.Linear(t,1)=msgs_this{t}.Twist.Twist.Linear.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Twist.Twist.Linear(t,2)=msgs_this{t}.Twist.Twist.Linear.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Twist.Twist.Linear(t,3)=msgs_this{t}.Twist.Twist.Linear.Z; 
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Twist.Twist.Angular(t,1)=msgs_this{t}.Twist.Twist.Angular.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Twist.Twist.Angular(t,2)=msgs_this{t}.Twist.Twist.Angular.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.odom_mavros.Twist.Twist.Angular(t,3)=msgs_this{t}.Twist.Twist.Angular.Z; 
                end
                clear bag_this msgs_this
            end
            
            %% /odometry/rtk_local_odom
            pat = '/odometry/rtk_local_odom';
            if contains(topicName,pat)
                bag_this = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.time = bag_this.MessageList.Time;
                msgs_this = readMessages(bag_this,'DataFormat','struct');
                for t = 1 : length(msgs_this)
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Pose.Pose.Position(t,1) = msgs_this{t}.Pose.Pose.Position.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Pose.Pose.Position(t,2) = msgs_this{t}.Pose.Pose.Position.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Pose.Pose.Position(t,3) = msgs_this{t}.Pose.Pose.Position.Z;   
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Twist.Twist.Linear(t,1)=msgs_this{t}.Twist.Twist.Linear.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Twist.Twist.Linear(t,2)=msgs_this{t}.Twist.Twist.Linear.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Twist.Twist.Linear(t,3)=msgs_this{t}.Twist.Twist.Linear.Z; 
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Twist.Twist.Angular(t,1)=msgs_this{t}.Twist.Twist.Angular.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Twist.Twist.Angular(t,2)=msgs_this{t}.Twist.Twist.Angular.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.odometry.rtk_local_odom.Twist.Twist.Angular(t,3)=msgs_this{t}.Twist.Twist.Angular.Z; 
                end
                clear bag_this msgs_this
            end
            
            %% /mavros/setpoint_raw/attitude
            pat = '/mavros/setpoint_raw/attitude';
            if contains(topicName,pat)
                bag_this = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.mavros.setpoint_raw.attitude.time = bag_this.MessageList.Time;
                msgs_this = readMessages(bag_this,'DataFormat','struct');
                for t = 1 : length(msgs_this)
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.setpoint_raw.attitude.thrust(t,1) = msgs_this{t}.Thrust;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.setpoint_raw.attitude.body_rate(t,1) = msgs_this{t}.BodyRate.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.setpoint_raw.attitude.body_rate(t,2) = msgs_this{t}.BodyRate.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.setpoint_raw.attitude.body_rate(t,3) = msgs_this{t}.BodyRate.Z;
                end
                clear bag_this msgs_this
            end
            
            %% /mavros/local_position/velocity_body
            pat = '/mavros/local_position/velocity_body';
            if contains(topicName,pat)
                bag_this = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.mavros.local_position.velocity_body.time = bag_this.MessageList.Time;
                msgs_this = readMessages(bag_this,'DataFormat','struct');
                for t = 1 : length(msgs_this)
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.local_position.velocity_body.Linear(t,1) = msgs_this{t}.Twist.Linear.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.local_position.velocity_body.Linear(t,2) = msgs_this{t}.Twist.Linear.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.local_position.velocity_body.Linear(t,3) = msgs_this{t}.Twist.Linear.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.local_position.velocity_body.Angular(t,1) = msgs_this{t}.Twist.Angular.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.local_position.velocity_body.Angular(t,2) = msgs_this{t}.Twist.Angular.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.local_position.velocity_body.Angular(t,3) = msgs_this{t}.Twist.Angular.Z;
                end
                clear bag_this msgs_this
            end
            
            %% /mavros/imu/data
            pat = '/mavros/imu/data';
            if contains(topicName,pat)
                bag_this = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.mavros.imu.data.time = bag_this.MessageList.Time;
                msgs_this = readMessages(bag_this,'DataFormat','struct');
                for t = 1 : length(msgs_this)
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.imu.data.LinearAcceleration(t,1) = msgs_this{t}.LinearAcceleration.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.imu.data.LinearAcceleration(t,2) = msgs_this{t}.LinearAcceleration.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.imu.data.LinearAcceleration(t,3) = msgs_this{t}.LinearAcceleration.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.imu.data.AngularVelocity(t,1) = msgs_this{t}.AngularVelocity.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.imu.data.AngularVelocity(t,2) = msgs_this{t}.AngularVelocity.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.mavros.imu.data.AngularVelocity(t,3) = msgs_this{t}.AngularVelocity.Z;
                    %% add orientation
                end
                clear bag_this msgs_this
            end
            
            %% /control_manager/cmd_odom
            pat = '/control_manager/cmd_odom';
            if contains(topicName,pat)
                bag_cmd_odom = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.time = bag_cmd_odom.MessageList.Time;
                msgs_cmd_odom = readMessages(bag_cmd_odom,'DataFormat','struct');
                for t = 1 : length(msgs_cmd_odom)
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Pose.Pose.Position(t,1) = msgs_cmd_odom{t}.Pose.Pose.Position.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Pose.Pose.Position(t,2) = msgs_cmd_odom{t}.Pose.Pose.Position.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Pose.Pose.Position(t,3) = msgs_cmd_odom{t}.Pose.Pose.Position.Z;   
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Twist.Twist.Linear(t,1)=msgs_cmd_odom{t}.Twist.Twist.Linear.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Twist.Twist.Linear(t,2)=msgs_cmd_odom{t}.Twist.Twist.Linear.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Twist.Twist.Linear(t,3)=msgs_cmd_odom{t}.Twist.Twist.Linear.Z; 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Twist.Twist.Angular(t,1)=msgs_cmd_odom{t}.Twist.Twist.Angular.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Twist.Twist.Angular(t,2)=msgs_cmd_odom{t}.Twist.Twist.Angular.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.cmd_odom.Twist.Twist.Angular(t,3)=msgs_cmd_odom{t}.Twist.Twist.Angular.Z; 
                %     [yaw, pitch, roll] = quat2angle([msgs_uavcmd_odom{t}.Pose.Pose.Orientation.W msgs_uavcmd_odom{t}.Pose.Pose.Orientation.X msgs_uavcmd_odom{t}.Pose.Pose.Orientation.Y msgs_uavcmd_odom{t}.Pose.Pose.Orientation.Z]);
                %     cmd_odom(t,4) = roll;
                %     cmd_odom(t,5) = pitch;
                %     cmd_odom(t,6) = yaw;
                end
                clear bag_cmd_odom msgs_cmd_odom
            end
            %% /control_manager/attitude_cmd
            pat = '/control_manager/attitude_cmd';
            if contains(topicName,pat)
                bag_attitude_cmd = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.control_manager.attitude_cmd.time = bag_attitude_cmd.MessageList.Time;
                msgs_attitude_cmd = readMessages(bag_attitude_cmd,'DataFormat','struct');
                for t = 1 : length(msgs_attitude_cmd)
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.attitude_cmd.Thrust(t,1) = msgs_attitude_cmd{t}.Thrust;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.attitude_cmd.AttitudeRate(t,1) = msgs_attitude_cmd{t}.AttitudeRate.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.attitude_cmd.AttitudeRate(t,2) = msgs_attitude_cmd{t}.AttitudeRate.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.attitude_cmd.AttitudeRate(t,3) = msgs_attitude_cmd{t}.AttitudeRate.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.attitude_cmd.DesiredAcceleration(t,1) = msgs_attitude_cmd{t}.DesiredAcceleration.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.attitude_cmd.DesiredAcceleration(t,2) = msgs_attitude_cmd{t}.DesiredAcceleration.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.attitude_cmd.DesiredAcceleration(t,3) = msgs_attitude_cmd{t}.DesiredAcceleration.Z;
                    %% add more relevant stuff
                end
                clear bag_attitude_cmd msgs_attitude_cmd
            end

            %% /control_manager/thrust_force
            pat = '/control_manager/thrust_force';
            if contains(topicName,pat)
                bag_this = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.control_manager.thrust_force.time = bag_this.MessageList.Time;
                msgs_this = readMessages(bag_this,'DataFormat','struct');
                for t = 1 : length(msgs_this)
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.thrust_force.value(t,1) = msgs_this{t}.Value;
                end
                clear bag_this msgs_this
            end
            
            %% /control_manager/position_cmd
            pat = '/control_manager/position_cmd';
            if contains(topicName,pat)
                bag_this = select(bag,'Topic', topicName); 
                taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.time = bag_this.MessageList.Time;
                msgs_this = readMessages(bag_this,'DataFormat','struct');
                for t = 1 : length(msgs_this)
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Position(t,1) = msgs_this{t}.Position.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Position(t,2) = msgs_this{t}.Position.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Position(t,3) = msgs_this{t}.Position.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Velocity(t,1) = msgs_this{t}.Velocity.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Velocity(t,2) = msgs_this{t}.Velocity.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Velocity(t,3) = msgs_this{t}.Velocity.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Acceleration(t,1) = msgs_this{t}.Acceleration.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Acceleration(t,2) = msgs_this{t}.Acceleration.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Acceleration(t,3) = msgs_this{t}.Acceleration.Z;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Snap(t,1) = msgs_this{t}.Snap.X;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Snap(t,2) = msgs_this{t}.Snap.Y;
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.position_cmd.Snap(t,3) = msgs_this{t}.Snap.Z;
                    % ADD MORE
                end
                clear bag_this msgs_this
            end
            

            %% custom_controller:
            pat = '_controller/';
            if contains(topicName, pat) % it's a controller's topic
                % extract the controller's name
                splitTopicNames = split(topicName,"/");
                strControllerName = splitTopicNames(4);
                taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.name = strControllerName;
                %% _controller/custom_thrust
                pat = '_controller/custom_thrust';
                if contains(topicName,pat)
                    bag_custom_thrust = select(bag,'Topic', topicName); 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.custom_thrust.time = bag_custom_thrust.MessageList.Time;
                    msgs_custom_thrust = readMessages(bag_custom_thrust,'DataFormat','struct');
                    for t = 1 : length(msgs_custom_thrust)
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.custom_thrust.Data(t,1) = msgs_custom_thrust{t}.Data;
                    end
                    clear bag_custom_thrust msgs_custom_thrust
                end
                %% _controller/custom_projected_thrust
                pat = '_controller/custom_projected_thrust';
                if contains(topicName,pat)
                    bag_custom_projected_thrust = select(bag,'Topic', topicName); 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.custom_projected_thrust.time = bag_custom_projected_thrust.MessageList.Time;
                    msgs_custom_projected_thrust = readMessages(bag_custom_projected_thrust,'DataFormat','struct');
                    for t = 1 : length(msgs_custom_projected_thrust)
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.custom_projected_thrust.Data(t,1) = msgs_custom_projected_thrust{t}.Data;
                    end
                    clear bag_custom_projected_thrust msgs_custom_projected_thrust
                end
                %% _controller/thrust_satlimit_physical
                pat = '_controller/thrust_satlimit_physical';
                if contains(topicName,pat)
                    bag_thrust_satlimit_physical = select(bag,'Topic', topicName); 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.thrust_satlimit_physical.time = bag_thrust_satlimit_physical.MessageList.Time;
                    msgs_thrust_satlimit_physical = readMessages(bag_thrust_satlimit_physical,'DataFormat','struct');
                    for t = 1 : length(msgs_thrust_satlimit_physical)
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.thrust_satlimit_physical.Data(t,1) = msgs_thrust_satlimit_physical{t}.Data;
                    end
                    clear bag_thrust_satlimit_physical msgs_thrust_satlimit_physical
                end
                %% _controller/thrust_satlimit
                pat = '_controller/thrust_satlimit'; 
                if contains(topicName,pat)
                    bag_thrust_satlimit = select(bag,'Topic', topicName); 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.thrust_satlimit.time = bag_thrust_satlimit.MessageList.Time; 
                    msgs_thrust_satlimit = readMessages(bag_thrust_satlimit,'DataFormat','struct');
                    for t = 1 : length(msgs_thrust_satlimit)
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.thrust_satlimit.Data(t,1) = msgs_thrust_satlimit{t}.Data;
                    end
                    clear bag_thrust_satlimit msgs_thrust_satlimit
                end
                %% _controller/thrust_satval
                pat = '_controller/thrust_satval'; 
                if contains(topicName,pat)
                    bag_thrust_satval = select(bag,'Topic', topicName); 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.thrust_satval.time = bag_thrust_satval.MessageList.Time;
                    msgs_thrust_satval = readMessages(bag_thrust_satval,'DataFormat','struct');
                    for t = 1 : length(msgs_thrust_satval)
                         taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.thrust_satval.Data(t,1) = msgs_thrust_satval{t}.Data;
                    end
                    clear bag_thrust_satval msgs_thrust_satval
                end
                %% _controller/hover_thrust
                pat = '_controller/hover_thrust'; 
                if contains(topicName,pat)
                    bag_hover_thrust = select(bag,'Topic', topicName); 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.hover_thrust.time = bag_hover_thrust.MessageList.Time;
                    msgs_hover_thrust = readMessages(bag_hover_thrust,'DataFormat','struct');
                    for t = 1 : length(msgs_hover_thrust)
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.controller.hover_thrust.Data(t,1) = msgs_hover_thrust{t}.Data;
                    end
                    clear bag_hover_thrust msgs_hover_thrust
                end
                
            end
            %% custom_tracker:
            pat = '_tracker/';
            if contains(topicName, pat) % it's a tracker's topic
                % extract the tracker's name
                splitTopicNames = split(topicName,"/");
                strTrackerName = splitTopicNames(4);
                taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.name = strTrackerName;
                %% _tracker/goal_pose
                pat = '_tracker/goal_pose';
                if (contains(topicName,pat))
                    bag_goal_pose = select(bag,'Topic', topicName); 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.goal_pose.time = bag_goal_pose.MessageList.Time;
                    msgs_goal_pose = readMessages(bag_goal_pose,'DataFormat','struct');
                    for t = 1 : length(msgs_goal_pose)
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.goal_pose.Reference.Position(t,1) = msgs_goal_pose{t}.Reference.Position.X;
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.goal_pose.Reference.Position(t,2) = msgs_goal_pose{t}.Reference.Position.Y;
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.goal_pose.Reference.Position(t,3) = msgs_goal_pose{t}.Reference.Position.Z;   
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.goal_pose.Reference.Heading = msgs_goal_pose{t}.Reference.Heading;
                    end
                    clear bag_goal_pose msgs_goal_pose
                end
                %% _tracker/custom_predicted_thrust
                pat = '_tracker/custom_predicted_thrust';
                if contains(topicName,pat)
                    bag_custom_predicted_thrust = select(bag,'Topic', topicName); 
                    time_temp = bag_custom_predicted_thrust.MessageList.Time;
                    msgs_custom_predicted_thrust = readMessages(bag_custom_predicted_thrust,'DataFormat','struct');
                    % preallocation
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_thrust.time = zeros(length(msgs_custom_predicted_thrust),length(msgs_custom_predicted_thrust{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_thrust.Data = zeros(length(msgs_custom_predicted_thrust),length(msgs_custom_predicted_thrust{1}.Poses(:)));
                    for t = 1 : length(msgs_custom_predicted_thrust)
                        for tpred = 1: length(msgs_custom_predicted_thrust{t}.Poses(:))
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_thrust.time(t,tpred) = time_temp(t) + tpred*custom_dt; 
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_thrust.Data(t,tpred) = msgs_custom_predicted_thrust{t}.Poses(tpred).Position.X;
                        end
                    end
                    clear bag_custom_predicted_thrust msgs_custom_predicted_thrust time_temp
                end
                %% _tracker/custom_predicted_poses
                pat = '_tracker/custom_predicted_poses';
                if contains(topicName,pat)
                    bag_custom_predicted_poses = select(bag,'Topic', topicName); 
                    time_temp = bag_custom_predicted_poses.MessageList.Time;
                    msgs_custom_predicted_poses = readMessages(bag_custom_predicted_poses,'DataFormat','struct');
                    % preallocation
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_poses.time = zeros(length(msgs_custom_predicted_poses),length(msgs_custom_predicted_poses{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_poses.Poses.Position.X = zeros(length(msgs_custom_predicted_poses),length(msgs_custom_predicted_poses{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_poses.Poses.Position.Y = zeros(length(msgs_custom_predicted_poses),length(msgs_custom_predicted_poses{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_poses.Poses.Position.Z = zeros(length(msgs_custom_predicted_poses),length(msgs_custom_predicted_poses{1}.Poses(:)));
                    for t = 1 : length(msgs_custom_predicted_poses)
                        for tpred = 1: length(msgs_custom_predicted_poses{t}.Poses(:))
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_poses.time(t,tpred) = time_temp(t) + tpred*custom_dt; 
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_poses.Poses.Position.X(t,tpred) = msgs_custom_predicted_poses{t}.Poses(tpred).Position.X;
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_poses.Poses.Position.Y(t,tpred) = msgs_custom_predicted_poses{t}.Poses(tpred).Position.Y;
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_poses.Poses.Position.Z(t,tpred) = msgs_custom_predicted_poses{t}.Poses(tpred).Position.Z;
                        end
                    end
                    clear bag_custom_predicted_poses msgs_custom_predicted_poses time_temp
                end
                %% _tracker/custom_predicted_vels
                pat = '_tracker/custom_predicted_vels';
                if contains(topicName,pat)
                    bag_custom_predicted_vels = select(bag,'Topic', topicName); 
                    time_temp = bag_custom_predicted_vels.MessageList.Time;
                    msgs_custom_predicted_vels = readMessages(bag_custom_predicted_vels,'DataFormat','struct');
                    % preallocation
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_vels.time = zeros(length(msgs_custom_predicted_vels),length(msgs_custom_predicted_vels{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_vels.Poses.Position.X = zeros(length(msgs_custom_predicted_vels),length(msgs_custom_predicted_vels{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_vels.Poses.Position.Y = zeros(length(msgs_custom_predicted_vels),length(msgs_custom_predicted_vels{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_vels.Poses.Position.Z = zeros(length(msgs_custom_predicted_vels),length(msgs_custom_predicted_vels{1}.Poses(:)));
                    for t = 1 : length(msgs_custom_predicted_vels)
                        for tpred = 1: length(msgs_custom_predicted_vels{t}.Poses(:))
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_vels.time(t,tpred) = time_temp(t) + tpred*custom_dt; 
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_vels.Poses.Position.X(t,tpred) = msgs_custom_predicted_vels{t}.Poses(tpred).Position.X;
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_vels.Poses.Position.Y(t,tpred) = msgs_custom_predicted_vels{t}.Poses(tpred).Position.Y;
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_vels.Poses.Position.Z(t,tpred) = msgs_custom_predicted_vels{t}.Poses(tpred).Position.Z;
                        end
                    end
                    clear bag_custom_predicted_vels msgs_custom_predicted_vels time_temp
                end
                %% _tracker/custom_predicted_accs
                pat = '_tracker/custom_predicted_accs';
                if contains(topicName,pat)
                    bag_custom_predicted_accs = select(bag,'Topic', topicName); 
                    time_temp = bag_custom_predicted_accs.MessageList.Time;
                    msgs_custom_predicted_accs = readMessages(bag_custom_predicted_accs,'DataFormat','struct');
                    % preallocation
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_accs.time = zeros(length(msgs_custom_predicted_accs),length(msgs_custom_predicted_accs{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_accs.Poses.Position.X = zeros(length(msgs_custom_predicted_accs),length(msgs_custom_predicted_accs{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_accs.Poses.Position.Y = zeros(length(msgs_custom_predicted_accs),length(msgs_custom_predicted_accs{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_accs.Poses.Position.Z = zeros(length(msgs_custom_predicted_accs),length(msgs_custom_predicted_accs{1}.Poses(:)));
                    for t = 1 : length(msgs_custom_predicted_accs)
                        for tpred = 1: length(msgs_custom_predicted_accs{t}.Poses(:))
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_accs.time(t,tpred) = time_temp(t) + tpred*custom_dt; 
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_accs.Poses.Position.X(t,tpred) = msgs_custom_predicted_accs{t}.Poses(tpred).Position.X;
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_accs.Poses.Position.Y(t,tpred) = msgs_custom_predicted_accs{t}.Poses(tpred).Position.Y;
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_accs.Poses.Position.Z(t,tpred) = msgs_custom_predicted_accs{t}.Poses(tpred).Position.Z;
                        end
                    end
                    clear bag_custom_predicted_accs msgs_custom_predicted_accs time_temp
                end
                %% _tracker/custom_predicted_attrate
                pat = '_tracker/custom_predicted_attrate';
                if contains(topicName,pat)
                    bag_custom_predicted_attrate = select(bag,'Topic', topicName); 
                    time_temp = bag_custom_predicted_attrate.MessageList.Time;
                    msgs_custom_predicted_attrate = readMessages(bag_custom_predicted_attrate,'DataFormat','struct'); 
                    % preallocation
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_attrate.time = zeros(length(msgs_custom_predicted_attrate),length(msgs_custom_predicted_attrate{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_attrate.Poses.Position.X = zeros(length(msgs_custom_predicted_attrate),length(msgs_custom_predicted_attrate{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_attrate.Poses.Position.Y = zeros(length(msgs_custom_predicted_attrate),length(msgs_custom_predicted_attrate{1}.Poses(:)));
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_attrate.Poses.Position.Z = zeros(length(msgs_custom_predicted_attrate),length(msgs_custom_predicted_attrate{1}.Poses(:)));
                    for t = 1 : length(msgs_custom_predicted_attrate)
                        for tpred = 1: length(msgs_custom_predicted_attrate{t}.Poses(:))
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_attrate.time(t,tpred) = time_temp(t) + tpred*custom_dt; 
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_attrate.Poses.Position.X(t,tpred) = msgs_custom_predicted_attrate{t}.Poses(tpred).Position.X;
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_attrate.Poses.Position.Y(t,tpred) = msgs_custom_predicted_attrate{t}.Poses(tpred).Position.Y;
                            taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.custom_predicted_attrate.Poses.Position.Z(t,tpred) = msgs_custom_predicted_attrate{t}.Poses(tpred).Position.Z;
                        end
                    end
                    clear bag_custom_predicted_attrate msgs_custom_predicted_attrate time_temp
                end
                %% _tracker/DSM   
                pat = '_tracker/DSM';
                if contains(topicName,pat)
                    bag_DSM = select(bag,'Topic', topicName); 
                    taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.DSM.time = bag_DSM.MessageList.Time;
                    msgs_DSM = readMessages(bag_DSM,'DataFormat','struct');
                    for t = 1 : length(msgs_DSM)
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.DSM.DSM(t,1) = msgs_DSM{t}.DSM;
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.DSM.DSM_s(t,1) = msgs_DSM{t}.DSMS;
                        taskRun{taskRunId,1}.uav{uavId,1}.control_manager.tracker.DSM.DSM_a(t,1) = msgs_DSM{t}.DSMA;
                    end
                    clear bag_DSM msgs_DSM
                end

            end  
        end
    end
end

%% test
% taskRun{taskRunId,1}.uav{2,1} = taskRun{taskRunId,1}.uav{1,1}
% taskRun{taskRunId,1}.uav{3,1} = taskRun{taskRunId,1}.uav{1,1}
%% save data in .mat file
matname = strcat(matfilename,'.mat');
save(matname,'taskRun');%,,'bag','allUavIds');
disp('Job done')