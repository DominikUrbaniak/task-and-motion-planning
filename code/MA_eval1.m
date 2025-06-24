close all

run_again = 1;
run_again_index = 500;
restore = 0;

if run_again
    sol_file_path = "run_again/sol_"+run_again_index+".txt";
    load("run_again/init_config_"+run_again_index+".mat")
end

global t_mp ins
t_mp = {};
t_mp{1} = 0;

id = 0;
if id >0
    task_file_name = "runs_size"+id;
    dir = "tests/new_size"+id;
    mkdir(dir)
else
    task_file_name = "save_init_10";
    dir = "tests/save_10";
    mkdir(dir)
end
task_directory_name = "C:/ubuntu_files/eval1";
dmp_transformation = 1;

global h_cube f_clear
f_clear = 0.02;
prev_f = 1;
f_sc = 1;
h_cube = f_sc*0.04;
h_cube = prev_f*0.04;
grid_thresh = h_cube+0.01;
gripper_width = h_cube+0.03;

scaler = [f_sc,f_sc];
h_grasp = 0.02;
h_clearance = 0.005;
window_pos_x = 1100;
window_pos_y = 100;
n_heuristic_polys = 21;
global nn_inputs
%pause on
disp('Program started');
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19991,true,true,5000,5);

simulation = 1;
n_cubes = 8;
n_cells = 9;
cubes_init_eul = [-pi,0,pi/2]; %same as the orientation of the gripper in the demonstration

font_size = 14;

load('NNets/data_norm3_50.mat') %dmp_nn1.mat: net1, dmp_nn2.mat: dmp_net2, data_norm3_50.mat: net
dmp_net = net;


%Setting attributes for the task file creation
objects = {};
objects{1} = ["cell","cube"];
objects{2} = [n_cells,n_cubes];

% Initializing the figure
f0 = subplot(2,2,3);
hold(f0,'on')
set(f0,'XColor','none','YColor','none');
title("Initial Configuration")
set(f0,'Tag','zeroth');
f1=subplot(2,2,4);
hold(f1,'on')
set(f1,'XColor','none','YColor','none');
title("Current Configuration")
set(gcf, 'Position',  [window_pos_x, window_pos_y, 1000, 1200])
f2=subplot(2,2,2);
set(f2,'Tag','second');
hold(f2,'on')
grid(f2,'on')
axis(f2,'equal')
%axis([-0.1,0.1,-0.02,0.4,-0.02,0.4])
ylim(f2,[-0.02,0.4])
zlim(f2,[-0.02,0.4])
view(90,0)
%view(10,30)
title("Trajectories")

figure(2)
set(gcf, 'Position',  [window_pos_x-1000, window_pos_y+200, 600, 720])
f5 = subplot(3,1,1);
hold(f5,'on')
set(f5,'Tag','fifth');
xlim(f5,[0 1])
b1 = barh(f5,zeros(2,1));

title(f5,"NN Input")
f6=subplot(3,1,2);
hold(f6,'on')
set(f6,'Tag','sixth');
title(f6,"NN Output: \theta_{i,X}")
ylim(f6,[-4 2])

f7=subplot(3,1,3);
hold(f7,'on')
set(f7,'Tag','seventh');
title(f7,"NN Output: \theta_{i,Z}")
ylim(f7,[-2 4])

col_pick = [1,0,1];
col_place = [0,0,1];

global patch_cells;
for i = 1:n_heuristic_polys
    patch_cells{i} = patch(f1,zeros(5,1),zeros(5,1),[0,0,0]);
end

if (clientID>-1)
    disp('Connected to remote API server');
     
    %% Initialize a struct for the v-rep cubes
    cube_names = strings(n_cubes,1);
    cube_handles = zeros(n_cubes,1);
    cube_colors = [0,1,0;0,0,1;1,0,0;0.5,0,0;0,0.5,0;0,0,0.5;1,1,0;0,1,1]; %RGB colors set for each cube
    cube_init_poses = zeros(n_cubes,6);
    cubes_init_z = 0.444; %tabletop at h_cube = 0.04
    cube_init_poses(:,3) = cubes_init_z;
    cube_goal_poses = cube_init_poses;
    cube_current_poses = cube_init_poses;
    load("cube_init2.mat")
    for i = 1:n_cubes
        cube_names(i) = "Cuboid"+i;
        name = convertStringsToChars(cube_names(i));
        [~,cube_handles(i)] = sim.simxGetObjectHandle(clientID,name,sim.simx_opmode_blocking);
        sim.simxSetObjectPosition(clientID,cube_handles(i),-1,cube_init_poses(i,1:3),sim.simx_opmode_blocking);
    end
    
    
    %initialize cube heights
    if restore
        load("restore_cubes")
        %send_inverted_scales = send_inverted_scales.^-1;
        send_inverted_scales = inverted_cube_scales;
        [returnCode,~,~,~,~]=sim.simxCallScriptFunction(clientID,'ROBOTIQ_85',sim.sim_scripttype_childscript,'initialize_block_heights',cube_handles,send_inverted_scales,'',[],sim.simx_opmode_blocking);
        disp("Cube sizes restored")
    else
        
        cube_scales = scaler(1)+(scaler(2)-scaler(1))*rand([n_cubes,1]);
    	%cube_scales = [1,1,1,1,1,1,1,1]';
        inverted_cube_scales = cube_scales.^-1;
        save("restore_cubes.mat","inverted_cube_scales")
    
        send_scales = [cube_scales];
        send_inverted_scales = [inverted_cube_scales];
        [returnCode,~,block_heights,~,~]=sim.simxCallScriptFunction(clientID,'ROBOTIQ_85',sim.sim_scripttype_childscript,'initialize_block_heights',cube_handles,send_scales,'',[],sim.simx_opmode_blocking)
    %grasp_heights = init_grasp_height + (cube_scales*cubes_init_height-ones(n_cubes,1)*cubes_init_height)/2
    end
    
    for i = 1:n_cubes
        %cube_names(i) = "Cuboid"+i;
        %name = convertStringsToChars(cube_names(i));
        %[~,cube_handles(i)] = sim.simxGetObjectHandle(clientID,name,sim.simx_opmode_blocking);
        [~,pos] = sim.simxGetObjectPosition(clientID,cube_handles(i),-1,sim.simx_opmode_blocking);
        [~,eul] = sim.simxGetObjectOrientation(clientID,cube_handles(i),-1,sim.simx_opmode_blocking);
        cube_init_poses(i,:) = [pos,cubes_init_eul(1:2),eul(3)]; %only orientation around z matters
        cube_current_poses(i,:) = [pos,cubes_init_eul(1:2),eul(3)];
    end
    
    cubes = struct("names",cube_names,"handles",cube_handles,"colors",cube_colors,"init_poses",cube_init_poses,"goal_poses",cube_goal_poses,"current_poses",cube_current_poses);
    grid_places = get_grid_coords(); %retrieves x,y-coordinates of all possible places
    
    
    if simulation %in real-world the cube poses will be observed
        %initilize random initial places for each cube
        % ROBOT
        % 3 2 1     -> place indeces
        % 6 5 4
        % 9 8 7
        if ~run_again
            rand_init_pos = randperm(n_cells);
            rand_init_pos = rand_init_pos(1:n_cubes); % last index marks the empty place
        end
        new_cube_poses = [grid_places(rand_init_pos,:),cubes.init_poses(:,3),ones(n_cubes,3).*[cubes_init_eul]];
        cubes.init_poses = new_cube_poses;
        cubes.current_poses = new_cube_poses;
        new_pos = new_cube_poses(:,1:3);
        new_eul = new_cube_poses(:,4:6);
        for i = 1:n_cubes
            sim.simxSetObjectPosition(clientID,cube_handles(i),-1,new_pos(i,:),sim.simx_opmode_blocking);
            sim.simxSetObjectOrientation(clientID,cube_handles(i),-1,cubes_init_eul,sim.simx_opmode_blocking);
        end
    end
    
    %initilize random goal places for each cube
    all_rand_goal_pos = randperm(n_cells);
    goal_env = all_rand_goal_pos;
    goal_env(goal_env == n_cells) = 0;
    rand_goal_pos = all_rand_goal_pos(1:n_cubes); % last index marks the empty place
    goal_cube_poses = cube_init_poses;
    constant_pose_elements = cube_init_poses(1,3:6); %Goal poses only vary in x,y-direction 
    goal_cube_poses(:,1:2) = grid_places(rand_goal_pos,:);
    cubes.goal_poses = goal_cube_poses;
    
    plot_cubes(f0,cubes,0,0,0,0); %plot into initial configuration
    [cube_polys,cube_plots,text_plots] = plot_cubes(f1,cubes,0,0,0,0); %initialize the current configuration plot
    grid_polys = get_grid_polys(grid_places,grid_thresh); %Retrieve polygons to easily verfy if a cube lies completely inside the goal area 
    
    [init_env, init_env_string] = get_environment(grid_polys,cube_polys); %Retrieve two representations of the current configuration (numerical and symbolic)
    goal_env_string = env2string(goal_env); %translate the numerical into a symbolic representation 
if ~run_again   
    task_file_path = task_directory_name+"/task_"+task_file_name+".pddl";
    create_task_file(task_file_path,init_env_string,goal_env_string,objects)
    
%%********************************************
%% Pause here to find a solution with an external task planner
% e.g. find a solution using fast downward -> save in file 

sol_file_path = task_directory_name+"/sol_"+task_file_name+".txt";
end
%%*********************************************

    %% Plot the solution from the task planner
    figure(1)
    fid = fopen(sol_file_path);
    i = 1;
    task = [];
    lines = {};
    textboxes = {};
    while i < 1000 %max plan length is 1000
        lin = fgetl(fid);
        if lin(1) ~= '('
            disp(lin(1))
            break
        end
        lines{i} = lin(2:end-1);
        title_box = annotation('textbox','String','Task Plan','FontSize',font_size+4,'FontWeight','bold','EdgeColor','none');
        title_box.Position(1:2) = [0.2,0.89];
        textboxes{i} = annotation('textbox','String',lines{i},'FontName','Courier New','FontSize',font_size);
        textboxes{i}.Position(1:2) = [0.1,0.88-i*0.032];
        task(i,:) = [str2double(lin(16)),str2double(lin(22)),str2double(lin(28))]; %indeces 17, 24 & 30 mark the intial place id, goal place id & cube id
        %***** adjust, if place id > 9
        
        i = i+1;
    end
    textboxes{i} = annotation('textbox','String','Pick','FontName','Courier New','FontSize',font_size,'EdgeColor',col_pick);
    textboxes{i+1} = annotation('textbox','String','Place','FontName','Courier New','FontSize',font_size,'EdgeColor',col_place);
    %textboxes{i+2} = annotation('textbox','FontSize',15,'Position',[0.05,0.5,0.45,0.05],'FitBoxToText','off');
    textboxes{i+2} = annotation('textbox','FontSize',font_size);
    textboxes{i}.Position(1:2) = [0.05,0.88-i*0.032];
    textboxes{i+1}.Position(1:2) = [0.414,0.88-i*0.032];
    textboxes{i+2}.Position(1:2) = [0.05,0.88-(i+1)*0.032];
    
    %create a list of expected changes to later compare them to the real
    %changes to evaluate the successful execution
    expected_env_steps = get_env_evolution(init_env,task);
    goal_pos = expected_env_steps(end,:);
    plot_goal_grid(f0,grid_places,goal_pos,cubes.colors,grid_thresh);
    plot_goal_grid(f1,grid_places,goal_pos,cubes.colors,grid_thresh);
    
    %% Create the DMPs to solve the task
    tic
    dmp_eval1 = dmp_6Dn4(6);
    dmp_eval1.rescale = dmp_transformation;
    training_file = "eval1_demo_low.txt";
    dmp_eval1 = dmp_eval1.train(training_file);
    b2 = bar(f6,zeros(dmp_eval1.n_rbf,1));
    
    b3 = bar(f7,zeros(dmp_eval1.n_rbf,1));
    
    dmp_eval1.x_end(4:6) = cubes_init_eul;
    t_mp{2} = toc;
    [plan_length,~] = size(task); 
    
    [returnCode,gripper_connector]=sim.simxGetObjectHandle(clientID,'ROBOTIQ_85_attachPoint',sim.simx_opmode_blocking);
    [returnCode,init_handler]=sim.simxGetObjectHandle(clientID,'cub_goal',sim.simx_opmode_blocking);
    [returnCode,tip_handler]=sim.simxGetObjectHandle(clientID,'tip',sim.simx_opmode_blocking); 
    [returnCode,init_pos]=sim.simxGetObjectPosition(clientID,tip_handler,-1,sim.simx_opmode_blocking);
    [returnCode,init_eul]=sim.simxGetObjectOrientation(clientID,tip_handler,-1,sim.simx_opmode_blocking);
    
    error = 0;
    swit = 0; %parameter to decide whether trajectory or grasp is executed 
    fail_counter = 0; 
    max_fails = 5; %count failed attempts and abort after reaching max_fails
    
    %Report the actual end poses of the gripper
    x_end_history = zeros(2*plan_length,dmp_eval1.n_dims); %each symbolic action will consist of two DMPs
    plan_execution_step = 1; %counter that tracks the task execution
    
    %start with an open gripper
    sub_task = "pick";
    
    while plan_execution_step <= plan_length   
        %Check the state of the simulation by a signal that is set when the
        %robot is in the start position
        [returnCode,traj_start_signal]=sim.simxGetIntegerSignal(clientID,'dmp_traj_start',sim.simx_opmode_oneshot_wait);%sim.simx_opmode_streaming
        
        if ~returnCode && ~traj_start_signal && ~swit
            swit = 1;
            %Get the initial positions for the DMPs
            [returnCode,tip_pos_init]=sim.simxGetObjectPosition(clientID,tip_handler,-1,sim.simx_opmode_blocking);
            [returnCode,tip_pos_eul]=sim.simxGetObjectOrientation(clientID,tip_handler,-1,sim.simx_opmode_blocking);
            
            dmp_eval1.x_init(1:3) = tip_pos_init;
            dmp_eval1.x_init(3) = dmp_eval1.x_init(3)+0.5*h_cube - h_grasp;
            dmp_eval1.x_init(4:6) = tip_pos_eul;
            current_cube = task(plan_execution_step,3);
                
            %Separate the symbolic action into two parts: movement to
            %"pick" and movement to "place"
            if sub_task == "pick" %requires the current pose of the cube
                [returnCode,cube_pos_init]=sim.simxGetObjectPosition(clientID,cubes.handles(current_cube),-1,sim.simx_opmode_blocking);
                [returnCode,cube_eul_init]=sim.simxGetObjectOrientation(clientID,cubes.handles(current_cube),-1,sim.simx_opmode_blocking);
                textboxes{end-2}.FontWeight = 'bold';
                textboxes{end-1}.FontWeight = 'normal';
                textboxes{plan_execution_step}.BackgroundColor = 'y';
                
                if cube_pos_init(3) < 0 %Quit, when a cube falls from the table
                    disp("Cube is not reachable -> task cannot be executed")
                    cube_pos_init(3)
                    break
                else
                    dmp_eval1.x_end(1:3) = cube_pos_init; %grasp height = center of the cube
                    dmp_eval1.x_end(6) = cube_eul_init(3);
                end
                [dmp_eval1,r_L,H] = set_means_from_nn(dmp_eval1,cube_polys,0,dmp_net,gripper_width,h_cube,h_clearance,h_grasp,n_heuristic_polys,plan_execution_step);
                
            else
                goal_pos_id = task(plan_execution_step,2); %marks the position to place the cube
                textboxes{end-2}.FontWeight = 'normal';
                textboxes{end-1}.FontWeight = 'bold';
                dmp_eval1.x_end(1:2) = grid_places(goal_pos_id,:);
                dmp_eval1.x_end(3) = cubes.init_poses(current_cube,3);
                dmp_eval1.x_end(4:6)=cubes_init_eul;
                
                [dmp_eval1,r_L,H] = set_means_from_nn(dmp_eval1,cube_polys,current_cube,dmp_net,gripper_width,h_cube,h_clearance,h_grasp,n_heuristic_polys,plan_execution_step);
                t_mp{2+plan_execution_step*2} = toc;
            end
            
            
            %if sub_task == "pick"
            %    savefig(dir+"/"+task_file_name+"_"+plan_execution_step+"_pick")
            %else
            %    savefig(dir+"/"+task_file_name+"_"+plan_execution_step+"_place")
            %end
               
            fprintf("** %d: %s...\n",plan_execution_step,sub_task)
            %% Plot and get the trajectory and send it to v-rep
            [r_c,z_height,d_g,d_H] = plot_traj(f2,dmp_eval1,cubes,current_cube,sub_task,r_L,H,n_heuristic_polys,col_pick,col_place)
            b1.YData = [r_c,r_L];
            
            b2.YData = dmp_eval1.means(:,2)';
            
            b3.YData = dmp_eval1.means(:,3)';
            if sub_task == "place"
                 b1.FaceColor = col_place;
                 b2.FaceColor = col_place;
                 b3.FaceColor = col_place;
            else
                 b1.FaceColor = col_pick;
                 b2.FaceColor = col_pick;
                 b3.FaceColor = col_pick;
            end
            
            [traj_send,x_end] = get_traj(dmp_eval1,dmp_eval1.means,dmp_eval1.x_end,cubes, current_cube, sub_task, plan_execution_step);
            drawnow
            [returnCode,~,~,~,~]=sim.simxCallScriptFunction(clientID,'ROBOTIQ_85',sim.sim_scripttype_childscript,'set_trajectory',[],traj_send,'Hi',[],sim.simx_opmode_blocking);
            if sub_task == "pick"
                x_end_history(2*plan_execution_step-1,:) = x_end;
            else
                x_end_history(2*plan_execution_step,:) = x_end;
            end
        elseif ~returnCode && traj_start_signal && swit
            disp("Waiting for simulation to finish...")
            sent = 1;
            while sent %Wait for the trajectory to finish -> then, close/open the gripper
                [returnCode,signal]=sim.simxGetIntegerSignal(clientID,'send_traj',sim.simx_opmode_oneshot_wait);
                if ~returnCode && ~signal
                    swit = 0;
                    if sub_task == "pick"
                        %close the gripper 
                        %tie the cube to the gripper (not physically accuarate)
                        [returnCode]=sim.simxSetIntegerSignal(clientID,'hand',0,sim.simx_opmode_oneshot_wait);
                        [returnCode]=sim.simxSetObjectParent(clientID,cubes.handles(current_cube),gripper_connector,1,sim.simx_opmode_oneshot_wait);
                        sub_task = "place";
                        drawnow
                        savefig(dir+"/"+task_file_name+"_"+plan_execution_step+"_pick")
                        saveas(1,dir+"/"+task_file_name+"_"+plan_execution_step+"_pick.png")
                    else
                        %open the gripper
                        %make the cube independent from the gripper again
                        [returnCode]=sim.simxSetIntegerSignal(clientID,'hand',1,sim.simx_opmode_oneshot_wait);
                        [returnCode]=sim.simxSetObjectParent(clientID,cubes.handles(current_cube),-1,1,sim.simx_opmode_oneshot_wait);
                        sub_task = "pick";
                        
                        %update the cube's pose and check if the action was
                        %successful
                        [~,new_pos] = sim.simxGetObjectPosition(clientID,cube_handles(current_cube),-1,sim.simx_opmode_blocking);
                        [~,new_eul] = sim.simxGetObjectOrientation(clientID,cube_handles(current_cube),-1,sim.simx_opmode_blocking);
                        cubes.current_poses(current_cube,:) = [new_pos,new_eul];
                        [cube_polys,cube_plots,text_plots] = plot_cubes(f1,cubes,current_cube,cube_polys,cube_plots,text_plots);
                        [env,env_str] = get_environment(grid_polys,cube_polys);
                        if isequal(env,expected_env_steps(plan_execution_step,:))
                            textboxes{plan_execution_step}.BackgroundColor = 'g';
                            if plan_execution_step == plan_length
                                textboxes{end}.String = "Task step #"+plan_execution_step+" was successful, full task is finished!";
                                textboxes{end-1}.FontWeight = 'normal';
                                textboxes{end}.FontWeight = 'bold';
                                textboxes{end}.BackgroundColor = 'g';
                            else
                                textboxes{end}.String = "Task step #"+plan_execution_step+" was successful, starting next action...";
                            end
                            fail_counter = 0;
                            plan_execution_step = plan_execution_step + 1;
                        else
                            fail_counter = fail_counter + 1;
                            textboxes{plan_execution_step}.BackgroundColor = 'r';
                            textboxes{end}.String = "Task step #"+plan_execution_step+" failed, try again...";
                            if fail_counter >= max_fails
                                disp("Task failed "+max_fails+" times, exiting program!")
                                error = 1;
                            end
                        end
                        drawnow
                        savefig(dir+"/"+task_file_name+"_"+plan_execution_step+"_place")
                        saveas(1,dir+"/"+task_file_name+"_"+plan_execution_step+"_place.png")
                    end
                    
                    %Retrieve data from the simulation (not used here)
                    %[~,~,data,~,~]=sim.simxCallScriptFunction(clientID,'ROBOTIQ_85',sim.sim_scripttype_childscript,'get_trajectory',[],traj_send,'Hi',[],sim.simx_opmode_blocking);
                    
                    %ready to start the next movement
                    [returnCode]=sim.simxSetIntegerSignal(clientID,'send_traj',1,sim.simx_opmode_oneshot_wait);
                    sent = 0;
                end
                
            end
        end
        if error
            disp("Finished with error")
            break
        end
    end
    
    %return to start position after finishing the task
    if ~error
        signal = 1;
        while signal
            [returnCode,signal]=sim.simxGetIntegerSignal(clientID,'dmp_traj_start',sim.simx_opmode_oneshot_wait);
            if ~signal
                disp("return to initial position") 
                [~,tip_pos_init]=sim.simxGetObjectPosition(clientID,tip_handler,-1,sim.simx_opmode_blocking);
                dmp_eval1.x_init(1:3) = tip_pos_init;
                dmp_eval1.x_end(1:3) = init_pos;
                [dmp_eval1,r_L,H] = set_means_from_nn(dmp_eval1,cube_polys,0,dmp_net,gripper_width,h_cube,h_clearance,h_grasp,n_heuristic_polys,plan_execution_step);
                [traj_send,x_end] = get_traj(dmp_eval1,dmp_eval1.means,dmp_eval1.x_end,cubes, current_cube, sub_task, plan_execution_step);
                [returnCode,~,~,~,~]=sim.simxCallScriptFunction(clientID,'ROBOTIQ_85',sim.sim_scripttype_childscript,'set_trajectory',[],traj_send,'Hi',[],sim.simx_opmode_blocking);
                
            end
        end
        
    else
        disp("########################    ERROR     #####################")
    end
    
    sim.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
save(dir+"/time_"+task_file_name,"t_mp")
save(dir+"/init_"+task_file_name,"rand_init_pos")
save(dir+"/ins_"+task_file_name,"ins")
sim.delete(); % call the destructor!
    
disp('Program ended');    

% Set the coordinates for the 9 places
function out = get_grid_coords()
    grid_nx = 3;
    grid_ny = 3;
    first_cell_coords = [-0.25,0.35]; %cell #1 is the bottom left (robot arm perspective)
    x_step = 0.1;
    y_step = -0.15;
    out = zeros(grid_nx*grid_ny,2);
    c = 0;
    for i = 1:grid_nx
        for j = 1:grid_ny
            c = c+1;
            out(c,:) = [first_cell_coords(1)+(i-1)*x_step,first_cell_coords(2)+(j-1)*y_step];
        end
    end
    %out = [-0.2,0;-0.25,0.1;-0.125,0.08;-0.025,0.125;-0.18,0.2;-0.1,0.25;-0.26,0.37;-0.025,0.4;0,0]
    
   
    %out_x = -0.25*rand([9,1]);
    %out_y = 0.5*rand([9,1]);
    %out = [out_x,out_y]
    % ROBOT
    % 3 2 1
    % 6 5 4
    % 9 8 7
end

%get the polygons of the places to evaluate the intersection with the
%cubes' polygons
function grid_polys = get_grid_polys(grid_coords,L) % larger L makes a success more likely
    [iters,~] = size(grid_coords);
    grid_polys = zeros(2,5,iters);
    for ii = 1:iters
        [r_x,r_y] = get_edge_coords(grid_coords(ii,1),grid_coords(ii,2),L);
        %r_x = goal_pos(ii,1)-0.5*L;
        %r_y = goal_pos(ii,2)-0.5*L;
        [rx_vec,ry_vec] = get_rec_coords(r_x,r_y,L);
        grid_polys(:,:,ii) = [rx_vec;ry_vec];
    end

end

function plot_goal_grid(ax1,goal_pos,indeces,colors,L)
    iters = length(indeces);
    for ii = 1:iters
        [r_x,r_y] = get_edge_coords(goal_pos(ii,1),goal_pos(ii,2),L);
        if indeces(ii) == 0
            rectangle(ax1,'Position',[r_x,r_y,L,L],'EdgeColor',[0,0,0],'LineWidth',2,'LineStyle','--')
        else
            rectangle(ax1,'Position',[r_x,r_y,L,L],'EdgeColor',colors(indeces(ii),:),'LineWidth',2)
        end
        text(ax1,r_x,r_y-0.005,num2str(ii),'HorizontalAlignment','left','FontSize',9)
    end
end

%Plot the initial configuaration or update the plot to the current
%configuration
function [cube_polys,cube_plots,text_plots] = plot_cubes(ax1,cube_stack,id,cube_polys,cube_plots,text_plots)
global h_cube
L = h_cube;
if id == 0  
    iters = length(cube_stack.handles);
    cube_polys = zeros(2,5,iters);
    cube_plots = {};
    text_plots = {};
    for ii = 1:iters
        x = cube_stack.current_poses(ii,1);
        y = cube_stack.current_poses(ii,2);
        [r_x,r_y] = get_edge_coords(x,y,L);
        [rx_vec,ry_vec] = get_rec_coords(r_x,r_y,L);
        cube_plots{ii} = patch(ax1,rx_vec,ry_vec,cube_stack.colors(ii,:));
        text_plots{ii} = text(ax1,x,y,num2str(ii),'HorizontalAlignment','center','FontSize',15);
        rot_deg = rad2deg(cube_stack.current_poses(ii,6));
        rotate(cube_plots{ii},[0,0,1],rot_deg,[x,y,0]) %rotate around z-axis from the center of the cube
        xv = cube_plots{ii}.Vertices(:,1)';
        yv = cube_plots{ii}.Vertices(:,2)';
        cube_polys(:,:,ii) = [xv;yv];
    end
else
    x = cube_stack.current_poses(id,1);
    y = cube_stack.current_poses(id,2);
    r_x = x-0.5*L;
    r_y = y-0.5*L;
    [rx_vec,ry_vec] = get_rec_coords(r_x,r_y,L);
    cube_plots{id}.Vertices = [rx_vec',ry_vec'];
    rot_deg = rad2deg(cube_stack.current_poses(id,6));
    rotate(cube_plots{id},[0,0,1],rot_deg,[x,y,0]) %rotate around z-axis from the center of the cube
    xv = cube_plots{id}.Vertices(:,1)';
    yv = cube_plots{id}.Vertices(:,2)';
    cube_polys(:,:,id) = [xv;yv];
    text_plots{id}.Position = [x,y,0];
end
end

function [dmp,in2,H] = set_means_from_nn(dmp,cube_polys,except_id,nn,width,h_cube,h_clearance,h_grasp,n_heuristic_polys,step)
    
    global patch_cells f_clear ins;
    poly_width = width/2;
    input_length = n_heuristic_polys;%nn.inputs{1}.size;
    x = dmp.x_init(1:2);
    y = dmp.x_end(1:2);
    if except_id %distinguishes the operation pick (except_id=0) and place
        vec = y-x;
        if vec(2) < 0
            ang = atan(vec(1)/vec(2))+pi;
        else
            ang = atan(vec(1)/vec(2));
        end
        poly_len = norm(vec)/input_length;
        poly_v = zeros(2,5,input_length);

        %Reduce computations by only considering objects that intersect with
        %the whole are in between initial and goal position, then search the
        %specific poly_v only for those objects
        poly_all = zeros(2,5);
        poly_all(:,1) = (x-poly_width*[cos(ang),-sin(ang)])+poly_len*[sin(ang),cos(ang)];
        poly_all(:,2) = (x+poly_width*[cos(ang),-sin(ang)])+poly_len*[sin(ang),cos(ang)];
        poly_all(:,3) = (y+poly_width*[cos(ang),-sin(ang)])-poly_len*[sin(ang),cos(ang)];
        poly_all(:,4) = (y-poly_width*[cos(ang),-sin(ang)])-poly_len*[sin(ang),cos(ang)];
        poly_all(:,5) = (x-poly_width*[cos(ang),-sin(ang)])+poly_len*[sin(ang),cos(ang)];
        %patch_cells{n_heuristic_polys+1}.Vertices = [poly_all(1,:)',poly_all(2,:)'];
        %patch_cells{n_heuristic_polys+1}.FaceColor = [0,0,1];
        %patch_cells{n_heuristic_polys+1}.FaceAlpha = 0.2;

        [~,~,n_obst] = size(cube_polys);
        in_poly_all = zeros(1,n_obst);
        for jjj = 1:n_obst
            obst_avoid = any(inpolygon(cube_polys(1,:,jjj),cube_polys(2,:,jjj),poly_all(1,:),poly_all(2,:)));
            if  obst_avoid && jjj ~= except_id %When placing the cube the gripper does not need to avoid the picked cube
                in_poly_all(jjj) = jjj;
            end
        end
        in_all = nonzeros(in_poly_all);
        in = zeros(input_length-1,1); % regards the borders of the inner

        f0 = findobj('Tag','zeroth');

        for iii = 2:input_length-1
            xi = x+(iii-1)*poly_len*[sin(ang),cos(ang)];
            yi = x+iii*poly_len*[sin(ang),cos(ang)];
            poly_v(:,1,iii) = xi-poly_width*[cos(ang),-sin(ang)];
            poly_v(:,2,iii) = xi+poly_width*[cos(ang),-sin(ang)];
            poly_v(:,3,iii) = yi+poly_width*[cos(ang),-sin(ang)];
            poly_v(:,4,iii) = yi-poly_width*[cos(ang),-sin(ang)];
            poly_v(:,5,iii) = xi-poly_width*[cos(ang),-sin(ang)];
            avoid = 0;
            for jjj = 1:length(in_all)

                obst_avoid = any(inpolygon(cube_polys(1,:,in_all(jjj)),cube_polys(2,:,in_all(jjj)),poly_v(1,:,iii),poly_v(2,:,iii)));

                if  obst_avoid && in_all(jjj) ~= except_id %When placing the cube the gripper does not need to avoid the picked cube
                    in(iii) = h_cube; 
                    in(iii-1) = h_cube;
                    avoid = 1;
                end

            end
            patch_cells{iii}.Vertices = [poly_v(1,:,iii)',poly_v(2,:,iii)'];
            if avoid %&& ~except_cube
                patch_cells{iii}.FaceColor = [1,0,0];
                patch_cells{iii}.FaceAlpha = 0.4;
            %elseif except_cube
            %    patch_cells{iii}.FaceColor = [0,1,0];
            %    patch_cells{iii}.FaceAlpha = 0.3;
            else
                %patch(f0,poly_v(1,:,iii),poly_v(2,:,iii),[1,0,0],'FaceAlpha',0.4);
                patch_cells{iii}.FaceColor = [0,1,1];
                patch_cells{iii}.FaceAlpha = 0.2;
            end
        end
        
        %objects closest to initial or goal position define the steepness of the trajectory, shapes are encoded in a descrete way from 1 (least steep) to 10 (steepest)
        %For the neural network these descrete values are normalized to
        %0.1,...,1
        if isempty(find(in))
            in1 = 0;
            in1_norm = f_clear;
            in2 = randi(10)/10; %in2 doesn't matter here
        else
            in1 = h_cube; %When a cube is in the gripper, the avoidance height increases
            in2 = max(round(abs(find(in)-n_heuristic_polys/2)))/10;
            l_norm = norm(dmp.x_end(1:2) - dmp.x_init(1:2));
            in1_norm = in1 / l_norm + f_clear
            ins(2*step,:) = [in1_norm,in2];
        end
    else
        for iii = 1:n_heuristic_polys
            patch_cells{iii}.Vertices = zeros(5,2);
        end
        in1 = h_grasp; %the object of maximum height defines the height of the trajectory
        in2 = 1;
        l_norm = norm(dmp.x_end(1:2) - dmp.x_init(1:2));
        in1_norm = in1 / l_norm + f_clear
        ins(2*step-1,:) = [in1_norm,in2];
    end
    
    
    
    
    in1
    in2
    global t_mp
    tic
    
    
    H = in1;
    
    nn_means_yz = nn([in1_norm,in2]');
    if except_id > 0
        t_mp{2+step*2} = toc;
    else
        t_mp{2+step*2-1} = toc;
    end
    dmp.means(:,2:3) = reshape(nn_means_yz,[dmp.n_rbf,2]);
    
end

function env_str = env2string(goal_env)
    iters = length(goal_env);
    env_str = strings(1,iters);
    for ii = 1:iters
        env_str(ii) = get_predicate(ii,goal_env(ii));
    end
end
function [x_vec,y_vec] = get_rec_coords(x,y,L)
    x_vec = [x,x,x+L,x+L,x];
    y_vec = [y,y+L,y+L,y,y];
end
function [x_edge,y_edge] = get_edge_coords(x,y,L)
    x_edge = x-0.5*L;
    y_edge = y-0.5*L;
end

function expected_env_steps = get_env_evolution(init_env,task)
    [r,~] = size(task);
    expected_env_steps = zeros(r,length(init_env));
    c_env = init_env;
    for i = 1:r
        old_cell = task(i,1);
        new_cell = task(i,2);
        cube = task(i,3);
        c_env(old_cell) = 0;
        c_env(new_cell) = cube;
        expected_env_steps(i,:) = c_env;
    end
end

function [env,env_str_array] = get_environment(grid_polys,cube_polys)
    [~,~,grid_elements] = size(grid_polys); %(x-y-dim,5 vertices,n_grid)
    [~,~,cube_elements] = size(cube_polys);
    env = zeros(1,grid_elements);
    string_counter = 1;
    for ii = 1:grid_elements
        air = 1;
        cube = 0;
        for jj = 1:cube_elements
            full_included = inpolygon(cube_polys(1,:,jj),cube_polys(2,:,jj),grid_polys(1,:,ii),grid_polys(2,:,ii));
            no_touch = ~full_included;
            if full_included                %cube is placed fully within the grid place
                cube = jj;
                air = 0;
                env_str_array(string_counter) = get_predicate(ii,jj);
                string_counter = string_counter + 1;
                break
            elseif ~no_touch                %cube touches the grid place -> grid place is occupied by the cube and by air (least favourable situation)
                cube = jj;
                break
            end
        end
        if air
            if cube                         %keep 0 when no cube is touching, otherwise report suboptimal result where cube is not fully included in the grid place
                env(ii) = -1;
                env_str_array(string_counter) = get_predicate(ii,jj);
                string_counter = string_counter + 1;
                env_str_array(string_counter) = get_predicate(ii,0);
                string_counter = string_counter + 1;
            else
                env_str_array(string_counter) = get_predicate(ii,0);
                string_counter = string_counter + 1;
            end
        else
            env(ii) = cube;
        end
            
    end
end

%translates the numerical description into the symbolic one
function predicate = get_predicate(cell_id,obj_id)
    if obj_id == 0
        predicate = "on cell"+cell_id+" air";
    else
        predicate = "on cell"+cell_id+" cube"+obj_id;
    end
end

%Create a task file to be used with fast-downward planner
function create_task_file(file_name,init_predicates, goal_predicates, objects)
    object_string = "";
    init_string = "";
    goal_string = "";
    problem = "task1";
    domain = "eval_1";
    for ii = 1:length(objects)
        for iii = 1:objects{2}(ii)
             if ii == length(objects) && iii == objects{2}(ii)
                 object_string = object_string + objects{1}(ii) + iii;
             else
                 object_string = object_string + objects{1}(ii) + iii + " ";
             end
        end
    end
    for ii = 1:length(init_predicates)
        if ii == length(init_predicates)
            init_string = init_string + "("+init_predicates(ii)+")";
            goal_string = goal_string + "("+goal_predicates(ii)+")";
        else
            init_string = init_string + "("+init_predicates(ii)+") ";
            goal_string = goal_string + "("+goal_predicates(ii)+") ";
        end
    end
    fid = fopen(file_name,'w');
    fprintf(fid, '(define (problem %s)\r\n',problem);
    fprintf(fid, '(:domain %s)\r\n',domain);
    fprintf(fid, '(:objects %s)\r\n',object_string);
    fprintf(fid, '(:init %s)\r\n',init_string);
    fprintf(fid, '(:goal (and %s)))',goal_string);
    fclose(fid);
end

function [r_c,z_height,d_g,d_H] = plot_traj(ax,dmp,cube_stack,id,action,r_L,H,n_heuristic_polys,col_pick,col_place)
    global f_clear
    x_init_orig = dmp.x_init;
    x_end_orig = dmp.x_end;
    cla(ax)
    x_init = zeros(1,dmp.n_dims);
    x_end = x_init;
    
    x_end(2) = norm(x_init_orig(1:2)-x_end_orig(1:2));
    
    dmp.x_init = x_init;
    dmp.x_end = x_end;
    
    m = dmp.dynamics(dmp.means,0,"");
    y_ = m(:,2:2+dmp.n_dims-1);
    
    line(ax,[0,0],[-0.02,0.4],[0,0],'Color','k','LineWidth',0.5);
    
    if action == "place"
        plot3(ax,y_(:,1),y_(:,2),y_(:,3),'Color',col_place,'LineWidth',1.5)
        plot3(ax,dmp.x_end(1),dmp.x_end(2),dmp.x_end(3),'Color',col_place,'Marker','*','LineWidth',1.5);
        
    else
        plot3(ax,y_(:,1),y_(:,2),y_(:,3),'Color',col_pick,'LineWidth',1.5)
    end
    
    
    weights=[1,1];
    limits_x=[];
    L = zeros(1,2);
    L(2) = r_L*10+10
    L(1) = n_heuristic_polys-L(2);
    limits_z = x_end(2)*L/n_heuristic_polys;
    f=1;
    if f==1
    [policy_cost,label_z,fin,y_borders] = evaluate_rollout(dmp,m,limits_x,limits_z,H,weights);
    z_height = -policy_cost(2);
    r_c = z_height/x_end(2);
    h_clear = f_clear*x_end(2);
    d_H = H-z_height+h_clear;
    d_g = policy_cost(3);
    %line([0,0],[y_borders(limit_u),y_borders(limit_u)],[0,z_height],'Color','k','LineStyle',':','LineWidth',2);
    %line([0,0],[y_borders(limit_o),y_borders(limit_o)],[0,z_height],'Color','k','LineStyle',':','LineWidth',2);
    for jj = 1:length(limits_z)
        line(ax,[0,0],[limits_z(jj),limits_z(jj)],[0,weights(jj)*H],'Color','k','LineStyle',':','LineWidth',2);
    end
    %line([0,0],[-0.01,-0.01],[-0.2,0.25],'Color','r','LineStyle','--','LineWidth',1);
    %line([0,0],[0.16,0.16],[-0.2,0.25],'Color','r','LineStyle','--','LineWidth',1);
    line(ax,[0,0],[-0.02,0.4],[z_height,z_height],'Color','k','LineStyle','--','LineWidth',1);
    if action == "place"
        line(ax,[0,0],[-0.02,0.4],[z_height-h_clear,z_height-h_clear],'Color',col_place,'LineStyle',':','LineWidth',1);
    else
        line(ax,[0,0],[-0.02,0.4],[z_height-h_clear,z_height-h_clear],'Color',col_pick,'LineStyle',':','LineWidth',1);
    end
    end
    
    
    
end

function [traj,y_end] = get_traj(dmp,means,goals,cube_stack, id, action, step)
    f2 = findobj('Tag','second');
    global t_mp
    tic
    m = dmp.dynamics(means,0,"");
    t_mp{1} = t_mp{1}+toc;
    y_ = m(:,2:2+dmp.n_dims-1);
    
    fprintf("max in Z: %d\n",max(y_(:,3))-dmp.x_init(3))
    fprintf("min in Z: %d\n",min(y_(:,3))-dmp.x_init(3))
    
    y_end = y_(end,:);
    yd_ = m(:,2+dmp.n_dims:2+2*dmp.n_dims-1);
    ydd_ = m(:,2+2*dmp.n_dims:2+3*dmp.n_dims-1);
    if action == "place"
        idx = dmp.n_time_steps/2;
        %plot3(f2,y_(:,1),y_(:,2),y_(:,3),'Color',cube_stack.colors(id,:),'LineWidth',1.5)
        %plot3(f2,goals(1),goals(2),goals(3),'Color',cube_stack.colors(id,:),'Marker','*','LineWidth',1.5);
        %text(f2,y_(idx,1),y_(idx,2),y_(idx,3),"Step "+step,'HorizontalAlignment','left')
        %drawnow
    else
        %plot3(f2,y_(:,1),y_(:,2),y_(:,3),'k--','LineWidth',1.5)
    end
    
    x_tars = zeros(length(y_)-1,dmp.n_dims);
    dx_tar = zeros(length(y_)-1,3);
    dx_cur = zeros(length(y_)-1,3);
    ddx_cur = zeros(length(y_)-1,3);
    for j = 1:dmp.n_dims
        x_tars(:,j) = y_(2:end,j);
        if j < 4
            dx_cur(:,j) = yd_(1:end-1,j);
            dx_tar(:,j) = yd_(2:end,j);
            ddx_cur(:,j) = ydd_(1:end-1,j);
        end
    end
    euls = zeros(length(y_)-1,3);
    if dmp.n_dims > 3
        eul = x_tars(:,4:end);
        x_tar = x_tars(:,1:3);
        if dmp.n_dims == 4
            euls(:,2) = eul;
        elseif dmp.n_dims == 5
            euls(:,1:2) = eul;
        elseif dmp.n_dims == 6
            euls = eul;
        else
            disp("Error in traj generation: dimensions greater than 6 is not available")
            eul
        end
        euls(:,1) = -pi*ones(length(y_)-1,1);
        euls(:,2) = zeros(length(y_)-1,1);
        euls(:,3) = linspace(dmp.x_init(6),dmp.x_end(6),length(y_)-1);
        quats = mEul2cQuat(euls);
    end
    
    thetas = zeros(length(dx_cur),1);
    traj2 = [x_tar,dx_cur,thetas,ddx_cur,dx_tar,quats];
    %flatten the matrix to 1D as required for v-rep input
    traj = reshape(traj2',1,[]);   
    
    %figure(2)
    %hold on
    %dmp2 = dmp;
    %dmp2.x_init(1:3) = [0,0,0];
    %dmp2.x_end(1:3) = [0,0,0];
    %dmp.x_end(2) = norm(dmp.x_end(1:2) - dmp.x_init(1:2));
    %dmp2.plot_rollout(dmp2.means,"b",0.5)
    %figure(1)
end
function coppeliaQuat = mEul2cQuat(matlabEul)
    mQuat = eul2quat(matlabEul,'XYZ');
    coppeliaQuat = [mQuat(:,2:4),mQuat(:,1)];
end
    

