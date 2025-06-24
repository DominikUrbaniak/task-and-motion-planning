classdef dmp_6Dn4
    %Train and optimize 3D-trajectories with dmps
    %Mandatory input: text-file with a trajectory to be imitated by dmp
    %with time and desired positions, velocities and acceleration per time
    %step
    
    %Methods to call 
    %train: learn mean_x and mean_y for the given trajectory
    %optimize: learn desired trajectory with learned initial mean_x and mean_y 
    
    %Necessary function files:
    %rbfn_fit.m
    %rbfn_predict.m
    %exponential_fcn.m
    %dE_spring3D.m
    
    %Examples
    %test.m: optimizing the throwing task
    %dmp_pp_test.m: test the rotation and dilatation invariance according
    %to the dmp++ paper
    
    properties
        
        damping_coefficient     %input for dmp
        spring_constant         %input for dmp, will be computed such that the system is critically damped
        mass                    %input for dmp
        alpha                   %for computing the exponential phase xs_phase (same for x and y-direction)
        xs_phase                %exponential input for the function approximation
        n_rbf                   %Number of basis functions for th function approximation
        bf_type                 %Basis function type: default is "mollifier", change to anything else for using radial basis functions
        bf_width                %factor that influences the number of intersections between the basis function
        centers                 %Basis function centers for n_rbf [n_rbf,1]
        widths                  %Basis function widths (differently computed for mollifier and rbf)
        means                    %Parameters [n_rbf x 1] for approximating the forcing terms in all dimensions
        covar                   %Parameters [n_rbf x n_rbf] for the exploration of mean
        x_init                  %Initial point [x,y,z,theta,...] of the trajectory
        x_end                   %Final point [x,y] of the trajectory
        tau                     %Duration of the trajectory execution, constant, according to imitated trajectory: t(end)-t(1)
        n_time_steps            %Selectable number of time steps to execute the trajectory
        learned_position        %Constant value initialized from imitating the original trajectory: x_end-x_init
        rescale                 %Boolean, initialized to 1, keeps the trajectory invariant to dilatation and rotation when selecting different initial or goal positions
        ball_init               %landing site of the ball of demostrated trajectory
        ball_goal               %desired landing site of the ball after optimization
        win_size                %window size of the figures
        init_pertub             %pertub the initial and end point of the trajectory
        goal_pertub
        sigma
        sigma_goal
        covar_goal
        cov_decay
        n_dims
        orientation_factor
        vp1
        vp2
        vp3
        param_pertub
        z_norm
    end
    
    methods
        function obj = dmp_6Dn4(n_dims)
            %DMP Construct an instance of this class
            obj.n_dims = n_dims;
            obj.alpha = 2;
            obj.damping_coefficient = 10; %default: 30
            obj.spring_constant = 1/4*obj.damping_coefficient^2;
            obj.mass = 1;
            obj.n_rbf = 10;
            obj.sigma = 0.5;
            obj.sigma_goal = 0.003;
            obj.covar = obj.sigma*obj.sigma*eye(obj.n_rbf).*ones(obj.n_rbf,obj.n_rbf,obj.n_dims);
            obj.covar_goal = obj.sigma_goal*obj.sigma_goal*eye(obj.n_dims);
            obj.cov_decay = 1;
            obj.n_time_steps = 200;
            obj.bf_type = "rbf"; % else "rbf"
            if obj.bf_type == "mollifier"
                obj.bf_width = 1.7;                     %good value for mollifier
            else
                obj.bf_width = 0.7;                     %good value for rbf
            end
            obj.rescale = 1;
            obj.win_size = [1300,800];
            obj.init_pertub = zeros(1,n_dims);           
            obj.goal_pertub = zeros(1,n_dims);
            obj.orientation_factor = 1000;
            obj.param_pertub = [1,1,1,0,0,0];
            obj.z_norm = 1;
        end
        
        function obj = train(obj,trajectory_file)
            plot_training = 0;
            %Imitate the desired trajectory from the trajectory_file with a dmp
            obj.covar = obj.sigma*obj.sigma*eye(obj.n_rbf).*ones(obj.n_rbf,obj.n_rbf,obj.n_dims);
            trajectory = readmatrix(trajectory_file);
            obj.tau = trajectory(end,1)-trajectory(1,1);
            xs_traj = trajectory(:,2:2+obj.n_dims-1);
            xds_traj = trajectory(:,2+obj.n_dims:2+2*obj.n_dims-1);
            xdds_traj = trajectory(:,2+2*obj.n_dims:2+3*obj.n_dims-1);
            obj.x_init = xs_traj(1,:)+obj.init_pertub;
            obj.x_end = xs_traj(end,:)+obj.goal_pertub;
            obj.learned_position = obj.x_end-obj.x_init;
            [obj.n_time_steps,~] = size(trajectory);
            
            [obj.xs_phase,~] = exponential_fcn(obj,obj.alpha,obj.tau,obj.n_time_steps,1,0);% changed from 1 to 2
            f_target = obj.tau*obj.tau*xdds_traj + (obj.spring_constant*(xs_traj - obj.x_end) + obj.damping_coefficient*obj.tau*xds_traj)/obj.mass;
            if plot_training
                figure('Position',[50 50 obj.win_size(1) obj.win_size(2)])
            end
            for i = 1:obj.n_dims
                [obj.means(:,i),obj.centers,obj.widths,activation_x] = rbfn_fit(obj,f_target(:,i));
                if plot_training
                    subplot(obj.n_dims,1,i)
                    sgtitle("Reproducing the forcing terms in X,Y,Z-Direction ("+obj.bf_type+", bf-width="+obj.bf_width+")")
                    hold on
                    weighted_ac = obj.means(:,i)'.*activation_x;
                    plot(f_target(:,i),'b','LineWidth',2)
                    plot(sum(weighted_ac,2),'k','LineWidth',2)
                    plot(weighted_ac,'k:')
                    xlabel("Time Steps")
                    ylabel("Forcing Terms in dim:"+i)
                    legend("Demonstration","Reproduction","Basis Functions")
                end
            end 
            if plot_training
                m = dynamics(obj,obj.means,0,"");
                ts = linspace(0,obj.tau,length(trajectory))';
                ts_exp = obj.xs_phase*obj.tau;
                figure('Position',[50 50 obj.win_size(1) obj.win_size(2)])
                sgtitle("The demonstrated Trajectory and its Reproduction")
                subplot(3,1,1)
                hold on
                d=plot(ts,xs_traj,'b','LineWidth',1);
                r=plot(ts(1:end,1),m(:,2:2+obj.n_dims-1),'k');
                xlabel("Time in s")
                ylabel("Position")
                legend([d(1),r(1)],"Demonstration in x,y,z","Reproduction in x,y,z")
                subplot(3,1,2)
                hold on
                plot(ts,xds_traj,'b')
                plot(ts(1:end),m(:,2+obj.n_dims:2+2*obj.n_dims-1),'k')
                xlabel("Time in s")
                ylabel("Velocity")
                subplot(3,1,3)
                hold on
                plot(ts,xdds_traj,'b')
                plot(ts(1:end),m(:,2+2*obj.n_dims:2+3*obj.n_dims-1),'k')
                xlabel("Time in s")
                ylabel("Acceleration") 
            end
        end
        
        function [obj,policy_cost,h,train,label_z,ii,ti,y_borders] = optimize(obj,max_runs,n_samples,x_vec,z_vec,saving,directory,max_val,wei)
            %figure('Position',[50 50 obj.win_size(1) obj.win_size(2)])
            %ax1 = subplot(2,1,1);
            h =10;
            %title("DMP Trajectories")
            %hold on
            %grid on
            %axis equal
            covar_decay_factor=1;
            %obj.plot_rollout(0,"r",2);
            train = zeros(max_runs,obj.n_rbf,obj.n_dims);
            label_z = zeros(max_runs,length(z_vec)+length(x_vec)+2);
            label_x = label_z;
            %cost_avg = zeros(max_runs,3);    
            %tic
            for ii = 1:max_runs
                
                mean_expl = explore(obj,n_samples);
                meanX = mean_expl(:,1:obj.n_rbf);
                meanY = mean_expl(:,obj.n_rbf+1:2*obj.n_rbf);
                meanZ = mean_expl(:,2*obj.n_rbf+1:end);
                costs = zeros(n_samples,5);
                if saving
                    if ~exist(directory, 'dir')
                        mkdir(directory)
                    end
                    mkdir(directory+"/run_"+ii)
                end
                for jj = 1:n_samples
                    means_e = mean_expl(jj,:,:);
                    means_e = reshape(means_e,[obj.n_rbf,obj.n_dims]);
                    m = dynamics(obj,means_e,saving,directory+"/run_"+ii+"/rollout_sample_"+jj+".txt");
                    [costs(jj,:),~,~] = evaluate_rollout(obj,m,x_vec,z_vec,max_val,wei);
                end
                weights = costs_to_weights(obj,costs(:,1),h);
                mean_new = mean(mean_expl.*weights)/mean(weights);
                mean_new = reshape(mean_new,[obj.n_rbf,obj.n_dims]);
                obj.means = mean_new;
                obj.covar = covar_decay_factor^2*obj.covar;
                m = dynamics(obj,obj.means,saving,directory+"/run_"+ii+"/rollout_sample_"+jj+".txt");
                [policy_cost,label_z(ii,:),fin,y_borders] = evaluate_rollout(obj,m,x_vec,z_vec,max_val,wei);
                
                %if mod(ii,100) == 0
                %    ii
                %    policy_cost
                %    figure(4)
                %    obj.plot_rollout(0,"k",0.1);
                %end
                train(ii,:,:) = mean_new;
                if saving
                    writematrix(mean_new,directory+"/run_"+ii+"/new_mean.txt");
                end
                cost_avg(ii,:) = mean(costs);
                %if cost_avg(ii,1) < 0.05
                    %break
                %end
                %Distinguish one optimization run and several runs (take the mean costs
                %from multiple optimizations)
                avg_costs = cost_avg;
                [~,n_costs] = size(avg_costs);
                if ii == max_runs || fin
                    ti = 0;%toc;
                    %h(1) = figure;
                    %subplot(2,1,1)
                    %obj.plot_rollout(0,"k",2);
                    %axis equal
                    %axis([-0.01,0.2,-0.02,0.2,-0.3,0.3])
                    %view([90,0,0])
                    %subplot(2,1,2)
                    %h(2) = figure;
                   % hold on
                    %grid on
                    %for jj = 1:n_costs
                    %    plot(avg_costs(:,jj))
                    %end
                    %legend("Full Costs","Avoidance Cost Z","Goal Cost")
                    %title("DMP Costs")
                    %xlabel("Iterations")
                    break
                else
                    %obj.plot_rollout(0,"c",0.5);
                end
            end
            
        end
        
        function mean_expl = explore(obj,n_samples)
            mean_expl = zeros(n_samples,obj.n_rbf,obj.n_dims);
            %goal_covar = obj.sigma_goal*obj.sigma_goal*eye(3);
            %goals_expl1 = mvnrnd(obj.x_end(1:3),goal_covar,n_samples);
            %goals_expl2 = mvnrnd(obj.x_end(4:6),obj.orientation_factor*goal_covar,n_samples); %pertubation of the angles needs higher values
            %goals_expl = [goals_expl1,goals_expl2].*goal_pertub;
            %goals_expl = goals_expl+obj.x_end.*~goal_pertub;
            for j = 1:obj.n_dims
                if obj.param_pertub(j)
                    mean_expl(:,:,j) = mvnrnd(obj.means(:,j),obj.covar(:,:,j),n_samples); %samples from distribution
                else
                    mean_expl(:,:,j) = zeros(n_samples,obj.n_rbf)+obj.means(:,j)';
                end
            end
        end
        
        function obj = set_exploration_rate(obj,sigma_traj, sigma_goal)
            obj.sigma = sigma_traj;
            obj.covar = obj.sigma*obj.sigma*eye(obj.n_rbf).*ones(obj.n_rbf,obj.n_rbf,obj.n_dims);
            obj.sigma_goal = sigma_goal;
            obj.covar_goal = obj.sigma_goal*obj.sigma_goal*eye(obj.n_dims);
        end
        
        function [costs,zi,fin,y_borders] = evaluate_rollout(obj,m,x,z,max_val,weights)
            
            
            
            y = m(:,2:2+obj.n_dims-1);
            y_end = y(end,1:3);
            yd = m(:,2+obj.n_dims:2+2*obj.n_dims-1);
            ydd = m(:,2+2*obj.n_dims:2+3*obj.n_dims-1);
            fin = 0;
            %y2_z = zeros(1,length(z));
            cost_z = zeros(1,length(z));
            y_borders = zeros(1,length(z)+length(x));
            heights_z = zeros(1,length(z)+length(x)+2);
            lows_z = cost_z;
            zi = zeros(1,length(z)+length(x)+2);
            %if weights(2)
            %    for iii = 1:length(z)
            %        %y2_z = iii*y_end(2)/(length(z)+1);
            %        [~,id] = min(abs(y(:,2)-z(iii)));

%                    if z(iii)%>0
%                        heights_z(iii) = 1/weights(2)*y(id,3);
%                        zi(iii) = 1/weights(2)*y(id,3);
%                        y_borders(iii) = y(id,2);
%                    end
%                end
%            end
                for iii = 1:length(z)
                    %y2_z = iii*y_end(2)/(length(z)+1);
                    [~,id] = min(abs(y(:,2)-z(iii)));
                    
                    if z(iii)%>0
                        if weights(iii) ~= 0
                            heights_z(iii) = 1/weights(iii)*y(id,3);
                            zi(iii) = 1/weights(iii)*y(id,3);
                            y_borders(iii) = y(id,2);
                            y_borders(iii) = id;
                        end
                    end
                end
            
            %if weights(1)
            %    for iii = 1:length(x)
            %        [~,id] = min(abs(y(:,2)-x(iii)));
            %        if x(iii)%>0
            %            heights_z(length(z)+iii) = 1/weights(1)*y(id,1);
            %            zi(length(z)+iii) = 1/weights(1)*y(id,1);
            %            y_borders(length(z)+iii) = y(id,2);
            %        end
            %    end
            %end
            %if weights(3) ~= 0 
            %    heights_z(end-1) = -0.1+1/weights(3)*(obj.x_init(2) - min(y(:,2)));
            %    zi(end-1) = -0.1+1/weights(3)*(obj.x_init(2) - min(y(:,2)));
            %end
            %if weights(4) ~= 0
            %    heights_z(end) = 0.001+1/weights(4)*-(obj.x_end(2) - max(y(:,2)));
            %    zi(end) = 0.001+1/weights(4)*-(obj.x_end(2) - max(y(:,2)));
            %end
            T = length(ydd);
            
            sum_ydd = sum(sum(ydd(1:3).^2));
            acc_weight = 1000;
            z_weight = 0.8;
            costs = zeros(5,1);
            r = [0.001,0.001,0.08];
            %heights_z
            if any(weights)
                [costs_2,min_id] = min(nonzeros(heights_z));%max(cost_z);
                costs(2) = -costs_2;
                min_id;
                if costs(2) < -max_val && costs(3) < 0.01
                    fin = 1;
                end
            end
            if nonzeros(z)~=length(z)
                costs(5) = 0;%0.1*sum(min(0,obj.x_end(2)-y(:,2)));%1*max(nonzeros(lows_z));%max(cost_z);
                %cost for datapoints z<0, y<y_init-0.01, y>y_end+0.01
                costs(4) = - sum(min(0,r(1)+y(:,2)-obj.x_init(2))) - sum(min(0,r(2)+obj.x_end(2)-y(:,2))) ...
                    - sum(min(0,r(3)+obj.x_end(3)-y(:,3))) - sum(min(0,y(id:end,3)-obj.x_end(3))) ;% - sum(min(0,y(:,3)))
            end
            costs(3) = 10*norm(y_end-obj.x_end(1:3));
            %costs(5) = 0;%acc_weight * sum_ydd / T;
            costs(1) = costs(2)+costs(3)+costs(4)+costs(5);
        end

        function weights = costs_to_weights(obj,full_costs,h)
            %Compute weigts to update the mean_x and mean_y of a dmp
            costs_range = max(full_costs)-min(full_costs);
            %weights = ones(length(full_costs),1);
            if costs_range == 0
                weights = ones(length(full_costs),1);
            else
                weights = exp(-h*(full_costs-min(full_costs))/costs_range);
            end
            weights = weights/sum(weights);
        end 
        
        function [yd_, zd_] = dE_spring_dims(obj,xs_spring,tau,spring_const,damping,x_end,mass)
            y_ = xs_spring(1,1:obj.n_dims);
            z_ = xs_spring(1,obj.n_dims+1:2*obj.n_dims);
            yd_ = z_/tau;
            zd_ = (-spring_const*(y_-x_end) - damping*z_)/(mass*tau);
        end
        
        function [xs,xds] = exponential_fcn(obj,alpha,tau,n_time_steps,initial_state, attractor_state)
            ts = linspace(0,1.5*tau,n_time_steps)';
            exp_term = exp(-alpha*ts/tau);
            xd = -alpha/tau * exp_term;
            val_range = initial_state - attractor_state;
            xs = val_range .* exp_term + attractor_state;
            xds = val_range .* xd;
            %figure
            %plot(ts, exp_term)
        end
        
        function m = dynamics(obj,means,saving,file_out)
            %Compute the dynamics (y,yd,ydd) from mean_x and mean_y
            [obj.xs_phase,~] = exponential_fcn(obj,obj.alpha,obj.tau,obj.n_time_steps,1,0);
            forcing_terms = zeros(obj.n_time_steps,obj.n_dims);
            for k = 1:obj.n_dims
                %if k == 1
                %    k
                %    mean(:,k)
                %end
                [forcing_terms(:,k),~] = rbfn_predict(obj,means(:,k));
            end
            %[out2,~] = rbfn_predict(obj.xs_phase,meanY,obj.centers,obj.widths,obj.bf_type);
            %[out3,~] = rbfn_predict(obj.xs_phase,meanZ,obj.centers,obj.widths,obj.bf_type);
            %forcing_terms = [out1',out2',out3'];
            if obj.rescale
                new_position = obj.x_end - obj.x_init;
                obj.learned_position;
                M = rotodilatation(obj,obj.learned_position,new_position);
                %M = M/norm(M);
                
                %forcing_terms = forcing_terms*M;
                forcing_terms = (M*forcing_terms')';
            end
            forcing_terms_flip = flip(forcing_terms);
            forcing_terms_flip = forcing_terms.*1;
            %other xs, xds values for SPRING by integration
            xs_spring = zeros(obj.n_time_steps,2*obj.n_dims); %3D for Y and Z parts of first order diff eq
            xds_spring = zeros(obj.n_time_steps,2*obj.n_dims);
            xs_spring(1,1:obj.n_dims) = obj.x_init; %see integrateStart: x.segment(0) = initial_state
            xs_spring(1,obj.n_dims+1:2*obj.n_dims) = zeros(1,obj.n_dims);

            [yd_,zd_] = dE_spring_dims(obj,xs_spring(1,:),obj.tau,obj.spring_constant,obj.damping_coefficient,obj.x_end,obj.mass);

            xds_spring(1,1:obj.n_dims) = yd_;
            xds_spring(1,obj.n_dims+1:2*obj.n_dims) = zd_;
            xds_spring(1,obj.n_dims+1:2*obj.n_dims) = xds_spring(1,obj.n_dims+1:2*obj.n_dims) + forcing_terms_flip(1,:)/obj.tau; 
            ts_exp = obj.tau*obj.xs_phase;
            ts_exp_flip = flip(ts_exp);
            ts = linspace(0,obj.tau,obj.n_time_steps);
            dts_exp_flip = zeros(obj.n_time_steps,1);
            for tts=2:obj.n_time_steps
                dts(tts) = ts(tts)-ts(tts-1);
                %dts = flip(dts);
                dts_exp_flip(tts) = ts_exp_flip(tts)-ts_exp_flip(tts-1);
            end
            for tt=2:obj.n_time_steps
                dt = dts(tt);
                xs_spring(tt,:) = xs_spring(tt-1,:) + dt*xds_spring(tt-1,:);
                xs_spring(tt,:);
                [yd_,zd_] = dE_spring_dims(obj,xs_spring(tt,:),obj.tau,obj.spring_constant,obj.damping_coefficient,obj.x_end,obj.mass);
                xds_spring(tt,1:obj.n_dims) = yd_;
                xds_spring(tt,obj.n_dims+1:2*obj.n_dims) = zd_;
                xds_spring(tt,obj.n_dims+1:2*obj.n_dims) = zd_ + forcing_terms_flip(tt,:)/obj.tau;
            end

            y_out = xs_spring(:,1:obj.n_dims);
            yd_out = xds_spring(:,1:obj.n_dims);
            ydd_out = xds_spring(:,obj.n_dims+1:2*obj.n_dims)/obj.tau;
            
            m = [ts_exp_flip,y_out,yd_out,ydd_out];
            
            if saving
                %disp("saved as: "+file_out)
                writematrix(m,file_out);
            end
        end
        
        function [params,centers,widths,ac] = rbfn_fit(obj,targets)

            weights = eye(obj.n_time_steps);
            xt = linspace(0,1,obj.n_time_steps);
            centers = linspace(0,1,obj.n_rbf);
            widths = ones(obj.n_rbf,1);
            c = zeros(1,obj.n_rbf);
            for i =1:obj.n_rbf
                %c(i) = exp(-tau*i*tau/n_basis_functions);
                c(i) = exp(-obj.tau*i*obj.tau/obj.n_rbf);
            end
            if obj.bf_type == "mollifier"
                %for basis function calculation from dmp++
                for i=2:obj.n_rbf
                    w = 1/(centers(i)-centers(i-1))/obj.bf_width;
                    widths(i) = w;
                end
                widths(1) = widths(2);
            else
                for i=1:obj.n_rbf-1
                    w = sqrt((centers(i+1)-centers(i))^2/(-8*log(obj.bf_width)));
                    widths(i) = w;
                end
                widths(obj.n_rbf) = widths(obj.n_rbf-1);
            end

            ac = zeros(obj.n_time_steps,obj.n_rbf);
            %figure(5)
            %hold on
            for i=1:obj.n_rbf 

                if obj.bf_type == "mollifier"
                    ac(:,i) = mollifier(obj,xt,centers(i),widths(i));
                    %plot(ac(:,i))
                else
                    ac(:,i) = rbf(obj,xt,centers(i),widths(i));
                end
            end
            params = (ac'*weights*ac)\(ac'*weights*targets);
            centers = centers';
        end
        
        %% Predict trajectories from the trained basis function networks

        function [out,rbfn] = rbfn_predict(obj,params)
            xt = linspace(0,1,obj.n_time_steps);
            n_x = length(obj.xs_phase);
            rbfn = zeros(obj.n_rbf,n_x);
            out = zeros(1,n_x);
            for i = 1:obj.n_rbf   
                if obj.bf_type == "mollifier"
                    rbfn(i,:) = params(i)*mollifier(obj,obj.xs_phase,obj.centers(i),obj.widths(i));
                else
                    rbfn(i,:) = params(i)*rbf(obj,xt,obj.centers(i),obj.widths(i));
                end
                out = out + rbfn(i,:);
            end

        end

        function activation = rbf(obj,x,c,w)
            activation = exp(-0.5*1/(w)^2 * (x-c).^2);
        end
        function activation = mollifier(obj,x,c,w)
            term = abs(w*(x-c));
            activation = exp(-1./(1-term.^2)) .* (term<1);
            activation(isnan(activation)) = 0;
        end
        
        %Plotting the trowing task
        function plot_rollout(obj,mean,col_ball,lineW_traj)
            if ~exist('col_ball','var')
                % third parameter does not exist, so default it to something
                col_ball = 'c';
            end
            if ~exist('lineW_traj','var')
                % third parameter does not exist, so default it to something
                lineW_traj = 0.3;
            
            end
            col_ball_traj = "--"+col_ball;
            x_left = -0.6;
            x_right = 0.4;
            %axis([x_left x_right -0.8 0.3])
            hold on
            grid on
            view([90,0,0])
            %plotting floor
            %x = linspace(-1,1,2);
            %y = linspace(-1,1,2);
            %z = -0.3*ones(2,2);
            %surf(x,y,z);
            
            [rows,cols]=size(mean);
            if cols == 3*obj.n_rbf
                for ii = 1:rows
                    m = dynamics(obj,mean(ii,:,:),0,"");
                    y = m(:,2:2+obj.n_dims-1);
                    
                    plot3(y(:,1),y(:,2),y(:,3),'LineWidth',0.3)
                end
            else
                m = dynamics(obj,obj.means,0,"");
                y = m(:,2:2+obj.n_dims-1);
                t_exp = m(:,1); 
                [min1,t1] = min(abs(y(:,2)-0.0075));
                [min2,t2] = min(abs(y(:,2)-0.075));
                [min3,t3] = min(abs(y(:,2)-0.1425));
                plot3(y(:,1),y(:,2),y(:,3),'Color',col_ball,'LineWidth',lineW_traj)
                plot3(obj.x_end(1),obj.x_end(2),obj.x_end(3),'k*')
            end
        end
        
        %Make trajectory invariant to rotodilatation
        function M = rotodilatation(obj,x0,x1)
            %v = cross(x0,x1);
            %if v == 0
                %M = eye(3);
            %else
                %vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
               % M = eye(3) + vx + vx^2*(1-dot(x0,x1))/(norm(cross(x0,x1))^2);
            %end
            x0 = x0(1:2);
            x1 = x1(1:2);
            x0_norm = x0/norm(x0);
            x1_norm = x1/norm(x1);
            
            M0 = fnAR(obj,x0_norm);
            M1 = fnAR(obj,x1_norm);
            M_2D = M1 * M0' * norm(x1) / norm(x0);
            %M_2D = eye(2) * norm(x1) / norm(x0);
            M = eye(obj.n_dims);
            M(1:2,1:2) = M_2D;
            if obj.z_norm
                M(3,3) = M(3,3) * norm(x1) / norm(x0); %norm of Z-coordinate to keep the same shape
            end
                %M(1,2) = -M(1,2);
            %M(2,1) = -M(2,1);
            %M(1:3,1:3) = M_3D;
            %M(1,1) = M_2D(1,1);
            %M(1,3) = M_2D(1,2);
            %M(3,1) = M_2D(2,1);
            %M(3,3) = M_2D(2,2);
        end
        
        %Compute the Rotation for the rotodilatation function
        function R = fnAR(obj,x)
            n = length(x);
            R = eye(n);
            step = 1;
            while (step<n)
                A = eye(n);
                it=1;
                while (it < n-step+1)
                    r2 = x(it)*x(it) + x(it+step) * x(it+step);
                    if (r2>0)
                        r = sqrt(r2);
                        pcos = x(it)/r;
                        psin = -x(it+step)/r;
                        A(it,it) = pcos;
                        A(it,it+step) = psin;
                        A(it+step,it) = -psin;
                        A(it+step,it+step) = pcos;
                    end
                    it = it+2*step;
                end
                step = step*2;
                x = x*A;
                R = A*R;
            end
        end
        function obj = set_damping(obj,new_damping_coefficient)
            obj.damping_coefficient = new_damping_coefficient;
            obj.spring_constant = 1/4*new_damping_coefficient^2;
        end
             
        %Plotting general trajectories
        function plot_trajectory(obj,mean)
            figure
            hold on
            grid on
            [rows,cols]=size(mean);
            if cols == 3*obj.n_rbf
                for ii = 1:rows
                    [y,~,~,y_ball,r_point,~] = dynamics(obj,mean(ii,1:obj.n_rbf),mean(ii,obj.n_rbf+1:2*obj.n_rbf),mean(ii,2*obj.n_rbf+1:end),0,"");
                    plot3(y(:,1),y(:,2),y(:,3))

                    %plotting ball curve
                    plot3(y_ball(:,1),y_ball(:,2),y_ball(:,3),'--c','LineWidth',0.3)
                    plot3(y_ball(1:10:end,1),y_ball(1:10:end,2),y_ball(1:10:end,3),'ko','LineWidth',0.3)
                    plot3(r_point(1),r_point(2),r_point(3),'g*');
                end
            else
                [y,~,~,y_ball,r_point,~] = dynamics(obj,obj.mean_x,obj.mean_y,obj.mean_z,0,"");
                plot3(y(1:end,1),y(1:end,2),y(1:end,3))
                plot3(y(1:20:end,1),y(1:20:end,2),y(1:20:end,3),'ko','LineWidth',0.3)
                plot3(y(1,1),y(1,2),y(1,3),'r+','MarkerSize',20,'LineWidth',2)
                plot3(y(end,1),y(end,2),y(end,3),'bx','MarkerSize',20,'LineWidth',2)
                diff_end = obj.x_end-y(end,:);
                fprintf("x_end goal difference to real x_end: (%f,%f,%f)\n", diff_end(1),diff_end(2),diff_end(3))
                %fprintf("x_init goal difference to real x_init: (%f,%f,%f)\n", diff_end(1),diff_end(2),diff_end(3))
                xlabel("X")
                ylabel("Y")
                zlabel("Z")
            end
        end
        
    end
end


