%--------------------------------------------------------------------------
%% Dynamic Programming based Hetergeneous Voronoi Diagram
% Input:
%       um  ----------- the original wind speed along x axis 
%       vm  ----------- the original wind speed along y axis
%      med  ----------- the hospital location
%  output_y ----------- the output file to save y (cluster index)
%  output_z ----------- the output file to save z (flight time)
%    weight ----------- wind weight 
% f_o_speed ----------- drone air speed (flight original speed)
%
% Output:
%         y ----------- the cluster index
%         z ----------- the flight time
%         t ----------- the running time for dynamic programming
% med_index ----------- the hospital index
%
% Notice:
%     um and vm may be extremely high (=200 * c) in obstacle or sea
%--------------------------------------------------------------------------
function [y,z,t, med_index] = hvd_dp(um, vm, med, output_y, output_z, weight, f_o_speed)

%% set parameter
c = 0.447;
obs = (um == 200*c) | (vm == 200*c);

% dimension
[n1, n2] = size(med);

% ss
ws_x = weight * um;
ws_y = -1 * weight * vm; % change direction

% initialize shortest time and cluster
z = 1e+10 * ones(n1,n2);   
y = zeros(n1,n2);

% facility (hospital) location
loc_binary = (med ~= 0);
[f_i_list,f_j_list] = find(loc_binary);

% set the shortest time as 0 for facility location
z(loc_binary) = 0;

% label the cluster index of facility location
for i = 1:length(f_i_list)
    y(f_i_list(i),f_j_list(i)) = i;  %% class
end

med_index = y; % record the hospital index

% initialize the distance for 16 directions
b = 1/sqrt(5);
a = 1/sqrt(2);
f_s_d_x = [b -b 2*b a 0 -a -2*b 1 -1 2*b a 0 -a -2*b b -b];
f_s_d_y = [2*b 2*b b a 1 a b 0 0 -b -a -1 -a -b -2*b -2*b];

% initialize the speed with 16 directions for obstacles
speed_0 = 1e-10 * ones(1,16);

% initialize the drone speed with 16 directions (wind speed considered)
% in obstacle location, speed = 1e-10; 
% else: the speed id combination of drone and wind speed
v_dir = cell(n1,n2);
for i = 1:n1
    for j = 1:n2
        f_s_list = zeros(1,16); 
        for k = 1:16
            speed = f_o_speed + ws_x(i,j) * f_s_d_x(k) + ws_y(i,j) * f_s_d_y(k); 
            f_s_list(k) = max(speed, 1e-10);
        end
        if obs(i,j) == 1
            v_dir{i,j} = speed_0;
        else
            v_dir{i,j} = f_s_list;
        end
    end
end

% dynamic programming
tic
z_last = zeros(n1,n2);
cell_size = 896.434;
n_z_last = 0;
n_iteration = 100; % the maximum number of iteration

for iter = 1:n_iteration
    for i = 3:n1-2
        for j = 3:n2-2
            if obs(i,j) == 0 % the inland area outside obstacle
                speed_d = cell(1,16);
                z_value = zeros(16,1);
                link = zeros(16,1); % flight time for 16 directions for z

                speed_d{1} = v_dir{i-2,j-1};
                speed_d{2} = v_dir{i-2,j+1};
                speed_d{3} = v_dir{i-1,j-2};
                speed_d{4} = v_dir{i-1,j-1};
                speed_d{5} = v_dir{i-1,j};
                speed_d{6} = v_dir{i-1,j+1};
                speed_d{7} = v_dir{i-1,j+2};
                speed_d{8} = v_dir{i,j-1};
                speed_d{9} = v_dir{i,j+1};
                speed_d{10} = v_dir{i+1,j-2};
                speed_d{11} = v_dir{i+1,j-1};
                speed_d{12} = v_dir{i+1,j};
                speed_d{13} = v_dir{i+1,j+1};
                speed_d{14} = v_dir{i+1,j+2};
                speed_d{15} = v_dir{i+2,j-1};
                speed_d{16} = v_dir{i+2,j+1};

                z_value(1) = z(i-2,j-1);
                z_value(2) = z(i-2,j+1);
                z_value(3) = z(i-1,j-2);
                z_value(4) = z(i-1,j-1);
                z_value(5) = z(i-1,j);
                z_value(6) = z(i-1,j+1);
                z_value(7) = z(i-1,j+2);
                z_value(8) = z(i,j-1);
                z_value(9) = z(i,j+1);
                z_value(10) = z(i+1,j-2);
                z_value(11) = z(i+1,j-1);
                z_value(12) = z(i+1,j);
                z_value(13) = z(i+1,j+1);
                z_value(14) = z(i+1,j+2);
                z_value(15) = z(i+2,j-1);
                z_value(16) = z(i+2,j+1);

                y_value(1) = y(i-2,j-1);
                y_value(2) = y(i-2,j+1);
                y_value(3) = y(i-1,j-2);
                y_value(4) = y(i-1,j-1);
                y_value(5) = y(i-1,j);
                y_value(6) = y(i-1,j+1);
                y_value(7) = y(i-1,j+2);
                y_value(8) = y(i,j-1);
                y_value(9) = y(i,j+1);
                y_value(10) = y(i+1,j-2);
                y_value(11) = y(i+1,j-1);
                y_value(12) = y(i+1,j);
                y_value(13) = y(i+1,j+1);
                y_value(14) = y(i+1,j+2);
                y_value(15) = y(i+2,j-1);
                y_value(16) = y(i+2,j+1);

                link_to_z = zeros(1,16);
                for dir = 1:16            
                    if dir == 5 || dir == 8 || dir == 9 || dir == 12
                        dis = cell_size;
                    elseif dir == 4 || dir == 6 || dir == 11 || dir == 13
                        dis = cell_size * sqrt(2);
                    elseif dir == 1 || dir == 2 || dir == 3 || dir == 7 || dir == 10 || dir == 14 || dir == 15 || dir == 16
                        dis = cell_size * sqrt(5);
                    end
                    
                    % flight time flying from "dir" to the point z(i,j)
                    link_to_z(dir) = dis/(2 * v_dir{i,j}(dir)) + dis/(2 * speed_d{dir}(dir));
                    
                    % flight time for direction "dir" = flight time in the
                    % neighbour point (from the direction "dir") + flight
                    % time flying from such neighbout point to the point z(i,j)
                    link(dir) = z_value(dir) + link_to_z(dir); 
                end
                z(i,j) = min(z(i,j), min(link));
                [~,I] = min(link);
                y(i,j) = y_value(I);
            end
        end
    end
    if norm(z_last - z, 'fro') < 1e-5
        break;
    end
    z_last = z;
    n_z_last = n_z_last + 1;
end
t = toc;

%% save result in output file
dlmwrite(output_y, y);
dlmwrite(output_z, z);

