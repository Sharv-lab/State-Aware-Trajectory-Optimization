clc;
close all;
clear all;

%   Define MATLAB Paths

% if(ispc)
%     addpath ..\..\gnss_ins_functions\;
% else
%     addpath ../../gnss_ins_functions/;
% end    

%   Save Sensor Data Flag

SENSOR_SAVE_FLAG = 1;           %   Sensor data gets saved if = 1.
                                %   If = 0, sensor data is not saved.

%   Select the Appropriate Motion Profile

%TRAJECTORY = 'A' --- >     heading North at a constant speed and sinusoidally varying heading 
%TRAJECTORY = 'B' --- >     heading North at constant speed and heading
%TRAJECTORY = 'C' --- >     heading North at a sinusoidally varying heading and speed    
%TRAJECTORY = 'D' --- >     heading North at constant heading but sinusoidally varying speed

TRAJECTORY = 'C';

% Generate the sensor and trajectory data
% Assuming that my quadcopter can generate that trajectory- Generate IMU


gen_traj_sensor_data_Quad(TRAJECTORY,SENSOR_SAVE_FLAG);


%   Select the Appropriate EKF FILTER_ARCHITECTURE
%   Note:  'H' = Heading derived from a Magnetometer/Compass.

%   'PVH'   --- >     Position, Velocity and Magnetometer/Compass Aiding
%   'PV'    --- >     Position and Velocity Aiding
%   'PH'    --- >     Position and Magnetometer/Compass Aiding
%   'P'     --- >     Position Aiding only

FILTER_ARCHITECTURE = 'PV';

%   Define EKF Run-time and Saving Parameters

LINEARIZE_ABT_TRUE = 0;     %   If = 1, EKF is linearized about
                            %   the true state trajectory (Idealized
                            %   scenario).  Otherwise, linearization occurs
                            %   about the estimated trajectory (realistic
                            %   real-time scenario).
                            
FILTER_FLAG = 1;            %   If = 0, inertial navigator is run in an open
                            %   open loop fashion (i.e., no aiding).  When
                            %   = 1, the system integrates inertial sensor
                            %   derived navigation information with the
                            %   aiding source information.

%   Simulation Data Save Flag

FILTER_SAVE_FLAG = 1;           %   Navigation algorithm output data gets saved 
                                %   if = 1.  If = 0, navigation algorithm data 
                                %   is not saved.

%   Simulate EKF

gnss_ins_EKF_2D_Quad(TRAJECTORY,FILTER_ARCHITECTURE,LINEARIZE_ABT_TRUE,...
                                    FILTER_FLAG, FILTER_SAVE_FLAG);
                                    
%%   Plot Simulation Results
 
plot_EKF_results_Quad(TRAJECTORY,FILTER_ARCHITECTURE);

% Plot Observability Grammian
pathName = '.\New_Data\';
eval(['load ',pathName,'TRAJECTORY_',TRAJECTORY,'_FILTER_ARCHITECTURE_',FILTER_ARCHITECTURE,'.mat']);

Wsym = 0.5*(W + W');
Wsym = Wsym / max(abs(Wsym(:)));

[V, D] = eig(Wsym);              % V - Evec , D -Eval
evals = diag(D);                 % eigenvalues (unsorted)

% Sort descending (largest -> smallest)
[evals_sorted, idx] = sort(evals, 'descend');
V_sorted = V(:, idx);


disp('Evals')
disp(V)

disp('Eigenvalues (descending):');
disp(evals_sorted);

Vn = V_sorted ./ max(abs(V_sorted), [], 1);
state_names = {'dpN','dpE','dvN','dvE','dpsi','bax','bay','bg'};

 % Dominant states in each eigenvector
for j = 1:8
    [~, order] = sort(abs(V_sorted(:,j)), 'descend');
    fprintf('\nMode %d: eval = %.6e\n', j, evals_sorted(j));
    for m = 1:3
        i = order(m);
        fprintf('  %s  (%+.3f)\n', state_names{i}, Vn(i,j));
    end
end

lambda_min = min(evals_sorted);

% Guard against tiny negative numerical noise
lambda_min = max(lambda_min, 0);

eta = 1 / sqrt(lambda_min);                 % local unobservability index

fprintf('eta = 1/sigma_min (local unobservability index) = %.12e\n', eta);