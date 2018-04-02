function varargout = runsim(stepsOrData, pauseLen, makeVideo)

global Param;
global Data;
global State;

if ~exist('pauseLen','var')
    pauseLen = 0.3; % seconds
end
if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(5, 1/pauseLen));
        open(vo);
    end
end
% Initalize Params
%===================================================
Param.initialStateMean = [180 50 0]';

% max number of landmark observations per timestep
Param.maxObs = 2;

% number of landmarks per sideline of field (minimum is 3)
Param.nLandmarksPerSide = 4;

% Motion noise (in odometry space, see p.134 in book).
Param.alphas = [0.05 0.001 0.05 0.01].^2; % std of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
Param.beta = [10, deg2rad(10)]; % [cm, rad]
Param.R = diag(Param.beta.^2);

% Step size between filter updates, can be less than 1.
Param.deltaT=0.1; % [s]

if isscalar(stepsOrData)
    % Generate a data set of motion and sensor info consistent with
    % noise models.
    numSteps = stepsOrData;
    Data = generateScript(Param.initialStateMean, numSteps, Param.maxObs, Param.alphas, Param.beta, Param.deltaT);
else
    % use a user supplied data set from a previous run
    Data = stepsOrData;
    numSteps = size(Data, 1);
    global FIELDINFO;
    FIELDINFO = getfieldinfo;
end
%===================================================

% Initialize State
%===================================================
State.Ekf.mu = Param.initialStateMean;
State.Ekf.Sigma = zeros(3);
State.Ekf.Observed_landmarks = [];
State.Ekf.data_asso = [];
N=0;
Param.updateMethod = 'seq'; %%%%MAKE THIS 'batch' for batch update

Param.vp=0;
Param.sim=1;

for t = 1:numSteps
    %plotsim(t);

    %=================================================
    % data available to your filter at this time step
    %=================================================
    u = getControl(t);
    z = getObservations(t);
    
    M = [(Param.alphas(1)*(u(1))^2 + Param.alphas(2)*(u(2))^2), 0, 0; ...
        0, (Param.alphas(3)*(u(2))^2+ Param.alphas(4)*(u(1))^2 +Param.alphas(4)*(u(3))^2), 0; ...
        0, 0, (Param.alphas(1)*(u(3))^2+Param.alphas(2)*(u(2))^2)]; %%%INITIALIZE  THIS NOISE MATRIX
    
    ekfpredict_sim(u,M);%%%PREDICT
    ekfupdate(z);%%%UPDATE
    our_data(t,:)= State.Ekf.mu(1:3);
    N = (size(State.Ekf.predMu,1)-3)/2;
    plotsim(t,our_data,z,State,N);
    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
        
    %=================================================
    %TODO: plot and evaluate filter results here
    %=================================================


    drawnow;
    if pauseLen > 0
        pause(pauseLen);
    end
    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
    
    N = size(State.Ekf.predMu,1);
    
    for i=1:(N-1)/2
       x1 = 2*i;
       if i == 1
           x1 = 1;
       end
       det_mat=State.Ekf.Sigma(x1:x1+1,x1:x1+1);
       det_v(i,t) = ((det(det_mat))^(1/4)); %%%STORE VALUES TO PLOT
    end
end

    figure;
    plotcorr(8);
    figure;
    N = size(State.Ekf.predMu,1);
    for i=2:(N-1)/2
        hold on;
    plot(1:t,det_v(i,:),'-');
    end
    legend('Landmark 3','Landmark 4','Landmark 5','Landmark 6','Landmark 7','Landmark 8','Landmark 1','Landmark 2');
    size(det_v)
    figure;
    plot(1:8,det_v(2:9,end),'*');
    

  if (strcmp(Param.dataAssociation, 'nn'))
    figure;
    hold on;
    observation = size(State.Ekf.data_asso(:,1),1);
    plot(1:observation, State.Ekf.data_asso(:,1),'*');
    plot(1:observation, State.Ekf.data_asso(:,2),'o');
    hold off;
end

if nargout >= 1
    varargout{1} = Data;
end

%==========================================================================
function u = getControl(t)
global Data;
% noisefree control command
u = Data.noisefreeControl(:,t);  % 3x1 [drot1; dtrans; drot2]


%==========================================================================
function z = getObservations(t)
global Data;
% noisy observations
z = Data.realObservation(:,:,t); % 3xn [range; bearing; landmark id]
ii = find(~isnan(z(1,:)));
z = z(:,ii);

%==========================================================================
function plotsim(t,our_data,z,State,N)
global Data;

%--------------------------------------------------------------
% Graphics
%--------------------------------------------------------------

NOISEFREE_PATH_COL = 'green';
ACTUAL_PATH_COL = 'blue';

NOISEFREE_BEARING_COLOR = 'cyan';
OBSERVED_BEARING_COLOR = 'red';

GLOBAL_FIGURE = 1;

%=================================================
% data *not* available to your filter, i.e., known
% only by the simulator, useful for making error plots
%=================================================
% actual position (i.e., ground truth)
x = Data.Sim.realRobot(1,t);
y = Data.Sim.realRobot(2,t);
theta = Data.Sim.realRobot(3,t);

% real observation
observation = Data.realObservation(:,:,t);

% noisefree observation
noisefreeObservation = Data.Sim.noisefreeObservation(:,:,t);

%=================================================
% graphics
%=================================================
figure(GLOBAL_FIGURE); clf; hold on; plotfield(observation(3,:));

% draw actual path (i.e., ground truth)
plot(Data.Sim.realRobot(1,1:t), Data.Sim.realRobot(2,1:t), 'Color', ACTUAL_PATH_COL);
plotrobot( x, y, theta, 'black', 1, ACTUAL_PATH_COL);

% draw noise free motion command path
plot(Data.Sim.noisefreeRobot(1,1:t), Data.Sim.noisefreeRobot(2,1:t), 'Color', NOISEFREE_PATH_COL);
plot(Data.Sim.noisefreeRobot(1,t), Data.Sim.noisefreeRobot(2,t), '*', 'Color', NOISEFREE_PATH_COL);

% plot([180 our_data(1,1)], [50 our_data(1,2)], 'Color', 'black');
% plot(our_data(1:t,1), our_data(1:t,2), 'Color', 'black');
% plot(our_data(t,1), our_data(t,2), '*', 'Color', 'black');

for i=1:N
    plotcov2d(State.Ekf.predMu(3+2*(i) - 1) ,State.Ekf.predMu(3+2*(i)) ,State.Ekf.predSigma((3+2*(i) - 1):(3+2*(i)),(3+2*(i) - 1):(3+2*(i))) ,'r', 1, 'y', 0, 3);
end
plotcov2d(State.Ekf.predMu(1) ,State.Ekf.predMu(2) ,State.Ekf.predSigma,'b', 1, 'y', 0, 3);
for k=1:size(observation,2)
    rng = Data.Sim.noisefreeObservation(1,k,t);
    ang = Data.Sim.noisefreeObservation(2,k,t);
    noisy_rng = observation(1,k);
    noisy_ang = observation(2,k);

    % indicate observed range and angle relative to actual position
    plot([x x+cos(theta+noisy_ang)*noisy_rng], [y y+sin(theta+noisy_ang)*noisy_rng], 'Color', OBSERVED_BEARING_COLOR);

    % indicate ideal noise-free range and angle relative to actual position
    plot([x x+cos(theta+ang)*rng], [y y+sin(theta+ang)*rng], 'Color', NOISEFREE_BEARING_COLOR);
end

function plotcorr( N)%%%%%FUNCTION TO PLOT
global State
vec = [1];
for i = 1:N
    vec = [vec;2+2*i];
    
end
for j = 1:length(vec)
    hold on
    corr_coeff_vec = [];
    for i = 1:length(vec)
        sigma1 = State.Ekf.Sigma(vec(j),vec(j));
        sigma2 = State.Ekf.Sigma(vec(i),vec(i));
        corr_coeff=State.Ekf.Sigma(vec(j),vec(i))/(sqrt(sigma1)*sqrt(sigma2));%%CALCULATING THE CORRELATION CO_EFFICIENT
        corr_coeff_vec = [corr_coeff_vec corr_coeff];   %%AUGMENTING THE CORRELATION CO_EFFICIENT
    end    
        plot((1:9),corr_coeff_vec,'-');
      
        ylim([-1,1]);
end
title('Graph for correlation');
xlabel('landmarks');
ylabel('correlation coefficients');        
legend('robot','3','4','5','6','7','8','1','2');

