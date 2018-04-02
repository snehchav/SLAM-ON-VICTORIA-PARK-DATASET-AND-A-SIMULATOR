function runvp(nSteps,pauseLen, makeVideo)

global Param;
global State;
global Data;

Param.vp=1;
Param.sim=0;

if ~exist('nSteps','var') || isempty(nSteps)
    nSteps = inf;
end

if ~exist('pauseLen','var')
    pauseLen = 0; % seconds
end

Data = load_vp_si();

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
% vehicle geometry
Param.a = 3.78; % [m]
Param.b = 0.50; % [m]
Param.L = 2.83; % [m]
Param.H = 0.76; % [m]

% 2x2 process noise on control input
sigma.vc = 0.02; % [m/s]
sigma.alpha = 2*pi/180; % [rad]
Param.Qu = diag([sigma.vc, sigma.alpha].^2);

% 3x3 process noise on model error
sigma.x = 0.1; % [m]
sigma.y = 0.1; % [m]
sigma.phi = 0.5*pi/180; % [rad]
Param.Qf = diag([sigma.x, sigma.y, sigma.phi].^2);

% 2x2 observation noise
sigma.r = 0.05; % [m]
sigma.beta = 1*pi/180; % [rad]
Param.R = diag([sigma.r, sigma.beta].^2);
%===================================================

% Initialize State
%===================================================
State.Ekf.mu = [Data.Gps.x(2), Data.Gps.y(2), 36*pi/180]';
State.Ekf.Sigma = zeros(3);
State.Ekf.aug_mu = State.Ekf.mu;
State.Ekf.Observed_landmarks = [];
State.Ekf.data_assoc = [];
State.Ekf.predMu = State.Ekf.mu;
State.Ekf.predSigma = State.Ekf.Sigma;
Param.updateMethod='seq'
State.Ekf.run_time_pred = zeros(nSteps);
State.Ekf.run_time_update = zeros(nSteps);
State.Ekf.run_time_landmarks=zeros(nSteps);
global AAr;
AAr = [0:360]*pi/360;


figure(1); clf;
axis equal;

ci = 1; % control index
t = min(Data.Laser.time(1), Data.Control.time(1));
for k=1:min(nSteps, length(Data.Laser.time))

    tic
    while (Data.Control.time(ci) < Data.Laser.time(k)) %PREDICT UNTIL YOU GET AN OBSERVATION
       % control available
       dt = Data.Control.time(ci) - t;
       t = Data.Control.time(ci);
       u = [Data.Control.ve(ci), Data.Control.alpha(ci)]';
        
       
       ekfpredict_vp(u, dt);    %%PREDICT
       
       ci = ci+1;
    end
    State.Ekf.run_time_pred(k) = toc;
    % observation available
    dt = Data.Laser.time(k) - t;
    t = Data.Laser.time(k);
    z = detectTreesI16(Data.Laser.ranges(k,:));
    
    z_2 = z;
    z_2(2,:) = z_2(2,:) - pi/2;
    
    tic
    ekfupdate(z_2); %%MAKE AN UPDATE
    State.Ekf.run_time_update(k) = toc;
    State.Ekf.run_time_landmarks(k) = length(State.Ekf.Observed_landmarks);
    State.Ekf.aug_mu = [State.Ekf.aug_mu, State.Ekf.mu(1:3)];
    
    doGraphics(z);
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
end
figure;
title('CPU TIME FOR PREDICTION AND UPDATE ');
plot(1:nSteps, State.Ekf.run_time_pred);
hold on;
plot(1:nSteps, State.Ekf.run_time_update);
hold off;
legend('Prediction','Update');
xlabel('nSteps');
ylabel('CPU time');
figure;
title('Number of landmarks with time');
plot(1:nSteps, State.Ekf.run_time_landmarks);
xlabel('Iteration');
ylabel('Number of Landmarks');

%==========================================================================
function doGraphics(z)
% Put whatever graphics you want here for visualization
%
% WARNING: this slows down your process time, so use sparingly when trying
% to crunch the whole data set!

global Param;
global State;
plot(State.Ekf.aug_mu(1,:), State.Ekf.aug_mu(2,:), '--b');
hold on
plotbot(State.Ekf.mu(1), State.Ekf.mu(2), State.Ekf.mu(3), 'black', 1, 'blue', 1);
hold on;

robot_graph =plotcov2d( State.Ekf.mu(1), State.Ekf.mu(2), State.Ekf.Sigma, 'blue', 0, 'blue', 0, 3);

BB=100;
axis([[-BB,BB]+State.Ekf.mu(1), [-BB,BB]+State.Ekf.mu(2)]);
textLabels = false; 
N=length(State.Ekf.Observed_landmarks);
if N > 0
    for i = 1:N
        plotcov2d(State.Ekf.mu(3+2*i-1,1), State.Ekf.mu(3+2*i,1), State.Ekf.Sigma(3+2*i-1:3+2*i,3+2*i-1:3+2*i), 'g', 0, 'y', 0.5, 3);
    end
end
title('Victoria Park')
xr = State.Ekf.mu(1);
yr = State.Ekf.mu(2);
tr = State.Ekf.mu(3);
for k=1:size(z,2)
    r = z(1,k);
    b = z(2,k);
    xl = xr + r*cos(b+tr-pi/2);
    yl = yr + r*sin(b+tr-pi/2);
    plot([xr; xl], [yr; yl],'r',xl,yl,'r*');
end

hold off;

