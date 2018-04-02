%generateScript: simulates the trajectory of the robot using square
%                path given by generate motion
%
%data=generateScript(initialstatemean,numSteps,alphas,betas)
%     generates data of the form 
%     [realObservation', noisefreeMotion', noisefreeObservation',
%     realRobot', noisefreeRobot']
%
%realObservation and noisefreeMotion is the only data available to
%filter.  All other data for debugging/display purposes.
%
%alphas are the 4-d noise for robot motion
%beta: noise for observations
%
%right now, observation ids not based on relationship between robot and marker
%
% TODO: update generateScript so that observations are for the
% closest marker

function Data = generateScript(initialStateMean, numSteps, maxObs, alphas, beta, deltaT)

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------

motionDim = 3;
observationDim = 3; % observation size (range, bearing, marker ID)

realRobot = initialStateMean;
noisefreeRobot = initialStateMean;

global Param;

% soccer field
nSideMarks = 3;
global FIELDINFO;
FIELDINFO = getfieldinfo(Param.nLandmarksPerSide);

Data.time = zeros(1, numSteps);
Data.noisefreeControl = zeros(motionDim, numSteps);
Data.realObservation = nan(observationDim, maxObs, numSteps);
Data.Sim.realRobot = zeros(motionDim, numSteps);
Data.Sim.noisefreeRobot = zeros(motionDim, numSteps);
Data.Sim.noisefreeObservation = nan(observationDim, maxObs, numSteps);

for n = 1:numSteps
 % --------------------------------------------
  % Simulate motion
  % --------------------------------------------

  t=n*deltaT;
  noisefreeMotion = generateMotion(t,deltaT);
  
  % Shift real robot
  prevNoisefreeRobot = noisefreeRobot;
  noisefreeRobot = sampleOdometry( noisefreeMotion, noisefreeRobot, [0 0 0 0]);

  
  % Move robot
  realRobot = sampleOdometry( noisefreeMotion, realRobot, alphas);
  
  %--------------------------------------------------------------
  % Simulate observation
  %--------------------------------------------------------------

  % Observation noise
  Q = diag([beta.^2, 0]);

  % select landmarks for sensing
  noisefreeObservation = senseLandmarks(realRobot, maxObs, FIELDINFO);
  numObs = size(noisefreeObservation,2);
  realObservation = nan(size(noisefreeObservation));
  for k=1:numObs
      observationNoise = sample( zeros(observationDim,1), Q, observationDim);
      realObservation(:,k) = noisefreeObservation(:,k) + observationNoise;
  end
  
  Data.time(n) = t;
  Data.noisefreeControl(:,n) = noisefreeMotion;
  Data.realObservation(:,1:numObs,n) = realObservation;
  Data.Sim.realRobot(:,n) = realRobot;
  Data.Sim.noisefreeRobot(:,n) = noisefreeRobot;
  Data.Sim.noisefreeObservation(:,1:numObs,n) = noisefreeObservation;
end

%==========================================================================
function noisefreeObservation = senseLandmarks(realRobot, maxObs, FIELDINFO, fov)

M = FIELDINFO.NUM_MARKERS;
noisefreeObservation = zeros(3, M);
for m=1:M
    noisefreeObservation(:,m) = observation( realRobot, FIELDINFO, m);
end

% orders landmarks with mininum bearing angle w.r.t robot
[dummy, ii] = sort(abs(noisefreeObservation(2,:)));
noisefreeObservation = noisefreeObservation(:,ii);

% keeps those within front plane
ii = find(-pi/2 < noisefreeObservation(2,:) & noisefreeObservation(2,:) < pi/2);
if length(ii) <= maxObs
    noisefreeObservation = noisefreeObservation(:,ii);
else
    noisefreeObservation = noisefreeObservation(:,1:maxObs);
end
