function ekfpredict_sim (u,M)
% EKF-SLAM prediction for simulator process model

global Param;
global State;

F=[eye(3),zeros(3,size(State.Ekf.mu,1)-3)]; %%%INITIALIZE F

State.Ekf.predMu = State.Ekf.mu;    

State.Ekf.predMu(1:3) = prediction(State.Ekf.mu(1:3),u);        %%APLLY CONTROL ACTION TO THE ROBOT
State.Ekf.predMu;

theta = State.Ekf.mu(3);
ang = minimizedAngle(theta + u(1));
G = eye(size(State.Ekf.mu,1)) + F'*[0 0 -u(2)*sin(ang); 0 0 u(2)*cos(ang); 0 0 0]*F; 
State.Ekf.predSigma = G*State.Ekf.Sigma*G'+ F'*M*F;     %%UPDATE THE CO_VARIANCE FOR THE ROBOT"S POSE
