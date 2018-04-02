function id_in_state = da_known(z)
% EKF-SLAM data association with known correspondences

global Param;
global State;

% z
% find(State.Ekf.Observed_landmarks == z)
% State.Ekf.Observed_landmarks
id_in_state = 3+2*(find(State.Ekf.Observed_landmarks == z));
