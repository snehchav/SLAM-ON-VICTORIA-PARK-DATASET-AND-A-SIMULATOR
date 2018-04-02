function id_in_state = da_nn(z, R, N)
% perform nearest-neighbor data association

global Param;
global State;

N = (size(State.Ekf.predMu,1)-3)/2;
A = (size(State.Ekf.predMu,1));
pi_k = zeros(N,1);
for k=1:N  %%%FOR ALL THE OBSERVED LANDMARKS TILL NOW
    del_x = State.Ekf.predMu(3+2*k - 1) - State.Ekf.predMu(1);
    del_y = State.Ekf.predMu(3+2*k) - State.Ekf.predMu(2);
    q=(del_x)^2+(del_y)^2;
    zhat = [sqrt(q); minimizedAngle(atan2(del_y, del_x) - State.Ekf.predMu(3))];    %%PREDICT THE MEASUREMENT FOR THE kth LANDMARK
    F_x_j=zeros(5,size(State.Ekf.predMu,1));
            F_x_j(1,1)= 1;
            F_x_j(2,2) = 1;
            F_x_j(3,3) = 1;
            F_x_j(4, 3+2*k -1) = 1;
            F_x_j(5,3+2*k) = 1;
            H=[-sqrt(q)*del_x,-sqrt(q)*del_y, 0, sqrt(q)*del_x,sqrt(q)*del_y ;...
                del_y, -del_x, -q, -del_y, del_x];
            H=(1/q)*H*F_x_j;
            Psi_k=H*State.Ekf.predSigma*H' + R;
            pi_k(k) = ((z-zhat)')*inv(Psi_k)*(z-zhat);      %%%CALCULATE MAHALANOBIS DISTANCE FOR EVERY LANDMARK
    end

if Param.sim==1
    pi_k(N)=23; %%%%VALUE FOR ALPHA FOR SIM
else
    pi_k(N)=300;    %%%VALUE OF ALPHA FOR NN
end
[~,index] = min(pi_k);

if index < N  %%UPDATE MEAN, SIGMA and LANDMARKS if a NEW LANDMARK HAS BEEN OBSERVED
   State.Ekf.predMu = State.Ekf.predMu(1:size(State.Ekf.predMu,1)-2);
   State.Ekf.predSigma = State.Ekf.predSigma(1:A-2,1:A-2);
   State.Ekf.Observed_landmarks = State.Ekf.Observed_landmarks(1:N-1);
end

id_in_state = 3 + 2*index;

