function N =ekfupdate_batch(z,N)

global Param;
global State;
            r = [Param.R(1,1); Param.R(2,2)];
            H_mat = [];
            zhat_mat = [];
            Q = [];
            z_mat = [];
            for i=1:size(z,2)
                switch lower(Param.dataAssociation)
                    case 'known'
                        j = z(3,i);
                        if (length(find(State.Ekf.Observed_landmarks == j))~=1)
                            initialize_new_landmark(z(:,i),Param.R);
                        end
                        id_in_state = da_known(z(3,i));
                        del_x = State.Ekf.predMu(3+2*id_in_state - 1) - State.Ekf.predMu(1);
                        del_y = State.Ekf.predMu(3+2*id_in_state) - State.Ekf.predMu(2);
                        q = (del_x)^2 + (del_y)^2;
                        zhat = [sqrt(q); minimizedAngle(atan2(del_y, del_x) - State.Ekf.predMu(3))];
                        zhat_mat = [zhat_mat; zhat];
                        n = size(State.Ekf.predMu,1); 
                        F_x_j=zeros(5,n);
                        F_x_j(1,1)= 1;
                        F_x_j(2,2) = 1;
                        F_x_j(3,3) = 1;
                        F_x_j(4, 3+2*id_in_state -1) = 1;
                        F_x_j(5,3+2*id_in_state) = 1;
                        H_final=[-sqrt(q)*del_x,-sqrt(q)*del_y, 0, sqrt(q)*del_x,sqrt(q)*del_y ;...
                                 del_y, -del_x, -q, -del_y, del_x];
                        H_final=(1/q)*H_final*F_x_j;
                        if size(H_mat,2)~=size(H_final,2) && ~(isempty(H_mat))
                            n = size(H_final,2) - size(H_mat,2);
                            H_mat = [H_mat zeros(size(H_mat,1),n)];
                        end
                        H_mat = [H_mat ; H_final];
                        Q =[Q; r];
                        z_mat = [z_mat; z(1:2,i)];
                    case 'nn'

                        [K,zhat,H_final,N] = da_nn(z(1:2,i), Param.R,N);

                    case 'jcbb'
                        id_in_state = da_jcbb(z(1:2,:), Param.R);
                    otherwise
                        error('unrecognized data association method: "%s"', Param.dataAssociation);

                end

            end
            Q = diag(Q);
            K = State.Ekf.predSigma*H_mat'*inv(H_mat*State.Ekf.predSigma*H_mat'+ Q);

            State.Ekf.predMu = State.Ekf.predMu + K*(z_mat-zhat_mat);
            State.Ekf.predSigma= (eye(size(K*H_mat))-K*H_mat)*State.Ekf.predSigma;

State.Ekf.mu = State.Ekf.predMu;
State.Ekf.Sigma = State.Ekf.predSigma;


