%-------------------------------------------------------
% noisy version of prediction
% predicts the new state given the current state and motion
% motion in form of [drot1,dtrans,drot2]
%-------------------------------------------------------
function state = sampleOdometry(motion, state, alphas)

drot1 = motion(1);
dtran = motion(2);
drot2 = motion(3);

noisymotion(1)=sample(motion(1),alphas(1)*drot1^2+alphas(2)*dtran^2,1);
noisymotion(2)=sample(motion(2),alphas(3)*dtran^2+alphas(4)*(drot1^2+drot2^2),1);
noisymotion(3)=sample(motion(3),alphas(1)*drot2^2+alphas(2)*dtran^2,1);

state = prediction(state, noisymotion);
