%-------------------------------------------------------
% predicts the new state given the current state and motion
% motion in form of [drot1,dtrans,drot2]
%-------------------------------------------------------
function state = prediction( state, motion)

state(3)=state(3)+motion(1);
state(1)=state(1)+motion(2)*cos(state(3));
state(2)=state(2)+motion(2)*sin(state(3));
state(3)=state(3)+motion(3);
state(3)=minimizedAngle(state(3));
