function plotdet(N)
global State
vec = [];
for i = 1:N
    vec = [vec;2+2*i];
end
for j = 1:length(vec)
    hold on
    cov=State.Ekf.Sigma(vec(j):vec(j)+1,vec(j):vec(j)+1);
    plot(j,(det(cov))^(1/4),'*');
    ylim([0,10]);
end
hold off
end
