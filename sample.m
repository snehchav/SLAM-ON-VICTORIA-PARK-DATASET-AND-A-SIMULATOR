%-------------------------------------------------------
% samples from a multivariate gaussian with given mean and covariance
%-------------------------------------------------------

function s = sample(mu, Sigma, dimensions)

[V,D] = eig(Sigma);

s=mu+V*sqrt(D)*randn(dimensions,1);
