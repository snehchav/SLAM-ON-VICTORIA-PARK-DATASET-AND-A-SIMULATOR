% Assumes equally weighted particles.
function [mu, Sigma] = meanAndVariance( samples, numSamples)

mu = mean(samples, 2);

% orientation is a bit more tricky.
sinSum = 0;
cosSum = 0;
for s=1:numSamples,
  cosSum = cosSum + cos( samples( 3,s));
  sinSum = sinSum + sin( samples( 3,s));
end
mu( 3) = atan2( sinSum, cosSum);

% Compute covariance.
zeroMean = samples - repmat(mu,1,numSamples);
for s=1:numSamples,
  zeroMean( 3,s) = minimizedAngle( zeroMean( 3,s));
end

Sigma = zeroMean*zeroMean' / numSamples;
