function [xmean, xvar, yopt] = CrossEntropyOptimization(cem_settings, costfcn)
%CROSSENTROPYOPTIMIZATION Uses CEM to optimize
%  
%tic
nsamples = cem_settings.nsamples;
xvar = cem_settings.var;
xmean = cem_settings.mean;
nelite = cem_settings.nelite;
xsize = cem_settings.xsize;

rng('default')

%sc = parallel.pool.Constant(RandStream('Threefry'));

for n = 1:cem_settings.max_iters
    %n
    ysample = zeros(nsamples + 1, 1);
    xsample = zeros(xsize, nsamples + 1);
    for j = 1:nsamples
        % Sample
        xsample(:,j) = mvnrnd(xmean', xvar, 1)'; %randn(xsize,1).*sqrt(xvar) + xmean;
        
        ysample(j) = costfcn(xsample(:,j));
    end
    
    % Do the previous mean too
    xsample(:, end) = xmean';
    ysample(end) = costfcn(xsample(:, end));

    [ysorted, I] = sort(ysample);
    xsorted = xsample(:, I);
    %ybest = ysorted(1)
    %ycheck = costfcn(xsorted(:,1))

    % Update mean
    xmean = zeros(xsize, 1);
    for i = 1:nelite
        xmean = xmean + xsorted(:, i);
    end
    xmean = xmean/nelite;
    
    % Update variance
    % TODO: Check this - should it be diagonal?
    % xvar = zeros(xsize);
    % for i = 1:nelite
    %    xvar = xvar + (xsorted(:, i) - xmean)*(xsorted(:, i) - xmean)';
    % end
    % xvar = xvar/nelite;

    % Plot the samples
    %hold on;
    %scatter(xsorted(1, 1:nelite), xsorted(2, 1:nelite));
    %hold off;

    %xmean
    %xvar
    mean(ysorted(1:nelite))
end

yopt = costfcn(xmean);
%toc

end

