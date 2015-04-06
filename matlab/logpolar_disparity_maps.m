function [cortical_disparity_maps] = logpolar_disparity_maps(RMIN,RMAX,NSECTORS,NRINGS,horizontal_disparities,horizontal_total_disparities)
%HUMPS  A function used by QUADDEMO, ZERODEMO and FPLOTDEMO.
%   Y = HUMPS(X) is a function with strong maxima near x = .3
%   and x = .9.
%
%   [X,Y] = HUMPS(X) also returns X.  With no input arguments,
%   HUMPS uses X = 0:.05:1.
%
%   Example:
%      plot(humps)
%
%   See QUADDEMO, ZERODEMO and FPLOTDEMO.

%   Copyright 1984-2014 The MathWorks, Inc.
t = logtform(RMIN, RMAX, NRINGS, NSECTORS);
cortical_disparity_maps=zeros(NSECTORS,NRINGS,2,horizontal_total_disparities);
for s=1:NSECTORS
    for r=1:NRINGS
        [x, y] = tforminv(t, r, s); % I^-1
        for disparity=1:horizontal_total_disparities
            dx=horizontal_disparities(disparity);            
            [p, theta] = tformfwd(t, x+dx, y); % I^-1
            if p<1 || p>NRINGS
                %cortical_disparity_maps(s,r,1,disparity)=Inf;
                %cortical_disparity_maps(s,r,2,disparity)=Inf;
                continue
            end
            if theta<1 || theta>NSECTORS
                %cortical_disparity_maps(s,r,1,disparity)=Inf;
                %cortical_disparity_maps(s,r,2,disparity)=Inf;
                continue
            end
            dtheta=theta-s;
            dp=p-r;
            cortical_disparity_maps(s,r,1,disparity)=dtheta;
            cortical_disparity_maps(s,r,2,disparity)=dp;
        end
    end
end

