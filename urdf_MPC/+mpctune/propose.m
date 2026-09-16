function theta=propose(ctx,i,budget,best,tried)
%PROPOSE Log-scale weight search and integer horizons; fixed physical limits.
lo=log10(ctx.cfg.weightMin); hi=log10(ctx.cfg.weightMax);
for attempt=1:100
    if i<=4 && attempt==1
        theta=ctx.baseline;
        % Early meaningful probes before broad random exploration.
        if i==2, theta(2)=theta(2)+log10(2);
        elseif i==3, theta(3)=theta(3)+log10(3);
        else, theta(1)=theta(1)+log10(3); theta(4)=theta(4)-log10(2); end
    elseif i<=ceil(0.6*budget) || attempt>20
        theta=[lo+rand(1,5).*(hi-lo), pick(ctx.cfg.predictionChoices),0];
        theta(7)=pick(ctx.cfg.controlChoices(ctx.cfg.controlChoices<=theta(6)));
    else
        theta=best; theta(1:5)=best(1:5)+0.30*randn(1,5);
        if rand<0.5
            [~,idx]=min(abs(ctx.cfg.predictionChoices-best(6)));
            idx=max(1,min(numel(ctx.cfg.predictionChoices),idx+randi(3)-2));
            theta(6)=ctx.cfg.predictionChoices(idx);
        end
        if rand<0.5
            theta(7)=pick(ctx.cfg.controlChoices(ctx.cfg.controlChoices<=theta(6)));
        end
    end
    theta(1:5)=max(lo,min(hi,theta(1:5)));
    theta(7)=min(theta(6),theta(7));
    if isempty(tried) || ~any(max(abs(tried-theta),[],2)<1e-9), return; end
end
error('mpctune:Search','Could not generate a distinct candidate.');
end

function v=pick(values)
assert(~isempty(values),'No admissible horizon choices.');
v=values(randi(numel(values)));
end
