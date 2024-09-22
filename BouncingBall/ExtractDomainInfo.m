function domains = ExtractDomainInfo(x0, T, cem_controller, BallParams, GuardParams)
%EXTRACTDOMAININFO Summary of this function goes here
%   

domains.num = 0;

tsim = 0;
sol = [];
dom_idx = 1;
while tsim < T
    sol = [sol, BallSimulation(tsim, T, x0, cem_controller, BallParams, GuardParams)];

    domains.num = domains.num + 1;

    nodes_per_sec = 60;

    domains.T(dom_idx) = sol(end).x(end) - sol(end).x(1);
    domains.nodes(dom_idx) = ceil(nodes_per_sec*domains.T(dom_idx));


    domains.type(dom_idx) = 1;

    tsim = sol(end).x(end);

    if ~isempty(sol(end).ie)
        domains.guard_idx(dom_idx) = sol(end).ie(1);
        if sol(end).ie(1) == 1 || sol(end).ie(1) == 2
            domains.type(dom_idx) = 1;

            % x or y and the bounds
            domains.variable(dom_idx) = GuardParams.Direction{sol(end).ie(1)}(2);
            col_idx = 1;
            if domains.variable(dom_idx) == 1
                col_idx = 2;
            end
            domains.guard_constraint{dom_idx}(1) = GuardParams.G{sol(end).ie(1)}(1, col_idx);
            domains.guard_constraint{dom_idx}(2) = GuardParams.G{sol(end).ie(1)}(2, col_idx);
            domains.guard_val(dom_idx) = GuardParams.Value(sol(end).ie(1));

            x0 = WallReset(sol(end).ie(1), sol(end).y(:, end), BallParams, GuardParams);
        else
            domains.variable(dom_idx) = GuardParams.Direction{sol(end).ie(1)}(2);
            col_idx = 1;
            if domains.variable(dom_idx) == 1
                col_idx = 2;
            end
            domains.type(dom_idx) = 1;

            % Exit if we hit this domain
            domains.T(dom_idx + 1) = T - sol(end).x(end);
            domains.type(dom_idx + 1) = 2;
            domains.num = domains.num + 1;
            domains.nodes(dom_idx + 1) = ceil(nodes_per_sec*domains.T(dom_idx + 1));
            tsim = T;
        end
    else
        domains.guard_idx(dom_idx) = 0; 
    end

    dom_idx = dom_idx + 1;

    x0 = ResetMap(sol(end), BallParams, GuardParams);
end


end

