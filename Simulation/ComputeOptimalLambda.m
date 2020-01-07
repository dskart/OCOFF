function opt_lam = ComputeOptimalLambda(alpha, beta, p0, delta0, info_state_size, N, t, var_proc_noise, L)
K1 = alpha^(-2)*p0^(-2)*delta0;
K2 = info_state_size*var_proc_noise;
K3 = sum(L^2)*(N*beta/alpha)^2;
fun = @(lam)(K1*lam.^(2.*t).*(1-lam)+K2.*(1-lam).^2.*(lam.^(N-1))+K3)./(lam.^(2.*(N-1)).*(1-lam));
opt_lam = fminbnd(fun,0,1);
if false
    figure;
    pos_lams = linspace(0, 0.999, 100);
    bound = fun(pos_lams);
    plot(pos_lams, bound)
    title("Bound on expected deviation")
    xlabel("Forgetting Factor")
    ylabel("Bound")
    close
end
end
