function [param_vect, P] = RLS_With_Forgetting(param_vect_min_1, P_min_1, info_vect, observation, lambda)
innov = observation - info_vect' * param_vect_min_1;


L = (P_min_1*info_vect)/(1+ info_vect'*P_min_1*info_vect);
P = 1/lambda * (eye(size(P_min_1))-L*info_vect)*P_min_1;
param_vect = param_vect_min_1 + P * info_vect * innov;
end