function out = pd_control(var,var_c,vard,vard_c,K_p,K_d)

out = - K_p * (var - var_c) - K_d * (vard - vard_c);
