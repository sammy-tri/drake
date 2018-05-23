d=importdata('bazel-bin/examples/contact_model/mbp/rigid_mug_gripper_mbp.runfiles/drake/nr_iteration.dat');
[N_niters,X_niters] = hist(d(:,3),0:5:60);

figure(1)
plot(X_niters,N_niters/sum(N_niters),'k-','LineWidth',2)
xlabel('Number of iterations [-]', 'FontName', 'Times', 'FontSize', 16)
ylabel('Frequency [-]', 'FontName', 'Times', 'FontSize', 16)
set(gca, 'FontName', 'Times', 'FontSize', 16)
%axis([0 1000 0 0.1])

figure(2)
[N_resid,X_resid] = hist(d(:,5),logspace(-10, -4, 40));
semilogx(X_resid,N_resid/sum(N_resid),'k-','LineWidth',2)
xlabel('Residual [N]', 'FontName', 'Times', 'FontSize', 16)
ylabel('Frequency [-]', 'FontName', 'Times', 'FontSize', 16)
set(gca, 'FontName', 'Times', 'FontSize', 16)


