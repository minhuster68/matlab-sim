function [A,B,C,D] = error_model(robot,q,dq,ddq,Ts,delta)
%ERROR_MODEL Reference-linearized error model, e=q_actual-q_ref.
% ZOH for the six mechanical states. Forward Euler for z=integral(e),
% matching the discrete integrator that is actually used in Simulink.
q=q(:); dq=dq(:); ddq=ddq(:);
M=massMatrix(robot,q);
assert(all(isfinite(M(:))) && rcond(M)>1e-12, ...
    'mpctune:MassMatrix','Mass matrix is nonfinite or nearly singular.');
Kq=zeros(3); Kv=zeros(3);
for j=1:3
    h=zeros(3,1); h(j)=delta;
    Kq(:,j)=(inverseDynamics(robot,q+h,dq,ddq) ...
            -inverseDynamics(robot,q-h,dq,ddq))/(2*delta);
    Kv(:,j)=(inverseDynamics(robot,q,dq+h,ddq) ...
            -inverseDynamics(robot,q,dq-h,ddq))/(2*delta);
end
Ac=[zeros(3),eye(3); -(M\Kq),-(M\Kv)];
Bc=[zeros(3);M\eye(3)];
E=expm([Ac Bc;zeros(3,9)]*Ts);
A=[eye(3),Ts*eye(3),zeros(3); zeros(6,3),E(1:6,1:6)];
B=[zeros(3);E(1:6,7:9)];
C=eye(9); D=zeros(9,3);
end
