% initall;
function RR=msolver(xG,sumr)

% syms erfa beta

% ep1=sumr(1)*cos(erfa)+sumr(3)*sin(beta);
% ep2=sumr(1)*sin(erfa)*sin(beta)+sumr(2)*cos(erfa)-sumr(1)*sin(erfa);
% ep3 = -sumr(1)*cos(erfa)*sin(beta)+sumr(2)*sin(erfa)+sumr(3)*cos(erfa)*cos(beta);

% ep1=sumr(1)*sin(erfa)*sin(beta)+sumr(2)*cos(erfa)-sumr(3)*sin(erfa)*cos(beta);
% ep2=sumr(1)*cos(beta)+sumr(3)*sin(beta);

% ep1=sumr(2)*cos(erfa)*sumr(3)*sin(erfa);
% ep2=sumr(1)*cos(beta)+sumr(2)*sin(erfa)*sin(beta)-sumr(3)*cos(erfa)*sin(beta);


op=fsolve(@(ang)ffau(ang,sumr),[0 0])


erfa=op(1);
beta=op(2);

% ep1=sumr(2)*cos(erfa)*sumr(3)*sin(erfa);
% ep2=sumr(1)*cos(beta)+sumr(2)*sin(erfa)*sin(beta)-sumr(3)*cos(erfa)*sin(beta);

ep1=sumr(1)*sin(erfa)*sin(beta)+sumr(2)*cos(erfa)-sumr(3)*sin(erfa)*cos(beta);
ep2=sumr(1)*cos(beta)+sumr(3)*sin(beta);

Rx = [1 0          0,
      0 cos(erfa) -sin(erfa),
      0 sin(erfa)  cos(erfa)];
  
Ry = [ cos(beta) 0 sin(beta),
       0         1 0,
      -sin(beta) 0 cos(beta)];

RR=(Rx*Ry)';
RR'*sumr'

test=cross([0 0 3]',RR'*sumr');




