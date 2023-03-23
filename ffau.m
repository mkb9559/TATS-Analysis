function f= ffau(ang,sumr)

f(1)=sumr(1)*sin(ang(1))*sin(ang(2))+sumr(2)*cos(ang(1))-sumr(3)*sin(ang(1))*cos(ang(2));
f(2)=sumr(1)*cos(ang(2))+sumr(3)*sin(ang(2));
% f(1)=sumr(2)*cos(ang(1))*sumr(3)*sin(ang(1));
% f(2)=sumr(1)*cos(ang(2))+sumr(2)*sin(ang(1))*sin(ang(2))-sumr(3)*cos(ang(1))*sin(ang(2));

