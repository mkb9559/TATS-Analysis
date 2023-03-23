clc;
clear all;

n=3;
m=ones(1,n)*2;
mL=1.5;
g=[0 0 -9.8]';

x1=[ 0               5*2*sqrt(3)/3  5];
x2=[ -7 -5*sqrt(3)/3    5];
x3=[  5 -5*sqrt(3)/3    5];
xL=[ 0 0 0]';
x=[x1' x2' x3'];

% % % % % % 
% solver initial
if(n==3)
    C=[ 1 -1  0,
        0  1 -1,
       -1  0  1];
end
C2=abs(C);
rouL = 0.05;

summp = mL*xL;
summ  = mL;
for i=1:n
    summp = summp + m(i)*x(:,i);
    summ  = summ + m(i);
end
for i=1:length(C(:,1))
    tp=[0 0 0]';
    rp=[0 0 0]';
    for j=1:length(C(1,:))
        tp = tp + C(j,i)*x(:,j);
        rp = rp + C2(j,i)*x(:,j);
    end
    summp = summp + norm(tp)*rouL*rp/2;
    summ  = summ  + norm(tp)*rouL;  
end
xC = summp/summ;

xG = xC';
rp1=x1-xG;
rp2=x2-xG;
rp3=x3-xG;
sumr= rp1+rp2+rp3;
RR=msolver(xG,sumr);

x1=(RR'*x1')';
x2=(RR'*x2')';
x3=(RR'*x3')';
% % % % % % % 


x=[x1' x2' x3'];
f_max = 30;
v=zeros(3,n);
f=zeros(1,n);
t=zeros(1,n);
for i=1:n
    v(:,i) = (xL-x(:,i))/norm(xL-x(:,i));
    f(1,i) = f_max;
    t(1,i) = -m(1,i)*v(:,i)'*g+sqrt(f(1,i)*f(1,i)+m(1,i)^2*g'*g*(-v(:,i)'*[0 0 1]'-1));
end
W=-v;
if(n==3)
    erfa = [0 0 0 0 1 1 1 1,
            0 0 1 1 0 0 1 1,
            0 1 0 1 0 1 0 1];
end
sp=zeros(3,length(erfa(1,:)));
for i=1:length(erfa(1,:))
    for j=1:n
        sp(:,i) = sp(:,i)+erfa(j,i)*t(1,j)*W(:,j);
    end
end
sp=sp';
k=convhull(sp);
% figure(1);
% hold on;
% trisurf(k,sp(:,1),sp(:,2),sp(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.5);
% axis equal;


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % 
delta_max=pi/6;
I33=[1 1 1]';
I3=diag(I33);


q=zeros(3,n);
W2=ones(6,n);
for i=1:n
    q(:,i)    = x(:,i)-xC;
    W2(4:6,i) = q(:,i);
end


delta = 0:(delta_max)/3:delta_max;
fai   = 0:pi/5:2*pi;

sp_f=g*(sum(m)-mL);
sp_w=[0 0 0]';
ent=2;
for i=1:length(delta)
    for j=1:length(fai)
        for o=1:length(erfa(1,:))
            sp_w(:,ent) = [0 0 0]';
            sp_f(:,ent) = g*(sum(m)-mL);
            for y=1:n
                sp_w(:,ent) = sp_w(:,ent) + cross(W2(4:6,y),erfa(y,o)*f(1,y)*[sin(delta(i))*cos(fai(j)) sin(delta(i))*sin(fai(j)) cos(delta(i))]');
                sp_f(:,ent) = sp_f(:,ent) + erfa(y,o)*f(1,y)*[sin(delta(i))*cos(fai(j)) sin(delta(i))*sin(fai(j)) cos(delta(i))]';
            end
            ent=ent+1;
        end
    end
end

for i=1:length(sp_f(1,:))
    sp_w2(:,i)=cross(xL-xC,sp_f(:,i));
end

t_f2=ones(1,n)*60;
sp_f2=zeros(3,length(erfa(1,:)));
for i=1:length(erfa(1,:))
    for j=1:n
        sp_f2(:,i) = sp_f2(:,i)+erfa(j,i)*t_f2(1,j)*W(:,j);
    end
end


sp_f=sp_f';
sp_w=sp_w';
sp_w2=sp_w2';
sp_f2=sp_f2';
k_f=convhull(sp_f);
k_w=convhull(sp_w);
k_w2=convhull(sp_w2);
k_f2=convhull(sp_f2);
% figure(2);
% hold on;
% trisurf(k_f,sp_f(:,1),sp_f(:,2),sp_f(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.5);
% axis equal;

figure(3);
hold on;
trisurf(k_w,sp_w(:,1),sp_w(:,2),sp_w(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.5);
trisurf(k_w2,sp_w2(:,1),sp_w2(:,2),sp_w2(:,3),'FaceColor',[1 0 0],'FaceAlpha',0.5);
v=xC-xL;
v=v/norm(v);
quiver3(0,0,0,100*v(1),100*v(2),100*v(3),'k','LineWidth',2);
axis equal;

% figure(4);
% hold on;
% trisurf(k,sp_f2(:,1),sp_f2(:,2),sp_f2(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.5);
% axis equal;

figure(10);
hold on;
trisurf(k,sp(:,1),sp(:,2),sp(:,3),'FaceColor',[0 0 1],'FaceAlpha',0.8);
trisurf(k_f,sp_f(:,1),sp_f(:,2),sp_f(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.4);
trisurf(k_f2,sp_f2(:,1),sp_f2(:,2),sp_f2(:,3),'FaceColor',[1 0 0],'FaceAlpha',0.6);
axis equal;










