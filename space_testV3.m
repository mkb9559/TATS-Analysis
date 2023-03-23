%% init
clc;
clear all;

n=3;
m=ones(1,n)*1.1;
mL=0.4;
g=[0 0 -9.8]';

x1=[ 0   5*2*sqrt(3)/3  5];
x2=[ -7 -5*sqrt(3)/3    5];
x3=[  5 -5*sqrt(3)/3    5];
xL=[ 0 0 0]';

x=[x1' x2' x3'];

%% classic cooperative transportation
f_max = 17;
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
    sp(:,i)=mL*g;
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

% xL()
% XL=zeros(length(sp(:,1))+1,1);
% for i=1:50
%     
% end

%% Tensegrity Aerial Transportation System
if(n==3)
    C=[ 1 -1  0,
        0  1 -1,
       -1  0  1];
end
C2=abs(C);
I33=[1 1 1]';
I3=diag(I33);


%% mass centroid
rouL = 0.005;
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
delta_max=pi/60;
% delta_max=pi/2;
% delta_max=0.6;
for i=1:length(sp(:,1))
    delta_max=max(delta_max,abs(atan(norm(sp(i,1:2))/(sp(i,3)+(summ-mL)*abs(g(3))))));
%     tp=abs(atan(norm(sp(i,1:2))/(sp(i,3)+(summ-mL)*abs(g(3)))))
%     a=1;
end
delta_max
%% prime
q=zeros(3,n);
W2=ones(6,n);
for i=1:n
    q(:,i)    = x(:,i)-xC;
    W2(4:6,i) = q(:,i);
end


delta = 0:(delta_max)/10:delta_max;
fai   = 0:pi/5:2*pi;

v = xC-xL;
v = v/norm(v);


%% tension for force wrench
t_f2=ones(1,n)*300;
sp_f2=zeros(3,length(erfa(1,:)));
for i=1:length(erfa(1,:))
    sp_f2(:,i)=mL*g;
    for j=1:n
        sp_f2(:,i) = sp_f2(:,i)+erfa(j,i)*t_f2(1,j)*W(:,j);
    end
end
sp_f2 = sp_f2';
k_f2 = convhull(sp_f2);

%% normal wrench space
sp_f  = g*(summ-mL);
sp_w  = [0 0 0]';

ent=2;
rL=xC-xL;

sp_f_int = sp;
sp_w_int = [0 0 0];
rnt = length(sp(:,1))+1;
for i=1:length(delta)
    for j=1:length(fai)
        for o=1:length(erfa(1,:))
%             sp_w(:,ent) = cross(xC-xL,mL*g);
            sp_w(:,ent) = [0 0 0]';
            sp_f(:,ent) = g*summ;
            for y=1:n
                sp_w(:,ent) = sp_w(:,ent) + cross(W2(4:6,y),erfa(y,o)*f(1,y)*[sin(delta(i))*cos(fai(j)) sin(delta(i))*sin(fai(j)) cos(delta(i))]');
                sp_f(:,ent) = sp_f(:,ent) + erfa(y,o)*f(1,y)*[sin(delta(i))*cos(fai(j)) sin(delta(i))*sin(fai(j)) cos(delta(i))]';
            end
            if(isequal(k_f2, convhull([sp_f2;sp_f(:,ent)'])))
                sp_f_int(rnt,:) = sp_f(:,ent)';
                sp_w_int(rnt-length(sp(:,1)),:) = sp_w(:,ent)';
                sp_w_L(rnt-length(sp(:,1)),:)   = cross(-sp_f(:,ent)',rL);
                sp_w_int(rnt-length(sp(:,1)),:) = sp_w_int(rnt-length(sp(:,1)),:)./norm(xC-xL);
                rnt=rnt+1;
            end
            sp_w(:,ent)   = sp_w(:,ent)./norm(xC-xL);
%             sp_w(:,ent)=sp_w(:,ent)./abs(xC-xL);
            ent=ent+1;
        end
    end
end

%% convhull 

sp_f  = sp_f';
sp_w  = sp_w';
% u=u';
k_f  = convhull(sp_f);
k_w  = convhull(sp_w);

k_f_int = convhull(sp_f_int);
k_w_int = convhull(sp_w_int);
k_w_L = convhull(sp_w_L);

%% capacity margin

[psf,pef,gamaf]=CapacityMargin([0 0 0],sp_f_int,k_f_int,1);
% [psw,pew,gamaw]=CapacityMargin([0 0 0],sp_w_int,k_w_int,2);
[psw,pew,gamaw]=CapacityMargin([0 0 0],sp_w,k_w,2);



%% data plots

figure(3);
hold on;
% trisurf(k_w,sp_w(:,1),sp_w(:,2),sp_w(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.3);
trisurf(k_w,sp_w(:,1),sp_w(:,2),sp_w(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.4,'EdgeAlpha',0.4);
% trisurf(k_w_int,sp_w_int(:,1),sp_w_int(:,2),sp_w_int(:,3),'FaceColor',[1 0 0],'FaceAlpha',0.5);
plot3(psw(1),psw(2),psw(3),'r*');
plot3(pew(1),pew(2),pew(3),'r*');
line([psw(1) pew(1)],[psw(2) pew(2)],[psw(3) pew(3)],'Color','r','LineWidth',2);
% trisurf(k_w_L,sp_w_L(:,1),sp_w_L(:,2),sp_w_L(:,3),'FaceColor',[0 0 1],'FaceAlpha',0.5);
axis equal;
view([pi/6 pi/6 pi/4]);
set(gca,'FontSize',16);


figure(10);
hold on;
trisurf(k,sp(:,1),sp(:,2),sp(:,3),'FaceColor',[0 0 1],'FaceAlpha',0.4,'EdgeAlpha',0.4);
trisurf(k_f,sp_f(:,1),sp_f(:,2),sp_f(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.2,'EdgeAlpha',0.4);
trisurf(k_f2,sp_f2(:,1),sp_f2(:,2),sp_f2(:,3),'FaceColor',[1 1 0],'FaceAlpha',0.3,'EdgeAlpha',0.4);
% trisurf(k_f_int,sp_f_int(:,1),sp_f_int(:,2),sp_f_int(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.4,'EdgeAlpha',0.4);
trisurf(k_f_int,sp_f_int(:,1),sp_f_int(:,2),sp_f_int(:,3),'FaceColor',[1 0 0],'FaceAlpha',0.4,'EdgeAlpha',0.4);
plot3(psf(1),psf(2),psf(3),'r*');
plot3(pef(1),pef(2),pef(3),'r*');
line([psf(1) pef(1)],[psf(2) pef(2)],[psf(3) pef(3)],'Color','r','LineWidth',2);
% quiver3(0,0,0,50*v(1),50*v(2),50*v(3),'k','LineWidth',2);
axis equal;
% view([pi/6 pi/6 pi/9]);
view(v)
% axis([-50 50 -50 50 -50 50]);
axis([-40 40 -40 40 -20 60]);
h=legend('$\tilde{\mathcal{W}}$','$\mathcal{W}_{1}$','$\mathcal{W}_{2}$','location','northeast');
set(h,'fontsize',12,'interpreter','latex');

vp = [pi/6 pi/6 pi/9];
figure(11);
subplot(1,4,1);
trisurf(k_f,sp_f(:,1),sp_f(:,2),sp_f(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.2,'EdgeAlpha',0.4);
axis equal;view(vp);
axis([-30 30 -30 30 -50 50]);title('$\mathcal{F}_{1}$','fontsize',24,'interpreter','latex');
h=legend('$\mathcal{F}_{1}$','location','northeast');
xlabel('${f}_{x}(N)$','interpreter','latex');
ylabel('${f}_{y}(N)$','interpreter','latex');
zlabel('${f}_{z}(N)$','interpreter','latex');
set(gca,'FontSize',16);set(h,'fontsize',24,'interpreter','latex');
subplot(1,4,2);
trisurf(k_f2,sp_f2(:,1),sp_f2(:,2),sp_f2(:,3),'FaceColor',[1 1 0],'FaceAlpha',0.3,'EdgeAlpha',0.4);
axis equal;view(vp);
axis([-30 30 -30 30 -10 50]);title('$\mathcal{F}_{2}$','fontsize',24,'interpreter','latex');
h=legend('$\mathcal{F}_{2}$','location','northeast');
xlabel('${f}_{x}(N)$','interpreter','latex');
ylabel('${f}_{y}(N)$','interpreter','latex');
zlabel('${f}_{z}(N)$','interpreter','latex');
set(gca,'FontSize',16);set(h,'fontsize',24,'interpreter','latex');
subplot(1,4,3);
hold on;
trisurf(k_f,sp_f(:,1),sp_f(:,2),sp_f(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.2,'EdgeAlpha',0.4);
trisurf(k_f2,sp_f2(:,1),sp_f2(:,2),sp_f2(:,3),'FaceColor',[1 1 0],'FaceAlpha',0.3,'EdgeAlpha',0.4);
axis equal;view(vp);
axis([-30 30 -30 30 -10 50]);title('Intersection','fontsize',24,'interpreter','latex');
h=legend('$\mathcal{F}_{1}$','$\mathcal{F}_{2}$','location','northeast');
xlabel('${f}_{x}(N)$','interpreter','latex');
ylabel('${f}_{y}(N)$','interpreter','latex');
zlabel('${f}_{z}(N)$','interpreter','latex');
set(gca,'FontSize',16);set(h,'fontsize',24,'Interpreter','latex');
subplot(1,4,4);
trisurf(k_f_int,sp_f_int(:,1),sp_f_int(:,2),sp_f_int(:,3),'FaceColor',[1 0 0],'FaceAlpha',0.4,'EdgeAlpha',0.4);
axis equal;view(vp);
axis([-30 30 -30 30 -10 50]);title('$\mathcal{F}$','fontsize',24,'interpreter','latex');
h=legend('$\mathcal{F}$','location','northeast');
xlabel('${f}_{x}(N)$','interpreter','latex');
ylabel('${f}_{y}(N)$','interpreter','latex');
zlabel('${f}_{z}(N)$','interpreter','latex');
set(gca,'FontSize',16);set(h,'fontsize',24,'interpreter','latex');












