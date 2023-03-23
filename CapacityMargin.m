function [ps,pe,gama] = CapacityMargin(ps,sp,k,index)

dis=zeros(length(k(:,1)),1);
ve =zeros(length(k(:,1)),3);

for i=1:length(k(:,1))
    disp(['processing: ' num2str(i*100/length(k(:,1))) '%'])
    a = sp(k(i,1),:);
    b = sp(k(i,2),:);
    c = sp(k(i,3),:);
    
    par = planefuc(a,b,c);
    
    A = double(par(1));
    B = double(par(2));
    C = double(par(3));
    D = double(par(4));
    
    
    dis(i)  = abs(A*ps(1)+B*ps(2)+C*ps(3)+D)/sqrt(A^2+B^2+C^2); 
    vn      = -sign(A*ps(1)+B*ps(2)+C*ps(3)+D)*[A,B,C]/norm([A,B,C]);
    v       = vn*dis(i);
    ve(i,:) = ps+v;
end

figure(index);
hold on;
trisurf(k,sp(:,1),sp(:,2),sp(:,3),'FaceColor',[0 1 0],'FaceAlpha',0.3);
plot3(ps(1),ps(2),ps(3),'r*');
plot3(ve(:,1),ve(:,2),ve(:,3),'r*');
for i=1:length(k(:,1))
    line([ps(1) ve(i,1)],[ps(2) ve(i,2)],[ps(3) ve(i,3)],'LineWidth',2);
end
axis equal;

[gama,id] = min(dis);
pe        = ve(id,:);
end