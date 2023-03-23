function fucc = planefuc(a,b,c)
% plane: A*x+B*y+C*z+D=0
syms x y z;
D=[ones(4,1),[[x,y,z];a;b;c]];
detd=det(D);
% disp(strcat('The function is ',char(detd),'=0'))

z=solve(detd,z);
% detd
% a
% b
% c
syms x y z;
fucc=zeros(4,1);
tp=detd;
tp=coeffs(tp(1),x);
tp=coeffs(tp(1),y);
tp=coeffs(tp(1),z);
rp=detd;
rp=coeffs(rp(1),y);
rp=coeffs(rp(1),z);
rp=coeffs(rp(1),x);
ep=detd;
ep=coeffs(ep(1),z);
ep=coeffs(ep(1),x);
ep=coeffs(ep(1),y);

if(isequal(size(tp),[1,1])&&isequal(size(rp),[1,1])&&isequal(size(ep),[1,1]))
    fucc(4) = 0;
    tp=detd+1;
    tp=coeffs(tp(1),x);
    if(isequal(size(tp),[1,1]))
        fucc(1) = 0;
    else
        fucc(1) = double(tp(2));
    end
    tp=detd+1;
    tp=coeffs(tp(1),y);
    if(isequal(size(tp),[1,1]))
        fucc(2) = 0;
    else
        fucc(2) = double(tp(2));
    end
    tp=detd+1;
    tp=coeffs(tp(1),z);
    if(isequal(size(tp),[1,1]))
        fucc(3) = 0;
    else
        fucc(3) = double(tp(2));
    end
else
    fucc(4) = double(tp(1));
    tp=detd;
    tp=coeffs(tp(1),x);
    if(isequal(size(tp),[1,1]))
        fucc(1) = 0;
    else
        fucc(1) = double(tp(2));
    end
    tp=detd;
    tp=coeffs(tp(1),y);
    if(isequal(size(tp),[1,1]))
        fucc(2) = 0;
    else
        fucc(2) = double(tp(2));
    end
    tp=detd;
    tp=coeffs(tp(1),z);
    if(isequal(size(tp),[1,1]))
        fucc(3) = 0;
    else
        fucc(3) = double(tp(2));
    end
end
% fucc

% A=fucc(1);
% B=fucc(2);
% C=fucc(3);
% D=fucc(4);

end