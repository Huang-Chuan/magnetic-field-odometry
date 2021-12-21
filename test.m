clear all;
clc

% Order
order=2;

% Generate h vector
L=nchoosek(order+1+3, 3)-1;     % (order+4)*(order+3)*(order+2)/6-1;
h=sym(zeros(1,L));              % h(r) in eq. (4) 
th=sym(zeros(1,L));             % \theta in eq. (4)
rx = sym('rx','real');          % Pos x-axis    
ry = sym('ry','real');          % Pos y-axis
rz = sym('rz','real');          % Pos z-axis
ctr=0;
for l=1:order+1
    for x=0:l
        for y=0:l
            for z=0:l
                tmp=x+y+z;
                if tmp==l
                    ctr=ctr+1;
                    h(ctr)=eval(['rx^' num2str(x) '*ry^' num2str(y) '*rz^' num2str(z)]);
                    th(ctr)=sym(['thx' num2str(x) 'y' num2str(y) 'z' num2str(z)], 'real');
                end
            end
        end
    end
end

% Display h(r) and \theta
disp('h(r) is')
disp(h)
disp('theta is')
disp(th)


% Calculate the A matrix
A=sym(zeros(3,L));
A(1,:)=diff(h,rx);
A(2,:)=diff(h,ry);
A(3,:)=diff(h,rz);

% Identify the B matrix
M=nchoosek(order-1+3, 3);
B=zeros(M,L);

Tx=diff(A(1,:),rx);
Ty=diff(A(2,:),ry);
Tz=diff(A(3,:),rz);
ctr=0;
for l=0:order-1
    for x=0:l
        for y=0:l
            for z=0:l
                tmp=x+y+z;
                if tmp==l
                    ctr=ctr+1;
                    for m=1:L
                        for coeff=1:(order+1)*order
                            if isequal(Tx(m),coeff*eval(['rx^' num2str(x) '*ry^' num2str(y) '*rz^' num2str(z)]))
                                B(ctr,m)=coeff;
                            end
                            if isequal(Ty(m),coeff*eval(['rx^' num2str(x) '*ry^' num2str(y) '*rz^' num2str(z)]))
                                B(ctr,m)=coeff;
                            end
                            if isequal(Tz(m),coeff*eval(['rx^' num2str(x) '*ry^' num2str(y) '*rz^' num2str(z)]))
                                B(ctr,m)=coeff;
                            end
                        end
                    end
                end
            end
        end
    end
end

% Get the null space
Bn=null(B,'r');

% Display A*Bn for location r (see eq. 8)
disp('A*Bn for location r')
disp(A*Bn)

% % Calculate 
% Hn = A*th'
% dHn = [diff(Hn, rx)';
%        diff(Hn, ry)';
%        diff(Hn, rz)']
