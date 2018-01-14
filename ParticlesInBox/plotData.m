clear all
filename = 'pos';
delimiterIn = '\t';
headerlinesIn = 1;
fileData = importdata(filename,delimiterIn,headerlinesIn);
datadims = size(fileData.data);
cols = datadims(2);
rows = datadims(1);
data = fileData.data(:,2:cols);
i_ptcl = (1:(cols-1)/3)-1;
loops = size(data(:,1));
F(loops) = struct('cdata',[],'colormap',[]);
axis vis3d

% for j = i_ptcl
%          plot3(data(:,j*3+1),data(:,j*3+2),data(:,j*3+3),'-');
%          hold on
% end


for time = 1:rows
    for j = i_ptcl
       plot3(data(time,j*3+1),data(time,j*3+2),data(time,j*3+3),'*');
       hold on
    end
%         plot3(data(time,0*3+1),data(time,0*3+2),data(time,0*3+3),'blueo');
%         hold on
%         plot3(data(time,1*3+1),data(time,1*3+2),data(time,1*3+3),'ro');
%         hold off
    axis([-1,1,-1,1,-1,1])
    F(time)=getframe(gcf);
end
xlabel('x')
ylabel('y')
zlabel('z')
%movie(F,1,1)
