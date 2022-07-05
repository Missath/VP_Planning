

clear
clc
close all


load('InputWall.txt') 
load('out_Patches.txt')
load('out_ScoreTable_final.txt')
load('out_VPCandidates.txt')
load('out_VP_Solution.txt')

set(gcf, 'position', [0 0 1000 1000]);
set(gca, 'FontSize',20);
xlabel('X (m)')
ylabel('Y (m)')


%% plot obstacles and restricted area
x = [];
y = [];
for i = find(InputWall(:,11) == 0)'
    x = [x InputWall(i,2)];
    y = [y InputWall(i,3)];
    if InputWall(i,1) == 2
        patch(x,y,[0.85 0.85 0.85],'EdgeColor',[0.85 0.85 0.85])
        x = [];
        y = [];
    end
end


for i = find(InputWall(:,11) == 2)'
    x = [x InputWall(i,2)];
    y = [y InputWall(i,3)];
    if InputWall(i,1) == 2
        patch(x,y,[0.85 0.85 0.85],'EdgeColor',[0.85 0.85 0.85])
        x = [];
        y = [];
    end
end

%% plot all wall patches (to scan walls) 
for i= 1: length(out_Patches)
    line([out_Patches(i,3),out_Patches(i,5)],[out_Patches(i,4),out_Patches(i,6)],'Color',[20 20 20]/256,'LineWidth',1.5);
end
hold on
for i = find(InputWall(:,11) == 2)'   
    line([InputWall(i,2),InputWall(i,5)],[InputWall(i,3),InputWall(i,6)],'Color',[20 20 20]/256,'LineStyle','--');
end
for i = find(InputWall(:,11) == 0)'   
    line([InputWall(i,2),InputWall(i,5)],[InputWall(i,3),InputWall(i,6)],'Color',[20 20 20]/256,'LineStyle','-');
end


%% plot VPs
data = out_VPCandidates;
for i = 1: size(out_VPCandidates,1)
       
    plot(data(i,1),data(i,2),'k.','MarkerSize',6);
    hold on

end

%% plot VPs
outVPfinals = out_VP_Solution;
a = 3;
b = 4;
c = 3;
for i = 1: size(outVPfinals,1)
       
    plot([outVPfinals(i,3),outVPfinals(i,3)-a],[outVPfinals(i,4),outVPfinals(i,4)-2*a],'-','LineWidth',2,'Color',[100,120,143]/256);
    hold on
    plot([outVPfinals(i,3),outVPfinals(i,3)+a],[outVPfinals(i,4),outVPfinals(i,4)-2*a],'-','LineWidth',2,'Color',[100,120,143]/256);
    hold on
    plot([outVPfinals(i,3)-b,outVPfinals(i,3)+b],[outVPfinals(i,4),outVPfinals(i,4)],'r-','LineWidth',2.5);
    hold on
    vp_num = sprintf('%d',i);
    set(text(outVPfinals(i,3)+c,outVPfinals(i,4),vp_num),'fontsize',12);

end







