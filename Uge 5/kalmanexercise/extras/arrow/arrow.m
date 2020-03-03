function [h] = arrow(start,stop,len,ba,tipa,w,p,col)
%     plot([start(1) stop(1)],[start(2) stop(2)],[col 'o']);
    u = stop(1)-start(1);
    v = stop(2)-start(2);
    
    quiver(start(1),start(2),u,v,col,'LineWidth',1,'MaxHeadSize',1)