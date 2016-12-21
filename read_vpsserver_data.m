% Requires variable 'port' to be set. 
% Reads and prints VPS points data. 

npts = 20;
loop = true;
set(gcf,'CurrentCharacter','@');

if ~exist('port')
    disp('Set the `port` variable first!')
else
    p = scatter3([],[],[],'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);
    xlabel('X')
    ylabel('Z')
    zlabel('Y')
    
    t = tcpip('localhost', port);
    t.InputBufferSize = npts*12;
    fopen(t);
    while loop
        while (t.BytesAvailable < npts*12) && loop
            loop = ~(get(gcf,'CurrentCharacter') == 'q');
            drawnow;
            pause(0.001);
        end
        
        if loop
            datain = fread(t);
			% This seems to change from Windows to OSX/Linux
            datain = typecast((uint8(datain)), 'single');

            v = 1:3:npts*3;
            xdata =  datain(v);
            zdata = -datain(v+1);
            ydata =  datain(v+2);
            set(p, 'XData', xdata, 'YData', ydata, 'ZData', zdata);
            axis([-0.7 0.7 0 2 -0.7 0.7]);
            drawnow;
        end
    end
    fclose(t);
    delete(t);
    clear t;
    close(gcf);
end

% Cleanly close socket with 'fclose(t)'