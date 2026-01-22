%% Simulink 3D Animation with Moving Target
figure;
xlimit = [-12 12];
ylimit = [-12 12];
zlimit = [-12 12];
width = 750;
height = 650;
NewFigure(xlimit,ylimit,zlimit,-43,25,width,height);

%VisAttitude([0,0,0],'black')
%VisAttitude(deg2rad(RefEuler),'g:')
pause(1)
AnimEulerTar(out.time1,out.XYZ1,out.EulerAngles1,out.VXYZ1,out.Tar1,out.time2,out.XYZ2,out.EulerAngles2,out.VXYZ2,out.Tar2)

%% Local Functions

function NewFigure(xlim,ylim,zlim,viewx,viewy,w,h)
    set(gca, 'XLim', xlim,'YLim',ylim,'ZLim',zlim);
    view(viewx,viewy)
    x0=10;
    y0=10;
    set(gcf,'position',[x0,y0,w,h])
    hold on;
    grid on;
end

function AnimEuler(t_plot,XYZs,EulerAngles,VXYZs)
    t_section = 0
    curve = animatedline('LineWidth',2,'LineStyle',':');
    for i = 1:length(t_plot)
        if abs( t_plot(i) - t_section) < 0.0001
            % Do Animation
            Euler = EulerAngles(i,:)
            XYZ = XYZs(i,:)
            VXYZ = VXYZs(i,:)
            O = eye(3);
            T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
            O_I = T_BtoI*O
            addpoints(curve, XYZ(1), XYZ(2),XYZ(3))
            line1 = drawline(XYZ,O_I(:,1),'b')
            line2 = drawline(XYZ,O_I(:,2),'g')
            line3 = drawline(XYZ,O_I(:,3),'r')
            line4 = extendline(XYZ,O_I(:,3),'r--')
            line5 = extendline(XYZ,O_I(:,1),'b:')
            drawnow
            pause(0.01)
            
            % labels
            %title('3D view')
          
            xlabel(string( num2str(t_plot(i),'%.1f') )+' sec   ');
            
            dispstr7 = string( num2str( VXYZ(1),'%.1f' ) );
            dispstr8 = string( num2str( VXYZ(2),'%.1f' ) );
            dispstr9 = string( num2str( VXYZ(3),'%.1f' ) );
            vstr = 'Velocity ['+dispstr7+ ' , '+dispstr8 + ' , ' + dispstr9 + ']'
            
            dispstr1 = string( num2str( rad2deg(Euler(1)),'%.1f' ) );
            dispstr2 = string( num2str( rad2deg(Euler(2)),'%.1f' ) );
            dispstr3 = string( num2str( rad2deg(Euler(3)),'%.1f' ) );
            dispstr4 = string( num2str( XYZ(1),'%.1f' ) );
            dispstr5 = string( num2str( XYZ(2),'%.1f' ) );
            dispstr6 = string( num2str( XYZ(3),'%.1f' ) );
            title('EulerAngle ['+dispstr1+' , '+dispstr2+' , '+dispstr3+']  XYZ [' + dispstr4...
                + ' , ' +dispstr5+ ' , '+dispstr6 +']   ' + vstr);
            
            t_section = t_section + 0.1            
            
            if i ~= length(EulerAngles)
                delete(line1)
                delete(line2)
                delete(line3)
                delete(line4)
                delete(line5)
            end
        end
    end    
end

function AnimEulerTar(t_plot1,XYZs1,EulerAngles1,VXYZs1,Tars1,t_plot2,XYZs2,EulerAngles2,VXYZs2,Tars2)
    t_section = 0
    curve1 = animatedline('LineWidth',0.5);
    curveTR1 = animatedline('LineWidth',1,'LineStyle',':');

    curve2 = animatedline('LineWidth',0.5);
    curveTR2 = animatedline('LineWidth',1,'LineStyle',':');
    if length(t_plot1) >= length(t_plot2)
        t_common = t_plot1;
    else
        t_common = t_plot2;
    end
    for i = 1:length(t_common)
        if abs( t_common(i) - t_section) < 0.0001
            % Do Animation
            Euler1 = EulerAngles1(i,:)
            XYZ1 = XYZs1(i,:)
            VXYZ1 = VXYZs1(i,:)
            TR1 = Tars1(i,:)
            
            % Do Animation
            Euler2 = EulerAngles2(i,:)
            XYZ2 = XYZs2(i,:)
            VXYZ2 = VXYZs2(i,:)
            TR2 = Tars2(i,:)


            O2 = eye(3);
            T2 = matrixB2I(Euler2(1),Euler2(2),Euler2(3));
            O2_I = T2*O2

            O1 = eye(3);
            T1 = matrixB2I(Euler1(1),Euler1(2),Euler1(3));
            O1_I = T1*O1

                        
%             pro1 = O_I(:,1)+O_I(:,2) + transpose(XYZ)           
%             pro2 = O_I(:,1)-O_I(:,2) + transpose(XYZ)            
%             pro3 = -O_I(:,1)+O_I(:,2) + transpose(XYZ)           
%             pro4 = -O_I(:,1)-O_I(:,2) + transpose(XYZ)
            
            addpoints(curve1, XYZ1(1), XYZ1(2),XYZ1(3))
            addpoints(curveTR1, TR1(1),TR1(2),TR1(3))
            head1 = scatter3(TR1(1),TR1(2),TR1(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black')

            addpoints(curve2, XYZ2(1), XYZ2(2),XYZ2(3))
           
            head2 = scatter3(TR2(1),TR2(2),TR2(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black')
            
            line11 = drawline(XYZ1,O1_I(:,1),'b--',1.5)
            line21 = drawline(XYZ1,O1_I(:,2),'g--',1.5)
            line31 = drawline(XYZ1,O1_I(:,3),'r--',1.5)
            line51 = extendline(XYZ1,O1_I(:,1),'b:')
            
            frame11 = drawline(XYZ1,0.5*O1_I(:,1)+0.5*O1_I(:,2),'black',2.5)
            frame21 = drawline(XYZ1,0.5*O1_I(:,1)-0.5*O1_I(:,2),'black',2.5)
            frame31 = drawline(XYZ1,-0.5*O1_I(:,1)+0.5*O1_I(:,2),'black',2.5)
            frame41 = drawline(XYZ1,-0.5*O1_I(:,1)-0.5*O1_I(:,2),'black',2.5) 

            line12 = drawline(XYZ2,O2_I(:,1),'b--',1.5)
            line22 = drawline(XYZ2,O2_I(:,2),'g--',1.5)
            line32 = drawline(XYZ2,O2_I(:,3),'r--',1.5)
            line52 = extendline(XYZ2,O2_I(:,1),'b:')
            
            frame12 = drawline(XYZ2,0.5*O2_I(:,1)+0.5*O2_I(:,2),'black',2.5)
            frame22 = drawline(XYZ2,0.5*O2_I(:,1)-0.5*O2_I(:,2),'black',2.5)
            frame32 = drawline(XYZ2,-0.5*O2_I(:,1)+0.5*O2_I(:,2),'black',2.5)
            frame42 = drawline(XYZ2,-0.5*O2_I(:,1)-0.5*O2_I(:,2),'black',2.5)     
%            
%             head1 = scatter3(pro1(1),pro1(2),pro1(3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b')
%             head2 = scatter3(pro2(1),pro2(2),pro2(3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b')
%             head3 = scatter3(pro3(1),pro3(2),pro3(3),'filled','MarkerFaceColor','g','MarkerEdgeColor','g')
%             head4 = scatter3(pro4(1),pro4(2),pro4(3),'filled','MarkerFaceColor','g','MarkerEdgeColor','g')
            
            drawnow
            pause(0.1)
            
            % logs     
            xlabel(string( num2str(t_plot1(i),'%.1f') )+' sec   ');
            
            dispstr7 = string( num2str( VXYZ1(1),'%.1f' ) );
            dispstr8 = string( num2str( VXYZ1(2),'%.1f' ) );
            dispstr9 = string( num2str( VXYZ1(3),'%.1f' ) );
            vstr = 'Velocity ['+dispstr7+ ' , '+dispstr8 + ' , ' + dispstr9 + ']'
            
            dispstr1 = string( num2str( rad2deg(Euler1(1)),'%.1f' ) );
            dispstr2 = string( num2str( rad2deg(Euler1(2)),'%.1f' ) );
            dispstr3 = string( num2str( rad2deg(Euler1(3)),'%.1f' ) );
            dispstr4 = string( num2str( XYZ1(1),'%.1f' ) );
            dispstr5 = string( num2str( XYZ1(2),'%.1f' ) );
            dispstr6 = string( num2str( XYZ1(3),'%.1f' ) );
            title('EulerAngle ['+dispstr1+' , '+dispstr2+' , '+dispstr3+']  XYZ [' + dispstr4...
                + ' , ' +dispstr5+ ' , '+dispstr6 +']   ' + vstr);
            
            t_section = t_section + 0.4            

            delete(line11)
            delete(line21)
            delete(line31)
            delete(line51)
                
            delete(frame11)
            delete(frame21)
            delete(frame31)
            delete(frame41)
                
            delete(head1)
           

            delete(line12)
            delete(line22)
            delete(line32)
            delete(line52)
                
            delete(frame12)
            delete(frame22)
            delete(frame32)
            delete(frame42)
                
            delete(head2)
        end



    end    
end

function SubAnimEuler(t_plot,XYZs,EulerAngles,TR)
    figure;
    t_section = 0  
    for i = 1:length(t_plot)
        if abs( t_plot(i) - t_section) < 0.0001
            % Do Animation    
            subplot(1,2,1)
            NewFigure([-3 3],[-1 5],[0 6],0,0,1200,600);
            scatter3(TR(1),TR(2),TR(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black')       
            Euler = EulerAngles(i,:)
            XYZ = XYZs(i,:)
            O = eye(3);
            T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
            O_I = T_BtoI*O
            line1 = drawline(XYZ,O_I(:,1),'b')
            line2 = drawline(XYZ,O_I(:,2),'g')
            line3 = drawline(XYZ,O_I(:,3),'r')
            line4 = extendline(XYZ,O_I(:,3),'r--')
            line5 = extendline(XYZ,O_I(:,1),'b:')
            
            % labels
            title('XZ plane (side view)')
            dispstr1 = string( num2str( rad2deg(Euler(1)),'%.1f' ) );
            dispstr2 = string( num2str( rad2deg(Euler(2)),'%.1f' ) );
            dispstr3 = string( num2str( rad2deg(Euler(3)),'%.1f' ) );
            dispstr4 = string( num2str( XYZ(1),'%.1f' ) );
            dispstr5 = string( num2str( XYZ(2),'%.1f' ) );
            dispstr6 = string( num2str( XYZ(3),'%.1f' ) );            
            mystr = string( num2str(t_plot(i),'%.1f'))+' sec 3  EulerAngle ['+dispstr1+' , '+dispstr2+' , '+dispstr3+']  XYZ [' + dispstr4...
            + ' , ' +dispstr5+ ' , '+dispstr6 +']';
            txt1 = annotation('textbox', [0.35, 0.9, 0.1, 0.1], 'string', mystr)

            
            subplot(1,2,2)
            NewFigure([-3 3],[-1 5],[0 6],0,90,1200,600);
            scatter3(TR(1),TR(2),TR(3),'filled','MarkerFaceColor','black','MarkerEdgeColor','black')       
            line6 = drawline(XYZ,O_I(:,1),'b')
            line7 = drawline(XYZ,O_I(:,2),'g')
            line8 = drawline(XYZ,O_I(:,3),'r')
            line9 = extendline(XYZ,O_I(:,3),'r--')
            line10 = extendline(XYZ,O_I(:,1),'b:')
            % labels
            title('XY plane (top view)')

            
            drawnow
            pause(0.01)        
            if i ~= length(EulerAngles)
                delete(line1)
                delete(line2)
                delete(line3)
                delete(line4)
                delete(line5)
                delete(line6)
                delete(line7)
                delete(line8)
                delete(line9)
                delete(line10)
                delete(txt1)
            end
                        
            t_section = t_section + 0.2
        end
    end    
end

function m = matrixB2I(phi,theta,psi)
    T_BtoV2 = [[1 0 0];[0 cos(-phi) sin(-phi)];[0 -sin(-phi) cos(-phi)]];
    T_V2toV1 = [[cos(-theta) 0 -sin(-theta)];[0 1 0];[sin(-theta) 0 cos(-theta)]];
    T_V1toI = [[cos(-psi) sin(-psi) 0];[-sin(-psi) cos(-psi) 0];[0 0 1]];
    m = T_V1toI*T_V2toV1*T_BtoV2;
end

function line = drawline(p1,p2,color,width)
% MYMEAN Local function that calculates mean of array.
    pt1 = p1
    pt2 = pt1 + transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',width);
end

function line = extendline(p1,p2,color)
% MYMEAN Local function that calculates mean of array.
    pt1 = p1
    pt2 = pt1 + 20*transpose(p2);
    pts = [pt1;pt2];
    line = plot3(pts(:,1), pts(:,2), pts(:,3),color,'LineWidth',0.5);
end

function VisAttitude(Euler,linsty)
    O = eye(3);
    T_BtoI = matrixB2I(Euler(1),Euler(2),Euler(3));
    O_I = T_BtoI*O
    for i = 1:length(O_I)
        drawline(O_I(:,i),linsty)
    end
    z = O_I(:,3)
    scatter3(z(1),z(2),z(3),'filled','MarkerFaceColor','b','MarkerEdgeColor','b')
end

function DrawSphereObstacle(XYZ_obst, r)
    % XYZ_obst: [x; y; z] center
    % r: radius of the sphere

    % Generate a unit sphere
    [Xs, Ys, Zs] = sphere(40);   % 40 faces for smoothness
    r_org = r-0.6;
    % Scale to radius r and shift to center
    Xs = Xs*r_org + XYZ_obst(1);
    Ys = Ys*r_org + XYZ_obst(2);
    Zs = Zs*r_org + XYZ_obst(3);

    % Plot the sphere
    h = surf(Xs, Ys, Zs, ...
        'FaceAlpha', 1, ...    % semi-transparent
        'EdgeColor', 'none', ...
        'FaceColor', [1 0 0]);   % red

    hold on

end
