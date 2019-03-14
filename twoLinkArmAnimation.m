classdef twoLinkArmAnimation < handle
    %TWOLINKARMANIMATION This class encapsulates a nice animation of the
    %arm
    %   
    
    properties
        bodyPoints;     %For Patch Objects
        jointT;         %Transforms of moveable joint
        staticT;          %Static Transforms
        patches;        %Patch Objects
        fig;
        ax;
        
        linkEdgeColor = [0,0,0];
        linkFaceColor = 0.6*[1,1,1];
        jointFaceColor = 0.4*[1,1,1];
        jointRadius = 0.02; %m
        
        vidObj;
        vidOpen;
        vidRes;
        
        p;
    end
    
    methods (Access = public)
        function obj = twoLinkArmAnimation(p_in)
            %TWOLINKARMANIMATION Construct an instance of this class
            %   This creates a 
            obj.p = p_in;
            obj.fig = figure;
            obj.ax = axes('parent',obj.fig);
            axis equal;   
            axis([-1,2.5,-0.5,2.5]);      
            
            obj = setupPatches(obj,true);
            obj = setupPose(obj,[0,0]);
            
            obj.vidOpen = false;
            obj.vidRes = [1080,1080];
            
            obj.fig.Position = [1964,110,obj.vidRes(1:2)];
            grid on
            drawnow();
        end
        
        function obj = setPose(obj,y)
            %SETPOS Summary of this method goes here
            %   Detailed explanation goes here
            
            % Make into a column Vector
            if size(y,1) < size(y,2)
                y = y';
            end
            
            %Size Check
            if isequal(size(y),[4,1])
               qpos = [y(1),y(3)];               
            elseif isequal(size(y),[2,1])
               qpos = y;   
            else
                warning(['qpos is wrong size.',newline,'Should be 7 or 9 elements, input is ',num2str(length(y))])
                return;
            end
            
            updatePose(obj,qpos);
            drawnow();
        end
    end
    
    methods (Access = private)
                
        function obj = setupPose(obj,qpos)
            %SETUPPOSE update internal display state
            %   This is where the transforms are calculated
            
            %Static Transform to body frame
            set(obj.staticT(1),'Matrix', eye(4)); 
            %Rotation of Link 1
            set(obj.jointT(1),'Matrix', makehgtform('zrotate',qpos(1)));
            
            %Translate along link 1
            set(obj.staticT(2),'Matrix',  makehgtform('translate', [obj.p.l1 0 0])); 
            %Rotation of link 2
            set(obj.jointT(2),'Matrix', makehgtform('zrotate', qpos(2)));      
            
            %Rotate to CM of link 2
            set(obj.staticT(3),'Matrix',  makehgtform('zrotate',obj.p.de));
        end
        
        function obj = updatePose(obj,qpos)
            %UPDATEPOSE update internal display state
            %   This is where the transforms are calculated
            
            %Rotation of Link 1
            set(obj.jointT(1),'Matrix', makehgtform('zrotate',qpos(1)));
            %Rotation of link 2
            set(obj.jointT(2),'Matrix', makehgtform('zrotate', qpos(2)));  
        end
        
        function obj = setupPatches(obj,ground)
           %SETUPPATCHES fill the interal patch objects and 
            %   This relys on the transforms already being calculated
%             ground = true;
            
            if ground == true
                obj.patches(1) = patch([-5,5,5,-5],[0,0,-5,-5],0.8*[1,1,1],'EdgeColor',[1,1,1]);
            else                
                obj.patches(1) = patch([-5,5,5,-5],[0,0,-5,-5],0.8*[1,1,1],'FaceAlpha',0);
            end
            
            %Link 1            
            obj.staticT(1) = hgtransform('Parent',obj.ax);
            obj.jointT(1) = hgtransform('Parent',obj.staticT(1));
            [x_out,y_out] = capsule(obj,0.05,[1,0]);
            obj.patches(2) = patch(  x_out,y_out,obj.linkFaceColor,'EdgeColor',obj.linkEdgeColor,...
                                    'Parent',obj.jointT(1));
            %Link 2
            obj.staticT(2) = hgtransform('Parent',obj.jointT(1));
            obj.jointT(2) =  hgtransform('Parent',obj.staticT(2));
            [x_out,y_out] = capsule(obj,0.05,[1,0]);
            obj.patches(3) = patch(  x_out,y_out,obj.linkFaceColor,'EdgeColor',obj.linkEdgeColor,...
                                    'Parent',obj.jointT(2));
                                
            %Tennis Racket
            [x_out,y_out] = capsule(obj,0.05,[0,0.8]);
            x_out = x_out + 1;
            obj.patches(4) = patch(  x_out,y_out,obj.linkFaceColor,'EdgeColor',obj.linkEdgeColor,...
                                    'Parent',obj.jointT(2));
                                
            %Center of Mass of Link 2                    
            obj.staticT(3) = hgtransform('Parent',obj.jointT(2));
                                                     
            [x_out,y_out] = capsule(obj,0.05,[0,0]);
            x_out = x_out + obj.p.lce;
            obj.patches(5) = patch(  x_out,y_out,obj.linkFaceColor,'EdgeColor',obj.linkEdgeColor,...
                                    'Parent',obj.staticT(3));
                
            %Joint circles
            for i = 1:2
                obj.patches(end+1) = patch( obj.jointRadius*sin(0:0.3:2*pi), obj.jointRadius*cos(0:0.3:2*pi),...
                obj.jointFaceColor,...
                'Parent',obj.jointT(i));
            end
            
        end
    
        function [x_out,y_out] = capsule(obj,radius,endpt)
            %CAPSULE generates a set of x,y points for patching a capsule 
            %from (0,0) to endpt with spec radius
            
            th = atan2(endpt(2),endpt(1));
            l = norm(endpt,2);
            
            x = [-radius*sin(0:0.1:pi),l-radius*sin(pi:0.1:2*pi)];
            y = [ -radius*cos(0:0.1:pi),-radius*cos(pi:0.1:2*pi)];
            
            x_out = -y*sin(th) + x*cos(th);
            y_out = x*sin(th) + y*cos(th);
        end
    end
end

