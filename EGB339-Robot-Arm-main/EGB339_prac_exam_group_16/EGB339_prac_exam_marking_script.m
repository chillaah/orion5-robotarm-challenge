clear all
close all

%this will be used to flag how you want to be assessed
fname = 'EGB339_prac_exam_group_16.json';
flags = jsondecode(fileread(fname));

%change this for different input sheets
init_image = imread('TestSheet1.jpg');
dest_image = imread('TestSheet8.jpg');

%this will not be made available but used to determine marks
%this example is for TestSheet1 and TestSheet8 for exam sheet 2
true_init_pos = [145.5, 422.5; 54.1, 410.8; 99.2, 80.3];
true_dest_pos = [240.2, 474.3; 214.9, 366.7; 190.5, 104.9];

cylinder_offsheet_positions = [200, -100; 300, -100; 400, -100];

%Your software will be tested twice using the list of run flags
%from your json file
for i = 1:2
    
    current_run_flags = flags.run_flags(i);
   
    dobot_sim = coppeliaRobot('Dobot');
    dobot_sim.startSim(current_run_flags.use_sync); 
    
    %use this to change the exam practice sheet (1 or 2)
    dobot_sim.showExamSheet(2)
    
    if current_run_flags.use_vision

        %first ask for image processing to return init and dest positions
        [ret_init_pos, ret_dest_pos] = EGB339_prac_exam_group_38(dobot_sim, init_image, dest_image, current_run_flags.use_vision);
        
        dobot_sim.setTargetGoalPositions(true_init_pos, true_dest_pos)
        
        if current_run_flags.cylinders_present
             dobot_sim.setCylinderPosition(true_init_pos)
        else
            dobot_sim.setCylinderPosition(cylinder_offsheet_positions)
        end

        %then repeat call to function now asking to execute pick and place and
        %input init and dest positions
        EGB339_prac_exam_group_38(dobot_sim, ret_init_pos, ret_dest_pos, false);
        
    else
        %set this to zero 
        ret_init_pos = 0
        ret_dest_pos = 0

        dobot_sim.setTargetGoalPositions(true_init_pos, true_dest_pos)
         
        if current_run_flags.cylinders_present
             dobot_sim.setCylinderPosition(true_init_pos)
        else
            dobot_sim.setCylinderPosition(cylinder_offsheet_positions)
        end
        
        %then call function but provide the real init and dest positions as
        %forfeited using vision marks
        EGB339_prac_exam_group_38(dobot_sim, true_init_pos, true_dest_pos, false);
    end
       
    actual_cyl_pos = dobot_sim.getCylinderPositions();
    actual_tool_pos = dobot_sim.tool_goal_dist;
     
    disp('Mark for this run');
    run_mark(i) = calculate_mark(ret_init_pos, ret_dest_pos, true_init_pos, true_dest_pos, actual_cyl_pos, actual_tool_pos, current_run_flags)
    
    dobot_sim.clearToolGoalDistances();
    clear dobot_sim;
end


function total_mark = calculate_mark(ret_init_pos, ret_dest_pos, true_init_pos, true_dest_pos, cyl_pos, tool_point_distance, run_flags)

   %evalaute performance of run
        
    vis_pos_mark = 0;
    tool_marks = 0;
    cyl_pos_mark = 0;
    
    %out of 3
    if run_flags.use_vision
        vision_init_error = sqrt(sum((ret_init_pos - true_init_pos).^2,2));
        vision_dest_error = sqrt(sum((ret_dest_pos - true_dest_pos).^2,2));
        
        for j = 1:3
            if vision_init_error(j) < 10
                vis_pos_mark = vis_pos_mark + 0.5
            elseif vision_init_error(j) < 20
                vis_pos_mark = vis_pos_mark + 0.25
            end
            
            if vision_dest_error(j) < 10
                vis_pos_mark = vis_pos_mark + 0.5
            elseif vision_dest_error(j) < 20
                vis_pos_mark = vis_pos_mark + 0.25
            end
        end
    end
    
    %out of 17
    if run_flags.cylinders_present
        tool_distances = cell2mat(tool_point_distance);
        min_tool_dist = min(tool_distances,[],2);
        
        tool_marks = (min_tool_dist < 75).'*[1; 1; 1; 2/3; 2/3; 2/3]
        
        %distance error
        cyl_pos_distance = sqrt(sum((cyl_pos - true_dest_pos).^2,2));
        cyl_init_pos_distance = sqrt(sum((cyl_pos - true_init_pos).^2,2));
        
        %for removing a cylinder + 1 mark
        for j = 1:3
            if cyl_init_pos_distance(j) > 50
                cyl_pos_mark = cyl_pos_mark + 1
            end
        end
        
        %accuracy of cylinder final position

        for j = 1:3
            if cyl_pos_distance(j) < 30
                cyl_pos_mark = cyl_pos_mark + 3
            elseif cyl_pos_distance(j) < 40
                cyl_pos_mark = cyl_pos_mark + 2
            elseif cyl_pos_distance(j) < 50
                cyl_pos_mark = cyl_pos_mark + 1
            end
        end
    else

        %out of 12 instead
        tool_distances = cell2mat(tool_point_distance);
        min_tool_dist = min(tool_distances,[],2);

        % 6 marks for general approach of tool to targets
        tool_marks = (min_tool_dist < 100).'*[1; 1; 1; 1; 1; 1]
        
        %marks for accuracy of reaching goal locations
        for j = 1:6
            if min_tool_dist(j) < 30
                tool_marks = tool_marks + 2
            elseif min_tool_dist(j) < 40
                tool_marks = tool_marks + 1

            end
        end
    end

        
    total_mark = vis_pos_mark + tool_marks + cyl_pos_mark;

end