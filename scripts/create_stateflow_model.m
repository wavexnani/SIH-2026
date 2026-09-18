function create_stateflow_model()
    % CREATE_STATEFLOW_MODEL Programmatically generates the SIH26037 Stateflow Supervisory Model
    %
    % Purpose:
    %   Creates SIH26037_SupervisoryArchitecture.slx containing the authoritative
    %   hierarchical state-machine model representing Stage 5 perception, prediction,
    %   coordination, motion planning, and safety supervision.
    
    model_name = 'SIH26037_SupervisoryArchitecture';
    
    % Close system if already loaded
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    
    fprintf('Creating new Simulink system: %s...\n', model_name);
    new_system(model_name, 'Model');
    load_system(model_name);
    
    % Add Stateflow chart block
    rt = sfroot;
    chart_block_path = [model_name '/SupervisoryArchitecture'];
    add_block('sflib/Chart', chart_block_path);
    
    chart = rt.find('-isa', 'Stateflow.Chart', 'Path', chart_block_path);
    if isempty(chart)
        error('Failed to locate Stateflow chart object.');
    end
    
    chart.Name = 'SupervisoryStateflow';
    chart.ActionLanguage = 'MATLAB';
    
    % Define Input Data Contracts
    i1 = Stateflow.Data(chart); i1.Name = 'lead_dx'; i1.Scope = 'Input'; i1.DataType = 'double';
    i2 = Stateflow.Data(chart); i2.Name = 'min_ttc'; i2.Scope = 'Input'; i2.DataType = 'double';
    i3 = Stateflow.Data(chart); i3.Name = 'W_avail'; i3.Scope = 'Input'; i3.DataType = 'double';
    i4 = Stateflow.Data(chart); i4.Name = 'mpc_status'; i4.Scope = 'Input'; i4.DataType = 'int32';
    i5 = Stateflow.Data(chart); i5.Name = 'v_ego'; i5.Scope = 'Input'; i5.DataType = 'double';
    i6 = Stateflow.Data(chart); i6.Name = 'clearance'; i6.Scope = 'Input'; i6.DataType = 'double';
    
    % Define Output Data Contracts
    o1 = Stateflow.Data(chart); o1.Name = 'macro_intent'; o1.Scope = 'Output'; o1.DataType = 'uint8';
    o2 = Stateflow.Data(chart); o2.Name = 'target_v'; o2.Scope = 'Output'; o2.DataType = 'double';
    o3 = Stateflow.Data(chart); o3.Name = 'locked_side'; o3.Scope = 'Output'; o3.DataType = 'uint8';
    o4 = Stateflow.Data(chart); o4.Name = 'safety_override'; o4.Scope = 'Output'; o4.DataType = 'boolean';
    
    % Set Chart Annotations / Documentation
    ann1 = Stateflow.Annotation(chart);
    ann1.Text = sprintf('SIH26037 HIERARCHICAL STATEFLOW SUPERVISORY ARCHITECTURE\nAuthoritative Stage 5 Execution Model & PPT Reference');
    ann1.Position = [50 20 0 0];
    
    % Create Top Level Supervisory States
    s_init = Stateflow.State(chart);
    s_init.Name = 'INIT';
    s_init.Position = [50 70 140 60];
    s_init.LabelString = sprintf('INIT\nentry:\n  macro_intent = uint8(0);\n  target_v = 0.0;\n  locked_side = uint8(0);\n  safety_override = false;');
    
    s_nav = Stateflow.State(chart);
    s_nav.Name = 'AUTONOMOUS_NAVIGATION';
    s_nav.Position = [50 160 520 340];
    s_nav.LabelString = sprintf('AUTONOMOUS_NAVIGATION\nentry:\n  safety_override = false;');
    
    s_emerg = Stateflow.State(chart);
    s_emerg.Name = 'EMERGENCY';
    s_emerg.Position = [610 160 150 70];
    s_emerg.LabelString = sprintf('EMERGENCY\nentry:\n  macro_intent = uint8(5);\n  target_v = 0.0;\n  locked_side = uint8(0);\n  safety_override = true;');
    
    s_stop = Stateflow.State(chart);
    s_stop.Name = 'SAFE_STOP';
    s_stop.Position = [610 270 150 70];
    s_stop.LabelString = sprintf('SAFE_STOP\nentry:\n  macro_intent = uint8(6);\n  target_v = 0.0;\n  locked_side = uint8(0);\n  safety_override = true;');
    
    s_complete = Stateflow.State(chart);
    s_complete.Name = 'MISSION_COMPLETE';
    s_complete.Position = [610 380 150 70];
    s_complete.LabelString = sprintf('MISSION_COMPLETE\nentry:\n  macro_intent = uint8(0);\n  target_v = 0.0;\n  locked_side = uint8(0);\n  safety_override = false;');
    
    % Default transition to INIT
    t_init = Stateflow.Transition(chart);
    t_init.Destination = s_init;
    t_init.SourceEndpoint = [20 90];
    t_init.DestinationEndpoint = [50 90];
    
    % Sub-States inside AUTONOMOUS_NAVIGATION
    s_cruise = Stateflow.State(s_nav);
    s_cruise.Name = 'CRUISE';
    s_cruise.Position = [70 200 130 60];
    s_cruise.LabelString = sprintf('CRUISE\nentry:\n  macro_intent = uint8(0);\n  target_v = 8.0;\n  locked_side = uint8(0);');
    
    s_follow = Stateflow.State(s_nav);
    s_follow.Name = 'FOLLOW';
    s_follow.Position = [250 200 130 60];
    s_follow.LabelString = sprintf('FOLLOW\nentry:\n  macro_intent = uint8(1);\n  locked_side = uint8(0);\nduring:\n  target_v = max(0.0, min(8.0, 8.0 + 0.40*(lead_dx - 15.0)));');
    
    s_yield = Stateflow.State(s_nav);
    s_yield.Name = 'YIELD';
    s_yield.Position = [70 320 130 60];
    s_yield.LabelString = sprintf('YIELD\nentry:\n  macro_intent = uint8(2);\n  locked_side = uint8(0);\nduring:\n  target_v = max(0.0, min(4.0, 0.50*(lead_dx - 15.0)));');
    
    s_overtake = Stateflow.State(s_nav);
    s_overtake.Name = 'OVERTAKE';
    s_overtake.Position = [250 320 130 60];
    s_overtake.LabelString = sprintf('OVERTAKE\nentry:\n  macro_intent = uint8(3);\n  locked_side = uint8(1);\n  target_v = max(4.0, v_ego + 2.0);');
    
    s_recover = Stateflow.State(s_nav);
    s_recover.Name = 'RECOVER';
    s_recover.Position = [420 260 130 60];
    s_recover.LabelString = sprintf('RECOVER\nentry:\n  macro_intent = uint8(4);\n  locked_side = uint8(0);\n  target_v = 8.0;');
    
    % Default sub-state transition to CRUISE
    t_cruise_def = Stateflow.Transition(s_nav);
    t_cruise_def.Destination = s_cruise;
    t_cruise_def.SourceEndpoint = [60 230];
    t_cruise_def.DestinationEndpoint = [70 230];
    
    % Navigation sub-state transitions
    t1 = Stateflow.Transition(chart);
    t1.Source = s_init;
    t1.Destination = s_nav;
    t1.LabelString = '[lead_dx > 0]';
    
    t2 = Stateflow.Transition(s_nav);
    t2.Source = s_cruise;
    t2.Destination = s_follow;
    t2.LabelString = '[lead_dx <= 40.0 && lead_dx > 28.0]';
    
    t3 = Stateflow.Transition(s_nav);
    t3.Source = s_cruise;
    t3.Destination = s_yield;
    t3.LabelString = '[min_ttc <= 6.0]';
    
    t4 = Stateflow.Transition(s_nav);
    t4.Source = s_follow;
    t4.Destination = s_overtake;
    t4.LabelString = '[lead_dx <= 28.0 && W_avail >= 4.10 && min_ttc > 6.0]';
    
    t5 = Stateflow.Transition(s_nav);
    t5.Source = s_follow;
    t5.Destination = s_yield;
    t5.LabelString = '[min_ttc <= 6.0 || W_avail < 4.10]';
    
    t6 = Stateflow.Transition(s_nav);
    t6.Source = s_yield;
    t6.Destination = s_overtake;
    t6.LabelString = '[min_ttc > 6.0 && W_avail >= 4.10 && lead_dx <= 28.0]';
    
    t7 = Stateflow.Transition(s_nav);
    t7.Source = s_yield;
    t7.Destination = s_follow;
    t7.LabelString = '[min_ttc > 6.0 && lead_dx > 28.0]';
    
    t8 = Stateflow.Transition(s_nav);
    t8.Source = s_overtake;
    t8.Destination = s_recover;
    t8.LabelString = '[lead_dx >= 7.50]';
    
    t9 = Stateflow.Transition(s_nav);
    t9.Source = s_overtake;
    t9.Destination = s_yield;
    t9.LabelString = '[min_ttc <= 6.0]'; % Abort overtake
    
    t10 = Stateflow.Transition(s_nav);
    t10.Source = s_recover;
    t10.Destination = s_cruise;
    t10.LabelString = '[v_ego >= 7.5]';
    
    % Top Level Emergency & Completion Transitions
    t_emerg = Stateflow.Transition(chart);
    t_emerg.Source = s_nav;
    t_emerg.Destination = s_emerg;
    t_emerg.LabelString = '[mpc_status == 0 || min_ttc < 1.0 || clearance <= 0.05]';
    
    t_stop = Stateflow.Transition(chart);
    t_stop.Source = s_emerg;
    t_stop.Destination = s_stop;
    t_stop.LabelString = '[v_ego < 0.10]';
    
    % Save model
    file_path = fullfile(pwd, [model_name '.slx']);
    save_system(model_name, file_path);
    close_system(model_name, 0);
    
    fprintf('SUCCESS: Stateflow Supervisory Architecture created at: %s\n', file_path);
end
