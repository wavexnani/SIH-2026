classdef ScenarioEventLogger < handle
    % SCENARIOEVENTLOGGER Real-Time Scenario Event & Timeline Recorder
    %
    % Purpose:
    %   Records high-level autonomous driving milestones (e.g. obstacle detection,
    %   intent transition, maneuver initiation, obstacle clearance, safety filter override)
    %   and formats a human-readable timeline for demonstrations and reports.
    
    properties
        event_log       struct = struct('t', {}, 'type', {}, 'desc', {}, 'x', {}, 'y', {}, 'v', {}, 'intent', {})
        last_intent     char = 'NONE'
        obstacle_logged logical = false
        cleared_logged  logical = false
    end
    
    methods
        function obj = ScenarioEventLogger()
            obj.reset();
        end
        
        function reset(obj)
            obj.event_log = struct('t', {}, 'type', {}, 'desc', {}, 'x', {}, 'y', {}, 'v', {}, 'intent', {});
            obj.last_intent = 'NONE';
            obj.obstacle_logged = false;
            obj.cleared_logged = false;
        end
        
        function update(obj, t, world, info)
            % UPDATE Evaluates simulation step telemetry and logs new events
            if nargin < 4 || isempty(info), return; end
            
            % 1. Detect Intent Changes
            curr_intent = 'MAINTAIN';
            if isfield(info, 'macro_intent') && ~isempty(info.macro_intent)
                curr_intent = info.macro_intent;
            end
            
            if ~strcmp(curr_intent, obj.last_intent) && ~strcmp(obj.last_intent, 'NONE')
                event_type = 'MACRO_INTENT_CHANGE';
                desc = sprintf('Macro intent changed to %s', curr_intent);
                obj.addEvent(t, event_type, desc, world.ego.x, world.ego.y, world.ego.v, curr_intent);
            end
            obj.last_intent = curr_intent;
            
            % 2. Obstacle Detection Event
            if isprop(world, 'n_static_obs') && world.n_static_obs > 0
                obs_x = world.static_obs(1, 1);
                if obs_x > world.ego.x && (obs_x - world.ego.x) < 30.0 && ~obj.obstacle_logged
                    obj.obstacle_logged = true;
                    desc = sprintf('Static obstacle detected ahead at d = %.1fm', obs_x - world.ego.x);
                    obj.addEvent(t, 'OBSTACLE_DETECTED', desc, world.ego.x, world.ego.y, world.ego.v, curr_intent);
                end
                
                % Obstacle Cleared Event
                if obj.obstacle_logged && world.ego.x > (obs_x + 3.0) && ~obj.cleared_logged
                    obj.cleared_logged = true;
                    desc = sprintf('Static obstacle cleared at x = %.1fm', obs_x);
                    obj.addEvent(t, 'OBSTACLE_CLEARED', desc, world.ego.x, world.ego.y, world.ego.v, curr_intent);
                end
            end
            
            % 3. Safety Filter Intervention Event
            if isfield(info, 'filter_active') && info.filter_active
                desc = sprintf('SafetyFilter intervention: %s', info.filter_reason);
                obj.addEvent(t, 'SAFETY_INTERVENTION', desc, world.ego.x, world.ego.y, world.ego.v, curr_intent);
            end
        end
        
        function addEvent(obj, t, type_str, desc_str, x_val, y_val, v_val, intent_str)
            evt.t = t;
            evt.type = type_str;
            evt.desc = desc_str;
            evt.x = x_val;
            evt.y = y_val;
            evt.v = v_val;
            evt.intent = intent_str;
            obj.event_log(end+1) = evt;
        end
        
        function printTimeline(obj)
            fprintf('\n----------------------------------------------------------------------------------------\n');
            fprintf('        SCENARIO EVENT TIMELINE LOG                                                     \n');
            fprintf('----------------------------------------------------------------------------------------\n');
            if isempty(obj.event_log)
                fprintf('  No major event milestones recorded.\n');
            else
                for e = 1:length(obj.event_log)
                    evt = obj.event_log(e);
                    fprintf('  t = %5.1fs | %-20s | (x=%5.1fm, y=%4.2fm, v=%4.1fm/s) | %s\n', ...
                        evt.t, evt.type, evt.x, evt.y, evt.v, evt.desc);
                end
            end
            fprintf('----------------------------------------------------------------------------------------\n\n');
        end
    end
end
