classdef ScenarioEventLogger < handle
    % SCENARIOEVENTLOGGER Chronological Closed-Loop Simulation Event Recorder
    %
    % Purpose:
    %   Logs state transitions, perception events, controller intent selections,
    %   actuator bias events, safety filter interventions, and collision/stop events.
    
    properties
        events  struct = struct('t', {}, 'event_type', {}, 'description', {})
    end
    
    methods
        function obj = ScenarioEventLogger()
            obj.reset();
        end
        
        function reset(obj)
            obj.events = struct('t', {}, 'event_type', {}, 'description', {});
        end
        
        function logEvent(obj, t, event_type, description)
            % LOGEVENT Record a single event at timestamp t
            if nargin < 4, description = ''; end
            
            idx = length(obj.events) + 1;
            obj.events(idx).t = t;
            obj.events(idx).event_type = event_type;
            obj.events(idx).description = description;
        end
        
        function printSummary(obj)
            fprintf('\n--- CHRONOLOGICAL EVENT LOG ---\n');
            if isempty(obj.events)
                fprintf('  (No events recorded)\n');
                return;
            end
            for i = 1:length(obj.events)
                e = obj.events(i);
                if isempty(e.description)
                    fprintf('  t = %5.2f s | %-28s\n', e.t, e.event_type);
                else
                    fprintf('  t = %5.2f s | %-28s | %s\n', e.t, e.event_type, e.description);
                end
            end
            fprintf('--------------------------------\n\n');
        end
    end
end
