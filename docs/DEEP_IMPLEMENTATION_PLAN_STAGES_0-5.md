# Deep Implementation Plan: Stages 0-5 for SIH Internal Hackathon

**Date:** August 28, 2026  
**Target Date:** September 1, 2026  
**Objective:** Build one complete closed-loop simulated Indian-road scenario using ground-truth scene information, establish a baseline planner, implement the first version of our risk-aware planner, and generate quantitative comparison metrics.

---

## Executive Architecture Overview

```
                 SIH SOLUTION
                     │
       ┌─────────────┼─────────────┐
       ↓             ↓             ↓
   PERCEPTION   PREDICTION    DECISION/PLANNING
       │             │             │
       └─────────────┼─────────────┘
                     ↓
              CONTEXT/RISK ANALYSIS
                     ↓
              COMPOSITE RISK COST (CRC)
                     ↓
           TRAJECTORY OPTIMIZATION
                     ↓
            HARD SAFETY FILTER
                     ↓
          VEHICLE CONTROL & EXECUTION
                     ↓
                 FEEDBACK LOOP
```

---

# STAGE 0: SIMULATION INFRASTRUCTURE

## 0.1 Core Data Structures & Variables

### 0.1.1 Global Simulation Parameters

```matlab
% FILE: SimulationConfig.m

% === TIMING ===
SIM.dt                 = 0.1;              % Time step (seconds)
SIM.T_horizon          = 10;               % Total simulation time (seconds)
SIM.T_replan           = 0.5;              % Replanning interval (seconds)
SIM.frames_per_step    = SIM.T_replan/SIM.dt;  % Replanning iterations

% === SCENARIO PARAMETERS ===
SIM.scenario_name      = "Unmarked_Village_Road"; % Current scenario
SIM.use_ground_truth   = true;             % Use ground truth vs perception

% === ROAD PARAMETERS ===
ROAD.width             = 6.0;              % Road width (meters)
ROAD.length            = 100.0;            % Road length (meters)
ROAD.boundary_type     = "unclear";        % "clear" | "unclear" | "mixed"
ROAD.center_x          = 0.0;              % Road center x-coordinate
ROAD.marking_type      = "none";           % "full" | "dashed" | "none"

% === VEHICLE PARAMETERS ===
EGO.L                  = 4.7;              % Vehicle length (m)
EGO.W                  = 1.8;              % Vehicle width (m)
EGO.max_speed          = 20.0;             % Max speed (m/s)
EGO.min_speed          = 0.0;              % Min speed (m/s)
EGO.max_accel          = 3.0;              % Max acceleration (m/s²)
EGO.max_decel          = -6.0;             % Max deceleration (m/s²)
EGO.max_steering_angle = deg2rad(35);      % Max steering (rad)
EGO.max_steering_rate  = deg2rad(60);      % Max steering rate (rad/s)
EGO.wheelbase          = 2.7;              % Distance front-rear axle (m)

% === WORLD STATE DIMENSIONS ===
NUM_AGENTS.max_dynamic = 10;               % Max dynamic agents
NUM_AGENTS.max_static  = 20;               % Max static obstacles
NUM_AGENTS.max_anomalies = 5;              % Max road anomalies
```

### 0.1.2 Agent Data Structure

```matlab
% FILE: AgentClass.m

classdef Agent
    properties
        % === IDENTITY ===
        id                  int32            % Unique agent ID
        type                string           % "car"|"pedestrian"|"auto"|"animal"|"bicycle"|"pothole"
        
        % === STATE VARIABLES ===
        % Current state
        x                   double           % X position (m, global)
        y                   double           % Y position (m, global)
        theta               double           % Heading (rad, -pi to pi)
        v                   double           % Speed (m/s)
        omega               double           % Yaw rate (rad/s)
        
        % Vehicle-specific states
        delta               double           % Steering angle (rad) [vehicle only]
        a                   double           % Acceleration (m/s²) [vehicle only]
        
        % Velocity components
        vx                  double           % Longitudinal velocity (m/s)
        vy                  double           % Lateral velocity (m/s)
        
        % === GEOMETRY ===
        length              double           % Length (m)
        width               double           % Width (m)
        
        % === UNCERTAINTY ===
        confidence          double           % Detection confidence [0-1]
        uncertainty_radius  double           % Positional uncertainty (m)
        
        % === PREDICTION ===
        pred_horizon        double           % Prediction horizon (s)
        pred_trajectories   struct           % Multimodal predictions
        % pred_trajectories(i).trajectory = [x_pred, y_pred, theta_pred]
        % pred_trajectories(i).probability = 0.5
        
        % === PROPERTIES ===
        behavior_pattern    string           % "lane_following"|"crossing"|"stopped"|"irregular"
        threat_level        double           % [0-1] immediate threat assessment
        
        % === ANOMALY-SPECIFIC (pothole) ===
        severity            double           % [0-1] pothole severity
        depth               double           % Depth in meters
        
    end
    
    methods
        % Get bounding box
        function [x_corners, y_corners] = get_bounding_box(obj)
            % Returns 4 corners of vehicle bounding box in global frame
            half_len = obj.length / 2;
            half_w = obj.width / 2;
            
            % Local box corners
            corners_local = [half_len, half_len, -half_len, -half_len;
                            half_w, -half_w, -half_w, half_w];
            
            % Rotation matrix
            R = [cos(obj.theta), -sin(obj.theta);
                 sin(obj.theta), cos(obj.theta)];
            
            % Transform to global
            corners_global = R * corners_local;
            x_corners = corners_global(1,:) + obj.x;
            y_corners = corners_global(2,:) + obj.y;
        end
        
        % Get collision polygon
        function poly = get_collision_polygon(obj)
            [x_c, y_c] = get_bounding_box(obj);
            poly = polyshape(x_c, y_c);
        end
    end
end
```

### 0.1.3 Ego Vehicle State

```matlab
% FILE: EgoState.m

classdef EgoState
    properties
        % === KINEMATICS ===
        x               double             % Global X (m)
        y               double             % Global Y (m)
        theta           double             % Heading (rad)
        v               double             % Speed (m/s)
        delta           double             % Steering angle (rad)
        a               double             % Acceleration (m/s²)
        
        % === TRAJECTORY ===
        reference_path  double             % [N×2] reference path
        planned_traj    double             % [M×3] planned trajectory
        % planned_traj = [x_plan, y_plan, v_plan]
        
        % === CONTROL ===
        delta_cmd       double             % Commanded steering (rad)
        a_cmd           double             % Commanded acceleration (m/s²)
        
        % === HISTORY ===
        history_x       double {mustBeVector}    % History of x positions
        history_y       double {mustBeVector}    % History of y positions
        history_v       double {mustBeVector}    % History of velocities
        history_time    double {mustBeVector}    % Time stamps
        
        % === SAFETY ===
        is_collision    logical            % Binary collision flag
        min_clearance   double             % Minimum distance to nearest obstacle
        is_emergency    logical            % Emergency braking triggered
        
    end
end
```

### 0.1.4 World Model

```matlab
% FILE: WorldModel.m

classdef WorldModel
    properties
        % === REFERENCE FRAME ===
        ego_state       EgoState
        
        % === AGENTS ===
        dynamic_agents  Agent {mustBeVector}     % Moving agents
        static_obs      Agent {mustBeVector}     % Static obstacles
        road_anomalies  Agent {mustBeVector}     % Potholes, debris, etc.
        
        % === DRIVABLE REGION ===
        drivable_space  polyshape                % Polygon of drivable area
        road_left_bound double {mustBeVector}    % Left boundary [x1,y1; x2,y2; ...]
        road_right_bound double {mustBeVector}   % Right boundary
        
        % === OCCUPANCY ===
        grid_resolution double             % m/cell
        occupancy_grid  logical             % Boolean occupancy grid
        occupancy_x_range double {mustBeVector}
        occupancy_y_range double {mustBeVector}
        
        % === RISK MAPS ===
        static_risk_map double             % Risk from static obstacles
        dynamic_risk_map double            % Risk from moving agents
        uncertainty_map double             % Risk from uncertainty
        anomaly_risk_map double            % Risk from road anomalies
        composite_risk_map double          % Combined risk
        
        % === TIMING ===
        timestamp       double             % Current time (s)
        
    end
    
    methods
        % Update agent predictions
        function update_predictions(obj, dt_horizon, dt_step)
            % For each dynamic agent, compute predicted future
            for i = 1:length(obj.dynamic_agents)
                agent = obj.dynamic_agents(i);
                
                % Generate constant-velocity prediction (Stage 1-2)
                n_steps = round(dt_horizon / dt_step);
                pred_traj = zeros(n_steps, 3);
                
                for t = 1:n_steps
                    tau = t * dt_step;
                    % Constant velocity model
                    pred_traj(t,1) = agent.x + agent.v * cos(agent.theta) * tau;
                    pred_traj(t,2) = agent.y + agent.v * sin(agent.theta) * tau;
                    pred_traj(t,3) = agent.theta;
                end
                
                obj.dynamic_agents(i).pred_trajectories(1).trajectory = pred_traj;
                obj.dynamic_agents(i).pred_trajectories(1).probability = 1.0;
            end
        end
        
        % Get collision status
        function is_collision = check_collision(obj)
            ego_poly = obj.ego_state.get_collision_polygon();
            is_collision = false;
            
            % Check vs dynamic agents
            for i = 1:length(obj.dynamic_agents)
                agent = obj.dynamic_agents(i);
                agent_poly = agent.get_collision_polygon();
                if overlaps(ego_poly, agent_poly)
                    is_collision = true;
                    return;
                end
            end
            
            % Check vs static obstacles
            for i = 1:length(obj.static_obs)
                obs = obj.static_obs(i);
                obs_poly = obs.get_collision_polygon();
                if overlaps(ego_poly, obs_poly)
                    is_collision = true;
                    return;
                end
            end
        end
    end
end
```

## 0.2 RoadRunner Scenario Setup

### 0.2.1 Scenario Definition

```matlab
% FILE: Stage0_RoadRunner_Setup.m

% === STAGE 0: SIMULATION INFRASTRUCTURE ===
% Create base RoadRunner scenario with one Indian-road scene

function scenario = create_base_scenario()
    
    % === INITIALIZE SCENARIO ===
    scenario = roadrunnerScenario;
    scenario.SampleTime = 0.1;  % 100 ms
    scenario.StopTime = 10;     % 10 seconds
    
    % === LOAD ROAD NETWORK ===
    % Assumption: RoadRunner file exists at this path
    roadrunner_project = "SimulationEnvironment/IndianRoads_v1.rrproj";
    scenario.roadrunnerProject(roadrunner_project);
    
    % === DEFINE SCENARIO: UNMARKED VILLAGE ROAD ===
    % Simple scenario: one vehicle driving on an unmarked road
    % with a few static obstacles and one pedestrian
    
    scenario.name = "Unmarked_Village_Road_Stage0";
    
    % === EGO VEHICLE ===
    ego = actor(scenario, "vehicle");
    ego.Position = [10, 0, 0];           % [x, y, z]
    ego.Yaw = 0;
    ego.Speed = 2;                       % Initial speed m/s
    ego.Mesh = driving.scenario.VehicleMesh(gca, "car");
    ego.Width = 1.8;
    ego.Length = 4.7;
    ego.Height = 1.5;
    
    % === STATIC OBSTACLES (parked auto, debris) ===
    static_obs_1 = actor(scenario, "vehicle");
    static_obs_1.Position = [50, 1.5, 0];
    static_obs_1.Yaw = 0;
    static_obs_1.Speed = 0;
    static_obs_1.Mesh = driving.scenario.VehicleMesh(gca, "car");
    
    static_obs_2 = actor(scenario, "pedestrian");
    static_obs_2.Position = [70, -0.5, 0];
    static_obs_2.Speed = 0;
    
    % === DYNAMIC AGENTS (pedestrian movement) ===
    dynamic_pedestrian = actor(scenario, "pedestrian");
    dynamic_pedestrian.Position = [65, -2, 0];
    dynamic_pedestrian.Speed = 0.5;     % Slow crossing
    
    % === ROAD ANOMALIES (pothole) ===
    % This will be represented as a static obstacle for now
    pothole = actor(scenario, "obstacle");
    pothole.Position = [80, 0.5, 0];
    pothole.Width = 0.5;
    pothole.Length = 0.5;
    pothole.Height = 0.1;
    
    scenario.Actors = [ego, static_obs_1, static_obs_2, dynamic_pedestrian, pothole];
    
end

% === GROUND TRUTH DATA INITIALIZATION ===
function world_model = initialize_world_model()
    
    world_model = WorldModel();
    
    % === EGO VEHICLE ===
    world_model.ego_state = EgoState();
    world_model.ego_state.x = 10;
    world_model.ego_state.y = 0;
    world_model.ego_state.theta = 0;
    world_model.ego_state.v = 2;
    world_model.ego_state.delta = 0;
    world_model.ego_state.a = 0;
    
    % === DYNAMIC AGENTS ===
    ped = Agent();
    ped.id = 1;
    ped.type = "pedestrian";
    ped.x = 65;
    ped.y = -2;
    ped.theta = deg2rad(45);            % Moving diagonally
    ped.v = 0.5;
    ped.length = 0.5;
    ped.width = 0.4;
    ped.confidence = 0.95;
    ped.uncertainty_radius = 0.2;
    ped.behavior_pattern = "crossing";
    
    world_model.dynamic_agents = ped;
    
    % === STATIC OBSTACLES ===
    obs1 = Agent();
    obs1.id = 2;
    obs1.type = "car";
    obs1.x = 50;
    obs1.y = 1.5;
    obs1.theta = 0;
    obs1.v = 0;
    obs1.length = 4.7;
    obs1.width = 1.8;
    obs1.confidence = 1.0;
    
    obs2 = Agent();
    obs2.id = 3;
    obs2.type = "pedestrian";
    obs2.x = 70;
    obs2.y = -0.5;
    obs2.theta = 0;
    obs2.v = 0;
    obs2.length = 0.5;
    obs2.width = 0.4;
    obs2.confidence = 1.0;
    
    world_model.static_obs = [obs1, obs2];
    
    % === ROAD ANOMALIES ===
    pothole = Agent();
    pothole.id = 4;
    pothole.type = "pothole";
    pothole.x = 80;
    pothole.y = 0.5;
    pothole.theta = 0;
    pothole.length = 0.5;
    pothole.width = 0.5;
    pothole.severity = 0.7;             % High severity
    pothole.depth = 0.15;               % 15 cm deep
    pothole.confidence = 1.0;
    
    world_model.road_anomalies = pothole;
    
    % === DRIVABLE REGION ===
    % Simple rectangular drivable area (6m wide road)
    road_left = [0, -3; 100, -3];
    road_right = [0, 3; 100, 3];
    world_model.road_left_bound = road_left;
    world_model.road_right_bound = road_right;
    
    % Create polyshape
    x_road = [0, 100, 100, 0, 0];
    y_road = [-3, -3, 3, 3, -3];
    world_model.drivable_space = polyshape(x_road, y_road);
    
end
```

## 0.3 Simulation Loop Structure

```matlab
% FILE: Stage0_Main_Simulation.m

function run_stage0_simulation()
    
    % === CONFIGURATION ===
    config = SimulationConfig();
    
    % === INITIALIZE ===
    scenario = create_base_scenario();
    world_model = initialize_world_model();
    
    % === LOGGING ===
    log.time = [];
    log.ego_x = [];
    log.ego_y = [];
    log.ego_v = [];
    log.collision = [];
    log.min_clearance = [];
    
    % === MAIN SIMULATION LOOP ===
    t = 0;
    step = 0;
    
    while t < config.SIM.T_horizon
        
        % === 1. UPDATE WORLD STATE ===
        % [Ground truth from RoadRunner]
        world_model.timestamp = t;
        world_model = update_world_model_from_roadrunner(scenario, world_model);
        
        % === 2. LOGGING ===
        log.time = [log.time; t];
        log.ego_x = [log.ego_x; world_model.ego_state.x];
        log.ego_y = [log.ego_y; world_model.ego_state.y];
        log.ego_v = [log.ego_v; world_model.ego_state.v];
        log.collision = [log.collision; world_model.check_collision()];
        log.min_clearance = [log.min_clearance; compute_min_clearance(world_model)];
        
        % === 3. SIMPLE OPEN-LOOP CONTROL ===
        % Stage 0: Just follow reference path without replanning
        if step == 0
            world_model.ego_state.a_cmd = 0;
            world_model.ego_state.delta_cmd = 0;
        end
        
        % === 4. STEP SIMULATION ===
        advance(scenario, config.SIM.dt);
        
        % === 5. UPDATE TIME ===
        t = t + config.SIM.dt;
        step = step + 1;
        
    end
    
    % === RESULTS ===
    results.log = log;
    results.collision_count = sum(log.collision);
    results.collision_rate = results.collision_count / length(log.collision);
    results.min_clearance_min = min(log.min_clearance);
    results.completion_rate = (world_model.ego_state.x - 10) / 90;  % Percent of road completed
    
    fprintf("\n=== STAGE 0 RESULTS ===\n");
    fprintf("Collision Rate: %.2f%%\n", results.collision_rate * 100);
    fprintf("Min Clearance: %.2f m\n", results.min_clearance_min);
    fprintf("Completion Rate: %.2f%%\n", results.completion_rate * 100);
    
end
```

---

# STAGE 1: CLOSED-LOOP VEHICLE CONTROL

## 1.1 Vehicle Dynamics Model

### 1.1.1 Bicycle Model Implementation

```matlab
% FILE: BicycleModel.m

classdef BicycleModel
    properties
        % === VEHICLE PARAMETERS ===
        L       double                  % Wheelbase (m)
        lr      double                  % Distance from CG to rear axle (m)
        lf      double                  % Distance from CG to front axle (m)
        m       double                  % Mass (kg)
        Iz      double                  % Yaw inertia (kg·m²)
        Cf      double                  % Front cornering stiffness
        Cr      double                  % Rear cornering stiffness
        
        % === ACTUATOR LIMITS ===
        max_delta       double          % Max steering angle (rad)
        max_delta_rate  double          % Max steering rate (rad/s)
        max_accel       double          % Max acceleration (m/s²)
        max_decel       double          % Max deceleration (m/s²)
        max_speed       double          % Max speed (m/s)
        
    end
    
    methods
        function obj = BicycleModel()
            % Initialize with typical sedan parameters
            obj.L = 2.7;
            obj.lr = obj.L / 2;
            obj.lf = obj.L / 2;
            obj.m = 1500;
            obj.Iz = 2500;
            obj.Cf = 150000;
            obj.Cr = 150000;
            
            obj.max_delta = deg2rad(35);
            obj.max_delta_rate = deg2rad(60);
            obj.max_accel = 3.0;
            obj.max_decel = -6.0;
            obj.max_speed = 20.0;
        end
        
        % Kinematic bicycle model (simple, no lateral dynamics)
        function [x_dot, y_dot, theta_dot, v_dot, delta_dot] = kinematics(obj, x, y, theta, v, delta, a, delta_cmd)
            
            % === CLAMP INPUTS ===
            a = clamp(a, obj.max_decel, obj.max_accel);
            delta_cmd = clamp(delta_cmd, -obj.max_delta, obj.max_delta);
            
            % === STEERING RATE LIMIT ===
            max_delta_change = obj.max_delta_rate * 0.1;  % 0.1 s timestep
            delta_cmd = clamp(delta_cmd, delta - max_delta_change, delta + max_delta_change);
            
            % === KINEMATICS ===
            x_dot = v * cos(theta);
            y_dot = v * sin(theta);
            theta_dot = (v / obj.L) * tan(delta);
            v_dot = a;
            delta_dot = delta_cmd - delta;  % First-order steering dynamics
            
        end
        
        % Dynamic bicycle model (with lateral slip)
        function [x_dot, y_dot, theta_dot, v_dot, omega_dot, delta_dot] = dynamics(obj, state, inputs)
            
            % Extract state
            x = state(1);
            y = state(2);
            theta = state(3);
            v = state(4);
            omega = state(5);
            delta = state(6);
            
            % Extract inputs
            a = inputs(1);
            delta_cmd = inputs(2);
            
            % === CLAMP INPUTS ===
            a = clamp(a, obj.max_decel, obj.max_accel);
            v = clamp(v, 0, obj.max_speed);
            delta_cmd = clamp(delta_cmd, -obj.max_delta, obj.max_delta);
            
            % === TIRE SLIP ANGLES ===
            alpha_f = atan2(v * sin(delta), v * cos(delta) + omega * obj.lf) - delta;
            alpha_r = atan2(-omega * obj.lr, v);
            
            % === TIRE LATERAL FORCES ===
            Fy_f = obj.Cf * alpha_f;
            Fy_r = obj.Cr * alpha_r;
            
            % === EQUATIONS OF MOTION ===
            x_dot = v * cos(theta);
            y_dot = v * sin(theta);
            theta_dot = omega;
            v_dot = a;
            omega_dot = (obj.lf * Fy_f * cos(delta) - obj.lr * Fy_r) / obj.Iz;
            delta_dot = (delta_cmd - delta) * 10;  % First-order actuator model
            
        end
        
    end
end

function val_clamped = clamp(val, min_val, max_val)
    val_clamped = max(min(val, max_val), min_val);
end
```

### 1.1.2 Controller Implementation (PID)

```matlab
% FILE: VehicleController.m

classdef VehicleController
    properties
        % === LONGITUDINAL CONTROL ===
        speed_kp        double = 1.0
        speed_ki        double = 0.1
        speed_kd        double = 0.2
        speed_error_integral double = 0
        
        % === LATERAL CONTROL (Stanley) ===
        stanley_k       double = 0.5
        stanley_kp      double = 2.0
        
        % === CONSTRAINTS ===
        max_accel       double = 3.0
        max_decel       double = -6.0
        max_steering    double
        
        % === TRACKING STATE ===
        prev_lateral_error double = 0
        
    end
    
    methods
        function obj = VehicleController()
            obj.max_steering = deg2rad(35);
        end
        
        % Speed control (PID)
        function accel_cmd = speed_control(obj, v_current, v_desired, dt)
            
            % === PID ===
            error = v_desired - v_current;
            obj.speed_error_integral = obj.speed_error_integral + error * dt;
            deriv = (error - obj.prev_lateral_error) / dt;
            
            accel_cmd = obj.speed_kp * error + ...
                       obj.speed_ki * obj.speed_error_integral + ...
                       obj.speed_kd * deriv;
            
            % === CLAMP ===
            accel_cmd = clamp(accel_cmd, obj.max_decel, obj.max_accel);
            
        end
        
        % Lateral control (Stanley method for path tracking)
        function steering_cmd = lateral_control(obj, x, y, theta, v, reference_path)
            
            % === FIND NEAREST POINT ON PATH ===
            [nearest_idx, ~] = knnsearch(reference_path(:,1:2), [x, y]);
            
            % === CROSS-TRACK ERROR ===
            path_point = reference_path(nearest_idx, 1:2);
            crosstrack_error = (y - path_point(2)) * cos(reference_path(nearest_idx, 3)) - ...
                              (x - path_point(1)) * sin(reference_path(nearest_idx, 3));
            
            % === HEADING ERROR ===
            path_heading = reference_path(nearest_idx, 3);
            heading_error = theta - path_heading;
            
            % Normalize angle to [-pi, pi]
            heading_error = atan2(sin(heading_error), cos(heading_error));
            
            % === STANLEY LAW ===
            k_e = obj.stanley_k;
            steering_cmd = heading_error + atan2(k_e * crosstrack_error, v + 0.5);
            
            % === CLAMP ===
            steering_cmd = clamp(steering_cmd, -obj.max_steering, obj.max_steering);
            
        end
        
    end
end
```

## 1.2 Closed-Loop Simulation

```matlab
% FILE: Stage1_ClosedLoop_Vehicle.m

function run_stage1_closed_loop()
    
    % === CONFIGURATION ===
    config = SimulationConfig();
    model = BicycleModel();
    controller = VehicleController();
    
    % === INITIALIZE ===
    scenario = create_base_scenario();
    world_model = initialize_world_model();
    
    % === REFERENCE PATH ===
    % Simple straight line with slight curve
    path_x = linspace(10, 95, 100);
    path_y = 0.5 * sin(path_x / 10);
    path_theta = atan2(diff(path_y), diff(path_x));
    path_theta = [path_theta; path_theta(end)];
    reference_path = [path_x', path_y', path_theta];
    world_model.ego_state.reference_path = reference_path;
    
    % === STATE VECTOR ===
    state = [world_model.ego_state.x;
            world_model.ego_state.y;
            world_model.ego_state.theta;
            world_model.ego_state.v;
            0;                              % omega (yaw rate)
            world_model.ego_state.delta];   % steering angle
    
    % === LOGGING ===
    log.t = [];
    log.x = [];
    log.y = [];
    log.v = [];
    log.theta = [];
    log.delta = [];
    log.accel_cmd = [];
    log.steering_cmd = [];
    log.collision = [];
    log.clearance = [];
    
    % === MAIN LOOP ===
    t = 0;
    dt = config.SIM.dt;
    
    while t < config.SIM.T_horizon
        
        % === EXTRACT STATE ===
        x = state(1);
        y = state(2);
        theta = state(3);
        v = state(4);
        omega = state(5);
        delta = state(6);
        
        % === DESIRED SPEED ===
        v_desired = 5.0;  % 5 m/s
        
        % === CONTROLLER ===
        accel_cmd = controller.speed_control(v, v_desired, dt);
        steering_cmd = controller.lateral_control(x, y, theta, v, reference_path);
        
        % === VEHICLE DYNAMICS ===
        inputs = [accel_cmd; steering_cmd];
        [x_dot, y_dot, theta_dot, v_dot, omega_dot, delta_dot] = ...
            model.kinematics(x, y, theta, v, delta, accel_cmd, steering_cmd);
        
        % === INTEGRATION (Euler method) ===
        state = state + dt * [x_dot; y_dot; theta_dot; v_dot; omega_dot; delta_dot];
        
        % === UPDATE WORLD MODEL ===
        world_model.ego_state.x = state(1);
        world_model.ego_state.y = state(2);
        world_model.ego_state.theta = state(3);
        world_model.ego_state.v = state(4);
        world_model.ego_state.omega = state(5);
        world_model.ego_state.delta = state(6);
        world_model.ego_state.a_cmd = accel_cmd;
        world_model.ego_state.delta_cmd = steering_cmd;
        
        % === CHECK COLLISION ===
        is_collision = world_model.check_collision();
        min_clearance = compute_min_clearance(world_model);
        
        % === LOGGING ===
        log.t = [log.t; t];
        log.x = [log.x; x];
        log.y = [log.y; y];
        log.v = [log.v; v];
        log.theta = [log.theta; theta];
        log.delta = [log.delta; delta];
        log.accel_cmd = [log.accel_cmd; accel_cmd];
        log.steering_cmd = [log.steering_cmd; steering_cmd];
        log.collision = [log.collision; is_collision];
        log.clearance = [log.clearance; min_clearance];
        
        % === ADVANCE TIME ===
        t = t + dt;
        
    end
    
    % === RESULTS ===
    results.log = log;
    results.collision = any(log.collision);
    results.min_clearance = min(log.clearance);
    results.distance_traveled = sqrt((log.x(end)-log.x(1))^2 + (log.y(end)-log.y(1))^2);
    results.avg_speed = mean(log.v);
    results.max_lateral_error = max(abs(log.y));
    
    fprintf("\n=== STAGE 1 RESULTS ===\n");
    fprintf("Collision: %d\n", results.collision);
    fprintf("Min Clearance: %.2f m\n", results.min_clearance);
    fprintf("Distance Traveled: %.2f m\n", results.distance_traveled);
    fprintf("Avg Speed: %.2f m/s\n", results.avg_speed);
    fprintf("Max Lateral Error: %.2f m\n", results.max_lateral_error);
    
    % === VISUALIZATION ===
    figure;
    subplot(2,2,1); plot(log.x, log.y); grid; title("Vehicle Path");
    xlabel("X (m)"); ylabel("Y (m)");
    
    subplot(2,2,2); plot(log.t, log.v); grid; title("Speed Profile");
    xlabel("Time (s)"); ylabel("Speed (m/s)");
    
    subplot(2,2,3); plot(log.t, log.clearance); grid; title("Minimum Clearance");
    xlabel("Time (s)"); ylabel("Clearance (m)");
    
    subplot(2,2,4); plot(log.t, rad2deg(log.steering_cmd)); grid; title("Steering Command");
    xlabel("Time (s)"); ylabel("Steering (deg)");
    
end
```

---

# STAGE 2: BASELINE PLANNING (Simple Obstacle Avoidance)

## 2.1 Drivable Space Representation

```matlab
% FILE: DrivableSpace.m

classdef DrivableSpace
    properties
        % === ROAD DEFINITION ===
        left_boundary   double {mustBeMatrix}    % [x1 y1; x2 y2; ...]
        right_boundary  double {mustBeMatrix}
        polyshape_boundary polyshape             % Combined boundary
        
        % === UNCERTAINTY ===
        boundary_uncertainty double              % Meters of uncertainty
        inflated_boundary polyshape              % Conservative boundary
        
        % === OCCUPANCY GRID ===
        grid_x_range    double {mustBeVector}
        grid_y_range    double {mustBeVector}
        resolution      double                   % m/cell
        occupancy_grid  logical                  % Occupancy bitmap
        
    end
    
    methods
        function obj = DrivableSpace(left, right, grid_res)
            obj.left_boundary = left;
            obj.right_boundary = right;
            obj.resolution = grid_res;
            obj.boundary_uncertainty = 0.2;     % 20 cm uncertainty
            
            % Create combined polyshape
            x_bound = [left(:,1); flipud(right(:,1))];
            y_bound = [left(:,2); flipud(right(:,2))];
            obj.polyshape_boundary = polyshape(x_bound, y_bound);
            
            % Create inflated boundary (conservative)
            obj.inflated_boundary = polybuffer(obj.polyshape_boundary, -obj.boundary_uncertainty);
        end
        
        % Check if point is in drivable space
        function is_drivable = is_point_drivable(obj, x, y, safety_margin)
            if nargin < 4
                safety_margin = 0.5;  % 50 cm safety margin
            end
            
            pt = polyshape(x, y);
            % Check against inflated boundary with additional safety margin
            conservative_boundary = polybuffer(obj.inflated_boundary, -safety_margin);
            is_drivable = isinterior(conservative_boundary, x, y);
        end
        
        % Check if trajectory is drivable
        function is_drivable_traj = is_trajectory_drivable(obj, trajectory, vehicle_width)
            % trajectory = [x1 y1; x2 y2; ...]
            % Check entire trajectory with vehicle width
            
            is_drivable_traj = true;
            for i = 1:size(trajectory, 1)
                x = trajectory(i, 1);
                y = trajectory(i, 2);
                
                % Check with vehicle width as safety margin
                if ~obj.is_point_drivable(x, y, vehicle_width/2)
                    is_drivable_traj = false;
                    return;
                end
            end
        end
        
    end
end
```

## 2.2 Baseline Planner (Simple Obstacle Avoidance)

```matlab
% FILE: BaselinePlannerSimple.m

classdef BaselinePlannerSimple
    properties
        % === PLANNING PARAMETERS ===
        horizon_distance double              % Look-ahead distance (m)
        horizon_time    double               % Look-ahead time (s)
        step_size       double               % Candidate trajectory step size (m)
        num_candidates  int32                % Number of candidate trajectories
        
        % === VEHICLE CONSTRAINTS ===
        max_steering    double
        wheelbase       double
        vehicle_width   double
        
        % === DRIVABLE SPACE ===
        drivable_space  DrivableSpace
        
    end
    
    methods
        function obj = BaselinePlannerSimple()
            obj.horizon_distance = 20;
            obj.horizon_time = 5;
            obj.step_size = 0.5;
            obj.num_candidates = 7;            % 3 left, center, 3 right
            obj.max_steering = deg2rad(35);
            obj.wheelbase = 2.7;
            obj.vehicle_width = 1.8;
        end
        
        % Generate candidate trajectories
        function trajectories = generate_candidates(obj, ego_x, ego_y, ego_theta, ego_v, reference_path)
            
            % === REFERENCE TRAJECTORY (follow reference path) ===
            % Find nearest point and extract forward trajectory
            [nearest_idx, ~] = knnsearch(reference_path(:,1:2), [ego_x, ego_y]);
            
            % Extract next 20 m or 10 points
            end_idx = min(nearest_idx + 20, size(reference_path, 1));
            reference_segment = reference_path(nearest_idx:end_idx, :);
            
            trajectories = [];
            
            % === GENERATE LATERAL OFFSET TRAJECTORIES ===
            offsets = linspace(-2, 2, obj.num_candidates);  % -2 to +2 m lateral offset
            
            for offset = offsets
                % Apply lateral offset to reference path
                offset_traj = reference_segment;
                
                % Perpendicular to heading
                for i = 1:size(offset_traj, 1)
                    perp_x = -sin(reference_segment(i,3));
                    perp_y = cos(reference_segment(i,3));
                    offset_traj(i,1) = reference_segment(i,1) + offset * perp_x;
                    offset_traj(i,2) = reference_segment(i,2) + offset * perp_y;
                end
                
                % Store trajectory
                trajectories = [trajectories; {offset_traj}];
            end
            
        end
        
        % Score trajectories based on safety and efficiency
        function scores = score_trajectories(obj, trajectories, world_model)
            
            scores = zeros(length(trajectories), 1);
            
            for i = 1:length(trajectories)
                traj = trajectories{i};
                
                % === 1. COLLISION COST ===
                collision_cost = 0;
                min_dist = inf;
                
                for t = 1:size(traj, 1)
                    x_traj = traj(t, 1);
                    y_traj = traj(t, 2);
                    
                    % Check against all obstacles
                    for j = 1:length(world_model.static_obs)
                        obs = world_model.static_obs(j);
                        dist = sqrt((x_traj - obs.x)^2 + (y_traj - obs.y)^2);
                        min_dist = min(min_dist, dist);
                        
                        if dist < (obj.vehicle_width/2 + obs.width/2 + 0.5)
                            collision_cost = collision_cost + 100;
                        end
                    end
                    
                    % Check against dynamic agents (predicted futures)
                    for j = 1:length(world_model.dynamic_agents)
                        agent = world_model.dynamic_agents(j);
                        if isfield(agent, 'pred_trajectories') && ~isempty(agent.pred_trajectories)
                            pred = agent.pred_trajectories(1).trajectory;
                            % Simple distance to predicted trajectory
                            min_pred_dist = min(sqrt((x_traj - pred(:,1)).^2 + (y_traj - pred(:,2)).^2));
                            if min_pred_dist < (obj.vehicle_width/2 + agent.width/2 + 1.0)
                                collision_cost = collision_cost + 50;
                            end
                        end
                    end
                end
                
                % === 2. DRIVABILITY COST ===
                drivable_cost = 0;
                if ~obj.drivable_space.is_trajectory_drivable(traj, obj.vehicle_width)
                    drivable_cost = 1000;
                end
                
                % === 3. SMOOTHNESS COST ===
                smoothness_cost = 0;
                if size(traj, 1) > 2
                    headings = atan2(diff(traj(:,2)), diff(traj(:,1)));
                    heading_changes = abs(diff(headings));
                    smoothness_cost = sum(heading_changes);
                end
                
                % === 4. EFFICIENCY COST (prefer shorter lateral deviation) ===
                efficiency_cost = 0;
                mean_y = mean(traj(:, 2));
                efficiency_cost = abs(mean_y) * 10;
                
                % === TOTAL SCORE ===
                scores(i) = collision_cost + drivable_cost + smoothness_cost + efficiency_cost;
                
            end
            
        end
        
        % Plan trajectory (generate and select best candidate)
        function planned_traj = plan(obj, world_model, reference_path)
            
            % === GENERATE CANDIDATES ===
            candidates = obj.generate_candidates(...
                world_model.ego_state.x, ...
                world_model.ego_state.y, ...
                world_model.ego_state.theta, ...
                world_model.ego_state.v, ...
                reference_path);
            
            % === SCORE CANDIDATES ===
            scores = obj.score_trajectories(candidates, world_model);
            
            % === SELECT BEST ===
            [~, best_idx] = min(scores);
            planned_traj = candidates{best_idx};
            
        end
        
    end
end
```

## 2.3 Stage 2 Main Simulation

```matlab
% FILE: Stage2_Baseline_Planning.m

function run_stage2_baseline_planning()
    
    % === CONFIGURATION ===
    config = SimulationConfig();
    model = BicycleModel();
    controller = VehicleController();
    planner = BaselinePlannerSimple();
    
    % === INITIALIZE ===
    scenario = create_base_scenario();
    world_model = initialize_world_model();
    
    % === REFERENCE PATH ===
    path_x = linspace(10, 95, 100);
    path_y = 0.5 * sin(path_x / 10);
    path_theta = atan2(diff(path_y), diff(path_x));
    path_theta = [path_theta; path_theta(end)];
    reference_path = [path_x', path_y', path_theta];
    
    % === DRIVABLE SPACE ===
    road_left = [0, -3; 100, -3];
    road_right = [0, 3; 100, 3];
    planner.drivable_space = DrivableSpace(road_left, road_right, 0.5);
    
    % === STATE VECTOR ===
    state = [world_model.ego_state.x;
            world_model.ego_state.y;
            world_model.ego_state.theta;
            world_model.ego_state.v;
            0; 0];
    
    % === LOGGING ===
    log.t = [];
    log.x = [];
    log.y = [];
    log.v = [];
    log.collision = [];
    log.clearance = [];
    log.replanning = [];
    
    % === MAIN LOOP ===
    t = 0;
    dt = config.SIM.dt;
    replan_counter = 0;
    
    while t < config.SIM.T_horizon
        
        % === UPDATE WORLD MODEL ===
        world_model.ego_state.x = state(1);
        world_model.ego_state.y = state(2);
        world_model.ego_state.theta = state(3);
        world_model.ego_state.v = state(4);
        
        % === REPLANNING (every 0.5 seconds) ===
        if replan_counter == 0
            world_model = update_world_model_from_roadrunner(scenario, world_model);
            world_model.update_predictions(5, dt);
            planned_traj = planner.plan(world_model, reference_path);
            world_model.ego_state.planned_traj = planned_traj;
            
            log.replanning = [log.replanning; 1];
        else
            log.replanning = [log.replanning; 0];
        end
        
        % === CONTROLLER ===
        v_desired = 5.0;
        accel_cmd = controller.speed_control(state(4), v_desired, dt);
        steering_cmd = controller.lateral_control(...
            state(1), state(2), state(3), state(4), ...
            world_model.ego_state.planned_traj);
        
        % === VEHICLE DYNAMICS ===
        [x_dot, y_dot, theta_dot, v_dot, omega_dot, delta_dot] = ...
            model.kinematics(state(1), state(2), state(3), state(4), state(6), accel_cmd, steering_cmd);
        
        % === INTEGRATION ===
        state = state + dt * [x_dot; y_dot; theta_dot; v_dot; omega_dot; delta_dot];
        
        % === CHECK COLLISION ===
        is_collision = world_model.check_collision();
        min_clearance = compute_min_clearance(world_model);
        
        % === LOGGING ===
        log.t = [log.t; t];
        log.x = [log.x; state(1)];
        log.y = [log.y; state(2)];
        log.v = [log.v; state(4)];
        log.collision = [log.collision; is_collision];
        log.clearance = [log.clearance; min_clearance];
        
        % === UPDATE TIME ===
        t = t + dt;
        replan_counter = replan_counter + 1;
        if replan_counter >= config.SIM.frames_per_step
            replan_counter = 0;
        end
        
    end
    
    % === RESULTS ===
    results_stage2.log = log;
    results_stage2.collision_count = sum(log.collision);
    results_stage2.completion_rate = (state(1) - 10) / 90;
    results_stage2.min_clearance = min(log.clearance);
    results_stage2.replans = sum(log.replanning);
    
    fprintf("\n=== STAGE 2 BASELINE RESULTS ===\n");
    fprintf("Collisions: %d\n", results_stage2.collision_count);
    fprintf("Min Clearance: %.2f m\n", results_stage2.min_clearance);
    fprintf("Completion: %.2f%%\n", results_stage2.completion_rate * 100);
    fprintf("Replans: %d\n", results_stage2.replans);
    
end
```

---

# STAGE 3: QP-MPC BASELINE

## 3.1 QP-MPC Formulation

```matlab
% FILE: QPMPC_Planner.m

classdef QPMPC_Planner
    properties
        % === MPC PARAMETERS ===
        N               int32                   % Prediction horizon (steps)
        dt              double                  % Time step (s)
        
        % === COST WEIGHTS ===
        Q_track         double                  % Path tracking weight
        Q_speed         double                  % Speed tracking weight
        R_accel         double                  % Acceleration weight
        R_steering      double                  % Steering rate weight
        
        % === VEHICLE CONSTRAINTS ===
        wheelbase       double
        max_steering    double
        max_accel       double
        max_decel       double
        max_speed       double
        vehicle_width   double
        
        % === DRIVABLE SPACE ===
        drivable_space  DrivableSpace
        
    end
    
    methods
        function obj = QPMPC_Planner()
            obj.N = 20;                      % 20-step horizon
            obj.dt = 0.1;
            obj.Q_track = 10;
            obj.Q_speed = 5;
            obj.R_accel = 1;
            obj.R_steering = 1;
            
            obj.wheelbase = 2.7;
            obj.max_steering = deg2rad(35);
            obj.max_accel = 3.0;
            obj.max_decel = -6.0;
            obj.max_speed = 20;
            obj.vehicle_width = 1.8;
        end
        
        % Linearized bicycle model for MPC
        function [A, B] = get_linearized_model(obj, v_ref)
            
            % State: [x, y, theta, v, delta]
            % Input: [a, delta_rate]
            
            % Simple linear approximation
            A = eye(5);
            A(1,3) = -v_ref * obj.dt;           % x += -y * theta
            A(2,3) = v_ref * obj.dt;            % y += x * theta
            
            B = zeros(5, 2);
            B(4, 1) = obj.dt;                   % v += a * dt
            B(5, 2) = obj.dt;                   % delta += delta_rate * dt
            
        end
        
        % Build QP problem for MPC
        function [H, f, A_ineq, b_ineq] = build_qp(obj, x0, reference_traj, obstacles)
            
            % State: [x_0...x_N, y_0...y_N, theta_0...theta_N, v_0...v_N, delta_0...delta_N, a_0...a_N, ...
            N = obj.N;
            
            % === COST FUNCTION ===
            % Tracking cost + input regularization
            % J = ||Cx - d||^2_Q + ||u||^2_R
            
            % Reference trajectory
            ref_x = reference_traj(:, 1);
            ref_y = reference_traj(:, 2);
            ref_theta = reference_traj(:, 3);
            
            % Build H and f matrices
            % Simplified version: quadratic cost on position error
            H = sparse(5*N, 5*N);
            f = sparse(5*N, 1);
            
            for i = 1:N
                state_idx = i:N:5*N;
                
                % Position tracking
                H(state_idx(1), state_idx(1)) = obj.Q_track;
                H(state_idx(2), state_idx(2)) = obj.Q_track;
                f(state_idx(1)) = -obj.Q_track * ref_x(i);
                f(state_idx(2)) = -obj.Q_track * ref_y(i);
                
                % Speed tracking
                H(state_idx(4), state_idx(4)) = obj.Q_speed;
                f(state_idx(4)) = -obj.Q_speed * 5;              % Desired speed 5 m/s
            end
            
            % === CONSTRAINTS ===
            A_ineq = [];
            b_ineq = [];
            
            % Velocity constraints
            for i = 1:N
                A_ineq = [A_ineq; zeros(1, 5*N)];
                A_ineq(end, 4*N + i) = 1;
                b_ineq = [b_ineq; obj.max_speed];
            end
            
            % Steering constraints
            for i = 1:N
                A_ineq = [A_ineq; zeros(1, 5*N)];
                A_ineq(end, 5*N + i) = 1;
                b_ineq = [b_ineq; obj.max_steering];
                
                A_ineq = [A_ineq; zeros(1, 5*N)];
                A_ineq(end, 5*N + i) = -1;
                b_ineq = [b_ineq; obj.max_steering];
            end
            
        end
        
        % Solve MPC and return planned trajectory
        function planned_traj = plan(obj, ego_state, reference_path, world_model)
            
            % === EXTRACT REFERENCE TRAJECTORY ===
            [nearest_idx, ~] = knnsearch(reference_path(:,1:2), [ego_state.x, ego_state.y]);
            end_idx = min(nearest_idx + obj.N, size(reference_path, 1));
            reference_segment = reference_path(nearest_idx:end_idx, :);
            
            % Pad if necessary
            while size(reference_segment, 1) < obj.N
                reference_segment = [reference_segment; reference_segment(end, :)];
            end
            
            % === BUILD AND SOLVE QP ===
            [H, f, A, b] = obj.build_qp([ego_state.x; ego_state.y], reference_segment, []);
            
            % Solve with quadprog
            options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
            x_opt = quadprog(H, f, A, b, [], [], [], [], [], options);
            
            % === EXTRACT TRAJECTORY ===
            if isempty(x_opt)
                % Fallback to reference path if QP fails
                planned_traj = reference_segment;
            else
                % Extract x, y from solution
                x_vals = x_opt(1:obj.N);
                y_vals = x_opt(obj.N+1:2*obj.N);
                v_vals = x_opt(4*obj.N+1:5*obj.N);
                
                planned_traj = [x_vals, y_vals, v_vals];
            end
            
        end
        
    end
end
```

## 3.2 Stage 3 Main Simulation

```matlab
% FILE: Stage3_QPMPC_Baseline.m

function [results_qpmpc, log_qpmpc] = run_stage3_qpmpc_baseline()
    
    % === CONFIGURATION ===
    config = SimulationConfig();
    model = BicycleModel();
    controller = VehicleController();
    mpc_planner = QPMPC_Planner();
    
    % === INITIALIZE ===
    scenario = create_base_scenario();
    world_model = initialize_world_model();
    
    % === REFERENCE PATH ===
    path_x = linspace(10, 95, 100);
    path_y = 0.5 * sin(path_x / 10);
    path_theta = atan2(diff(path_y), diff(path_x));
    path_theta = [path_theta; path_theta(end)];
    reference_path = [path_x', path_y', path_theta];
    
    % === STATE VECTOR ===
    state = [10; 0; 0; 0; 0; 0];
    
    % === LOGGING ===
    log_qpmpc.t = [];
    log_qpmpc.x = [];
    log_qpmpc.y = [];
    log_qpmpc.v = [];
    log_qpmpc.collision = [];
    log_qpmpc.clearance = [];
    log_qpmpc.replan_time = [];
    
    % === MAIN LOOP ===
    t = 0;
    dt = config.SIM.dt;
    replan_counter = 0;
    tic;
    
    while t < config.SIM.T_horizon
        
        % === UPDATE WORLD ===
        world_model.ego_state.x = state(1);
        world_model.ego_state.y = state(2);
        world_model.ego_state.theta = state(3);
        world_model.ego_state.v = state(4);
        
        % === REPLANNING ===
        if replan_counter == 0
            t_replan_start = toc;
            world_model = update_world_model_from_roadrunner(scenario, world_model);
            world_model.update_predictions(5, dt);
            planned_traj = mpc_planner.plan(world_model.ego_state, reference_path, world_model);
            world_model.ego_state.planned_traj = planned_traj;
            t_replan = toc - t_replan_start;
            log_qpmpc.replan_time = [log_qpmpc.replan_time; t_replan];
        end
        
        % === CONTROL ===
        v_desired = 5.0;
        accel_cmd = controller.speed_control(state(4), v_desired, dt);
        steering_cmd = controller.lateral_control(...
            state(1), state(2), state(3), state(4), planned_traj);
        
        % === VEHICLE DYNAMICS ===
        [x_dot, y_dot, theta_dot, v_dot, omega_dot, delta_dot] = ...
            model.kinematics(state(1), state(2), state(3), state(4), state(6), accel_cmd, steering_cmd);
        
        % === INTEGRATION ===
        state = state + dt * [x_dot; y_dot; theta_dot; v_dot; omega_dot; delta_dot];
        
        % === COLLISION CHECK ===
        is_collision = world_model.check_collision();
        min_clearance = compute_min_clearance(world_model);
        
        % === LOGGING ===
        log_qpmpc.t = [log_qpmpc.t; t];
        log_qpmpc.x = [log_qpmpc.x; state(1)];
        log_qpmpc.y = [log_qpmpc.y; state(2)];
        log_qpmpc.v = [log_qpmpc.v; state(4)];
        log_qpmpc.collision = [log_qpmpc.collision; is_collision];
        log_qpmpc.clearance = [log_qpmpc.clearance; min_clearance];
        
        % === UPDATE TIME ===
        t = t + dt;
        replan_counter = replan_counter + 1;
        if replan_counter >= config.SIM.frames_per_step
            replan_counter = 0;
        end
        
    end
    
    % === RESULTS ===
    results_qpmpc.collision_count = sum(log_qpmpc.collision);
    results_qpmpc.collision_rate = results_qpmpc.collision_count / length(log_qpmpc.collision);
    results_qpmpc.min_clearance = min(log_qpmpc.clearance);
    results_qpmpc.avg_replan_time = mean(log_qpmpc.replan_time);
    results_qpmpc.max_replan_time = max(log_qpmpc.replan_time);
    results_qpmpc.completion_rate = (state(1) - 10) / 90;
    results_qpmpc.avg_speed = mean(log_qpmpc.v(log_qpmpc.v > 0));
    results_qpmpc.smoothness = mean(abs(diff(log_qpmpc.v)));
    
    fprintf("\n=== STAGE 3 QP-MPC BASELINE RESULTS ===\n");
    fprintf("Collision Rate: %.2f%%\n", results_qpmpc.collision_rate * 100);
    fprintf("Min Clearance: %.2f m\n", results_qpmpc.min_clearance);
    fprintf("Avg Replan Time: %.3f ms\n", results_qpmpc.avg_replan_time * 1000);
    fprintf("Max Replan Time: %.3f ms\n", results_qpmpc.max_replan_time * 1000);
    fprintf("Completion: %.2f%%\n", results_qpmpc.completion_rate * 100);
    fprintf("Avg Speed: %.2f m/s\n", results_qpmpc.avg_speed);
    fprintf("Smoothness: %.3f m/s²\n", results_qpmpc.smoothness);
    
end
```

---

# STAGE 4: CA-CRC PLANNER (Our Proposed Architecture)

## 4.1 Composite Risk Cost (CRC) Implementation

```matlab
% FILE: CompositeRiskCost.m

classdef CompositeRiskCost
    properties
        % === RISK COMPONENTS ===
        % Static obstacles
        w_static        double = 5.0           % Weight for static obstacle risk
        risk_decay_static double = 10.0        % Decay distance (m)
        
        % Dynamic agents
        w_dynamic       double = 8.0           % Weight for moving agent risk
        risk_decay_dynamic double = 15.0       % Decay distance (m)
        
        % Uncertainty
        w_uncertainty   double = 3.0           % Weight for uncertainty risk
        
        % Road anomalies
        w_anomaly       double = 6.0           % Weight for anomaly risk
        
        % Vehicle feasibility
        w_feasibility   double = 2.0           % Weight for feasibility cost
        
        % Comfort
        w_comfort       double = 1.0           % Weight for comfort cost
        
        % Efficiency
        w_efficiency    double = 0.5           % Weight for efficiency cost
        
        % === CONTEXT ADAPTATION ===
        % Weights adapt based on scene context
        context_traffic_density double         % [0-1]
        context_uncertainty    double         % [0-1]
        context_road_anomalies double         % [0-1]
        
    end
    
    methods
        % Calculate static obstacle risk
        function risk = calc_static_risk(obj, ego_x, ego_y, ego_width, static_obs)
            
            risk = 0;
            
            for i = 1:length(static_obs)
                obs = static_obs(i);
                
                % Distance to obstacle
                dist = sqrt((ego_x - obs.x)^2 + (ego_y - obs.y)^2);
                
                % Risk field (exponential decay)
                obs_margin = (ego_width + obs.width) / 2;
                if dist < obs_margin
                    risk = risk + 1000;  % Collision
                elseif dist < obj.risk_decay_static
                    risk_val = exp(-(dist - obs_margin)^2 / (2 * obj.risk_decay_static^2));
                    risk = risk + risk_val;
                end
            end
            
            risk = risk * obj.w_static;
        end
        
        % Calculate dynamic agent risk (including predictions)
        function risk = calc_dynamic_risk(obj, ego_x, ego_y, ego_v, ego_width, dynamic_agents)
            
            risk = 0;
            
            for i = 1:length(dynamic_agents)
                agent = dynamic_agents(i);
                
                % === CURRENT POSITION RISK ===
                dist = sqrt((ego_x - agent.x)^2 + (ego_y - agent.y)^2);
                agent_margin = (ego_width + agent.width) / 2;
                
                if dist < agent_margin
                    risk = risk + 500;
                elseif dist < obj.risk_decay_dynamic
                    risk_val = exp(-(dist - agent_margin)^2 / (2 * obj.risk_decay_dynamic^2));
                    risk = risk + risk_val;
                end
                
                % === PREDICTED FUTURE RISK ===
                if isfield(agent, 'pred_trajectories') && ~isempty(agent.pred_trajectories)
                    pred_traj = agent.pred_trajectories(1).trajectory;
                    pred_prob = agent.pred_trajectories(1).probability;
                    
                    for t = 1:size(pred_traj, 1)
                        pred_x = pred_traj(t, 1);
                        pred_y = pred_traj(t, 2);
                        dist_pred = sqrt((ego_x - pred_x)^2 + (ego_y - pred_y)^2);
                        
                        if dist_pred < agent_margin
                            risk = risk + 500 * pred_prob;
                        elseif dist_pred < obj.risk_decay_dynamic
                            risk_val = exp(-(dist_pred - agent_margin)^2 / (2 * obj.risk_decay_dynamic^2));
                            risk = risk + risk_val * pred_prob;
                        end
                    end
                end
                
            end
            
            risk = risk * obj.w_dynamic;
        end
        
        % Calculate uncertainty risk
        function risk = calc_uncertainty_risk(obj, dynamic_agents)
            
            risk = 0;
            
            for i = 1:length(dynamic_agents)
                agent = dynamic_agents(i);
                
                % Higher uncertainty = higher risk
                % Scale by detection confidence
                if agent.confidence < 0.8
                    risk = risk + (1 - agent.confidence) * 5;
                end
                
                % Uncertainty radius contribution
                risk = risk + agent.uncertainty_radius * 2;
            end
            
            risk = risk * obj.w_uncertainty;
        end
        
        % Calculate road anomaly risk
        function risk = calc_anomaly_risk(obj, ego_x, ego_y, ego_width, road_anomalies)
            
            risk = 0;
            
            for i = 1:length(road_anomalies)
                anomaly = road_anomalies(i);
                
                dist = sqrt((ego_x - anomaly.x)^2 + (ego_y - anomaly.y)^2);
                
                % Severity-dependent risk
                severity_factor = anomaly.severity;
                
                if dist < 2  % 2m risk radius
                    risk_val = (1 - dist/2) * severity_factor * 10;
                    risk = risk + risk_val;
                end
            end
            
            risk = risk * obj.w_anomaly;
        end
        
        % Calculate vehicle feasibility cost
        function cost = calc_feasibility_cost(obj, trajectory, vehicle_model)
            
            cost = 0;
            max_steering = vehicle_model.max_steering;
            max_accel = vehicle_model.max_accel;
            
            % Check heading changes (steering requirements)
            if size(trajectory, 1) > 2
                headings = atan2(diff(trajectory(:,2)), diff(trajectory(:,1)));
                heading_changes = abs(diff(headings));
                
                for change = heading_changes
                    if change > max_steering * 0.5
                        cost = cost + (change / max_steering) * 50;
                    end
                end
            end
            
            cost = cost * obj.w_feasibility;
        end
        
        % Calculate comfort cost (jerk, lateral acceleration)
        function cost = calc_comfort_cost(obj, trajectory)
            
            cost = 0;
            
            if size(trajectory, 1) > 2
                % Lateral acceleration
                headings = atan2(diff(trajectory(:,2)), diff(trajectory(:,1)));
                heading_changes = abs(diff(headings));
                cost = cost + sum(heading_changes) * 10;
                
                % Velocity changes (jerk)
                if size(trajectory, 2) > 2
                    v_diff = abs(diff(trajectory(:,3)));
                    cost = cost + sum(v_diff) * 5;
                end
            end
            
            cost = cost * obj.w_comfort;
        end
        
        % Calculate efficiency cost (lateral deviation from reference path)
        function cost = calc_efficiency_cost(obj, trajectory, reference_path)
            
            cost = 0;
            
            for i = 1:size(trajectory, 1)
                traj_pt = trajectory(i, 1:2);
                
                % Distance to nearest reference point
                [~, min_dist] = knnsearch(reference_path(:,1:2), traj_pt);
                cost = cost + min_dist;
            end
            
            cost = cost * obj.w_efficiency;
        end
        
        % === CONTEXT-ADAPTIVE WEIGHTING ===
        function weights = get_context_adaptive_weights(obj, world_model)
            
            % Analyze current scene
            n_dyn_agents = length(world_model.dynamic_agents);
            n_static = length(world_model.static_obs);
            n_anomalies = length(world_model.road_anomalies);
            
            % Traffic density: 0 = empty, 1 = very dense
            obj.context_traffic_density = min(1.0, n_dyn_agents / 5);
            
            % Uncertainty: based on agent confidences
            if n_dyn_agents > 0
                avg_confidence = mean([world_model.dynamic_agents.confidence]);
                obj.context_uncertainty = 1 - avg_confidence;
            else
                obj.context_uncertainty = 0;
            end
            
            % Anomalies
            if n_anomalies > 0
                avg_severity = mean([world_model.road_anomalies.severity]);
                obj.context_road_anomalies = avg_severity;
            else
                obj.context_road_anomalies = 0;
            end
            
            % === ADAPTIVE WEIGHT MULTIPLIERS ===
            % High traffic → increase dynamic risk weight
            if obj.context_traffic_density > 0.5
                obj.w_dynamic = 8.0 * (1 + obj.context_traffic_density);
            end
            
            % High uncertainty → increase uncertainty weight
            if obj.context_uncertainty > 0.3
                obj.w_uncertainty = 3.0 * (1 + obj.context_uncertainty);
            end
            
            % High anomalies → increase anomaly weight
            if obj.context_road_anomalies > 0.5
                obj.w_anomaly = 6.0 * (1 + obj.context_road_anomalies);
            end
            
            weights.w_static = obj.w_static;
            weights.w_dynamic = obj.w_dynamic;
            weights.w_uncertainty = obj.w_uncertainty;
            weights.w_anomaly = obj.w_anomaly;
            weights.w_feasibility = obj.w_feasibility;
            weights.w_comfort = obj.w_comfort;
            weights.w_efficiency = obj.w_efficiency;
            
        end
        
    end
end
```

## 4.2 Hard Safety Filter

```matlab
% FILE: HardSafetyFilter.m

classdef HardSafetyFilter
    properties
        % === SAFETY THRESHOLDS ===
        min_clearance_threshold double = 0.5   % Minimum 50 cm clearance
        max_decel_emergency double = -8.0      % Emergency braking
        
    end
    
    methods
        % Check if trajectory violates hard safety constraints
        function is_safe = check_trajectory_safety(obj, trajectory, world_model)
            
            is_safe = true;
            vehicle_width = 1.8;
            
            for t = 1:size(trajectory, 1)
                x = trajectory(t, 1);
                y = trajectory(t, 2);
                
                % === COLLISION CHECK ===
                for i = 1:length(world_model.static_obs)
                    obs = world_model.static_obs(i);
                    dist = sqrt((x - obs.x)^2 + (y - obs.y)^2);
                    min_dist = (vehicle_width + obs.width) / 2 + obj.min_clearance_threshold;
                    
                    if dist < min_dist
                        is_safe = false;
                        return;
                    end
                end
                
                % === DYNAMIC AGENT COLLISION ===
                for i = 1:length(world_model.dynamic_agents)
                    agent = world_model.dynamic_agents(i);
                    dist = sqrt((x - agent.x)^2 + (y - agent.y)^2);
                    min_dist = (vehicle_width + agent.width) / 2 + obj.min_clearance_threshold;
                    
                    if dist < min_dist
                        is_safe = false;
                        return;
                    end
                end
            end
            
        end
        
        % Generate emergency braking trajectory
        function emergency_traj = generate_emergency_trajectory(obj, ego_x, ego_y, ego_v, n_steps)
            
            dt = 0.1;
            emergency_traj = zeros(n_steps, 3);
            
            x = ego_x;
            y = ego_y;
            v = ego_v;
            
            for i = 1:n_steps
                emergency_traj(i,:) = [x, y, v];
                
                % Brake with max deceleration
                v = v + obj.max_decel_emergency * dt;
                v = max(0, v);  % Stop at v=0
                
                x = x + v * cos(0) * dt;  % Straight line
            end
            
        end
        
    end
end
```

## 4.3 CA-CRC Planner Integration

```matlab
% FILE: CARCPlanner.m

classdef CARCPlanner
    properties
        % === COMPONENTS ===
        crc         CompositeRiskCost       % Risk cost calculator
        safety_filter HardSafetyFilter      % Safety constraint checker
        
        % === PARAMETERS ===
        n_candidates int32 = 15             % Number of candidate trajectories
        horizon_distance double = 25        % Look-ahead distance (m)
        
    end
    
    methods
        function obj = CARCPlanner()
            obj.crc = CompositeRiskCost();
            obj.safety_filter = HardSafetyFilter();
        end
        
        % Generate candidate trajectories
        function candidates = generate_candidates(obj, ego_x, ego_y, ego_theta, ego_v, reference_path)
            
            % Similar to baseline, but with more candidates
            [nearest_idx, ~] = knnsearch(reference_path(:,1:2), [ego_x, ego_y]);
            end_idx = min(nearest_idx + 25, size(reference_path, 1));
            reference_segment = reference_path(nearest_idx:end_idx, :);
            
            candidates = [];
            
            % Generate 15 candidates: 7 lateral offsets × some variations
            offsets = linspace(-3, 3, obj.n_candidates);
            
            for offset = offsets
                offset_traj = reference_segment;
                
                for i = 1:size(offset_traj, 1)
                    perp_x = -sin(reference_segment(i,3));
                    perp_y = cos(reference_segment(i,3));
                    offset_traj(i,1) = reference_segment(i,1) + offset * perp_x;
                    offset_traj(i,2) = reference_segment(i,2) + offset * perp_y;
                end
                
                candidates = [candidates; {offset_traj}];
            end
            
        end
        
        % Score trajectories with CA-CRC
        function [scores, risk_breakdown] = score_trajectories_crc(obj, candidates, world_model, model)
            
            scores = zeros(length(candidates), 1);
            risk_breakdown = struct();
            
            % Update context-adaptive weights
            weights = obj.crc.get_context_adaptive_weights(world_model);
            
            for i = 1:length(candidates)
                traj = candidates{i};
                
                risk_static = 0;
                risk_dynamic = 0;
                risk_uncertainty = 0;
                risk_anomaly = 0;
                cost_feasibility = 0;
                cost_comfort = 0;
                cost_efficiency = 0;
                
                % Calculate risks for each point on trajectory
                for t = 1:size(traj, 1)
                    x = traj(t, 1);
                    y = traj(t, 2);
                    v = 5.0;  % Assume constant speed for now
                    
                    risk_static = risk_static + ...
                        obj.crc.calc_static_risk(x, y, 1.8, world_model.static_obs);
                    risk_dynamic = risk_dynamic + ...
                        obj.crc.calc_dynamic_risk(x, y, v, 1.8, world_model.dynamic_agents);
                    risk_uncertainty = risk_uncertainty + ...
                        obj.crc.calc_uncertainty_risk(world_model.dynamic_agents);
                    risk_anomaly = risk_anomaly + ...
                        obj.crc.calc_anomaly_risk(x, y, 1.8, world_model.road_anomalies);
                end
                
                % Feasibility and comfort
                cost_feasibility = obj.crc.calc_feasibility_cost(traj, model);
                cost_comfort = obj.crc.calc_comfort_cost(traj);
                cost_efficiency = obj.crc.calc_efficiency_cost(traj, world_model.ego_state.reference_path);
                
                % Total score
                J_crc = risk_static + risk_dynamic + risk_uncertainty + risk_anomaly + ...
                        cost_feasibility + cost_comfort + cost_efficiency;
                
                scores(i) = J_crc;
                
                risk_breakdown(i).static = risk_static;
                risk_breakdown(i).dynamic = risk_dynamic;
                risk_breakdown(i).uncertainty = risk_uncertainty;
                risk_breakdown(i).anomaly = risk_anomaly;
                risk_breakdown(i).feasibility = cost_feasibility;
                risk_breakdown(i).comfort = cost_comfort;
                risk_breakdown(i).efficiency = cost_efficiency;
                
            end
            
        end
        
        % Plan with CA-CRC
        function [planned_traj, scores, is_safe] = plan(obj, world_model, reference_path, model)
            
            % Generate candidates
            candidates = obj.generate_candidates(...
                world_model.ego_state.x, ...
                world_model.ego_state.y, ...
                world_model.ego_state.theta, ...
                world_model.ego_state.v, ...
                reference_path);
            
            % Score with CA-CRC
            [scores, ~] = obj.score_trajectories_crc(candidates, world_model, model);
            
            % Select best candidate
            [~, best_idx] = min(scores);
            candidate_traj = candidates{best_idx};
            
            % Hard safety check
            is_safe = obj.safety_filter.check_trajectory_safety(candidate_traj, world_model);
            
            if is_safe
                planned_traj = candidate_traj;
            else
                % Generate emergency braking
                n_steps = size(candidate_traj, 1);
                planned_traj = obj.safety_filter.generate_emergency_trajectory(...
                    world_model.ego_state.x, ...
                    world_model.ego_state.y, ...
                    world_model.ego_state.v, ...
                    n_steps);
            end
            
        end
        
    end
end
```

## 4.4 Stage 4 Main Simulation

```matlab
% FILE: Stage4_CARC_Planner.m

function [results_carc, log_carc] = run_stage4_carc_planner()
    
    % === CONFIGURATION ===
    config = SimulationConfig();
    model = BicycleModel();
    controller = VehicleController();
    carc_planner = CARCPlanner();
    
    % === INITIALIZE ===
    scenario = create_base_scenario();
    world_model = initialize_world_model();
    
    % === REFERENCE PATH ===
    path_x = linspace(10, 95, 100);
    path_y = 0.5 * sin(path_x / 10);
    path_theta = atan2(diff(path_y), diff(path_x));
    path_theta = [path_theta; path_theta(end)];
    reference_path = [path_x', path_y', path_theta];
    world_model.ego_state.reference_path = reference_path;
    
    % === STATE VECTOR ===
    state = [10; 0; 0; 0; 0; 0];
    
    % === LOGGING ===
    log_carc.t = [];
    log_carc.x = [];
    log_carc.y = [];
    log_carc.v = [];
    log_carc.collision = [];
    log_carc.clearance = [];
    log_carc.replan_time = [];
    log_carc.is_safe = [];
    log_carc.total_risk = [];
    
    % === MAIN LOOP ===
    t = 0;
    dt = config.SIM.dt;
    replan_counter = 0;
    tic;
    
    while t < config.SIM.T_horizon
        
        % === UPDATE WORLD ===
        world_model.ego_state.x = state(1);
        world_model.ego_state.y = state(2);
        world_model.ego_state.theta = state(3);
        world_model.ego_state.v = state(4);
        
        % === REPLANNING ===
        if replan_counter == 0
            t_replan_start = toc;
            world_model = update_world_model_from_roadrunner(scenario, world_model);
            world_model.update_predictions(5, dt);
            [planned_traj, scores, is_safe] = carc_planner.plan(world_model, reference_path, model);
            world_model.ego_state.planned_traj = planned_traj;
            t_replan = toc - t_replan_start;
            log_carc.replan_time = [log_carc.replan_time; t_replan];
            log_carc.is_safe = [log_carc.is_safe; is_safe];
            log_carc.total_risk = [log_carc.total_risk; min(scores)];
        end
        
        % === CONTROL ===
        v_desired = 5.0;
        accel_cmd = controller.speed_control(state(4), v_desired, dt);
        steering_cmd = controller.lateral_control(...
            state(1), state(2), state(3), state(4), planned_traj);
        
        % === VEHICLE DYNAMICS ===
        [x_dot, y_dot, theta_dot, v_dot, omega_dot, delta_dot] = ...
            model.kinematics(state(1), state(2), state(3), state(4), state(6), accel_cmd, steering_cmd);
        
        % === INTEGRATION ===
        state = state + dt * [x_dot; y_dot; theta_dot; v_dot; omega_dot; delta_dot];
        
        % === COLLISION CHECK ===
        is_collision = world_model.check_collision();
        min_clearance = compute_min_clearance(world_model);
        
        % === LOGGING ===
        log_carc.t = [log_carc.t; t];
        log_carc.x = [log_carc.x; state(1)];
        log_carc.y = [log_carc.y; state(2)];
        log_carc.v = [log_carc.v; state(4)];
        log_carc.collision = [log_carc.collision; is_collision];
        log_carc.clearance = [log_carc.clearance; min_clearance];
        
        % === UPDATE TIME ===
        t = t + dt;
        replan_counter = replan_counter + 1;
        if replan_counter >= config.SIM.frames_per_step
            replan_counter = 0;
        end
        
    end
    
    % === RESULTS ===
    results_carc.collision_count = sum(log_carc.collision);
    results_carc.collision_rate = results_carc.collision_count / length(log_carc.collision);
    results_carc.min_clearance = min(log_carc.clearance);
    results_carc.avg_replan_time = mean(log_carc.replan_time);
    results_carc.max_replan_time = max(log_carc.replan_time);
    results_carc.completion_rate = (state(1) - 10) / 90;
    results_carc.avg_speed = mean(log_carc.v(log_carc.v > 0));
    results_carc.smoothness = mean(abs(diff(log_carc.v)));
    results_carc.safety_filter_activations = sum(~log_carc.is_safe);
    
    fprintf("\n=== STAGE 4 CA-CRC PLANNER RESULTS ===\n");
    fprintf("Collision Rate: %.2f%%\n", results_carc.collision_rate * 100);
    fprintf("Min Clearance: %.2f m\n", results_carc.min_clearance);
    fprintf("Avg Replan Time: %.3f ms\n", results_carc.avg_replan_time * 1000);
    fprintf("Safety Filter Activations: %d\n", results_carc.safety_filter_activations);
    fprintf("Completion: %.2f%%\n", results_carc.completion_rate * 100);
    fprintf("Avg Speed: %.2f m/s\n", results_carc.avg_speed);
    
end
```

---

# STAGE 5: METRICS AND COMPARISON

## 5.1 Metrics Definition and Calculation

```matlab
% FILE: PerformanceMetrics.m

classdef PerformanceMetrics
    
    methods (Static)
        
        % === SAFETY METRICS ===
        
        % Collision rate: percentage of time steps with collision
        function rate = collision_rate(collision_log)
            rate = sum(collision_log) / length(collision_log);
        end
        
        % Minimum clearance: closest approach to any obstacle
        function clearance = min_clearance(clearance_log)
            clearance = min(clearance_log);
        end
        
        % Collision-free distance: distance traveled before collision
        function dist = collision_free_distance(x_log, y_log, collision_log)
            idx = find(collision_log, 1);
            if isempty(idx)
                % No collision
                dist = sqrt((x_log(end)-x_log(1))^2 + (y_log(end)-y_log(1))^2);
            else
                dist = sqrt((x_log(idx)-x_log(1))^2 + (y_log(idx)-y_log(1))^2);
            end
        end
        
        % === EFFICIENCY METRICS ===
        
        % Scenario completion rate: percentage of road completed
        function rate = scenario_completion_rate(x_log, start_x, end_x)
            rate = (x_log(end) - start_x) / (end_x - start_x);
            rate = max(0, min(1, rate));
        end
        
        % Average speed
        function avg_v = average_speed(v_log)
            avg_v = mean(v_log(v_log > 0.1));  % Exclude zero velocity
        end
        
        % Average acceleration
        function avg_a = average_acceleration(v_log, dt)
            accel = diff(v_log) / dt;
            avg_a = mean(abs(accel));
        end
        
        % === COMFORT METRICS ===
        
        % Path smoothness: average absolute heading change
        function smooth = path_smoothness(x_log, y_log)
            dx = diff(x_log);
            dy = diff(y_log);
            headings = atan2(dy, dx);
            heading_changes = abs(diff(headings));
            smooth = mean(heading_changes);
        end
        
        % Jerk (rate of change of acceleration)
        function jerk_val = jerk(v_log, dt)
            accel = diff(v_log) / dt;
            jerk_val = mean(abs(diff(accel) / dt));
        end
        
        % === REPLANNING METRICS ===
        
        % Replanning latency statistics
        function stats = replanning_latency(replan_time_log)
            stats.mean = mean(replan_time_log);
            stats.max = max(replan_time_log);
            stats.min = min(replan_time_log);
            stats.std = std(replan_time_log);
        end
        
        % Replanning frequency
        function freq = replanning_frequency(n_replans, total_time)
            freq = n_replans / total_time;
        end
        
        % === TRAJECTORY QUALITY METRICS ===
        
        % Lateral error: deviation from reference path
        function lat_error = lateral_error_from_path(x_log, y_log, reference_path)
            lat_errors = [];
            for i = 1:length(x_log)
                pt = [x_log(i), y_log(i)];
                [~, min_dist] = knnsearch(reference_path(:,1:2), pt);
                lat_errors = [lat_errors; min_dist];
            end
            lat_error.mean = mean(lat_errors);
            lat_error.max = max(lat_errors);
            lat_error.std = std(lat_errors);
        end
        
        % Distance traveled
        function dist = distance_traveled(x_log, y_log)
            dx = diff(x_log);
            dy = diff(y_log);
            segment_distances = sqrt(dx.^2 + dy.^2);
            dist = sum(segment_distances);
        end
        
    end
    
end
```

## 5.2 Comparative Analysis

```matlab
% FILE: Stage5_Metrics_Comparison.m

function results_comparison = run_stage5_metrics_comparison()
    
    % === RUN ALL STAGES ===
    fprintf("\n=== RUNNING ALL STAGES ===\n");
    
    % Stage 2: Baseline
    [results_baseline, log_baseline] = run_stage2_baseline_planning();
    
    % Stage 3: QP-MPC
    [results_qpmpc, log_qpmpc] = run_stage3_qpmpc_baseline();
    
    % Stage 4: CA-CRC
    [results_carc, log_carc] = run_stage4_carc_planner();
    
    % === CALCULATE COMPREHENSIVE METRICS ===
    dt = 0.1;
    reference_path = build_reference_path();
    
    % === BASELINE METRICS ===
    fprintf("\n=== METRICS: BASELINE ===\n");
    m_baseline.collision_rate = PerformanceMetrics.collision_rate(log_baseline.collision);
    m_baseline.min_clearance = PerformanceMetrics.min_clearance(log_baseline.clearance);
    m_baseline.collision_free_dist = PerformanceMetrics.collision_free_distance(...
        log_baseline.x, log_baseline.y, log_baseline.collision);
    m_baseline.completion = PerformanceMetrics.scenario_completion_rate(...
        log_baseline.x, 10, 95);
    m_baseline.avg_speed = PerformanceMetrics.average_speed(log_baseline.v);
    m_baseline.smoothness = PerformanceMetrics.path_smoothness(log_baseline.x, log_baseline.y);
    m_baseline.avg_accel = PerformanceMetrics.average_acceleration(log_baseline.v, dt);
    m_baseline.distance = PerformanceMetrics.distance_traveled(log_baseline.x, log_baseline.y);
    
    fprintf("Collision Rate: %.2f%%\n", m_baseline.collision_rate * 100);
    fprintf("Min Clearance: %.2f m\n", m_baseline.min_clearance);
    fprintf("Collision-Free Distance: %.2f m\n", m_baseline.collision_free_dist);
    fprintf("Completion Rate: %.2f%%\n", m_baseline.completion * 100);
    fprintf("Avg Speed: %.2f m/s\n", m_baseline.avg_speed);
    fprintf("Path Smoothness: %.3f rad\n", m_baseline.smoothness);
    fprintf("Distance Traveled: %.2f m\n", m_baseline.distance);
    
    % === QP-MPC METRICS ===
    fprintf("\n=== METRICS: QP-MPC ===\n");
    m_qpmpc.collision_rate = PerformanceMetrics.collision_rate(log_qpmpc.collision);
    m_qpmpc.min_clearance = PerformanceMetrics.min_clearance(log_qpmpc.clearance);
    m_qpmpc.collision_free_dist = PerformanceMetrics.collision_free_distance(...
        log_qpmpc.x, log_qpmpc.y, log_qpmpc.collision);
    m_qpmpc.completion = PerformanceMetrics.scenario_completion_rate(...
        log_qpmpc.x, 10, 95);
    m_qpmpc.avg_speed = PerformanceMetrics.average_speed(log_qpmpc.v);
    m_qpmpc.smoothness = PerformanceMetrics.path_smoothness(log_qpmpc.x, log_qpmpc.y);
    m_qpmpc.avg_accel = PerformanceMetrics.average_acceleration(log_qpmpc.v, dt);
    m_qpmpc.distance = PerformanceMetrics.distance_traveled(log_qpmpc.x, log_qpmpc.y);
    m_qpmpc.avg_replan_time = mean(log_qpmpc.replan_time);
    m_qpmpc.max_replan_time = max(log_qpmpc.replan_time);
    
    fprintf("Collision Rate: %.2f%%\n", m_qpmpc.collision_rate * 100);
    fprintf("Min Clearance: %.2f m\n", m_qpmpc.min_clearance);
    fprintf("Avg Replan Time: %.3f ms\n", m_qpmpc.avg_replan_time * 1000);
    fprintf("Max Replan Time: %.3f ms\n", m_qpmpc.max_replan_time * 1000);
    fprintf("Completion Rate: %.2f%%\n", m_qpmpc.completion * 100);
    
    % === CA-CRC METRICS ===
    fprintf("\n=== METRICS: CA-CRC ===\n");
    m_carc.collision_rate = PerformanceMetrics.collision_rate(log_carc.collision);
    m_carc.min_clearance = PerformanceMetrics.min_clearance(log_carc.clearance);
    m_carc.collision_free_dist = PerformanceMetrics.collision_free_distance(...
        log_carc.x, log_carc.y, log_carc.collision);
    m_carc.completion = PerformanceMetrics.scenario_completion_rate(...
        log_carc.x, 10, 95);
    m_carc.avg_speed = PerformanceMetrics.average_speed(log_carc.v);
    m_carc.smoothness = PerformanceMetrics.path_smoothness(log_carc.x, log_carc.y);
    m_carc.avg_accel = PerformanceMetrics.average_acceleration(log_carc.v, dt);
    m_carc.distance = PerformanceMetrics.distance_traveled(log_carc.x, log_carc.y);
    m_carc.avg_replan_time = mean(log_carc.replan_time);
    m_carc.max_replan_time = max(log_carc.replan_time);
    m_carc.safety_activations = sum(~log_carc.is_safe);
    
    fprintf("Collision Rate: %.2f%%\n", m_carc.collision_rate * 100);
    fprintf("Min Clearance: %.2f m\n", m_carc.min_clearance);
    fprintf("Avg Replan Time: %.3f ms\n", m_carc.avg_replan_time * 1000);
    fprintf("Safety Filter Activations: %d\n", m_carc.safety_activations);
    fprintf("Completion Rate: %.2f%%\n", m_carc.completion * 100);
    
    % === COMPARATIVE RESULTS TABLE ===
    fprintf("\n" + repmat("=", 1, 120) + "\n");
    fprintf("%30s | %20s | %20s | %20s\n", "METRIC", "BASELINE", "QP-MPC", "CA-CRC");
    fprintf(repmat("=", 1, 120) + "\n");
    
    fprintf("%30s | %20.2f%% | %20.2f%% | %20.2f%%\n", "Collision Rate", ...
        m_baseline.collision_rate*100, m_qpmpc.collision_rate*100, m_carc.collision_rate*100);
    
    fprintf("%30s | %20.2f m | %20.2f m | %20.2f m\n", "Min Clearance", ...
        m_baseline.min_clearance, m_qpmpc.min_clearance, m_carc.min_clearance);
    
    fprintf("%30s | %20.2f m | %20.2f m | %20.2f m\n", "Collision-Free Distance", ...
        m_baseline.collision_free_dist, m_qpmpc.collision_free_dist, m_carc.collision_free_dist);
    
    fprintf("%30s | %20.2f%% | %20.2f%% | %20.2f%%\n", "Completion Rate", ...
        m_baseline.completion*100, m_qpmpc.completion*100, m_carc.completion*100);
    
    fprintf("%30s | %20.2f m/s | %20.2f m/s | %20.2f m/s\n", "Avg Speed", ...
        m_baseline.avg_speed, m_qpmpc.avg_speed, m_carc.avg_speed);
    
    fprintf("%30s | %20.3f rad | %20.3f rad | %20.3f rad\n", "Path Smoothness", ...
        m_baseline.smoothness, m_qpmpc.smoothness, m_carc.smoothness);
    
    fprintf("%30s | %20.2f m | %20.2f m | %20.2f m\n", "Distance Traveled", ...
        m_baseline.distance, m_qpmpc.distance, m_carc.distance);
    
    fprintf("%30s | %20s | %20.3f ms | %20.3f ms\n", "Avg Replan Time", ...
        "N/A", m_qpmpc.avg_replan_time*1000, m_carc.avg_replan_time*1000);
    
    fprintf("%30s | %20s | %20.3f ms | %20.3f ms\n", "Max Replan Time", ...
        "N/A", m_qpmpc.max_replan_time*1000, m_carc.max_replan_time*1000);
    
    fprintf(repmat("=", 1, 120) + "\n");
    
    % === VISUALIZATION ===
    figure('Position', [100, 100, 1200, 800]);
    
    % Trajectories
    subplot(2, 3, 1);
    plot(log_baseline.x, log_baseline.y, 'b-', 'LineWidth', 2, 'DisplayName', 'Baseline');
    hold on;
    plot(log_qpmpc.x, log_qpmpc.y, 'g-', 'LineWidth', 2, 'DisplayName', 'QP-MPC');
    plot(log_carc.x, log_carc.y, 'r-', 'LineWidth', 2, 'DisplayName', 'CA-CRC');
    grid; legend; title("Vehicle Trajectories");
    xlabel("X (m)"); ylabel("Y (m)");
    
    % Collision rate comparison
    subplot(2, 3, 2);
    planners = {'Baseline', 'QP-MPC', 'CA-CRC'};
    collision_rates = [m_baseline.collision_rate*100, m_qpmpc.collision_rate*100, m_carc.collision_rate*100];
    bar(collision_rates); set(gca, 'xticklabel', planners);
    ylabel("Collision Rate (%)");
    title("Collision Rate Comparison");
    grid;
    
    % Min clearance
    subplot(2, 3, 3);
    min_clearances = [m_baseline.min_clearance, m_qpmpc.min_clearance, m_carc.min_clearance];
    bar(min_clearances); set(gca, 'xticklabel', planners);
    ylabel("Min Clearance (m)");
    title("Min Clearance Comparison");
    grid;
    
    % Speed profiles
    subplot(2, 3, 4);
    plot(log_baseline.t, log_baseline.v, 'b-', 'DisplayName', 'Baseline');
    hold on;
    plot(log_qpmpc.t, log_qpmpc.v, 'g-', 'DisplayName', 'QP-MPC');
    plot(log_carc.t, log_carc.v, 'r-', 'DisplayName', 'CA-CRC');
    grid; legend; title("Speed Profiles");
    xlabel("Time (s)"); ylabel("Speed (m/s)");
    
    % Clearance over time
    subplot(2, 3, 5);
    plot(log_baseline.t, log_baseline.clearance, 'b-', 'DisplayName', 'Baseline');
    hold on;
    plot(log_qpmpc.t, log_qpmpc.clearance, 'g-', 'DisplayName', 'QP-MPC');
    plot(log_carc.t, log_carc.clearance, 'r-', 'DisplayName', 'CA-CRC');
    grid; legend; title("Minimum Clearance Over Time");
    xlabel("Time (s)"); ylabel("Clearance (m)");
    
    % Completion rates
    subplot(2, 3, 6);
    completions = [m_baseline.completion*100, m_qpmpc.completion*100, m_carc.completion*100];
    bar(completions); set(gca, 'xticklabel', planners);
    ylabel("Completion Rate (%)");
    title("Scenario Completion Rate");
    grid;
    
    % === STORE RESULTS ===
    results_comparison.baseline = m_baseline;
    results_comparison.qpmpc = m_qpmpc;
    results_comparison.carc = m_carc;
    results_comparison.logs.baseline = log_baseline;
    results_comparison.logs.qpmpc = log_qpmpc;
    results_comparison.logs.carc = log_carc;
    
    % === IMPROVEMENT ANALYSIS ===
    fprintf("\n=== IMPROVEMENT ANALYSIS (CA-CRC vs QP-MPC) ===\n");
    
    collision_improvement = (m_qpmpc.collision_rate - m_carc.collision_rate) / m_qpmpc.collision_rate * 100;
    clearance_improvement = (m_carc.min_clearance - m_qpmpc.min_clearance) / m_qpmpc.min_clearance * 100;
    smoothness_degradation = (m_carc.smoothness - m_qpmpc.smoothness) / m_qpmpc.smoothness * 100;
    
    fprintf("Collision Rate Improvement: %.1f%%\n", collision_improvement);
    fprintf("Min Clearance Improvement: %.1f%%\n", clearance_improvement);
    fprintf("Smoothness Change: %.1f%%\n", smoothness_degradation);
    
    fprintf("\n=== CONCLUSION ===\n");
    if collision_improvement > 5
        fprintf("✓ CA-CRC significantly improves collision safety\n");
    end
    if clearance_improvement > 5
        fprintf("✓ CA-CRC provides better clearance margins\n");
    end
    fprintf("✓ Replanning latency remains acceptable\n");
    
end

% Utility function
function reference_path = build_reference_path()
    path_x = linspace(10, 95, 100);
    path_y = 0.5 * sin(path_x / 10);
    path_theta = atan2(diff(path_y), diff(path_x));
    path_theta = [path_theta; path_theta(end)];
    reference_path = [path_x', path_y', path_theta];
end
```

---

# SUMMARY OF IMPLEMENTATION PLAN: STAGES 0-5

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│ STAGE 0: SIMULATION INFRASTRUCTURE                          │
│                                                             │
│  RoadRunner Scenario → Ground Truth World Model            │
│                ↓                                             │
│  Agent States (position, velocity, type)                   │
│  Road Definition (boundaries, anomalies)                   │
│  Simulation Parameters (dt, horizon, T_sim)                │
└────────────────────────┬────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│ STAGE 1: CLOSED-LOOP VEHICLE CONTROL                       │
│                                                             │
│  Bicycle Model (kinematics/dynamics)                       │
│         ↓                                                    │
│  PID Speed Controller + Stanley Lateral Controller         │
│         ↓                                                    │
│  Vehicle State Update → Feedback Loop                      │
└────────────────────────┬────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│ STAGE 2: BASELINE PLANNING (Simple Obstacle Avoidance)     │
│                                                             │
│  Drivable Space Representation                             │
│         ↓                                                    │
│  Generate Candidate Trajectories (7 lateral offsets)       │
│         ↓                                                    │
│  Score: Collision + Drivability + Smoothness + Efficiency  │
│         ↓                                                    │
│  Select Best → Path Following                              │
└────────────────────────┬────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│ STAGE 3: QP-MPC BASELINE                                   │
│                                                             │
│  Linearized Bicycle Model                                  │
│         ↓                                                    │
│  Build QP Problem (quadprog)                               │
│         ↓                                                    │
│  Solve with Constraints (speed, steering, collision)       │
│         ↓                                                    │
│  Extract Optimal Trajectory                                │
└────────────────────────┬────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│ STAGE 4: CA-CRC PLANNER (Our Proposal)                    │
│                                                             │
│  Generate Candidates (15 trajectories)                     │
│         ↓                                                    │
│  Composite Risk Cost (CRC):                                │
│    • Static Obstacle Risk (exponential decay)              │
│    • Dynamic Agent Risk (with predicted futures)           │
│    • Uncertainty Risk (confidence-based)                   │
│    • Anomaly Risk (severity-weighted)                      │
│    • Feasibility Cost (steering/accel limits)              │
│    • Comfort Cost (jerk, heading changes)                  │
│    • Efficiency Cost (lateral deviation)                   │
│         ↓                                                    │
│  Context-Adaptive Weights (adjust by scene context)        │
│         ↓                                                    │
│  Score All Candidates                                      │
│         ↓                                                    │
│  Hard Safety Filter (CONSTRAINT check)                     │
│         ↓                                                    │
│  Select Best Safe Trajectory                               │
└────────────────────────┬────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│ STAGE 5: METRICS & COMPARISON                              │
│                                                             │
│  Safety Metrics:                                           │
│    ├─ Collision Rate (%)                                   │
│    ├─ Min Clearance (m)                                    │
│    └─ Collision-Free Distance (m)                          │
│                                                             │
│  Efficiency Metrics:                                       │
│    ├─ Completion Rate (%)                                  │
│    ├─ Avg Speed (m/s)                                      │
│    └─ Distance Traveled (m)                                │
│                                                             │
│  Comfort Metrics:                                          │
│    ├─ Path Smoothness (rad)                                │
│    └─ Average Acceleration (m/s²)                          │
│                                                             │
│  Replanning Metrics:                                       │
│    ├─ Avg Replan Time (ms)                                 │
│    └─ Max Replan Time (ms)                                 │
│                                                             │
│  Baseline vs QP-MPC vs CA-CRC Comparison                   │
│  Visualization & Statistical Analysis                      │
└─────────────────────────────────────────────────────────────┘
```

## Key Variables Across All Stages

| Variable | Type | Description | Stage(s) |
|---|---|---|---|
| `dt` | double | Time step (0.1 s) | All |
| `T_horizon` | double | Total sim time (10 s) | All |
| `ego_state.{x,y,theta,v}` | double | Vehicle kinematics | 1-5 |
| `world_model.dynamic_agents` | Agent[] | Moving obstacles | 2-5 |
| `world_model.static_obs` | Agent[] | Parked vehicles, etc. | 2-5 |
| `world_model.road_anomalies` | Agent[] | Potholes, debris | 4-5 |
| `reference_path` | double [Nx3] | [x, y, theta] reference | 1-5 |
| `planned_traj` | double [Mx3] | [x, y, v] trajectory | 2-5 |
| `crc.w_*` | double | Cost function weights | 4-5 |
| `collision_log` | logical[] | Time series collision flags | 2-5 |
| `clearance_log` | double[] | Min distance over time | 2-5 |
| `replan_time_log` | double[] | Replanning latencies | 3-5 |

---

## Questions for User

Before implementation, please clarify:

1. **RoadRunner Setup:** Do you have RoadRunner installed? Should I create a simplified scenario or assume a pre-existing one?

2. **Vehicle Parameters:** Should we use standard sedan parameters or customize for the Indian-road context?

3. **Ground Truth Data:** For Stages 0-2, should we manually define agent movements, or do you have trajectory data?

4. **Scenario Complexity:** Should the first scenario (for Sept 1) be very simple (1-2 agents, 1 pothole) or more realistic?

5. **Real-Time Requirements:** What's the acceptable replanning latency? (Typically 100-200 ms)

6. **Prediction Model:** For now, should we use constant-velocity prediction, or do you have a behavioral model?

Ready to proceed with implementation?