% TEST_CLEAN_ARCHITECTURE Validates class instantiation in decoupled architecture
clear; clc;
fprintf('=== RUNNING CLEAN ARCHITECTURE PATH & INSTANTIATION TEST ===\n');

% Configure paths
setup_paths();

% Test Config
cfg = SimulationConfig();
fprintf('✓ SimulationConfig initialized\n');

% Test Simulation Environment Layer
world = WorldState(cfg);
fprintf('✓ WorldState initialized (world/)\n');

scen_gen = ScenarioGenerator();
fprintf('✓ ScenarioGenerator initialized (scenarios/)\n');

traf_gen = StochasticTrafficGenerator(42);
fprintf('✓ StochasticTrafficGenerator initialized (traffic/)\n');

obs_mod = ObservationModel('ideal', 42);
fprintf('✓ ObservationModel initialized (sensors/)\n');

plant = BicycleModel(cfg);
fprintf('✓ BicycleModel initialized (plant/)\n');

% Test Autonomy Pipeline Layer
ctrl = Stage5CoordinationController(cfg);
fprintf('✓ Stage5CoordinationController initialized (autonomy_pipeline/)\n');

planner = QPMPCPlanner(cfg);
fprintf('✓ QPMPCPlanner initialized (optimization_qp/)\n');

cacrc = CACRCPlanner(cfg);
fprintf('✓ CACRCPlanner initialized (planning/)\n');

sf = SafetyFilter(cfg);
fprintf('✓ SafetyFilter initialized (safety/)\n');

dec_layer = CoordinationDecisionLayer(8.0, 1.5, 5.0);
fprintf('✓ CoordinationDecisionLayer initialized (decision/)\n');

det = MultiVehicleDetector.detect(world, world.ego);
fprintf('✓ MultiVehicleDetector executed (perception_tracking/)\n');

fprintf('\n>>> ALL 11 ARCHITECTURAL SUBSYSTEMS VERIFIED SUCCESSFULLY! <<<\n');
