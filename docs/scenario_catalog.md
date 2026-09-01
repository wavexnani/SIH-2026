# Phase 12D Standardized Scenario Catalog (10 Levels)

| Level | Name | Key | Description | Key Challenges |
|---|---|---|---|---|
| Level 1 | Nominal Straight | `clear` | Straight open road with ego initialized at centerline $y=1.80\text{m}$ | Baseline tracking precision under nominal conditions |
| Level 2 | Static Obstacle | `static` | Single static obstacle requiring lateral clearance | Static geometric avoidance |
| Level 3 | Road Geometry | `multi_obstacle_sequence` | Narrowing boundary sequence and obstacle choke points | Tight free-space corridor navigation |
| Level 4 | Dynamic Following | `multi_vehicle_following` | Slow lead vehicle decelerating dynamically at $t=3\text{s}$ | Longitudinal car-following and distance keeping |
| Level 5 | Yield Interaction | `multi_vehicle_yield_overtake` | Conflicting dynamic vehicle crossing ego lane | Multi-vehicle macro-intent arbitration and yielding |
| Level 6 | Overtaking | `overtaking` | Slow dynamic lead vehicle requiring lateral passing | High-speed lateral passing maneuver under narrow corridor |
| Level 7 | Oncoming Conflict | `multi_vehicle_oncoming_conflict` | High-speed opposing dynamic vehicle in adjacent lane | Dynamic clearance preservation under high relative velocity |
| Level 8 | Combined Environment | `complex` | Simultaneous road narrowing, static obstacle, and dynamic vehicle | Multi-modal spatio-temporal boundary constraints |
| Level 9 | Disturbed Environment | `complex` | Level 8 combined with 100 ms perception delay, noise, and +0.8° steering bias | Robustness under simultaneous perception and actuation uncertainties |
| Level 10 | Deliberately Infeasible | `impassable_center` | Total free-space collapse across drivable road width | Controlled safe-stop execution before barrier |
