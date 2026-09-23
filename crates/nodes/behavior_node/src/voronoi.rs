use coordinate_systems::{Field, Ground};
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Pose2, point, vector};
use ordered_float::NotNan;
use types::{behavior_tree::Status, obstacles::Obstacle, rule_obstacles::RuleObstacle};
use voronoi::{Ownership, VoronoiGrid};

use crate::node::Blackboard;

pub fn calculate_voronoi_grid(blackboard: &mut Blackboard) -> Status {
    // Both behavior runtimes clear the map before each tree tick. Reuse it if
    // another branch requests the same tick's map.
    if blackboard.voronoi_map.is_some() {
        return Status::Success;
    }

    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field {
        let sites = collect_sites(blackboard, ground_to_field.as_pose());
        for (pose, _) in &sites {
            blackboard.voronoi_inputs.push(*pose);
        }

        blackboard.voronoi_map = Some(build_grid(blackboard, ground_to_field, &sites));
        Status::Success
    } else {
        Status::Failure
    }
}

fn build_grid(
    blackboard: &Blackboard,
    ground_to_field: Isometry2<Ground, Field>,
    sites: &[(Pose2<Field>, PlayerNumber)],
) -> VoronoiGrid {
    let length_half = blackboard.field_dimensions.length / 2.0;
    let width_half = blackboard.field_dimensions.width / 2.0;
    let parameters = &blackboard.parameters.voronoi;
    let padding = parameters.padding;
    let mut map = VoronoiGrid::new(
        point!(-length_half - padding, -width_half - padding),
        point!(length_half + padding, width_half + padding),
        parameters.grid_resolution,
    );
    // Ownership distances describe the robot center, so reserve its walking footprint.
    let robot_radius = blackboard.parameters.walking.path_planning.robot_radius;
    let obstacles: Vec<_> = blackboard
        .world_state
        .obstacles
        .iter()
        .map(|obstacle| Obstacle {
            radius_at_hip_height: obstacle.radius_at_hip_height + robot_radius,
            radius_at_foot_height: obstacle.radius_at_foot_height + robot_radius,
            ..*obstacle
        })
        .collect();
    let rule_obstacles: Vec<_> = blackboard
        .world_state
        .rule_obstacles
        .iter()
        .copied()
        .map(|mut obstacle| {
            match &mut obstacle {
                RuleObstacle::Circle(circle) => circle.radius += robot_radius,
                RuleObstacle::Rectangle(rectangle) => {
                    let margin = vector!(robot_radius, robot_radius);
                    rectangle.min -= margin;
                    rectangle.max += margin;
                }
            }
            obstacle
        })
        .collect();
    map.initialize_obstacles(&obstacles, &rule_obstacles, ground_to_field);
    map.multi_source_dijkstra(sites);
    map
}

pub(crate) fn closest_player_to_ball(blackboard: &Blackboard) -> Option<PlayerNumber> {
    let ball = blackboard.ball.as_ref()?;
    let map = blackboard.voronoi_map.as_ref()?;
    match map.ownership_at(ball.position)? {
        Ownership::Robot(player) => Some(player),
        Ownership::Blocked => {
            let robot_pose = blackboard.world_state.robot.ground_to_field?.as_pose();
            // A blocked ball has no path-distance label. Compare the actual sites rather
            // than choosing the owner of an arbitrarily selected obstacle boundary cell.
            collect_sites(blackboard, robot_pose)
                .into_iter()
                .filter_map(|(pose, player)| {
                    let distance =
                        NotNan::new((pose.position() - ball.position).norm_squared()).ok()?;
                    (distance.is_finite() && map.ownership_at(pose.position()).is_some())
                        .then_some((distance, player))
                })
                .min()
                .map(|(_, player)| player)
        }
        Ownership::Free => None,
    }
}

fn collect_sites(
    blackboard: &Blackboard,
    robot_pose: Pose2<Field>,
) -> Vec<(Pose2<Field>, PlayerNumber)> {
    let robot_player_number = blackboard.world_state.robot.player_number;
    let mut sites = vec![(robot_pose, robot_player_number)];

    for (player_number, player_state) in blackboard.world_state.player_states.iter() {
        if let Some(player_state) = player_state
            && player_number != robot_player_number
        {
            sites.push((player_state.pose, player_number));
        }
    }

    sites
}

#[cfg(test)]
mod tests {
    use geometry::circle::Circle;
    use linear_algebra::Isometry2;
    use types::{rule_obstacles::RuleObstacle, world_state::PlayerState};

    use crate::test_utils::{ball_at, blackboard};

    use super::*;

    #[test]
    fn ball_ownership_depends_on_sites_not_goalkeeper_role() {
        for own_player in [PlayerNumber::One, PlayerNumber::Two] {
            for goalkeeper in [PlayerNumber::One, PlayerNumber::Two] {
                for blocked in [false, true] {
                    let mut blackboard = blackboard();
                    blackboard.parameters.goalkeeper.player_number = goalkeeper;
                    blackboard.world_state.robot.player_number = own_player;
                    for (player, position) in [
                        (PlayerNumber::One, point!(-4.2, 0.0)),
                        (PlayerNumber::Two, point!(4.0, 0.0)),
                    ] {
                        blackboard.world_state.player_states[player] = Some(PlayerState {
                            pose: Pose2::from(position),
                            ball_position: None,
                        });
                        if player == own_player {
                            blackboard.world_state.robot.ground_to_field =
                                Some(Isometry2::from(position));
                        }
                    }
                    blackboard.ball = Some(ball_at(point!(-0.8, 0.0)));
                    if blocked {
                        blackboard.world_state.rule_obstacles =
                            vec![RuleObstacle::Circle(Circle::new(point!(-0.8, 0.0), 0.75))];
                    }
                    assert_eq!(calculate_voronoi_grid(&mut blackboard), Status::Success);

                    assert_eq!(
                        closest_player_to_ball(&blackboard),
                        Some(PlayerNumber::One),
                        "own_player={own_player:?}, goalkeeper={goalkeeper:?}, blocked={blocked}"
                    );
                }
            }
        }
    }

    #[test]
    fn repeated_requests_reuse_map_until_tick_reset() {
        let mut blackboard = blackboard();
        assert_eq!(calculate_voronoi_grid(&mut blackboard), Status::Success);
        let initial_map = blackboard.voronoi_map.clone();
        let initial_inputs = blackboard.voronoi_inputs.clone();

        // Changing an input makes an unintended same-tick recomputation observable.
        blackboard.world_state.player_states[PlayerNumber::Three] = Some(PlayerState {
            pose: Pose2::from(point!(2.0, 0.0)),
            ball_position: None,
        });
        assert_eq!(calculate_voronoi_grid(&mut blackboard), Status::Success);
        assert!(
            blackboard.voronoi_map == initial_map,
            "repeated request rebuilt the cached map"
        );
        assert_eq!(blackboard.voronoi_inputs, initial_inputs);

        // Both behavior runtimes reset these outputs before the next tree tick.
        blackboard.voronoi_map = None;
        blackboard.voronoi_inputs.clear();
        assert_eq!(calculate_voronoi_grid(&mut blackboard), Status::Success);
        assert!(
            blackboard.voronoi_map != initial_map,
            "tick reset did not refresh the map"
        );
        assert_eq!(blackboard.voronoi_inputs.len(), 2);
        assert_eq!(
            blackboard
                .voronoi_map
                .as_ref()
                .unwrap()
                .ownership_at(point!(2.0, 0.0)),
            Some(Ownership::Robot(PlayerNumber::Three))
        );
    }

    #[test]
    fn missing_localization_does_not_populate_cache() {
        let mut blackboard = blackboard();
        blackboard.world_state.robot.ground_to_field = None;
        assert_eq!(calculate_voronoi_grid(&mut blackboard), Status::Failure);
        assert!(blackboard.voronoi_map.is_none());
        assert!(blackboard.voronoi_inputs.is_empty());

        blackboard.world_state.robot.ground_to_field = Some(Isometry2::identity());
        assert_eq!(calculate_voronoi_grid(&mut blackboard), Status::Success);
        assert!(blackboard.voronoi_map.is_some());
        assert_eq!(blackboard.voronoi_inputs.len(), 1);
    }

    #[test]
    fn grid_reserves_robot_clearance_around_physical_and_rule_obstacles() {
        use geometry::rectangle::Rectangle;
        use types::obstacles::Obstacle;

        let mut blackboard = blackboard();
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        blackboard.world_state.obstacles = vec![Obstacle::ball(point!(0.0, 0.0), 0.2)];
        blackboard.world_state.rule_obstacles = vec![
            RuleObstacle::Circle(Circle::new(point!(2.0, 0.0), 0.3)),
            RuleObstacle::Rectangle(Rectangle {
                min: point!(-3.0, -0.5),
                max: point!(-2.0, 0.5),
            }),
        ];
        calculate_voronoi_grid(&mut blackboard);
        let grid = blackboard.voronoi_map.as_ref().unwrap();

        for point in [point!(0.4, 0.0), point!(2.6, 0.0), point!(-1.8, 0.0)] {
            assert_eq!(
                grid.ownership_at(point),
                Some(Ownership::Blocked),
                "{point:?}"
            );
        }
        assert_eq!(
            grid.ownership_at(point!(0.0, 1.0)),
            Some(Ownership::Robot(PlayerNumber::Two))
        );
    }
}
