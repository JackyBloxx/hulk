use coordinate_systems::{Field, Ground};
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Pose2, point};
use ordered_float::NotNan;
use types::behavior_tree::Status;
use voronoi::{Ownership, VoronoiGrid};

use crate::{goalkeeper::goalkeeper_can_pursue_ball, node::Blackboard};

pub fn calculate_voronoi_grid(blackboard: &mut Blackboard) -> Status {
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
    map.initialize_obstacles(
        &blackboard.world_state.obstacles,
        &blackboard.world_state.rule_obstacles,
        ground_to_field,
    );
    map.multi_source_dijkstra(sites);
    map
}

pub(crate) fn closest_player_to_ball(blackboard: &Blackboard) -> Option<PlayerNumber> {
    let ball = blackboard.ball.as_ref()?;
    let map = blackboard.voronoi_map.as_ref()?;
    match map.ownership_at(ball.position)? {
        Ownership::Robot(player)
            if player == blackboard.parameters.goalkeeper.player_number
                && !goalkeeper_can_pursue_ball(blackboard) =>
        {
            // Keep the goalkeeper in the coverage map, but re-elect among players
            // that can actually pursue this ball using the same obstacle distances.
            let ground_to_field = blackboard.world_state.robot.ground_to_field?;
            let sites = collect_eligible_sites(blackboard, ground_to_field.as_pose());
            let election_map = build_grid(blackboard, ground_to_field, &sites);
            match election_map.ownership_at(ball.position)? {
                Ownership::Robot(player) => Some(player),
                _ => None,
            }
        }
        Ownership::Robot(player) => Some(player),
        Ownership::Blocked => {
            let robot_pose = blackboard.world_state.robot.ground_to_field?.as_pose();
            // A blocked ball has no path-distance label. Compare the actual sites rather
            // than choosing the owner of an arbitrarily selected obstacle boundary cell.
            collect_eligible_sites(blackboard, robot_pose)
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

fn collect_eligible_sites(
    blackboard: &Blackboard,
    robot_pose: Pose2<Field>,
) -> Vec<(Pose2<Field>, PlayerNumber)> {
    let mut sites = collect_sites(blackboard, robot_pose);
    if !goalkeeper_can_pursue_ball(blackboard) {
        sites.retain(|(_, player)| *player != blackboard.parameters.goalkeeper.player_number);
    }
    sites
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
    use hsl_network_messages::{SubState, Team};
    use linear_algebra::Isometry2;
    use types::{
        filtered_game_controller_state::FilteredGameControllerState, rule_obstacles::RuleObstacle,
        world_state::PlayerState,
    };

    use crate::test_utils::{ball_at, blackboard};

    use super::*;

    #[test]
    fn ineligible_goalkeeper_keeps_coverage_but_not_ball_ownership() {
        for own_player in [PlayerNumber::One, PlayerNumber::Two] {
            let mut blackboard = blackboard();
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
                    blackboard.world_state.robot.ground_to_field = Some(Isometry2::from(position));
                }
            }
            blackboard.ball = Some(ball_at(point!(-0.8, 0.0)));
            calculate_voronoi_grid(&mut blackboard);

            assert_eq!(
                blackboard
                    .voronoi_map
                    .as_ref()
                    .unwrap()
                    .ownership_at(point!(-0.8, 0.0)),
                Some(Ownership::Robot(PlayerNumber::One))
            );
            assert_eq!(closest_player_to_ball(&blackboard), Some(PlayerNumber::Two));
        }
    }

    #[test]
    fn goalkeeper_election_respects_normal_and_set_play_eligibility() {
        let cases = [
            (-0.8, None, None, PlayerNumber::Two),
            (-2.0, None, None, PlayerNumber::One),
            (
                -0.8,
                Some(SubState::GoalKick),
                Some(Team::Hulks),
                PlayerNumber::One,
            ),
            (
                -2.0,
                Some(SubState::GoalKick),
                Some(Team::Opponent),
                PlayerNumber::Two,
            ),
            (
                -2.0,
                Some(SubState::CornerKick),
                Some(Team::Hulks),
                PlayerNumber::Two,
            ),
        ];
        for (ball_x, sub_state, kicking_team, expected) in cases {
            for blocked in [false, true] {
                let mut blackboard = blackboard();
                blackboard.world_state.robot.ground_to_field =
                    Some(Isometry2::from(point!(4.0, 0.0)));
                blackboard.world_state.player_states[PlayerNumber::One] = Some(PlayerState {
                    pose: Pose2::from(point!(-4.2, 0.0)),
                    ball_position: None,
                });
                blackboard.ball = Some(ball_at(point!(ball_x, 0.0)));
                blackboard.world_state.filtered_game_controller_state =
                    Some(FilteredGameControllerState {
                        sub_state,
                        kicking_team,
                        ..Default::default()
                    });
                if blocked {
                    blackboard.world_state.rule_obstacles =
                        vec![RuleObstacle::Circle(Circle::new(point!(ball_x, 0.0), 0.75))];
                }
                calculate_voronoi_grid(&mut blackboard);

                assert_eq!(
                    closest_player_to_ball(&blackboard),
                    Some(expected),
                    "ball_x={ball_x}, sub_state={sub_state:?}, team={kicking_team:?}, blocked={blocked}"
                );
            }
        }
    }
}
