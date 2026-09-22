use coordinate_systems::Field;
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Pose2, point};
use ordered_float::NotNan;
use types::behavior_tree::Status;
use voronoi::{Ownership, VoronoiGrid};

use crate::node::Blackboard;

pub fn calculate_voronoi_grid(blackboard: &mut Blackboard) -> Status {
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field {
        let field_dimensions = &blackboard.field_dimensions;
        let voronoi_parameters = &blackboard.parameters.voronoi;
        let obstacles = &blackboard.world_state.obstacles;
        let rule_obstacles = &blackboard.world_state.rule_obstacles;

        let sites = collect_sites(blackboard, ground_to_field.as_pose());
        for (pose, _) in &sites {
            blackboard.voronoi_inputs.push(*pose);
        }

        let length_half = field_dimensions.length / 2.0;
        let width_half = field_dimensions.width / 2.0;
        let padding = voronoi_parameters.padding;

        let grid_min = point!(-length_half - padding, -width_half - padding);
        let grid_max = point!(length_half + padding, width_half + padding);

        let mut map = VoronoiGrid::new(grid_min, grid_max, voronoi_parameters.grid_resolution);
        map.initialize_obstacles(obstacles, rule_obstacles, ground_to_field);
        map.multi_source_dijkstra(&sites);
        blackboard.voronoi_map = Some(map);
        Status::Success
    } else {
        Status::Failure
    }
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
