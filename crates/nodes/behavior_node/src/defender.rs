use coordinate_systems::Field;
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Orientation2, Point2, Pose2, point};
use types::{behavior_tree::Status, motion_command::OrientationMode};
use voronoi::Ownership;

use crate::{node::Blackboard, walk::walk_to};

pub fn walk_to_defender_position(blackboard: &mut Blackboard) -> Status {
    let Some(position) = cover_position(blackboard) else {
        blackboard.defending_player = None;
        return Status::Failure;
    };
    blackboard.defending_player = select_defender(blackboard, position);
    if blackboard.defending_player != Some(blackboard.player_number) {
        return Status::Failure;
    }
    if let (Some(ground_to_field), Some(ball), Some(target)) = (
        blackboard.world_state.robot.ground_to_field,
        blackboard.ball.as_ref(),
        defensive_target(blackboard, position),
    ) {
        let field_to_ground = ground_to_field.inverse();
        let pose = Pose2::from_parts(target, Orientation2::from_vector(ball.position - target));
        let orientation_mode = OrientationMode::LookAt {
            target: field_to_ground * ball.position,
            tolerance: blackboard
                .parameters
                .walking
                .walk_and_stand
                .orientation_tolerance,
        };
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        walk_to(
            blackboard,
            field_to_ground * pose,
            blackboard.parameters.walking.speed.kicking,
            orientation_mode,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.hysteresis,
        )
    } else {
        Status::Failure
    }
}

fn select_defender(blackboard: &Blackboard, position: Point2<Field>) -> Option<PlayerNumber> {
    let ball = blackboard.ball.as_ref()?.position;
    let map = blackboard.voronoi_map.as_ref()?;
    let ownership = match map.ownership_at(ball)? {
        Ownership::Blocked => map.nearest_non_blocked_ownership(ball)?,
        ownership => ownership,
    };
    let Ownership::Robot(ball_owner) = ownership else {
        return None;
    };

    let mut positions = blackboard
        .world_state
        .player_states
        .as_ref()
        .map(|state| state.as_ref().map(|state| state.pose.position()));
    positions[blackboard.player_number] =
        Some(blackboard.world_state.robot.ground_to_field?.translation());
    let goalkeeper = blackboard.parameters.goalkeeper.player_number;
    let distance = |player: PlayerNumber| {
        if player == goalkeeper || player == ball_owner {
            return None;
        }
        positions[player].map(|player_position| (player_position - position).norm())
    };
    let (best_player, best_distance) = positions
        .iter()
        .filter_map(|(player, _)| distance(player).map(|distance| (player, distance)))
        .min_by(
            |(left_player, left_distance), (right_player, right_distance)| {
                left_distance
                    .total_cmp(right_distance)
                    .then_with(|| left_player.cmp(right_player))
            },
        )?;

    let switch_margin = blackboard.parameters.defender.relative_switch_hysteresis
        * blackboard.field_dimensions.width;
    if let Some(previous) = blackboard.defending_player
        && let Some(previous_distance) = distance(previous)
        && previous_distance <= best_distance + switch_margin
    {
        return Some(previous);
    }
    Some(best_player)
}

fn cover_position(blackboard: &Blackboard) -> Option<Point2<Field>> {
    let ball = blackboard.ball.as_ref()?.position;
    let field = &blackboard.field_dimensions;
    let half_length = field.length / 2.0;
    let goal_to_ball_x = ball.x() + half_length;
    if goal_to_ball_x <= 0.0 {
        return None;
    }
    let x = ((ball.x() - half_length) / 2.0)
        .min(blackboard.parameters.defender.relative_maximum_x * half_length)
        .max(-half_length + field.goal_box_area_length);
    let fraction = ((x + half_length) / goal_to_ball_x).clamp(0.0, 1.0);
    Some(point![x, ball.y() * fraction])
}

fn defensive_target(blackboard: &Blackboard, position: Point2<Field>) -> Option<Point2<Field>> {
    let field = &blackboard.field_dimensions;
    let ground_to_field = blackboard.world_state.robot.ground_to_field?;
    let robot_radius = blackboard.parameters.walking.path_planning.robot_radius;
    let is_legal = |point: Point2<Field>| {
        point.x().abs() <= field.length / 2.0
            && point.y().abs() <= field.width / 2.0
            && blackboard
                .world_state
                .rule_obstacles
                .iter()
                .all(|obstacle| !obstacle.contains(point))
            && blackboard.world_state.obstacles.iter().all(|obstacle| {
                let radius = obstacle
                    .radius_at_hip_height
                    .max(obstacle.radius_at_foot_height)
                    + robot_radius;
                (point - ground_to_field * obstacle.position).norm_squared() > radius * radius
            })
    };
    is_legal(position).then_some(position)
}
