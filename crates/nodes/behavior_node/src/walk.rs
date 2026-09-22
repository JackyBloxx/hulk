use coordinate_systems::{Field, Ground};
use filtering::hysteresis::less_than_with_relative_hysteresis;
use geometry::Distance;
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Orientation2, Point, Point2, Pose2, point};
use path_planner::path_planner::PathPlanner;
use types::{
    behavior_tree::Status,
    motion_command::{BodyMotion, MotionCommand, OrientationMode},
    motion_type::MotionType,
    path::{Path, direct_path},
    path_obstacles::PathObstacleShape,
};
use voronoi::{Ownership, VoronoiGrid};

use crate::{
    action,
    actions::stand,
    behavior_tree::Node,
    condition,
    conditions::hulks_is_kicking_team,
    kick::{kick, select_kick_target, use_last_kick_power},
    node::Blackboard,
    selection, sequence, subtree,
    switch_motion_type::{is_last_motion_type, switch_motion_type},
};

pub fn plan(
    blackboard: &mut Blackboard,
    target_in_ground: Point2<Ground>,
    ground_to_field: Isometry2<Ground, Field>,
) -> Path {
    let mut planner = create_path_planner(blackboard, ground_to_field);
    let field_dimensions = blackboard.field_dimensions;
    let target_in_field = ground_to_field * target_in_ground;
    let x_max = field_dimensions.length / 2.0 + field_dimensions.border_strip_width;
    let y_max = field_dimensions.width / 2.0 + field_dimensions.border_strip_width;
    let clamped_target_in_robot = ground_to_field.inverse()
        * point![
            target_in_field.x().clamp(-x_max, x_max),
            target_in_field.y().clamp(-y_max, y_max)
        ];

    let path = planner
        .plan(Point::origin(), clamped_target_in_robot)
        .unwrap();
    blackboard.path_obstacles_output = planner.obstacles;
    path.unwrap_or_else(|| direct_path(Point::origin(), target_in_ground))
}

fn create_path_planner(
    blackboard: &Blackboard,
    ground_to_field: Isometry2<Ground, Field>,
) -> PathPlanner {
    let parameters: &types::parameters::PathPlanningParameters =
        &blackboard.parameters.walking.path_planning;
    let field_dimensions = blackboard.field_dimensions;

    let mut planner = PathPlanner {
        obstacle_escape_spline_segments: parameters.obstacle_escape_spline_segments,
        ..Default::default()
    };
    planner.with_last_motion(
        &blackboard.last_motion_command,
        parameters.rotation_penalty_factor,
    );
    planner.with_obstacles(&blackboard.world_state.obstacles, parameters.robot_radius);
    planner.with_rule_obstacles(
        ground_to_field.inverse(),
        &blackboard.world_state.rule_obstacles,
        parameters.robot_radius,
    );
    planner.with_field_borders(
        ground_to_field,
        field_dimensions.length,
        field_dimensions.width,
        field_dimensions.border_strip_width,
        parameters.field_border_weight,
    );
    planner.with_goal_support_structures(ground_to_field.inverse(), &field_dimensions);
    let ball_obstacle = blackboard.world_state.ball.map(|ball| ball.ball_in_ground);

    if let Some(ball_position) = ball_obstacle {
        planner.with_ball(
            ball_position,
            parameters.ball_obstacle_radius,
            parameters.robot_radius,
        );
    }

    planner
}

pub fn walk_to(
    blackboard: &mut Blackboard,
    target_pose: Pose2<Ground>,
    maximal_walk_speed: f32,
    orientation_mode: OrientationMode,
    distance_to_be_aligned: f32,
    hysteresis: nalgebra::Vector2<f32>,
) -> Status {
    if let Some(ground_to_field) = blackboard.world_state.robot.ground_to_field {
        let parameters = &blackboard.parameters.walking.walk_and_stand;
        let distance_to_walk = target_pose.position().coords().norm();
        let angle_to_walk = target_pose.orientation().angle();
        let was_standing_last_cycle =
            matches!(blackboard.last_motion_command, MotionCommand::Stand { .. });
        let is_reached = less_than_with_relative_hysteresis(
            was_standing_last_cycle,
            distance_to_walk,
            parameters.target_reached_thresholds.x,
            0.0..=hysteresis.x,
        ) && less_than_with_relative_hysteresis(
            was_standing_last_cycle,
            angle_to_walk.abs(),
            parameters.target_reached_thresholds.y,
            0.0..=hysteresis.y,
        );

        let minimal_walk_speed = blackboard.parameters.walking.speed.minimum_speed;
        let velocity_fade_distance = blackboard.parameters.walking.speed.velocity_fade_distance;

        // Desmos: https://www.desmos.com/calculator/ss94dje2ke
        let walk_speed = maximal_walk_speed
            - (maximal_walk_speed - minimal_walk_speed)
                * (-(2.0 * distance_to_walk / velocity_fade_distance).powf(2.0)).exp();

        if is_reached {
            blackboard.body_motion = Some(BodyMotion::Stand);
            Status::Success
        } else {
            let path = plan(blackboard, target_pose.position(), ground_to_field);
            blackboard.body_motion = Some(BodyMotion::Walk {
                path,
                orientation_mode,
                target_orientation: target_pose.orientation(),
                distance_to_be_aligned,
                speed: walk_speed,
            });
            Status::Success
        }
    } else {
        Status::Failure
    }
}

pub fn walk_to_ball(blackboard: &mut Blackboard) -> Status {
    if let (Some(ball), Some(ground_to_field)) = (
        &blackboard.last_ball,
        &blackboard.world_state.robot.ground_to_field,
    ) {
        let field_to_ground = ground_to_field.inverse();
        let ball_in_ground = field_to_ground * ball.position;
        let goal_position = field_to_ground * point!(blackboard.field_dimensions.length / 2.0, 0.0);
        let orientation = Orientation2::from_vector(goal_position - ball_in_ground);
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let kicking_speed = blackboard.parameters.walking.speed.kicking;

        let target_position = ball_in_ground
            - (goal_position - ball_in_ground).normalize()
                * blackboard.parameters.kicking.kick_position_ball_distance;
        walk_to(
            blackboard,
            Pose2::from_parts(target_position, orientation),
            kicking_speed,
            OrientationMode::AlignWithPath,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.hysteresis,
        )
    } else {
        Status::Failure
    }
}

pub fn walk_to_ball_subtree() -> Node<Blackboard> {
    switch_motion_type(
        MotionType::Walk,
        action!(walk_to_ball),
        subtree!(walk_alternatives_subtree),
    )
}

pub fn walk_alternatives_subtree() -> Node<Blackboard> {
    selection!(
        sequence!(
            condition!(is_last_motion_type, MotionType::Kick),
            sequence!(
                action!(kick),
                action!(select_kick_target),
                action!(use_last_kick_power),
            )
        ),
        action!(stand)
    )
}

pub fn walk_to_block_position(blackboard: &mut Blackboard) -> Status {
    if let (Some(block_position), Some(ball), Some(ground_to_field)) = (
        &blackboard.walk_position,
        &blackboard.last_ball,
        blackboard.world_state.robot.ground_to_field,
    ) {
        let ball_position = ground_to_field.inverse() * ball.position;
        let orientation = Orientation2::from_vector(ball_position - *block_position);
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let blocking_speed = blackboard.parameters.walking.speed.blocking;

        walk_to(
            blackboard,
            Pose2::from_parts(*block_position, orientation),
            blocking_speed,
            OrientationMode::LookAt {
                target: ball_position,
                tolerance: walk_and_stand.orientation_tolerance,
            },
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.goalkeeper_hysteresis,
        )
    } else {
        Status::Failure
    }
}

pub fn walk_to_kickoff_pose(blackboard: &mut Blackboard) -> Status {
    if let (Some(ground_to_field), player_number) = (
        blackboard.world_state.robot.ground_to_field,
        blackboard.world_state.robot.player_number,
    ) {
        let field_to_ground = ground_to_field.inverse();
        let kickoff = &blackboard.parameters.kickoff;
        let standard_pose = kickoff.standard_positions[player_number];
        let striker_position = kickoff.striker_position;
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let walk_to_kickoff_speed = blackboard.parameters.walking.speed.walk_to_kickoff;

        let mut target_position = standard_pose.position;

        if hulks_is_kicking_team(blackboard) && player_number == PlayerNumber::Three {
            target_position = striker_position;
        }

        let kickoff_pose_in_field =
            Pose2::from_parts(target_position, Orientation2::new(standard_pose.rotation));

        let kickoff_pose_in_ground = field_to_ground * kickoff_pose_in_field;

        walk_to(
            blackboard,
            kickoff_pose_in_ground,
            walk_to_kickoff_speed,
            OrientationMode::AlignWithPath,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.hysteresis,
        );
        Status::Success
    } else {
        Status::Failure
    }
}

pub fn walk_to_voronoi_position(blackboard: &mut Blackboard) -> Status {
    if let (Some(ground_to_field), Some(map)) = (
        blackboard.world_state.robot.ground_to_field,
        &blackboard.voronoi_map,
    ) && let Some(target_position) = target_player_position(map, blackboard, ground_to_field)
    {
        let walk_and_stand = blackboard.parameters.walking.walk_and_stand;
        let kicking_speed = blackboard.parameters.walking.speed.kicking;
        let orientation_mode = if let Some(ball) = &blackboard.ball {
            OrientationMode::LookAt {
                target: ground_to_field.inverse() * ball.position,
                tolerance: walk_and_stand.orientation_tolerance,
            }
        } else {
            OrientationMode::AlignWithPath
        };

        walk_to(
            blackboard,
            Pose2::from(ground_to_field.inverse() * target_position),
            kicking_speed,
            orientation_mode,
            walk_and_stand.normal_distance_to_be_aligned,
            walk_and_stand.hysteresis,
        )
    } else {
        Status::Failure
    }
}

fn target_player_position(
    map: &VoronoiGrid,
    blackboard: &Blackboard,
    ground_to_field: Isometry2<Ground, Field>,
) -> Option<Point2<Field>> {
    let player = blackboard.world_state.robot.player_number;
    let ball_position = blackboard.ball.as_ref().map(|ball| ball.position);
    let field_dimensions = &blackboard.field_dimensions;
    let parameters = &blackboard.parameters.voronoi;
    let planner = create_path_planner(blackboard, ground_to_field);
    let field_to_ground = ground_to_field.inverse();
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut count = 0;
    let mut candidates = Vec::new();

    for (point, ownership) in map.cells() {
        if ownership != Ownership::Robot(player)
            || point.x().abs() > field_dimensions.length / 2.0
            || point.y().abs() > field_dimensions.width / 2.0
        {
            continue;
        }

        // Also account for the ball and goal structures, which are walking obstacles
        // but must not remove ball ownership from the coverage grid.
        let point_in_ground = field_to_ground * point;
        if planner
            .obstacles
            .iter()
            .any(|obstacle| match obstacle.shape {
                PathObstacleShape::Circle(circle) => circle.contains(point_in_ground),
                PathObstacleShape::LineSegment(line) => {
                    line.distance_to(point_in_ground) <= f32::EPSILON
                }
            })
        {
            continue;
        }

        candidates.push(point);

        sum_x += point.x();
        sum_y += point.y();
        count += 1;
    }

    if count == 0 {
        return None;
    }

    let inv_count = 1.0 / count as f32;
    let centroid: Point2<Field> = point![sum_x * inv_count, sum_y * inv_count];

    let Some(ball_position) = ball_position else {
        // A centroid of a non-convex region need not be a feasible point in that region.
        return candidates.into_iter().min_by(|a, b| {
            (*a - centroid)
                .norm_squared()
                .total_cmp(&(*b - centroid).norm_squared())
        });
    };

    let half_length = field_dimensions.length / 2.0 + parameters.padding;
    let ball_x = ball_position.x();
    let ball_y = ball_position.y();
    let side_factor = (ball_x / half_length).clamp(-1.0, 1.0);

    let resolution = map.resolution();

    let support_distance = parameters.ball_support_distance.max(resolution);
    let support_sigma = parameters.ball_support_sigma.max(resolution);
    let inv_two_support_sigma_sq = 1.0 / (2.0 * support_sigma * support_sigma);

    let centroid_sigma = parameters.centroid_anchor_sigma.max(resolution);

    let mut best_target = None;

    for point in candidates {
        let forward_norm = point.x() / half_length;
        let forward_term = parameters.forward_weight * side_factor * forward_norm;

        let dx_ball = point.x() - ball_x;
        let dy_ball = point.y() - ball_y;
        let ball_distance = (dx_ball * dx_ball + dy_ball * dy_ball).sqrt();
        let support_distance_error = ball_distance - support_distance;
        let ball_term = parameters.ball_weight
            * (-(support_distance_error * support_distance_error) * inv_two_support_sigma_sq).exp();

        let dx_centroid = point.x() - centroid.x();
        let dy_centroid = point.y() - centroid.y();
        let centroid_penalty = parameters.centroid_anchor_weight
            * (dx_centroid * dx_centroid + dy_centroid * dy_centroid).sqrt()
            / centroid_sigma;

        let score = forward_term + ball_term - centroid_penalty;
        if best_target.is_none_or(|(best_score, _)| score > best_score) {
            best_target = Some((score, point));
        }
    }

    best_target.map(|(_, point)| point)
}

#[cfg(test)]
mod tests {
    use types::{
        obstacles::Obstacle,
        path::traits::EndPoints,
        world_state::{BallState, PlayerState},
    };

    use crate::{
        test_utils::{ball_at, blackboard},
        voronoi::calculate_voronoi_grid,
    };

    use super::*;

    fn support_target(blackboard: &Blackboard) -> Option<Point2<Field>> {
        target_player_position(
            blackboard.voronoi_map.as_ref().unwrap(),
            blackboard,
            blackboard.world_state.robot.ground_to_field.unwrap(),
        )
    }

    #[test]
    fn support_target_respects_walking_clearance() {
        let mut blackboard = blackboard();
        blackboard.world_state.robot.ground_to_field = Some(Isometry2::from(point!(-2.8, 0.0)));
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        blackboard.world_state.obstacles = vec![Obstacle::ball(point!(0.65, 0.11), 0.2)];
        blackboard.world_state.player_states[PlayerNumber::Three] = Some(PlayerState {
            pose: Pose2::from(point!(2.0, 0.0)),
            ball_position: None,
        });
        blackboard.ball = Some(ball_at(point!(2.4, 0.0)));
        calculate_voronoi_grid(&mut blackboard);

        let target = support_target(&blackboard).unwrap();
        assert!((target - point!(-2.15, 0.11)).norm() > 0.6, "{target:?}");
        assert_eq!(
            blackboard
                .voronoi_map
                .as_ref()
                .unwrap()
                .ownership_at(target),
            Some(Ownership::Robot(PlayerNumber::Two))
        );

        let ground_to_field = blackboard.world_state.robot.ground_to_field.unwrap();
        assert_eq!(walk_to_voronoi_position(&mut blackboard), Status::Success);
        let Some(BodyMotion::Walk { path, .. }) = &blackboard.body_motion else {
            panic!("supporter should walk to its clear target");
        };
        let endpoint = ground_to_field * path.last_segment().end_point();
        assert!(
            (endpoint - target).norm() < 1e-5,
            "planner moved {target:?} to {endpoint:?}"
        );
    }

    #[test]
    fn support_target_avoids_ball_clearance_even_without_cached_ball() {
        let mut blackboard = blackboard();
        blackboard.parameters.walking.path_planning.robot_radius = 0.4;
        blackboard
            .parameters
            .walking
            .path_planning
            .ball_obstacle_radius = 0.05;
        blackboard.world_state.ball = Some(BallState::default());
        calculate_voronoi_grid(&mut blackboard);

        let target = support_target(&blackboard).unwrap();
        assert!(target.coords().norm() > 0.45, "{target:?}");
        assert_eq!(
            blackboard
                .voronoi_map
                .as_ref()
                .unwrap()
                .ownership_at(target),
            Some(Ownership::Robot(PlayerNumber::Two))
        );
    }

    #[test]
    fn no_support_target_when_walking_clearance_covers_the_field() {
        let mut blackboard = blackboard();
        blackboard.parameters.walking.path_planning.robot_radius = 10.0;
        blackboard.world_state.ball = Some(BallState::default());
        calculate_voronoi_grid(&mut blackboard);

        assert!(support_target(&blackboard).is_none());
    }
}
