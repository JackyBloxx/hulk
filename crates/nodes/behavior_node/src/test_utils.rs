use std::time::Duration;

use coordinate_systems::Field;
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, Point2, Vector2};
use ros_z::time::Time;
use types::{
    field_dimensions::{FieldDimensions, Side},
    parameters::{BehaviorParameters, GoalkeeperParameters, VoronoiParameters},
    world_state::{RobotState, WorldState},
};

use crate::node::{Blackboard, LastBall};

pub(crate) fn blackboard() -> Blackboard {
    Blackboard {
        field_dimensions: FieldDimensions::SPL_2025,
        parameters: BehaviorParameters {
            goalkeeper: GoalkeeperParameters {
                player_number: PlayerNumber::One,
                striker_distance: 3.5,
                ..Default::default()
            },
            voronoi: VoronoiParameters {
                grid_resolution: 0.2,
                padding: 0.5,
                forward_weight: 0.2,
                ball_weight: 1.0,
                ball_support_distance: 1.5,
                ball_support_sigma: 0.5,
                centroid_anchor_weight: 0.6,
                centroid_anchor_sigma: 1.0,
                minimum_centroid_margin_from_own_side: 2.0,
            },
            ..Default::default()
        },
        world_state: WorldState {
            robot: RobotState {
                ground_to_field: Some(Isometry2::identity()),
                player_number: PlayerNumber::Two,
                ..Default::default()
            },
            ..Default::default()
        },
        path_obstacles_output: Vec::new(),
        time_since_last_switch: Duration::ZERO,
        direction_difference: 0.0,
        voronoi_inputs: Vec::new(),
        ball: None,
        visual_kick_ball_position: None,
        last_ball: None,
        last_close_enough_to_kick: false,
        last_kick_target: None,
        last_motion_command: Default::default(),
        last_motion_switch_time: Time::zero(),
        last_motion_type: None,
        last_sent_game_controller_return_message_time: None,
        last_sent_hsl_message_time: None,
        last_closest_to_ball: false,
        closest_to_ball_entered_area_since: None,
        closest_to_ball_left_area_since: None,
        is_injected_motion_command: false,
        walk_position: None,
        body_motion: None,
        head_motion: None,
        voronoi_map: None,
    }
}

pub(crate) fn ball_at(position: Point2<Field>) -> LastBall {
    LastBall {
        position,
        velocity: Vector2::zeros(),
        age: Time::zero(),
        field_side: Side::Left,
    }
}
