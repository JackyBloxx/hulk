use types::{
    behavior_tree::Status,
    motion_command::{ArmMotion, HeadMotion, ImageRegion, MotionCommand, OrientationMode},
    path::direct_path,
};

use linear_algebra::{Orientation2, Point};

use crate::behavior::node::CaptainBlackboard;

pub fn injected_motion_command(context: &mut CaptainBlackboard) -> Status {
    if let Some(injected_motion_command) = &context.parameters.injected_motion_command {
        context.output = Some(injected_motion_command.clone());
        Status::Success
    } else {
        Status::Failure
    }
}

pub fn prepare(context: &mut CaptainBlackboard) -> Status {
    context.output = Some(MotionCommand::Prepare);
    Status::Success
}

pub fn stand(context: &mut CaptainBlackboard) -> Status {
    context.output = Some(MotionCommand::Stand {
        head: HeadMotion::Center {
            image_region_target: ImageRegion::Top,
        },
    });
    Status::Success
}

pub fn stand_up(context: &mut CaptainBlackboard) -> Status {
    context.output = Some(MotionCommand::StandUp);
    Status::Success
}

pub fn walk_to_ball(context: &mut CaptainBlackboard) -> Status {
    if let Some(ball) = &context.world_state.ball {
        context.output = Some(MotionCommand::WalkWithVelocity {
            head: HeadMotion::LookAt {
                target: ball.ball_in_ground,
                image_region_target: ImageRegion::Top,
            },
            velocity: ball.ball_in_ground.coords(),
            angular_velocity: 0.0,
        });
        Status::Success
    } else {
        Status::Failure
    }
}

pub fn walk_to_voronoi(context: &mut CaptainBlackboard) -> Status {
    let own_centroid = context
        .world_state
        .centroids
        .first()
        .and_then(|centroid| *centroid);
    let Some(target_in_field) = own_centroid else {
        return Status::Failure;
    };

    let Some(ground_to_field) = context.world_state.robot.ground_to_field else {
        return Status::Failure;
    };
    let target_position = ground_to_field.inverse() * target_in_field;

    context.output = Some(MotionCommand::Walk {
        head: HeadMotion::LookAt {
            target: target_position,
            image_region_target: ImageRegion::Center,
        },
        path: direct_path(Point::origin(), target_position),
        left_arm: ArmMotion::Swing,
        right_arm: ArmMotion::Swing,
        orientation_mode: OrientationMode::AlignWithPath,
        target_orientation: Orientation2::new(0.0),
        distance_to_be_aligned: 0.2,
        speed: 0.5,
    });
    Status::Success
}
