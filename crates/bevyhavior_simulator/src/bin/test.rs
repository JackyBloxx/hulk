use bevy::prelude::*;
use bevyhavior_simulator::behavior_tree_simulator::{
    BehaviorTreeSimulatorSet, SimulatorBall, SimulatorClock, SimulatorFieldDimensions,
    SimulatorRobotBundle, SimulatorTimeline, default_behavior_parameters,
};
use coordinate_systems::{Ground, World};
use hsl_network_messages::PlayerNumber;
use linear_algebra::{Isometry2, point, vector};
use scenario::scenario;
use std::time::Duration;
use types::{field_dimensions::Side, motion_command::MotionCommand, primary_state::PrimaryState};

const RUN_DURATION: Duration = Duration::from_secs(40);
const TELEPORT_INTERVAL: Duration = Duration::from_secs(5);

#[derive(Resource, Default)]
struct PrintedFrames(usize);

#[derive(Resource, Default)]
struct BallTeleportState {
    last_slot: u64,
}

#[scenario]
fn behavior_tree_ball_outside_field_smoke(app: &mut App) {
    app.init_resource::<PrintedFrames>()
        .init_resource::<BallTeleportState>()
        .add_systems(Startup, startup)
        .add_systems(
            Update,
            teleport_ball.in_set(BehaviorTreeSimulatorSet::BeforeWorldState),
        )
        .add_systems(
            Update,
            report_and_exit.in_set(BehaviorTreeSimulatorSet::Scenario),
        );
}
fn startup(
    mut commands: Commands,
    mut ball: ResMut<SimulatorBall>,
    field_dimensions: Res<SimulatorFieldDimensions>,
) {
    let mut parameters =
        default_behavior_parameters().expect("failed to load default behavior parameters");
    parameters.goal_keeper_number = PlayerNumber::One;
    parameters.last_ball_timeout = Duration::from_secs(2);
    commands.spawn(
        SimulatorRobotBundle::new(PlayerNumber::Three, pose(2.0, 3.0, 0.0), parameters)
            .expect("failed to create robot bundle")
            .with_primary_state(PrimaryState::Playing),
    );
    let length = field_dimensions.0.length;
    let width = field_dimensions.0.width;
    ball.state = Some(
        bevyhavior_simulator::behavior_tree_simulator::SimulatedBall {
            position: point![-length / 4.0, -width / 4.0],
            velocity: vector![0.0, 0.0],
            field_side: Side::Left,
        },
    );
}

fn teleport_ball(
    clock: Res<SimulatorClock>,
    field_dimensions: Res<SimulatorFieldDimensions>,
    mut ball: ResMut<SimulatorBall>,
    mut state: ResMut<BallTeleportState>,
) {
    let elapsed = clock
        .now
        .duration_since(std::time::SystemTime::UNIX_EPOCH)
        .expect("simulator time should not move backwards");
    let slot = elapsed.as_secs() / TELEPORT_INTERVAL.as_secs();
    if slot == 0 || slot == state.last_slot {
        return;
    }

    let length = field_dimensions.0.length;
    let width = field_dimensions.0.width;
    let positions = [
        point![length / 4.0, -width / 4.0],
        point![length / 4.0, width / 4.0],
        point![-length / 4.0, width / 4.0],
        point![0.0, 0.0],
        point![length / 3.0, 0.0],
        point![0.0, -width / 3.0],
    ];
    let position = positions[(slot as usize - 1) % positions.len()];
    ball.state = Some(
        bevyhavior_simulator::behavior_tree_simulator::SimulatedBall {
            position,
            velocity: vector![0.0, 0.0],
            field_side: Side::Left,
        },
    );
    state.last_slot = slot;
    println!(
        "teleport_ball elapsed={:.2} position=({:.2}, {:.2})",
        elapsed.as_secs_f32(),
        position.x(),
        position.y()
    );
}

fn report_and_exit(
    timeline: Res<SimulatorTimeline>,
    mut printed_frames: ResMut<PrintedFrames>,
    mut exit: MessageWriter<AppExit>,
) {
    let mut has_violation = false;
    for (index, frame) in timeline.frames.iter().enumerate().skip(printed_frames.0) {
        println!(
            "frame={index} violations={}",
            frame.invariant_violations.len()
        );
        for (player_number, robot_frame) in &frame.robot_frames {
            println!(
                "  robot={player_number} motion={}",
                motion_name(&robot_frame.motion_command)
            );
        }
        for violation in &frame.invariant_violations {
            has_violation = true;
            println!(
                "  invariant={} robot={:?} severity={:?} message={}",
                violation.check_name,
                violation.player_number,
                violation.severity,
                violation.message
            );
        }
    }
    printed_frames.0 = timeline.frames.len();
    if has_violation {
        println!(
            "result=fail frames={} reason=invariant_violation",
            timeline.frames.len()
        );
        exit.write(AppExit::from_code(1));
        return;
    }
    if timeline.frames.len() as u32 >= frames_to_run() {
        println!("result=ok frames={}", timeline.frames.len());
        exit.write(AppExit::Success);
    }
}
fn frames_to_run() -> u32 {
    (RUN_DURATION.as_secs_f32()
        / bevyhavior_simulator::behavior_tree_simulator::DEFAULT_TICK_DURATION.as_secs_f32())
    .ceil() as u32
}
fn pose(x: f32, y: f32, yaw: f32) -> Isometry2<Ground, World> {
    Isometry2::from_parts(vector![x, y], yaw)
}
fn motion_name(motion_command: &MotionCommand) -> &'static str {
    match motion_command {
        MotionCommand::Prepare => "prepare",
        MotionCommand::Stand { .. } => "stand",
        MotionCommand::StandUp => "stand_up",
        MotionCommand::VisualKick { .. } => "visual_kick",
        MotionCommand::Walk { .. } => "walk",
        MotionCommand::WalkWithVelocity { .. } => "walk_with_velocity",
    }
}
