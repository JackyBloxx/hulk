use color_eyre::Result;
use types::{configuration::Behavior, PathSegment};

use std::time::Duration;

use context_attribute::context;
#[context]
pub struct CycleContext {
    pub dribble_path: Input<Option<Vec<PathSegment>>, "dribble_path?">,
    pub time_to_reach_kick_position: PersistentState<Duration, "time_to_reach_kick_position">,
    pub configuration: Parameter<Behavior, "behavior">,
    pub stand_up_back_estimated_remaining_duration:
        Input<Option<Duration>, "stand_up_back_estimated_remaining_duration?">,
    pub stand_up_front_estimated_remaining_duration:
        Input<Option<Duration>, "stand_up_front_estimated_remaining_duration?">,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct MainOutputs {}

pub struct TimeToReachKickPosition {}

impl TimeToReachKickPosition {
    pub fn new(_: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, context: CycleContext) -> Result<MainOutputs> {
        let walk_time = context
            .dribble_path
            .as_ref()
            .map(|path| {
                path.iter()
                    .map(|segment: &PathSegment| {
                        let length = segment.length();
                        match segment {
                            PathSegment::LineSegment(_) => {
                                length / context.configuration.path_planning.line_walking_speed
                            }
                            PathSegment::Arc(_, _) => {
                                length / context.configuration.path_planning.arc_walking_speed
                            }
                        }
                    })
                    .sum()
            })
            .map(Duration::from_secs_f32);
        let time_to_reach_kick_position = walk_time.map(|walk_time| {
            walk_time
                + *context
                    .stand_up_back_estimated_remaining_duration
                    .unwrap_or(&Duration::ZERO)
                + *context
                    .stand_up_front_estimated_remaining_duration
                    .unwrap_or(&Duration::ZERO)
        });

        *context.time_to_reach_kick_position =
            time_to_reach_kick_position.unwrap_or(Duration::from_secs(1800));
        /*1800 seconds is 30 minutes, which is essentially max as it pertains to game time and prevents Duration::MAX from breaking the behavior sim
         */
        Ok(MainOutputs {})
    }
}
