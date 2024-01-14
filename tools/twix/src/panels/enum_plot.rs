use std::{
    collections::hash_map::DefaultHasher,
    hash::{Hash, Hasher},
    str::FromStr,
    sync::Arc,
};

use communication::client::CyclerOutput;
use eframe::{
    egui::{show_tooltip_at_pointer, ComboBox, Response, Ui, Widget},
    epaint::{Color32, Stroke},
};
use egui_plot::{Plot, PlotBounds, PlotPoint, PlotUi, Polygon, Text};
use itertools::Itertools;
use log::{error, info};
use serde_json::Value;

use crate::{
    change_buffer::{Change, ChangeBuffer},
    completion_edit::CompletionEdit,
    nao::Nao,
    panel::Panel,
};

fn color_hash(value: impl Hash) -> Color32 {
    let mut hasher = DefaultHasher::new();
    value.hash(&mut hasher);

    let hash = hasher.finish();

    let [r, g, b, ..] = hash.to_le_bytes();

    Color32::from_rgb(r, g, b)
}

#[derive(Clone)]
struct Segment {
    start: usize,
    end: usize,
    value: Value,
}

impl Segment {
    fn name(&self) -> String {
        match &self.value {
            Value::Null => "<Null>".into(),
            Value::Bool(value) => value.to_string(),
            Value::Number(_) => "<Number>".into(),
            Value::String(string) => string.clone(),
            Value::Array(_) => "<Array>".into(),
            Value::Object(map) => {
                // There is only one key in the map.
                // This is probably an enum variant.
                if let [(key, _value)] = &map.iter().collect::<Vec<_>>().as_slice() {
                    (*key).clone()
                } else {
                    "<Object>".into()
                }
            }
        }
    }

    fn tooltip(&self) -> Option<String> {
        match &self.value {
            Value::Number(number) => Some(number.to_string()),
            Value::String(string) => Some(string.clone()),
            Value::Object(map) => {
                if let [(key, value)] = &map.iter().collect::<Vec<_>>().as_slice() {
                    Some(format!("{key} {value:#}"))
                } else {
                    Some(format!("{map:#?}"))
                }
            }
            Value::Array(array) => Some(format!("{array:#?}")),
            _ => None,
        }
    }
}

#[derive(Debug, PartialEq)]
enum ViewportMode {
    // Always keep every segment in view
    Full,
    // Variable-sized window, fixed on the right side to the most recent data
    Follow,
    // The viewport does not follow new data and has to be positioned manually
    Free,
}

pub struct EnumPlotPanel {
    nao: Arc<Nao>,
    changes: Vec<Change>,
    messages_count: usize,
    scroll_pos: f64,
    viewport_width: f64,
    change_buffer: Option<ChangeBuffer>,
    output_key: String,
    viewport_mode: ViewportMode,
}

impl Panel for EnumPlotPanel {
    const NAME: &'static str = "Enum Plot";

    fn new(nao: Arc<Nao>, _value: Option<&serde_json::Value>) -> Self {
        Self {
            nao,
            changes: Vec::new(),
            messages_count: 0,
            change_buffer: None,
            scroll_pos: 0.0,
            viewport_width: 1.0,
            output_key: String::new(),
            viewport_mode: ViewportMode::Full,
        }
    }
}

impl EnumPlotPanel {
    fn plot_segment(plot_ui: &mut PlotUi, segment: &Segment) {
        // Leave small gaps on the top and bottom,
        // to avoid clipping half the segment border
        const VERTICAL_MARGIN: f64 = 0.05;
        const BORDER_WIDTH: f32 = 2.0;

        let name = segment.name();
        let color = color_hash(&name);

        let start = segment.start as f64;
        let end = segment.end as f64;

        plot_ui.polygon(
            Polygon::new(vec![
                [start, 0.0 + VERTICAL_MARGIN],
                [start, 1.0 - VERTICAL_MARGIN],
                [end, 1.0 - VERTICAL_MARGIN],
                [end, 0.0 + VERTICAL_MARGIN],
            ])
            .fill_color(color.gamma_multiply(0.5))
            .name(&name)
            .stroke(Stroke::new(BORDER_WIDTH, color)),
        );
        plot_ui.text(
            Text::new(
                PlotPoint {
                    x: start + 0.05,
                    y: 0.9,
                },
                name,
            )
            .color(Color32::WHITE)
            .anchor(eframe::emath::Align2::LEFT_TOP),
        );
    }

    fn update_changes(&mut self) {
        if let Some(change_buffer) = &self.change_buffer {
            match change_buffer.get_and_reset() {
                Ok(change_buffer_update) => {
                    self.messages_count = change_buffer_update.message_count;
                    self.changes.extend(change_buffer_update.updates);
                }
                Err(error) => {
                    error!("{error}");
                }
            }
        }
    }

    fn segments(&self) -> Vec<Segment> {
        let mut segments = Vec::new();

        for (start, end) in self.changes.iter().tuple_windows() {
            segments.push(Segment {
                start: start.message_number,
                end: end.message_number,
                value: start.value.clone(),
            });
        }

        if let Some(last_change) = self.changes.last() {
            segments.push(Segment {
                start: last_change.message_number,
                end: self.messages_count,
                value: last_change.value.clone(),
            });
        }

        segments
    }

    fn process_user_input(&mut self, plot_ui: &PlotUi) {
        let drag_delta = f64::from(plot_ui.pointer_coordinate_drag_delta().x);

        let cursor_position = plot_ui.pointer_coordinate();
        let scroll_delta = plot_ui.ctx().input(|input| input.scroll_delta);

        let normalized_cursor_position = cursor_position
            .map_or(0.0, |plot_point| plot_point.x - self.scroll_pos)
            / self.viewport_width;

        let previous_viewport_width = self.viewport_width;

        self.viewport_width =
            (self.viewport_width * 0.99f64.powf(f64::from(scroll_delta.y))).max(1.0);

        let zoom_difference = self.viewport_width - previous_viewport_width;
        let zoom_scroll_compensation = zoom_difference * normalized_cursor_position;

        self.scroll_pos -= drag_delta
            + self.viewport_width * f64::from(scroll_delta.x) / 400.0
            + zoom_scroll_compensation;
    }

    fn show_plot(&mut self, plot_ui: &mut PlotUi) {
        const OVERSCAN_FACTOR: f64 = 0.02;

        if plot_ui.response().hovered() {
            self.process_user_input(plot_ui);
        }

        match self.viewport_mode {
            ViewportMode::Full => {
                self.viewport_width = self.messages_count.max(1) as f64 * (1.0 + OVERSCAN_FACTOR);
                self.scroll_pos = 0.0;
            }
            ViewportMode::Follow => {
                self.scroll_pos = self.messages_count.max(1) as f64
                    - self.viewport_width * (1.0 - OVERSCAN_FACTOR);
            }
            ViewportMode::Free => {}
        }

        plot_ui.set_plot_bounds(PlotBounds::from_min_max(
            [self.scroll_pos, 0.0],
            [self.scroll_pos + self.viewport_width, 1.0],
        ));

        for segment in self.segments() {
            Self::plot_segment(plot_ui, &segment);
        }
    }

    fn plot(&mut self, ui: &mut Ui) -> Response {
        self.update_changes();

        let plot = Plot::new("Jürgen")
            .height(64.0)
            .show_y(false)
            .y_axis_width(0)
            .y_grid_spacer(|_| vec![])
            .show_grid(false)
            .allow_scroll(false)
            .allow_drag(false)
            .allow_zoom(false)
            .label_formatter(|_name, plot_point| format!("Message #{}", plot_point.x as isize))
            .show(ui, |plot_ui| self.show_plot(plot_ui));

        if plot.response.double_clicked() {
            self.viewport_width = (self.messages_count as f64).max(1.0);
            self.scroll_pos = 0.0;
        }

        if let Some(hover_pos) = plot.response.hover_pos() {
            let plot_hover_pos = plot.transform.value_from_position(hover_pos).x;

            let hovered_segment = self
                .segments()
                .iter()
                .find(|segment| {
                    (segment.start as f64) < plot_hover_pos && plot_hover_pos < (segment.end as f64)
                })
                .cloned();

            if let Some(tooltip) = hovered_segment.and_then(|segment| segment.tooltip()) {
                show_tooltip_at_pointer(
                    ui.ctx(),
                    eframe::egui::Id::new("enum_plot_segment_tooltip"),
                    |ui| {
                        ui.label(tooltip);
                    },
                );
            }
        }

        plot.response
    }

    fn subscribe_key(&mut self, nao: Arc<Nao>) {
        self.change_buffer = match CyclerOutput::from_str(&self.output_key) {
            Ok(output) => {
                let buffer = nao.subscribe_changes(output);
                Some(buffer)
            }
            Err(error) => {
                error!("Failed to subscribe: {:#}", error);
                None
            }
        };
    }
}

impl Widget for &mut EnumPlotPanel {
    fn ui(self, ui: &mut Ui) -> Response {
        ui.vertical(|ui| {
            self.plot(ui);
            ui.horizontal_top(|ui| {
                let subscription_field = ui.add(CompletionEdit::outputs(
                    &mut self.output_key,
                    self.nao.as_ref(),
                ));
                if subscription_field.changed() {
                    info!("Subscribing: {}", self.output_key);
                    self.changes.clear();
                    self.subscribe_key(self.nao.clone());
                }
                if ui.button("Clear").clicked() {
                    let last_change = self.changes.drain(..).next_back();
                    if let Some(change_buffer) = &self.change_buffer {
                        change_buffer.reset();
                    }
                    if let Some(last_change) = last_change {
                        self.changes.push(Change {
                            message_number: 0,
                            value: last_change.value,
                        });
                    }
                }
                ui.label("Viewport mode:");
                ComboBox::new("viewport_mode", "")
                    .selected_text(format!("{:?}", self.viewport_mode))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.viewport_mode, ViewportMode::Full, "Full");
                        ui.selectable_value(
                            &mut self.viewport_mode,
                            ViewportMode::Follow,
                            "Follow",
                        );
                        ui.selectable_value(&mut self.viewport_mode, ViewportMode::Free, "Free");
                    })
            });
        })
        .response
    }
}
