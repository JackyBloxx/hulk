use color_eyre::Result;
use context_attribute::context;
use coordinate_systems::{Field, Ground};
use framework::{AdditionalOutput, MainOutput};
use linear_algebra::{Isometry2, Point2, point};
use serde::{Deserialize, Serialize};
use types::field_dimensions::FieldDimensions;

#[derive(Deserialize, Serialize)]
pub struct TargetPositionComposer {}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    ground_to_field: Input<Option<Isometry2<Ground, Field>>, "ground_to_field?">,
    field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    fake_robot_position: Parameter<Vec<Point2<Field>>, "behavior.fake_robot_position">,

    input_points: AdditionalOutput<Vec<Point2<Field>>, "voronoi.input_points">,
}

#[context]
pub struct MainOutputs {
    pub centroids: MainOutput<Vec<Option<Point2<Field>>>>,
    pub voronoi_cells: MainOutput<Vec<Vec<Point2<Field>>>>,
}

impl TargetPositionComposer {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, mut context: CycleContext) -> Result<MainOutputs> {
        let mut target_positions = Vec::new();
        let mut voronoi_cells = Vec::new();

        if let Some(ground_to_field) = context.ground_to_field {
            let pose = ground_to_field.as_pose();
            let half_length = context.field_dimensions.length / 2.0;
            let half_width = context.field_dimensions.width / 2.0;

            let mut sites = vec![(pose.position().x(), pose.position().y())];
            for fake_position in context.fake_robot_position.into_iter() {
                sites.push((fake_position.x(), fake_position.y()));
            }
            context.input_points.fill_if_subscribed(|| {
                sites
                    .iter()
                    .map(|(x, y)| point![<Field>, *x, *y])
                    .collect::<Vec<Point2<Field>>>()
            });

            for (index, site) in sites.iter().copied().enumerate() {
                if site.0.abs() > half_length || site.1.abs() > half_width {
                    continue;
                }

                let mut vertices = field_rectangle(half_length, half_width);
                for (other_index, other_site) in sites.iter().copied().enumerate() {
                    if index == other_index {
                        continue;
                    }

                    vertices = clip_polygon_to_bisector_half_plane(&vertices, site, other_site);
                    if vertices.is_empty() {
                        break;
                    }
                }

                let vertices = deduplicate_polygon(vertices);

                if let Some(target) = calculate_centroid_field(&vertices) {
                    target_positions.push(Some(target));
                    voronoi_cells.push(vertices);
                }
            }
        }

        Ok(MainOutputs {
            centroids: target_positions.into(),
            voronoi_cells: voronoi_cells.into(),
        })
    }
}

fn field_rectangle(half_length: f32, half_width: f32) -> Vec<Point2<Field>> {
    vec![
        point![<Field>, -half_length, -half_width],
        point![<Field>, half_length, -half_width],
        point![<Field>, half_length, half_width],
        point![<Field>, -half_length, half_width],
    ]
}

fn clip_polygon_to_bisector_half_plane(
    polygon: &[Point2<Field>],
    site: (f32, f32),
    other_site: (f32, f32),
) -> Vec<Point2<Field>> {
    if polygon.is_empty() {
        return Vec::new();
    }

    let dx = other_site.0 - site.0;
    let dy = other_site.1 - site.1;
    let c = 0.5
        * ((other_site.0 * other_site.0 + other_site.1 * other_site.1)
            - (site.0 * site.0 + site.1 * site.1));

    let mut clipped = Vec::new();

    for i in 0..polygon.len() {
        let current = polygon[i];
        let next = polygon[(i + 1) % polygon.len()];

        let current_value = current.x() * dx + current.y() * dy - c;
        let next_value = next.x() * dx + next.y() * dy - c;

        let current_inside = current_value <= f32::EPSILON;
        let next_inside = next_value <= f32::EPSILON;

        match (current_inside, next_inside) {
            (true, true) => clipped.push(next),
            (true, false) => {
                if let Some(intersection) = edge_half_plane_intersection(
                    current,
                    next,
                    current_value,
                    next_value,
                    f32::EPSILON,
                ) {
                    clipped.push(intersection);
                }
            }
            (false, true) => {
                if let Some(intersection) = edge_half_plane_intersection(
                    current,
                    next,
                    current_value,
                    next_value,
                    f32::EPSILON,
                ) {
                    clipped.push(intersection);
                }
                clipped.push(next);
            }
            (false, false) => {}
        }
    }

    clipped
}

fn edge_half_plane_intersection(
    start: Point2<Field>,
    end: Point2<Field>,
    start_value: f32,
    end_value: f32,
    epsilon: f32,
) -> Option<Point2<Field>> {
    let denominator = start_value - end_value;
    if denominator.abs() <= epsilon {
        return None;
    }

    let t = (start_value / denominator).clamp(0.0, 1.0);
    Some(point![
        <Field>,
        start.x() + t * (end.x() - start.x()),
        start.y() + t * (end.y() - start.y())
    ])
}

fn deduplicate_polygon(vertices: Vec<Point2<Field>>) -> Vec<Point2<Field>> {
    let mut deduplicated = Vec::with_capacity(vertices.len());

    for vertex in vertices {
        let is_duplicate = deduplicated.last().is_some_and(|last: &Point2<Field>| {
            (last.x() - vertex.x()).abs() < f32::EPSILON
                && (last.y() - vertex.y()).abs() < f32::EPSILON
        });
        if !is_duplicate {
            deduplicated.push(vertex);
        }
    }

    if deduplicated.len() > 1 {
        let first = deduplicated[0];
        let last = deduplicated[deduplicated.len() - 1];
        if (first.x() - last.x()).abs() < f32::EPSILON
            && (first.y() - last.y()).abs() < f32::EPSILON
        {
            deduplicated.pop();
        }
    }

    deduplicated
}

fn calculate_centroid_field(vertices: &[Point2<Field>]) -> Option<Point2<Field>> {
    if vertices.len() < 3 {
        return None;
    }

    let mut area = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;

    for i in 0..vertices.len() {
        let p1 = vertices[i];
        let p2 = vertices[(i + 1) % vertices.len()];

        let cross = p1.x() * p2.y() - p2.x() * p1.y();
        area += cross;
        cx += (p1.x() + p2.x()) * cross;
        cy += (p1.y() + p2.y()) * cross;
    }

    let final_area = area / 2.0;
    if final_area.abs() < 1e-6 {
        return None;
    }

    Some(point![<Field>, cx / (6.0 * final_area), cy / (6.0 * final_area)])
}
