use crate::sim::aircraft::AircraftTelemetry;

#[cfg(feature = "visualization")]
use bevy::prelude::*;

#[cfg(feature = "visualization")]
#[derive(Component)]
pub struct AircraftEntity;

#[cfg(feature = "visualization")]
#[derive(Component)]
pub struct TrailPoint {
    pub age: f32,
}

#[cfg(feature = "visualization")]
#[derive(Resource)]
pub struct SimulationData {
    pub telemetry: Vec<(f64, AircraftTelemetry)>,
    pub current_index: usize,
    pub playing: bool,
    pub speed: f32,
    pub timer: f32,
}

pub struct Visualization;

impl Visualization {
    pub fn run_visualization(telemetry: Vec<(f64, AircraftTelemetry)>) {
        #[cfg(feature = "visualization")]
        {
            use bevy::prelude::*;
            App::new()
                .add_plugins(DefaultPlugins.set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Eclipse Flight Simulator".to_string(),
                        resolution: (1200.0, 800.0).into(),
                        ..default()
                    }),
                    ..default()
                }))
                .insert_resource(SimulationData {
                    telemetry,
                    current_index: 0,
                    playing: true,
                    speed: 1.0,
                    timer: 0.0,
                })
                .add_systems(Startup, setup_scene)
                .add_systems(Update, (
                    update_aircraft_position,
                    update_trail,
                    handle_input,
                    update_camera,
                ))
                .run();
        }
        
        #[cfg(not(feature = "visualization"))]
        {
            println!("Visualization not available. Build with --features visualization to enable.");
            println!("Telemetry points: {}", telemetry.len());
        }
    }
}

#[cfg(feature = "visualization")]
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(PbrBundle {
        mesh: meshes.add(Cuboid::new(2.0, 0.2, 4.0)),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.4, 0.8),
            ..default()
        }),
        transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
        ..default()
    }).insert(AircraftEntity);

    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(100.0, 100.0)),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.8, 0.3),
            ..default()
        }),
        transform: Transform::from_translation(Vec3::new(0.0, -0.1, 0.0)),
        ..default()
    });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.5, -0.3, 0.0)),
        ..default()
    });

    commands.spawn(Camera3dBundle {
        transform: Transform::from_translation(Vec3::new(0.0, 20.0, 30.0))
            .looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}

#[cfg(feature = "visualization")]
fn update_aircraft_position(
    mut aircraft_query: Query<&mut Transform, With<AircraftEntity>>,
    mut sim_data: ResMut<SimulationData>,
    time: Res<Time>,
) {
    if !sim_data.playing || sim_data.telemetry.is_empty() {
        return;
    }

    sim_data.timer += time.delta_seconds() * sim_data.speed;
    
    if sim_data.timer >= 0.1 && sim_data.current_index < sim_data.telemetry.len() - 1 {
        sim_data.current_index += 1;
        sim_data.timer = 0.0;
    }

    if let Some((_, telemetry)) = sim_data.telemetry.get(sim_data.current_index) {
        if let Ok(mut transform) = aircraft_query.get_single_mut() {
            let pos = telemetry.position;
            let (roll, pitch, yaw) = telemetry.attitude;
            
            transform.translation = Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32);
            transform.rotation = Quat::from_euler(EulerRot::ZYX, yaw as f32, pitch as f32, roll as f32);
        }
    }
}

#[cfg(feature = "visualization")]
fn update_trail(
    mut commands: Commands,
    mut trail_query: Query<(Entity, &mut TrailPoint, &mut Transform)>,
    sim_data: Res<SimulationData>,
    time: Res<Time>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (entity, mut trail_point, mut transform) in trail_query.iter_mut() {
        trail_point.age += time.delta_seconds();
        
        if trail_point.age > 10.0 {
            commands.entity(entity).despawn();
        } else {
            let alpha = 1.0 - (trail_point.age / 10.0);
            transform.scale = Vec3::splat(alpha * 0.1);
        }
    }

    if sim_data.current_index % 10 == 0 {
        if let Some((_, telemetry)) = sim_data.telemetry.get(sim_data.current_index) {
            let pos = telemetry.position;
            
            commands.spawn(PbrBundle {
                mesh: meshes.add(Sphere::new(0.1)),
                material: materials.add(StandardMaterial {
                    base_color: Color::srgba(1.0, 0.5, 0.0, 0.8),
                    ..default()
                }),
                transform: Transform::from_translation(Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32)),
                ..default()
            }).insert(TrailPoint { age: 0.0 });
        }
    }
}

#[cfg(feature = "visualization")]
fn handle_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut sim_data: ResMut<SimulationData>,
) {
    if keys.just_pressed(KeyCode::Space) {
        sim_data.playing = !sim_data.playing;
    }
    
    if keys.just_pressed(KeyCode::ArrowLeft) && sim_data.current_index > 0 {
        sim_data.current_index -= 1;
    }
    
    if keys.just_pressed(KeyCode::ArrowRight) && sim_data.current_index < sim_data.telemetry.len() - 1 {
        sim_data.current_index += 1;
    }
    
    if keys.pressed(KeyCode::Digit1) {
        sim_data.speed = 0.1;
    } else if keys.pressed(KeyCode::Digit2) {
        sim_data.speed = 0.5;
    } else if keys.pressed(KeyCode::Digit3) {
        sim_data.speed = 1.0;
    } else if keys.pressed(KeyCode::Digit4) {
        sim_data.speed = 2.0;
    } else if keys.pressed(KeyCode::Digit5) {
        sim_data.speed = 5.0;
    }
}

#[cfg(feature = "visualization")]
fn update_camera(
    aircraft_query: Query<&Transform, (With<AircraftEntity>, Without<Camera>)>,
    mut camera_query: Query<&mut Transform, (With<Camera>, Without<AircraftEntity>)>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    if let (Ok(aircraft_transform), Ok(mut camera_transform)) = 
        (aircraft_query.get_single(), camera_query.get_single_mut()) {
        
        if keys.pressed(KeyCode::KeyF) {
            let follow_distance = 30.0;
            let follow_height = 15.0;
            
            let target_pos = aircraft_transform.translation + Vec3::new(0.0, follow_height, follow_distance);
            camera_transform.translation = camera_transform.translation.lerp(target_pos, 0.1);
            
            let look_target = aircraft_transform.translation;
            let direction = (look_target - camera_transform.translation).normalize();
            camera_transform.look_to(direction, Vec3::Y);
        }
    }
}