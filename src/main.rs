//! Displays a multicolored sphere to the user.

extern crate amethyst;
extern crate genmesh;
extern crate ncollide;
extern crate nalgebra;

mod physics;
use physics::*;

use amethyst::components::transform::{LocalTransform, Transform};
use amethyst::engine::{Application, State, Trans};
use amethyst::config::Element;
use amethyst::ecs::{RunArg, World, Processor, Component, VecStorage, Join, Entity};
use amethyst::gfx_device::DisplayConfig;
use amethyst::asset_manager::{AssetManager, DirectoryStore};
use amethyst::event::WindowEvent;
use amethyst::renderer::{VertexPosNormal, Pipeline};

use self::genmesh::generators::SphereUV;
use self::genmesh::{MapToVertices, Triangulate, Vertices};

use self::nalgebra::{Vector3, Point3, Isometry3, Rotation3, dot};
use self::ncollide::world::{CollisionWorld, CollisionObject3, CollisionGroups, GeometricQueryType};
use self::ncollide::narrow_phase::{ContactHandler, ContactAlgorithm3};
use self::ncollide::shape::ShapeHandle3;

struct CameraRotation {
    pitch: f32,
    yaw: f32,
    roll: f32,
}

struct DeltaMosue {
    last_x: i32,
    last_y: i32,
    delta_x: i32,
    delta_y: i32,
}

impl DeltaMosue {
    fn new() -> DeltaMosue {
        DeltaMosue {
            last_x: 0,
            last_y: 0,
            delta_x: 0,
            delta_y: 0,
        }
    }
}

struct Moveable {
    velocity: Vector3<f32>,
}

impl Component for Moveable {
    type Storage = VecStorage<Moveable>;
}

struct MouseLook;

use amethyst::world_resources::camera::Camera;
fn mouse_look(camera: &mut Camera, delta_yaw: f32, delta_pitch: f32) {
    use nalgebra::{BaseFloat, Unit, Norm, Rotate, UnitQuaternion, Quaternion, Point3, Vector3};
    if delta_yaw != 0.0 || delta_pitch != 0.0 {
        /*let dt = time.delta_time.subsec_nanos() as f32 / 1.0e9;

        if input_handler.key_down(VirtualKeyCode::W) {
            camera.eye = *(Point3::<f32>::from(&camera.eye) + unit_dir * 2.0 * dt).as_ref();
        }*/

        let pos = Point3::from(&camera.eye);
        let target = Point3::from(&camera.target);
        let mut forward = target - pos;
        let mut up = Vector3::from(&camera.up);

        // Set the yaw
        let quat =
            UnitQuaternion::new(
                &Quaternion::from_polar_decomposition( 1.0, delta_yaw,
                                                       Unit::new( &Vector3::y() ) ) );
        forward = quat.rotate( &forward ).normalize();
        up = quat.rotate( &up ).normalize();

        // Set the pitch
        let angle = nalgebra::angle_between( &forward, &Vector3::y() );
        let horizontal = nalgebra::cross( &forward, &up );
        if angle < delta_pitch {
            forward = Vector3::y();
            up = nalgebra::cross( &horizontal, &forward ).normalize();
        } else if <f32 as BaseFloat>::pi() - angle < -delta_pitch {
            forward = -Vector3::y();
            up = nalgebra::cross( &horizontal, &forward ).normalize();
        } else {
            let quat =
                UnitQuaternion::new(
                    &Quaternion::from_polar_decomposition( 1.0, delta_pitch,
                                                           Unit::new( &horizontal ) ) );
            forward = quat.rotate( &forward ).normalize();
            up = quat.rotate( &up ).normalize();
        }

        camera.target = *(pos + forward).as_ref();
        camera.up = *up.as_ref();
    }
}

struct Game;

impl State for Game {
    fn on_start(&mut self,
                world: &mut World,
                asset_manager: &mut AssetManager,
                pipeline: &mut Pipeline) {

        use amethyst::renderer::pass::{Clear, DrawShaded};
        use amethyst::renderer::{Layer, Light};
        use amethyst::world_resources::camera::{Projection, Camera};
        use amethyst::world_resources::{ScreenDimensions, InputHandler};
        use amethyst::components::rendering::{Texture, Mesh, Renderable};

        let layer = Layer::new("main",
                               vec![Clear::new([0.0, 0.0, 0.0, 1.0]),
                                    DrawShaded::new("main", "main")]);
        pipeline.layers = vec![layer];


        {
            let dimensions = world.read_resource::<ScreenDimensions>();
            let mut camera = world.write_resource::<Camera>();
            camera.projection = Projection::Perspective {
                fov: 110.0,
                aspect_ratio: dimensions.aspect_ratio,
                near: 0.1,
                far: 100.0,
            };
            camera.eye = [0.0, 0.0, -10.0];
            camera.target = [0.0, 0.0, 0.0];
            camera.up = [0.0, 1.0, 0.0];
        }
        asset_manager.register_store(DirectoryStore::new(format!("{}/resources",
                                                                 env!("CARGO_MANIFEST_DIR"))));
        asset_manager.register_asset::<Mesh>();
        asset_manager.register_asset::<Texture>();
        println!("Loading box");
        asset_manager.load_asset::<Mesh>("flat", "obj");
        println!("Box loaded");
        asset_manager.load_asset_from_data::<Texture, [f32; 4]>("dark_blue", [0.1, 0.1, 0.1, 1.0]);
        asset_manager.load_asset_from_data::<Texture, [f32; 4]>("green", [0.9, 0.9, 0.9, 1.0]);
        let flat = asset_manager.create_renderable("flat", "dark_blue", "green").unwrap();
        world.add_resource::<DeltaMosue>(DeltaMosue::new());
        world.add_resource::<InputHandler>(InputHandler::new());
        world.add_resource::<CameraRotation>(CameraRotation {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        });

        world.register::<Moveable>();
        world.register::<Collider>();

        let mut tr = LocalTransform::default();
        tr.translation = [0.0, 10.0, -10.0];
        world.create_now()
            .with::<Renderable>(flat.clone())
            .with(tr)
            .with(Transform::default())
            .build();

        let mut tr = LocalTransform::default();
        tr.translation = [0.0, -3.0, -10.0];
        world.create_now()
            .with::<Renderable>(flat.clone())
            .with(tr)
            .with(Transform::default())
            .build();

        let mut tr = LocalTransform::default();
        tr.translation = [0.0, 0.0, 0.0];
        let mut tr2 = LocalTransform::default();
        tr2.translation = tr.translation;
        let t = world.create_now()
            .with::<Renderable>(flat.clone())
            .with(tr)
            .with(Transform::default())
            .with(Collider::new(ColliderShape::new_cuboid(Vector3::new(0.5, 0.5, 0.5)),
                                ColliderLayer::Static))
            .build();

        let mut tr = LocalTransform::default();
        tr.translation = [0.0, 2.0, 0.0];
        let mut tr2 = LocalTransform::default();
        tr2.translation = tr.translation;
        let t = world.create_now()
            .with::<Renderable>(flat)
            .with(tr)
            .with(Transform::default())
            .with(Moveable { velocity: Vector3::new(0.0, -1.0, 0.0) })
            .with(Collider::new(ColliderShape::new_cuboid(Vector3::new(0.5, 0.5, 0.5)),
                                ColliderLayer::Object))
            .build();

        let light = Light {
            color: [1.0, 1.0, 1.0, 1.0],
            radius: 10.0,
            center: [0.0, 2.0, -3.0],
            propagation_constant: 0.0,
            propagation_linear: 0.0,
            propagation_r_square: 10.0,
        };

        world.create_now()
            .with::<Light>(light)
            .build();

        let sw = StaticWorld::construct(world.entities(),
                                        world.read::<Collider>(),
                                        world.read::<LocalTransform>());
        world.add_resource(sw);
    }

    fn handle_events(&mut self,
                     events: &[WindowEvent],
                     world: &mut World,
                     _: &mut AssetManager,
                     _: &mut Pipeline)
                     -> Trans {
        use amethyst::world_resources::InputHandler;
        use amethyst::event::*;

        let mut input_handler = world.write_resource::<InputHandler>();
        input_handler.update(events);
        for event in events {
            match event.payload {
                Event::MouseMoved(x, y) => {
                    let mut dm = world.write_resource::<DeltaMosue>();
                    let mut camera = world.write_resource::<Camera>();
                    dm.delta_x = x - dm.last_x;
                    dm.delta_y = y - dm.last_y;
                    dm.last_x = x;
                    dm.last_y = y;
                    mouse_look(&mut camera,
                               -dm.delta_x as f32 * 0.01,
                               -dm.delta_y as f32 * 0.01);
                }
                Event::KeyboardInput(_, _, Some(VirtualKeyCode::Escape)) => return Trans::Quit,
                Event::Closed => return Trans::Quit,
                _ => (),
            }
        }
        Trans::None
    }

    fn fixed_update(&mut self,
                    world: &mut World,
                    asset_manager: &mut AssetManager,
                    pipeline: &mut Pipeline)
                    -> Trans {
        use amethyst::world_resources::Time;

        let dt;
        {
            let time = world.read_resource::<Time>();
            dt = time.fixed_step.subsec_nanos() as f32 / 1.0e9;
        }
        let mut physics = world.write_resource::<StaticWorld>();
        let (moveables, mut colliders, mut transforms) =
            (world.read::<Moveable>(), world.write::<Collider>(), world.write::<LocalTransform>());

        for (mov, tr) in (&moveables, &mut transforms).iter() {
            tr.translation = *(Vector3::from(&tr.translation) + mov.velocity * dt).as_ref();
        }

        physics.resolve_collisions(world.entities(), colliders, transforms);

        Trans::None
    }
}

fn main() {
    let path = format!("{}/resources/config.yml", env!("CARGO_MANIFEST_DIR"));
    let display_config = DisplayConfig::from_file(path).unwrap();
    let mut game = Application::build(Game, display_config).done();
    game.run();
}
