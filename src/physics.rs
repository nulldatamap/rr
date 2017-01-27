use ::std::sync::{RwLockReadGuard, RwLockWriteGuard};
use ::nalgebra::{Vector3, Point3, Isometry3, Rotation3};
use ::ncollide::shape;
use ::ncollide::shape::{ShapeHandle3, Shape};
use ::ncollide::bounding_volume::{HasBoundingVolume, AABB};
use ::ncollide::partitioning::{BVT, BoundingVolumeInterferencesCollector};
use ::amethyst::ecs::{Entity, Entities, Component, VecStorage, JoinIter, Join, Storage, Allocator,
                      MaskedStorage};
use ::amethyst::components::transform::{LocalTransform, Transform};

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum ColliderLayer {
    Static,
    Object,
}

pub enum ColliderShape {
    Sphere(shape::Ball<f32>),
    Cuboid(shape::Cuboid3<f32>),
    Mesh(shape::TriMesh<Point3<f32>>),
}

impl ColliderShape {
    pub fn new_cuboid(half_extends: Vector3<f32>) -> ColliderShape {
        ColliderShape::Cuboid(shape::Cuboid3::new(half_extends))
    }

    fn aabb(&self, m: &Isometry3<f32>) -> AABB<Point3<f32>> {
        use self::ColliderShape::*;

        match self {
            &Sphere(ref sp) => sp.aabb(m),
            &Cuboid(ref cu) => cu.aabb(m),
            &Mesh(ref me) => {
                me.bvt()
                    .root_bounding_volume()
                    .expect("Can't use an empty mesh as a collision mesh!")
                    .clone()
            }
        }
    }

    fn shape(&self) -> &Shape<Point3<f32>, Isometry3<f32>> {
        use self::ColliderShape::*;

        match self {
            &Sphere(ref sp) => sp,
            &Cuboid(ref cu) => cu,
            &Mesh(ref me) => me,
        }
    }
}

pub struct Collider {
    shape: ColliderShape,
    layer: ColliderLayer,
}

impl Collider {
    pub fn new(shape: ColliderShape, layer: ColliderLayer) -> Collider {
        Collider {
            shape: shape,
            layer: layer,
        }
    }
}

impl Component for Collider {
    type Storage = VecStorage<Collider>;
}

pub struct StaticWorld {
    bvt: BVT<Entity, AABB<Point3<f32>>>,
}

pub fn local_transform_to_isometry(local: &LocalTransform) -> Isometry3<f32> {
    let rot_vec = [local.rotation[1], local.rotation[2], local.rotation[3]];
    Isometry3 {
        rotation: Rotation3::<f32>::new(Vector3::<f32>::from(&rot_vec)),
        translation: Vector3::<f32>::from(&local.translation),
    }
}

type StorageRead<'a, T> = Storage<T,
                                  RwLockReadGuard<'a, Allocator>,
                                  RwLockReadGuard<'a, MaskedStorage<T>>>;
type StorageWrite<'a, T> = Storage<T,
                                   RwLockReadGuard<'a, Allocator>,
                                   RwLockWriteGuard<'a, MaskedStorage<T>>>;

impl StaticWorld {
    pub fn construct(entities: Entities,
                     mut colliders: StorageRead<Collider>,
                     mut locals: StorageRead<LocalTransform>)
                     -> StaticWorld {
        let leaves: Vec<(Entity, AABB<Point3<f32>>)> = (&entities, &colliders, &locals)
            .iter()
            .filter_map(|(ent, col, tr)| if col.layer == ColliderLayer::Static {
                Some((ent, col.shape.aabb(&local_transform_to_isometry(tr))))
            } else {
                None
            })
            .collect();

        StaticWorld { bvt: BVT::new_balanced(leaves) }
    }

    pub fn resolve_collisions(&mut self,
                              entities: Entities,
                              mut colliders: StorageWrite<Collider>,
                              mut locals: StorageWrite<LocalTransform>) {
        use ::ncollide::query;

        let mut collisions = Vec::new();

        for (ent, col, tr) in (&entities, &colliders, &locals)
            .iter()
            .filter(|&(_, col, _): &(_, &Collider, _)| col.layer == ColliderLayer::Object) {

            let _: &Collider = col;
            let aabb = col.shape.aabb(&local_transform_to_isometry(tr));
            let mut possible_collisions = Vec::new();
            self.bvt
                .visit(&mut BoundingVolumeInterferencesCollector::new(&aabb,
                                                                      &mut possible_collisions));
            collisions.extend(possible_collisions.into_iter().map(|col_ent| (ent, col_ent)));
        }

        for (e, se) in collisions {
            let contact;
            {
                let et = locals.get(e).unwrap();
                let set = locals.get(se).unwrap();
                let ec = colliders.get(e).unwrap();
                let sec = colliders.get(se).unwrap();
                contact = query::contact(&local_transform_to_isometry(et),
                                         ec.shape.shape(),
                                         &local_transform_to_isometry(set),
                                         ec.shape.shape(),
                                         0.0);
            }
            if let Some(c) = contact {
                let et = locals.get_mut(e).unwrap();
                let new_pos = Vector3::from(&et.translation) + c.normal * -c.depth;
                et.translation = *new_pos.as_ref();
                et.flag(true);
            }
        }
    }
}