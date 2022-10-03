using System.Runtime.CompilerServices;
using Unity.Assertions;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Transforms;
using UnityEngine.Profiling;

namespace Rival
{
    public static class PhysicsUtilities
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe bool GetHitFaceNormal(RigidBody hitBody, ColliderKey colliderKey, out float3 faceNormal)
        {
            faceNormal = default;

            if (hitBody.Collider.Value.GetLeaf(colliderKey, out ChildCollider hitChildCollider))
            {
                ColliderType colliderType = hitChildCollider.Collider->Type;

                if (colliderType == ColliderType.Triangle || colliderType == ColliderType.Quad)
                {
                    BlobArray.Accessor<float3> verticesAccessor = ((PolygonCollider*)hitChildCollider.Collider)->Vertices;
                    float3 localFaceNormal = math.normalizesafe(math.cross(verticesAccessor[1] - verticesAccessor[0], verticesAccessor[2] - verticesAccessor[0]));
                    faceNormal = math.rotate(hitBody.WorldFromBody, localFaceNormal);

                    return true;
                }
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool DoesBodyHavePhysicsVelocityAndMass(in CollisionWorld collisionWorld, int rigidbodyIndex)
        {
            if (rigidbodyIndex < collisionWorld.NumDynamicBodies)
            {
                return true;
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool IsBodyKinematic(in ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity, Entity entity)
        {
            if (physicsMassFromEntity.HasComponent(entity) && physicsMassFromEntity[entity].InverseMass <= 0f)
            {
                return true;
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool IsBodyDynamic(in PhysicsWorld physicsWorld, int rigidbodyIndex)
        {
            if (DoesBodyHavePhysicsVelocityAndMass(in physicsWorld.CollisionWorld, rigidbodyIndex))
            {
                if (physicsWorld.MotionVelocities[rigidbodyIndex].InverseMass > 0f)
                {
                    return true;
                }
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool IsBodyDynamic(in ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity, Entity entity)
        {
            if (physicsMassFromEntity.HasComponent(entity) && physicsMassFromEntity[entity].InverseMass > 0f)
            {
                return true;
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool IsCollidable(in Material material)
        {
            if (material.CollisionResponse == CollisionResponsePolicy.Collide ||
                material.CollisionResponse == CollisionResponsePolicy.CollideRaiseCollisionEvents)
            {
                return true;
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool SetCollisionResponse(RigidBody rigidBody, ColliderKey colliderKey, CollisionResponsePolicy collisionResponse)
        {
            if (rigidBody.Collider.Value.GetLeaf(colliderKey, out ChildCollider leafCollider))
            {
                if (leafCollider.Collider->CollisionType == CollisionType.Convex)
                {
                    ConvexCollider* colliderPtr = (ConvexCollider*)leafCollider.Collider;
                    Material material = colliderPtr->Material;
                    material.CollisionResponse = collisionResponse;
                    colliderPtr->Material = material;
                }
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static bool SetCollisionResponse(RigidBody rigidBody, CollisionResponsePolicy collisionResponse)
        {
            if (rigidBody.Collider.Value.CollisionType == CollisionType.Convex)
            {
                ConvexCollider* colliderPtr = (ConvexCollider*)rigidBody.Collider.GetUnsafePtr();
                Material material = colliderPtr->Material;
                material.CollisionResponse = collisionResponse;
                colliderPtr->Material = material;
            }

            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SolveCollisionImpulses(
            in PhysicsVelocity physicsVelA,
            in PhysicsVelocity physicsVelB,
            in PhysicsMass physicsMassA,
            in PhysicsMass physicsMassB,
            in RigidTransform transformA,
            in RigidTransform transformB,
            float3 collisionPoint,
            float3 collisionNormalBToA,
            out float3 impulseOnA,
            out float3 impulseOnB)
        {
            impulseOnA = default;
            impulseOnB = default;

            Translation translationA = new Translation { Value = transformA.pos };
            Translation translationB = new Translation { Value = transformB.pos };
            Rotation rotationA = new Rotation { Value = transformA.rot };
            Rotation rotationB = new Rotation { Value = transformB.rot };

            float3 pointVelocityA = physicsVelA.GetLinearVelocity(physicsMassA, translationA, rotationA, collisionPoint);
            float3 pointVelocityB = physicsVelB.GetLinearVelocity(physicsMassB, translationB, rotationB, collisionPoint);

            float3 centerOfMassA = physicsMassA.GetCenterOfMassWorldSpace(translationA, rotationA);
            float3 centerOfMassB = physicsMassA.GetCenterOfMassWorldSpace(translationB, rotationB);
            float3 centerOfMassAToPoint = collisionPoint - centerOfMassA;
            float3 centerOfMassBToPoint = collisionPoint - centerOfMassB;

            float3 relativeVelocityAToB = pointVelocityB - pointVelocityA;
            float relativeVelocityOnNormal = math.dot(relativeVelocityAToB, collisionNormalBToA);

            float3 crossA = math.cross(centerOfMassAToPoint, collisionNormalBToA);
            float3 crossB = math.cross(collisionNormalBToA, centerOfMassBToPoint);
            float3 angularA = math.mul(new Math.MTransform(transformA).InverseRotation, crossA).xyz;
            float3 angularB = math.mul(new Math.MTransform(transformB).InverseRotation, crossB).xyz;
            float3 temp = angularA * angularA * physicsMassA.InverseInertia + angularB * angularB * physicsMassB.InverseInertia;
            float invEffectiveMass = temp.x + temp.y + temp.z + (physicsMassA.InverseMass + physicsMassB.InverseMass);

            if (invEffectiveMass > 0f)
            {
                float effectiveMass = 1f / invEffectiveMass;

                float impulseScale = -relativeVelocityOnNormal * effectiveMass;
                float3 totalImpulse = collisionNormalBToA * impulseScale;

                impulseOnA = -totalImpulse;
                impulseOnB = totalImpulse;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static PhysicsMass GetKinematicCharacterPhysicsMass(StoredKinematicCharacterBodyProperties characterBodyProperties)
        {
            return new PhysicsMass
            {
                AngularExpansionFactor = 0f,
                InverseInertia = float3.zero,
                InverseMass = characterBodyProperties.SimulateDynamicBody ? (1f / characterBodyProperties.Mass) : 0f,
                Transform = new RigidTransform(quaternion.identity, float3.zero),
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static PhysicsMass GetKinematicCharacterPhysicsMass(KinematicCharacterBody characterBody)
        {
            return new PhysicsMass
            {
                AngularExpansionFactor = 0f,
                InverseInertia = float3.zero,
                InverseMass = characterBody.SimulateDynamicBody ? (1f / characterBody.Mass) : 0f,
                Transform = new RigidTransform(quaternion.identity, float3.zero),
            };
        }
    }
}