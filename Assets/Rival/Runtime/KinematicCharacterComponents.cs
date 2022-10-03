using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Authoring;
using Unity.Transforms;
using UnityEngine;

namespace Rival
{
    [System.Serializable]
    public struct AuthoringKinematicCharacterBody
    {
        [Header("General Properties")]
        [Tooltip("Physics tags to be added to the character's physics body")]
        public CustomPhysicsBodyTags CustomPhysicsBodyTags;
        [Tooltip("Enables interpolating the character's translation between fixed update steps, for smoother movement")]
        public bool InterpolateTranslation;
        [Tooltip("Enables interpolating the character's rotation between fixed update steps, for smoother movement")]
        public bool InterpolateRotation;

        [Header("Grounding")]
        [Tooltip("Enables detecting ground and evaluating grounding for each hit")]
        public bool EvaluateGrounding;
        [Tooltip("Enables snapping to the ground surface below the character")]
        public bool SnapToGround;
        [Tooltip("Distance to snap to ground")]
        public float GroundSnappingDistance;
        [Tooltip("The max slope angle that the character can be considered grounded on")]
        public float MaxGroundedSlopeAngle;

        [Header("Collisions")]
        [Tooltip("Enables detecting and solving movement collisions with a collider cast, based on character's velocity")]
        public bool DetectMovementCollisions;
        [Tooltip("Enables detecting and solving overlaps")]
        public bool DecollideFromOverlaps;
        [Tooltip("Enables doing an extra physics check to project velocity on initial overlaps before the character moves. This can help with tunneling issues with non-circular character colliders, but has a performance cost.")]
        public bool ProjectVelocityOnInitialOverlaps;
        [Tooltip("The maximum amount of times per frame that the character should try to cast its collider for detecting hits")]
        public byte MaxContinuousCollisionsIterations;
        [Tooltip("The maximum amount of times per frame that the character should try to decollide itself from overlaps")]
        public byte MaxOverlapDecollisionIterations;
        [Tooltip("Whether we should reset the remaining move distance to zero when the character exceeds the maximum collision iterations")]
        public bool DiscardMovementWhenExceedMaxIterations;
        [Tooltip("Whether we should reset the velocity to zero when the character exceeds the maximum collision iterations")]
        public bool KillVelocityWhenExceedMaxIterations;
        [Tooltip("Enables doing a collider cast to detect obstructions when being moved by a parent body, instead of simply moving the character transform along")]
        public bool DetectObstructionsForParentBodyMovement;

        [Header("Dynamics")]
        [Tooltip("Enables physics interactions (push and be pushed) with other dynamic bodies")]
        public bool SimulateDynamicBody;
        [Tooltip("The mass used to simulate dynamic body interactions")]
        public float Mass;

        public static AuthoringKinematicCharacterBody GetDefault()
        {
            AuthoringKinematicCharacterBody c = new AuthoringKinematicCharacterBody
            {
                // Body Properties
                CustomPhysicsBodyTags = CustomPhysicsBodyTags.Nothing,
                InterpolateTranslation = true,
                InterpolateRotation = false,

                // Grounding
                EvaluateGrounding = true,
                SnapToGround = true,
                GroundSnappingDistance = 0.3f,
                MaxGroundedSlopeAngle = 60f,

                // Collisions
                DetectMovementCollisions = true,
                DecollideFromOverlaps = true,
                ProjectVelocityOnInitialOverlaps = false,
                MaxContinuousCollisionsIterations = 8,
                MaxOverlapDecollisionIterations = 2,
                DiscardMovementWhenExceedMaxIterations = true,
                KillVelocityWhenExceedMaxIterations = true,
                DetectObstructionsForParentBodyMovement = false,

                // Dynamics
                SimulateDynamicBody = false,
                Mass = 1f,
            };
            return c;
        }
    }

    [System.Serializable]
    public struct KinematicCharacterBody : IComponentData
    {
        public bool EvaluateGrounding;
        public bool SnapToGround;
        public float GroundSnappingDistance;
        public float MaxGroundedSlopeDotProduct;

        public bool DetectMovementCollisions;
        public bool DecollideFromOverlaps;
        public bool ProjectVelocityOnInitialOverlaps;
        public byte MaxContinuousCollisionsIterations;
        public byte MaxOverlapDecollisionIterations;
        public bool DiscardMovementWhenExceedMaxIterations;
        public bool KillVelocityWhenExceedMaxIterations;
        public bool DetectObstructionsForParentBodyMovement;

        public bool SimulateDynamicBody;
        public float Mass;

        [HideInInspector]
        public float3 RelativeVelocity;
        [HideInInspector]
        public bool IsGrounded;
        [HideInInspector]
        public BasicHit GroundHit;
        [HideInInspector]
        public Entity ParentEntity;
        [HideInInspector]
        public float3 ParentVelocity;
        [HideInInspector]
        public float3 ParentAnchorPoint;
        [HideInInspector]
        public quaternion RotationFromParent;
        [HideInInspector]
        public Entity PreviousParentEntity;
        [HideInInspector]
        public bool WasGroundedBeforeCharacterUpdate;

        public KinematicCharacterBody(AuthoringKinematicCharacterBody forAuthoring)
        {
            EvaluateGrounding = forAuthoring.EvaluateGrounding;
            SnapToGround = forAuthoring.SnapToGround;
            GroundSnappingDistance = forAuthoring.GroundSnappingDistance;
            MaxGroundedSlopeDotProduct = MathUtilities.AngleRadiansToDotRatio(math.radians(forAuthoring.MaxGroundedSlopeAngle));

            DetectMovementCollisions = forAuthoring.DetectMovementCollisions;
            DecollideFromOverlaps = forAuthoring.DecollideFromOverlaps;
            ProjectVelocityOnInitialOverlaps = forAuthoring.ProjectVelocityOnInitialOverlaps;
            MaxContinuousCollisionsIterations = forAuthoring.MaxContinuousCollisionsIterations;
            MaxOverlapDecollisionIterations = forAuthoring.MaxOverlapDecollisionIterations;
            DiscardMovementWhenExceedMaxIterations = forAuthoring.DiscardMovementWhenExceedMaxIterations;
            KillVelocityWhenExceedMaxIterations = forAuthoring.KillVelocityWhenExceedMaxIterations;
            DetectObstructionsForParentBodyMovement = forAuthoring.DetectObstructionsForParentBodyMovement;

            SimulateDynamicBody = forAuthoring.SimulateDynamicBody;
            Mass = forAuthoring.Mass;

            RelativeVelocity = default;
            IsGrounded = default;
            ParentEntity = default;
            GroundHit = default;
            ParentVelocity = default;
            ParentAnchorPoint = default;
            RotationFromParent = quaternion.identity;
            PreviousParentEntity = default;
            WasGroundedBeforeCharacterUpdate = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ShouldIgnoreDynamicBodies()
        {
            return !SimulateDynamicBody;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Unground()
        {
            IsGrounded = false;
            GroundHit = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool HasBecomeGrounded()
        {
            return !WasGroundedBeforeCharacterUpdate && IsGrounded;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool HasBecomeUngrounded()
        {
            return WasGroundedBeforeCharacterUpdate && !IsGrounded;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetCollisionDetectionActive(bool active)
        {
            EvaluateGrounding = active;
            DetectMovementCollisions = active;
            DecollideFromOverlaps = active;
        }
    }

    [System.Serializable]
    public struct StoredKinematicCharacterBodyProperties : IComponentData
    {
        public bool SimulateDynamicBody;
        public float Mass;
        public float3 RelativeVelocity;
        public float3 ParentVelocity;

        public void FromCharacterBody(in KinematicCharacterBody characterBody)
        {
            SimulateDynamicBody = characterBody.SimulateDynamicBody;
            Mass = characterBody.Mass;
            RelativeVelocity = characterBody.RelativeVelocity;
            ParentVelocity = characterBody.ParentVelocity;
        }
    }

    [Serializable]
    [InternalBufferCapacity(1)]
    public struct KinematicCharacterDeferredImpulse : IBufferElementData
    {
        public Entity OnEntity;
        public float3 LinearVelocityChange;
        public float3 AngularVelocityChange;
        public float3 Displacement;
    }

    [Serializable]
    [InternalBufferCapacity(2)]
    public struct KinematicCharacterHit : IBufferElementData
    {
        public Entity Entity;
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
        public float3 Position;
        public float3 Normal;
        public Unity.Physics.Material Material;
        public bool WasCharacterGroundedOnHitEnter;
        public bool IsGroundedOnHit;
        public float3 CharacterVelocityBeforeHit;
        public float3 CharacterVelocityAfterHit;
    }

    [Serializable]
    [InternalBufferCapacity(2)]
    public struct KinematicVelocityProjectionHit : IBufferElementData
    {
        public Entity Entity;
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
        public float3 Position;
        public float3 Normal;
        public Unity.Physics.Material Material;
        public bool IsGroundedOnHit;

        public KinematicVelocityProjectionHit(KinematicCharacterHit hit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.Normal;
            Material = hit.Material;
            IsGroundedOnHit = hit.IsGroundedOnHit;
        }

        public KinematicVelocityProjectionHit(BasicHit hit, bool isGroundedOnHit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.Normal;
            Material = hit.Material;
            IsGroundedOnHit = isGroundedOnHit;
        }

        public KinematicVelocityProjectionHit(float3 normal, float3 position, bool isGroundedOnHit)
        {
            Entity = Entity.Null;
            RigidBodyIndex = -1;
            ColliderKey = default;
            Position = position;
            Normal = normal;
            Material = default;
            IsGroundedOnHit = isGroundedOnHit;
        }
    }

    [Serializable]
    [InternalBufferCapacity(2)]
    public struct StatefulKinematicCharacterHit : IBufferElementData
    {
        public CharacterHitState State;
        public KinematicCharacterHit Hit;

        public StatefulKinematicCharacterHit(KinematicCharacterHit characterHit)
        {
            State = default;
            Hit = characterHit;
        }
    }
}