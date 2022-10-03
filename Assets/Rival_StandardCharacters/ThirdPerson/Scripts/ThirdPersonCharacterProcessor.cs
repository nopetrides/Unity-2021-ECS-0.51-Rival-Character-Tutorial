using ECS.Components;
using Unity.Physics.Authoring;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Rival;
using Unity.Physics;

public struct ThirdPersonCharacterProcessor : IKinematicCharacterProcessor
{
    public float DeltaTime;
    public CollisionWorld CollisionWorld;

    public ComponentDataFromEntity<StoredKinematicCharacterBodyProperties> StoredKinematicCharacterBodyPropertiesFromEntity;
    public ComponentDataFromEntity<PhysicsMass> PhysicsMassFromEntity;
    public ComponentDataFromEntity<PhysicsVelocity> PhysicsVelocityFromEntity;
    public ComponentDataFromEntity<TrackedTransform> TrackedTransformFromEntity;
    public ComponentDataFromEntity<CharacterFrictionSurface> CharacterFrictionSurfaceFromEntity;

    public NativeList<int> TmpRigidbodyIndexesProcessed;
    public NativeList<RaycastHit> TmpRaycastHits;
    public NativeList<ColliderCastHit> TmpColliderCastHits;
    public NativeList<DistanceHit> TmpDistanceHits;

    public Entity Entity;
    public float3 Translation;
    public quaternion Rotation;
    public PhysicsCollider PhysicsCollider;
    public KinematicCharacterBody CharacterBody;
    public ThirdPersonCharacterComponent ThirdPersonCharacter;
    public ThirdPersonCharacterInputs ThirdPersonCharacterInputs;

    public float3 GroundingUp;

    public DynamicBuffer<KinematicCharacterHit> CharacterHitsBuffer;
    public DynamicBuffer<KinematicCharacterDeferredImpulse> CharacterDeferredImpulsesBuffer;
    public DynamicBuffer<KinematicVelocityProjectionHit> VelocityProjectionHitsBuffer;
    public DynamicBuffer<StatefulKinematicCharacterHit> StatefulCharacterHitsBuffer;

    #region Processor Getters
    public CollisionWorld GetCollisionWorld => CollisionWorld;
    public ComponentDataFromEntity<StoredKinematicCharacterBodyProperties> GetStoredCharacterBodyPropertiesFromEntity => StoredKinematicCharacterBodyPropertiesFromEntity;
    public ComponentDataFromEntity<PhysicsMass> GetPhysicsMassFromEntity => PhysicsMassFromEntity;
    public ComponentDataFromEntity<PhysicsVelocity> GetPhysicsVelocityFromEntity => PhysicsVelocityFromEntity;
    public ComponentDataFromEntity<TrackedTransform> GetTrackedTransformFromEntity => TrackedTransformFromEntity;
    public NativeList<int> GetTmpRigidbodyIndexesProcessed => TmpRigidbodyIndexesProcessed;
    public NativeList<RaycastHit> GetTmpRaycastHits => TmpRaycastHits;
    public NativeList<ColliderCastHit> GetTmpColliderCastHits => TmpColliderCastHits;
    public NativeList<DistanceHit> GetTmpDistanceHits => TmpDistanceHits;
    #endregion

    #region Processor Callbacks
    public bool CanCollideWithHit(in BasicHit hit)
    {
        bool canCollide = true;
        if (!KinematicCharacterUtilities.DefaultMethods.CanCollideWithHit(in hit,
                in StoredKinematicCharacterBodyPropertiesFromEntity))
        {
            canCollide = false;
        }
        else if (ThirdPersonCharacter.IgnoredPhysicsTags.Value > CustomPhysicsBodyTags.Nothing.Value)
        {
            if ((CollisionWorld.Bodies[hit.RigidBodyIndex].CustomTags & ThirdPersonCharacter.IgnoredPhysicsTags.Value) 
                > 0)
            {
                canCollide = false;
            }
        }
        return canCollide;
    }

    public bool IsGroundedOnHit(in BasicHit hit, int groundingEvaluationType)
    {
        return KinematicCharacterUtilities.DefaultMethods.IsGroundedOnHit(
            ref this,
            in hit,
            in CharacterBody,
            in PhysicsCollider,
            Entity,
            ThirdPersonCharacter.GroundingUp,
            groundingEvaluationType,
            ThirdPersonCharacter.StepHandling,
            ThirdPersonCharacter.MaxStepHeight,
            ThirdPersonCharacter.ExtraStepChecksDistance);
    }

    public void OnMovementHit(
            ref KinematicCharacterHit hit,
            ref float3 remainingMovementDirection,
            ref float remainingMovementLength,
            float3 originalVelocityDirection,
            float hitDistance)
    {
        KinematicCharacterUtilities.DefaultMethods.OnMovementHit(
            ref this,
            ref hit,
            ref CharacterBody,
            ref VelocityProjectionHitsBuffer,
            ref Translation,
            ref remainingMovementDirection,
            ref remainingMovementLength,
            in PhysicsCollider,
            Entity,
            Rotation,
            ThirdPersonCharacter.GroundingUp,
            originalVelocityDirection,
            hitDistance,
            ThirdPersonCharacter.StepHandling,
            ThirdPersonCharacter.MaxStepHeight);
    }

    public void OverrideDynamicHitMasses(
        ref PhysicsMass characterMass,
        ref PhysicsMass otherMass,
        Entity characterEntity,
        Entity otherEntity,
        int otherRigidbodyIndex)
    {
    }

    public void ProjectVelocityOnHits(
        ref float3 velocity,
        ref bool characterIsGrounded,
        ref BasicHit characterGroundHit,
        in DynamicBuffer<KinematicVelocityProjectionHit> hits,
        float3 originalVelocityDirection)
    {
        // The last hit in the "hits" buffer is the latest hit. The rest of the hits are all hits so far in the movement iterations
        KinematicCharacterUtilities.DefaultMethods.ProjectVelocityOnHits(
            ref velocity,
            ref characterIsGrounded,
            ref characterGroundHit,
            in hits,
            originalVelocityDirection,
            ThirdPersonCharacter.GroundingUp,
            ThirdPersonCharacter.ConstrainVelocityToGroundPlane);
    }
    #endregion

    public unsafe void OnUpdate()
    {
        ThirdPersonCharacter.GroundingUp = -math.normalizesafe(ThirdPersonCharacter.Gravity);

        KinematicCharacterUtilities.InitializationUpdate(ref CharacterBody, ref CharacterHitsBuffer, ref VelocityProjectionHitsBuffer, ref CharacterDeferredImpulsesBuffer);
        KinematicCharacterUtilities.ParentMovementUpdate(ref this, ref Translation, ref CharacterBody, in PhysicsCollider, DeltaTime, Entity, Rotation, ThirdPersonCharacter.GroundingUp, CharacterBody.WasGroundedBeforeCharacterUpdate); // safe to remove if not needed
        KinematicCharacterUtilities.GroundingUpdate(ref this, ref Translation, ref CharacterBody, ref CharacterHitsBuffer, ref VelocityProjectionHitsBuffer, in PhysicsCollider, Entity, Rotation, ThirdPersonCharacter.GroundingUp);

		// Character velocity control is updated AFTER the ground has been detected, but BEFORE the character tries to move & collide with that velocity
        HandleCharacterControl();

        PreventGroundingFromFutureSlopeChange();

        if (CharacterBody.IsGrounded && CharacterBody.SimulateDynamicBody)
        {
            KinematicCharacterUtilities.DefaultMethods.UpdateGroundPushing(ref this, ref CharacterDeferredImpulsesBuffer, ref CharacterBody, DeltaTime, Entity, ThirdPersonCharacter.Gravity, Translation, Rotation, 1f); // safe to remove if not needed
        }

        KinematicCharacterUtilities.MovementAndDecollisionsUpdate(ref this, ref Translation, ref CharacterBody, ref CharacterHitsBuffer, ref VelocityProjectionHitsBuffer, ref CharacterDeferredImpulsesBuffer, in PhysicsCollider, DeltaTime, Entity, Rotation, ThirdPersonCharacter.GroundingUp);
        KinematicCharacterUtilities.DefaultMethods.MovingPlatformDetection(ref TrackedTransformFromEntity, ref StoredKinematicCharacterBodyPropertiesFromEntity, ref CharacterBody); // safe to remove if not needed
        KinematicCharacterUtilities.ParentMomentumUpdate(ref TrackedTransformFromEntity, ref CharacterBody, in Translation, DeltaTime, ThirdPersonCharacter.GroundingUp); // safe to remove if not needed
        KinematicCharacterUtilities.ProcessStatefulCharacterHits(ref StatefulCharacterHitsBuffer, in CharacterHitsBuffer); // safe to remove if not needed
    }

    public unsafe void PreventGroundingFromFutureSlopeChange()
    {
        if (CharacterBody.IsGrounded && (ThirdPersonCharacter.PreventGroundingWhenMovingTowardsNoGrounding || ThirdPersonCharacter.HasMaxDownwardSlopeChangeAngle))
        {
            KinematicCharacterUtilities.DefaultMethods.DetectFutureSlopeChange(
                ref this,
                in CharacterBody.GroundHit,
                in CharacterBody,
                in PhysicsCollider,
                Entity,
                CharacterBody.RelativeVelocity,
                ThirdPersonCharacter.GroundingUp,
                0.05f, // verticalOffset
                0.05f, // downDetectionDepth
                DeltaTime, // deltaTimeIntoFuture
                0.25f, // secondaryNoGroundingCheckDistance
                ThirdPersonCharacter.StepHandling,
                ThirdPersonCharacter.MaxStepHeight,
                out bool isMovingTowardsNoGrounding,
                out bool foundSlopeHit,
                out float futureSlopeChangeAnglesRadians,
                out RaycastHit futureSlopeHit);
            if ((ThirdPersonCharacter.PreventGroundingWhenMovingTowardsNoGrounding && isMovingTowardsNoGrounding) ||
                (ThirdPersonCharacter.HasMaxDownwardSlopeChangeAngle && foundSlopeHit && math.degrees(futureSlopeChangeAnglesRadians) < -ThirdPersonCharacter.MaxDownwardSlopeChangeAngle))
            {
                CharacterBody.IsGrounded = false;
            }
        }
    }

    public unsafe void HandleCharacterControl()
    {
        if (CharacterBody.IsGrounded)
        {
            // Move on ground
            float3 targetVelocity = ThirdPersonCharacterInputs.MoveVector * ThirdPersonCharacter.GroundMaxSpeed;

            if (ThirdPersonCharacterInputs.Spriting)
            {
                targetVelocity *= ThirdPersonCharacter.SprintSpeedMultiplier;
            }

            if (CharacterFrictionSurfaceFromEntity.HasComponent(CharacterBody.GroundHit.Entity))
            {
                targetVelocity *= CharacterFrictionSurfaceFromEntity[CharacterBody.GroundHit.Entity].VelocityFactor;
            }
            
            // Apply final velocity
            CharacterControlUtilities.StandardGroundMove_Interpolated(ref CharacterBody.RelativeVelocity, targetVelocity, ThirdPersonCharacter.GroundedMovementSharpness, DeltaTime, ThirdPersonCharacter.GroundingUp, CharacterBody.GroundHit.Normal);

            // Jump
            if (ThirdPersonCharacterInputs.JumpRequested && ThirdPersonCharacter.CurrentJumpsUsed < ThirdPersonCharacter.TotalJumps)
            {
                CharacterControlUtilities.StandardJump(ref CharacterBody, ThirdPersonCharacter.GroundingUp * ThirdPersonCharacter.JumpSpeed, true, ThirdPersonCharacter.GroundingUp);
                //increment jumps used
                ThirdPersonCharacter.CurrentJumpsUsed++;
            }
            else
            {
                ThirdPersonCharacter.CurrentJumpsUsed = 0;
            }

        }
        else
        {
            // Move in air
            float3 airAcceleration = ThirdPersonCharacterInputs.MoveVector * ThirdPersonCharacter.AirAcceleration;
            CharacterControlUtilities.StandardAirMove(ref CharacterBody.RelativeVelocity, airAcceleration, ThirdPersonCharacter.AirMaxSpeed, ThirdPersonCharacter.GroundingUp, DeltaTime, false);

            if (ThirdPersonCharacterInputs.JumpRequested && ThirdPersonCharacter.CurrentJumpsUsed < ThirdPersonCharacter.TotalJumps)
            {
                CharacterControlUtilities.StandardJump(ref CharacterBody, ThirdPersonCharacter.GroundingUp * ThirdPersonCharacter.JumpSpeed, true, ThirdPersonCharacter.GroundingUp);
                //increment jumps used
                ThirdPersonCharacter.CurrentJumpsUsed++;
            }
            
            // Gravity
            CharacterControlUtilities.AccelerateVelocity(ref CharacterBody.RelativeVelocity, ThirdPersonCharacter.Gravity, DeltaTime);

            // Drag
            CharacterControlUtilities.ApplyDragToVelocity(ref CharacterBody.RelativeVelocity, DeltaTime, ThirdPersonCharacter.AirDrag);
        }
    }
}
