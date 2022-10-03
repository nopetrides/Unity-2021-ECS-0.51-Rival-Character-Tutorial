using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Authoring;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Profiling;
using Unity.Transforms;

namespace Rival
{
    public enum CharacterHitState
    {
        Enter,
        Stay,
        Exit,
    }

    public enum GroundingEvaluationType
    {
        Default,
        GroundProbing,
        OverlapDecollision,
        InitialOverlaps,
        MovementHit,
        StepUpHit,
    }

    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(ExportPhysicsWorld))]
    [UpdateBefore(typeof(EndFramePhysicsSystem))]
    [UpdateAfter(typeof(TrackedTransformFixedSimulationSystem))]
    public class KinematicCharacterUpdateGroup : ComponentSystemGroup
    {
    }

    public interface IKinematicCharacterProcessor
    {
        CollisionWorld GetCollisionWorld { get; }
        ComponentDataFromEntity<StoredKinematicCharacterBodyProperties> GetStoredCharacterBodyPropertiesFromEntity { get; }
        ComponentDataFromEntity<PhysicsMass> GetPhysicsMassFromEntity { get; }
        ComponentDataFromEntity<PhysicsVelocity> GetPhysicsVelocityFromEntity { get; }
        ComponentDataFromEntity<TrackedTransform> GetTrackedTransformFromEntity { get; }
        NativeList<int> GetTmpRigidbodyIndexesProcessed { get; }
        NativeList<RaycastHit> GetTmpRaycastHits { get; }
        NativeList<ColliderCastHit> GetTmpColliderCastHits { get; }
        NativeList<DistanceHit> GetTmpDistanceHits { get; }

        bool CanCollideWithHit(in BasicHit hit);

        bool IsGroundedOnHit(in BasicHit hit, int groundingEvaluationType);

        void OnMovementHit(
            ref KinematicCharacterHit hit,
            ref float3 remainingMovementDirection,
            ref float remainingMovementLength,
            float3 originalVelocityDirection,
            float hitDistance);

        void ProjectVelocityOnHits(
            ref float3 velocity,
            ref bool characterIsGrounded,
            ref BasicHit characterGroundHit,
            in DynamicBuffer<KinematicVelocityProjectionHit> hits,
            float3 originalVelocityDirection);

        void OverrideDynamicHitMasses(
            ref PhysicsMass characterMass,
            ref PhysicsMass otherMass,
            Entity characterEntity,
            Entity otherEntity,
            int otherRigidbodyIndex);
    }

    public struct HitFractionComparer : IComparer<ColliderCastHit>
    {
        public int Compare(ColliderCastHit x, ColliderCastHit y)
        {
            if (x.Fraction > y.Fraction)
            {
                return 1;
            }
            else if (x.Fraction < y.Fraction)
            {
                return -1;
            }
            else
            {
                return 0;
            }
        }
    }

    [System.Serializable]
    public struct BasicHit
    {
        public Entity Entity;
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
        public float3 Position;
        public float3 Normal;
        public Material Material;

        public BasicHit(RaycastHit hit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.SurfaceNormal;
            Material = hit.Material;
        }

        public BasicHit(ColliderCastHit hit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.SurfaceNormal;
            Material = hit.Material;
        }

        public BasicHit(DistanceHit hit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.SurfaceNormal;
            Material = hit.Material;
        }

        public BasicHit(KinematicCharacterHit hit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.Normal;
            Material = hit.Material;
        }

        public BasicHit(KinematicVelocityProjectionHit hit)
        {
            Entity = hit.Entity;
            RigidBodyIndex = hit.RigidBodyIndex;
            ColliderKey = hit.ColliderKey;
            Position = hit.Position;
            Normal = hit.Normal;
            Material = hit.Material;
        }
    }

    public static class KinematicCharacterUtilities
    {
        public struct Constants
        {
            public const float CollisionOffset = 0.01f;
            public const float MinVelocityLengthSqForGroundingIgnoreCheck = 0.01f * 0.01f;
            public const float DotProductSimilarityEpsilon = 0.001f;
            public const float DefaultReverseProjectionMaxLengthRatio = 10f;
            public const float GroundedHitDistanceTolerance = CollisionOffset * 6f;
            public const float GroundedHitDistanceToleranceSq = GroundedHitDistanceTolerance * GroundedHitDistanceTolerance;
            public const float StepGroundingDetectionHorizontalOffset = 0.01f;
            public const float MinDotRatioForVerticalDecollision = 0.1f;
        }

#if RIVAL_PROFILING
        public const string ProfilingPrefix = "KinematicCharacter.ProfilerMarkers";
        private static readonly ProfilerMarker _groundDetectionProfilerMarker = new ProfilerMarker(ProfilingPrefix + "GroundDetection");
        private static readonly ProfilerMarker _moveProfilerMarker = new ProfilerMarker(ProfilingPrefix + "Move");
        private static readonly ProfilerMarker _decollideProfilerMarker = new ProfilerMarker(ProfilingPrefix + "Decollide");
        private static readonly ProfilerMarker _dynamicsProfilerMarker = new ProfilerMarker(ProfilingPrefix + "Dynamics");
        private static readonly ProfilerMarker _isGroundedOnHitProfilerMarker = new ProfilerMarker(ProfilingPrefix + "IsGroundedOnHit");
#endif

        public static ComponentType[] GetCoreCharacterComponentTypes()
        {
            return new ComponentType[]
            {
                typeof(Translation),
                typeof(Rotation),
                typeof(PhysicsCollider),
                typeof(PhysicsVelocity),
                typeof(PhysicsMass),
                typeof(KinematicCharacterBody),
                typeof(StoredKinematicCharacterBodyProperties),
                typeof(KinematicCharacterHit),
                typeof(KinematicVelocityProjectionHit),
                typeof(KinematicCharacterDeferredImpulse),
                typeof(StatefulKinematicCharacterHit),
            };
        }

        /// <summary>
        /// Adds all the required character components to an entity
        /// </summary>
        /// <param name="dstManager"></param>
        /// <param name="entity"></param>
        /// <param name="authoringProperties"></param> 
        public static void CreateCharacter(
            EntityManager dstManager,
            Entity entity,
            AuthoringKinematicCharacterBody authoringProperties)
        {
            // Base character components
            dstManager.AddComponentData(entity, new KinematicCharacterBody(authoringProperties));
            dstManager.AddComponentData(entity, new StoredKinematicCharacterBodyProperties());

            var characterHitsBuffer = dstManager.AddBuffer<KinematicCharacterHit>(entity);
            var velocityProjectionHitsBuffer = dstManager.AddBuffer<KinematicVelocityProjectionHit>(entity);
            var deferredImpulsesBuffer = dstManager.AddBuffer<KinematicCharacterDeferredImpulse>(entity);
            var statefulHitsBuffer = dstManager.AddBuffer<StatefulKinematicCharacterHit>(entity);

            // Kinematic physics body components
            dstManager.AddComponentData(entity, new PhysicsVelocity());
            dstManager.AddComponentData(entity, PhysicsMass.CreateKinematic(MassProperties.UnitSphere));
            dstManager.AddComponentData(entity, new PhysicsGravityFactor { Value = 0f });
            dstManager.AddComponentData(entity, new PhysicsCustomTags { Value = authoringProperties.CustomPhysicsBodyTags.Value });

            // Interpolation
            if (authoringProperties.InterpolateTranslation || authoringProperties.InterpolateRotation)
            {
                dstManager.AddComponentData(entity, new CharacterInterpolation
                {
                    InterpolateRotation = authoringProperties.InterpolateRotation ? (byte)1 : (byte)0,
                    InterpolateTranslation = authoringProperties.InterpolateTranslation ? (byte)1 : (byte)0,
                });
            }
        }

        /// <summary>
        /// Adds all the required character components to an entity
        /// </summary>
        /// <param name="commandBuffer"></param>
        /// <param name="entity"></param>
        /// <param name="authoringProperties"></param>
        public static void CreateCharacter(
            EntityCommandBuffer commandBuffer,
            Entity entity,
            AuthoringKinematicCharacterBody authoringProperties)
        {
            // Base character components
            commandBuffer.AddComponent(entity, new KinematicCharacterBody(authoringProperties));
            commandBuffer.AddComponent(entity, new StoredKinematicCharacterBodyProperties());

            var characterHitsBuffer = commandBuffer.AddBuffer<KinematicCharacterHit>(entity);
            var velocityProjectionHitsBuffer = commandBuffer.AddBuffer<KinematicVelocityProjectionHit>(entity);
            var deferredImpulsesBuffer = commandBuffer.AddBuffer<KinematicCharacterDeferredImpulse>(entity);
            var statefulHitsBuffer = commandBuffer.AddBuffer<StatefulKinematicCharacterHit>(entity);

            // Kinematic physics body components
            commandBuffer.AddComponent(entity, new PhysicsVelocity());
            commandBuffer.AddComponent(entity, PhysicsMass.CreateKinematic(MassProperties.UnitSphere));
            commandBuffer.AddComponent(entity, new PhysicsGravityFactor { Value = 0f });
            commandBuffer.AddComponent(entity, new PhysicsCustomTags { Value = authoringProperties.CustomPhysicsBodyTags.Value });

            // Interpolation
            if (authoringProperties.InterpolateTranslation || authoringProperties.InterpolateRotation)
            {
                commandBuffer.AddComponent(entity, new CharacterInterpolation());
            }
        }

        /// <summary>
        /// handles the conversion from GameObject to Entity for a character
        /// </summary>
        /// <param name="dstManager"></param>
        /// <param name="entity"></param>
        /// <param name="authoringGameObject"></param>
        /// <param name="authoringProperties"></param>
        public static void HandleConversionForCharacter(
            EntityManager dstManager,
            Entity entity,
            UnityEngine.GameObject authoringGameObject,
            AuthoringKinematicCharacterBody authoringProperties)
        {
            if (authoringGameObject.transform.lossyScale != UnityEngine.Vector3.one)
            {
                UnityEngine.Debug.LogError("ERROR: kinematic character objects do not support having a scale other than (1,1,1). Conversion will be aborted");
                return;
            }
            if (authoringGameObject.gameObject.GetComponent<PhysicsBodyAuthoring>() != null)
            {
                UnityEngine.Debug.LogError("ERROR: kinematic character objects cannot have a PhysicsBodyAuthoring component. The correct physics components will be setup automatically during conversion. Conversion will be aborted");
                return;
            }

            CreateCharacter(dstManager, entity, authoringProperties);
        }

        /// <summary>
        ///  clears some core character variables and buffers at the start of the update.
        /// </summary>
        /// <param name="characterBody"></param>
        /// <param name="characterHitsBuffer"></param>
        /// <param name="velocityProjectionHitsBuffer"></param>
        /// <param name="characterDeferredImpulsesBuffer"></param>
        /// <param name="characterRotation"></param>
        public static void InitializationUpdate(
            ref KinematicCharacterBody characterBody,
            ref DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            ref DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHitsBuffer,
            ref DynamicBuffer<KinematicCharacterDeferredImpulse> characterDeferredImpulsesBuffer)
        {
            // Initialize data for update
            velocityProjectionHitsBuffer.Clear();
            characterHitsBuffer.Clear();
            characterDeferredImpulsesBuffer.Clear();

            characterBody.WasGroundedBeforeCharacterUpdate = characterBody.IsGrounded;
            characterBody.ParentVelocity = default;
            characterBody.RotationFromParent = quaternion.identity;
            characterBody.PreviousParentEntity = characterBody.ParentEntity;
            characterBody.Unground();
        }

        /// <summary>
        /// handles moving the character based on its currently-assigned ParentEntity, if any.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="processor"></param>
        /// <param name="characterTranslation"></param>
        /// <param name="characterRotation"></param>
        /// <param name="characterBody"></param>
        /// <param name="groundingUp"></param>
        /// <param name="characterPhysicsCollider"></param>
        /// <param name="deltaTime"></param>
        /// <param name="characterEntity"></param>
        public static unsafe void ParentMovementUpdate<T>(
            ref T processor,
            ref float3 characterTranslation,
            ref KinematicCharacterBody characterBody,
            in PhysicsCollider characterPhysicsCollider,
            float deltaTime,
            Entity characterEntity,
            quaternion characterRotation,
            float3 groundingUp,
            bool constrainRotationToGroundingUp) where T : struct, IKinematicCharacterProcessor
        {
            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            ComponentDataFromEntity<TrackedTransform> trackedTransformFromEntity = processor.GetTrackedTransformFromEntity;
            NativeList<ColliderCastHit> tmpColliderCastHits = processor.GetTmpColliderCastHits;

            // Reset parent if parent entity doesn't exist anymore
            if (characterBody.ParentEntity != Entity.Null && !trackedTransformFromEntity.HasComponent(characterBody.ParentEntity))
            {
                characterBody.ParentEntity = Entity.Null;
            }
            if (characterBody.PreviousParentEntity != Entity.Null && !trackedTransformFromEntity.HasComponent(characterBody.PreviousParentEntity))
            {
                characterBody.PreviousParentEntity = Entity.Null;
            }

            // Movement from parent body
            // This needs to be after physics update in order to use the true solved position of the parent body as a target (otherwise there is drifting when applying a force to a dynamic body we are standing on)
            // And this also needs to be before grounding, because we must move to our new location before detecting the up-to-date ground for this frame
            characterBody.ParentVelocity = default;
            if (characterBody.ParentEntity != Entity.Null)
            {
                TrackedTransform parentTrackedTransform = trackedTransformFromEntity[characterBody.ParentEntity];

                // Translation
                float3 previousLocalTranslation = math.transform(math.inverse(parentTrackedTransform.PreviousFixedRateTransform), characterTranslation);
                float3 targetTranslation = math.transform(parentTrackedTransform.CurrentFixedRateTransform, previousLocalTranslation);

                // Anchor point
                float3 previousLocalAnchorPoint = math.transform(math.inverse(parentTrackedTransform.PreviousFixedRateTransform), characterBody.ParentAnchorPoint);
                float3 targetAnchorPoint = math.transform(parentTrackedTransform.CurrentFixedRateTransform, previousLocalAnchorPoint);

                // Rotation
                quaternion previousLocalRotation = math.mul(math.inverse(parentTrackedTransform.PreviousFixedRateTransform.rot), characterRotation);
                quaternion targetRotation = math.mul(parentTrackedTransform.CurrentFixedRateTransform.rot, previousLocalRotation);

                // Rotation up correction
                if (constrainRotationToGroundingUp)
                {
                    quaternion correctedRotation = MathUtilities.CreateRotationWithUpPriority(groundingUp, MathUtilities.GetForwardFromRotation(targetRotation));
                    MathUtilities.SetRotationAroundPoint(ref targetRotation, ref targetTranslation, targetAnchorPoint, correctedRotation);
                }
                 
                // Store data about parent movement
                float3 displacementFromParentMovement = targetTranslation - characterTranslation;
                characterBody.ParentVelocity = (targetAnchorPoint - characterBody.ParentAnchorPoint) / deltaTime;
                characterBody.RotationFromParent = math.mul(math.inverse(characterRotation), targetRotation);

                // Move translation
                if (characterBody.DetectMovementCollisions &&
                    characterBody.DetectObstructionsForParentBodyMovement &&
                    math.lengthsq(displacementFromParentMovement) > math.EPSILON)
                {
                    float3 castDirection = math.normalizesafe(displacementFromParentMovement);
                    float castLength = math.length(displacementFromParentMovement);

                    ColliderCastInput castInput = new ColliderCastInput(characterPhysicsCollider.Value, characterTranslation, characterTranslation + (castDirection * castLength), characterRotation);
                    tmpColliderCastHits.Clear();
                    AllHitsCollector<ColliderCastHit> collector = new AllHitsCollector<ColliderCastHit>(1f, ref tmpColliderCastHits);
                    collisionWorld.CastCollider(castInput, ref collector);
                    if (FilterColliderCastHitsForMove(ref processor, ref tmpColliderCastHits, characterBody.ShouldIgnoreDynamicBodies(), !characterBody.SimulateDynamicBody, characterEntity, castDirection, characterBody.ParentEntity, out ColliderCastHit closestHit, out bool foundAnyOverlaps))
                    {
                        characterTranslation += castDirection * closestHit.Fraction * castLength;
                    }
                    else
                    {
                        characterTranslation += displacementFromParentMovement;
                    }
                }
                else
                {
                    characterTranslation += displacementFromParentMovement;
                }
            }
        }

        /// <summary>
        /// handles detecting character grounding
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="processor"></param>
        /// <param name="characterTranslation"></param>
        /// <param name="characterRotation"></param>
        /// <param name="characterBody"></param>
        /// <param name="characterHitsBuffer"></param>
        /// <param name="velocityProjectionHitsBuffer"></param>
        /// <param name="characterPhysicsCollider"></param>
        /// <param name="characterEntity"></param>
        /// <param name="groundingUp"></param>
        public static unsafe void GroundingUpdate<T>(
            ref T processor,
            ref float3 characterTranslation,
            ref KinematicCharacterBody characterBody,
            ref DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            ref DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHitsBuffer,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            quaternion characterRotation,
            float3 groundingUp) where T : struct, IKinematicCharacterProcessor
        {
            // Detect ground
            bool newIsGrounded = false;
            BasicHit newGroundHit = default;
            if (characterBody.EvaluateGrounding)
            {
#if STRIDRIVAL_PROFILINGER_PROFILING
                _groundDetectionProfilerMarker.Begin();
#endif

                // Calculate ground probe length based on circumstances
                float groundDetectionLength = Constants.CollisionOffset * 3f;
                if (characterBody.SnapToGround && characterBody.WasGroundedBeforeCharacterUpdate)
                {
                    groundDetectionLength = characterBody.GroundSnappingDistance;
                }

                GroundDetection(
                    ref processor,
                    in characterBody,
                    in characterPhysicsCollider,
                    characterEntity,
                    characterTranslation,
                    characterRotation,
                    groundingUp,
                    groundDetectionLength,
                    out newIsGrounded,
                    out newGroundHit,
                    out float distanceToGround);

                // Ground snapping
                if (characterBody.SnapToGround && newIsGrounded)
                {
                    characterTranslation -= groundingUp * distanceToGround;
                    characterTranslation += groundingUp * Constants.CollisionOffset;
                }

                // Add ground hit as a character hit and project velocity
                if (newIsGrounded)
                {
                    KinematicCharacterHit groundCharacterHit = CreateCharacterHit(
                        in newGroundHit,
                        characterBody.WasGroundedBeforeCharacterUpdate,
                        characterBody.RelativeVelocity,
                        newIsGrounded);
                    velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit(groundCharacterHit));

                    bool tmpIsGrounded = characterBody.WasGroundedBeforeCharacterUpdate;
                    processor.ProjectVelocityOnHits(
                        ref characterBody.RelativeVelocity,
                        ref tmpIsGrounded,
                        ref newGroundHit, // in theory this should be the previous ground hit instead, but since it will be a ground-to-ground projection, it doesn't matter. Using previous ground normal here would force us to sync it for networking
                        in velocityProjectionHitsBuffer,
                        math.normalizesafe(characterBody.RelativeVelocity));

                    groundCharacterHit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;
                    characterHitsBuffer.Add(groundCharacterHit);
                }

#if RIVAL_PROFILING
                _groundDetectionProfilerMarker.End();
#endif
            }

            characterBody.IsGrounded = newIsGrounded;
            characterBody.GroundHit = newGroundHit;
        }

        /// <summary>
        /// handles moving the character and solving collisions, based on `KinematicCharacterBody.RelativeVelocity`, rotation, character grounding, and various other properties in `KinematicCharacterBody`.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="processor"></param>
        /// <param name="characterTranslation"></param>
        /// <param name="characterBody"></param>
        /// <param name="characterHitsBuffer"></param>
        /// <param name="velocityProjectionHitsBuffer"></param>
        /// <param name="characterDeferredImpulsesBuffer"></param>
        /// <param name="characterPhysicsCollider"></param>
        /// <param name="deltaTime"></param>
        /// <param name="characterEntity"></param>
        /// <param name="characterRotation"></param>
        /// <param name="groundingUp"></param>
        public static void MovementAndDecollisionsUpdate<T>(
            ref T processor,
            ref float3 characterTranslation,
            ref KinematicCharacterBody characterBody,
            ref DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            ref DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHitsBuffer,
            ref DynamicBuffer<KinematicCharacterDeferredImpulse> characterDeferredImpulsesBuffer,
            in PhysicsCollider characterPhysicsCollider,
            float deltaTime,
            Entity characterEntity,
            quaternion characterRotation,
            float3 groundingUp) where T : struct, IKinematicCharacterProcessor
        {
            bool moveConfirmedThereWereNoOverlaps = false;
            float3 originalVelocityDirectionBeforeMove = math.normalizesafe(characterBody.RelativeVelocity);

#if RIVAL_PROFILING
            _moveProfilerMarker.Begin();
#endif

            // Move character based on relativeVelocity
            MoveWithCollisions(
                ref processor,
                ref characterTranslation,
                ref characterBody,
                ref characterHitsBuffer,
                ref velocityProjectionHitsBuffer,
                in characterPhysicsCollider,
                deltaTime,
                characterEntity,
                groundingUp,
                originalVelocityDirectionBeforeMove,
                characterRotation,
                out moveConfirmedThereWereNoOverlaps);

#if RIVAL_PROFILING
            _moveProfilerMarker.End();
#endif

#if RIVAL_PROFILING
            _decollideProfilerMarker.Begin();
#endif

            // This has to be after movement has been processed, in order to let our movement to take us 
            // out of the collision with a platform before we try to decollide from it
            if (characterBody.DecollideFromOverlaps && !moveConfirmedThereWereNoOverlaps)
            {
                SolveOverlaps(
                    ref processor,
                    ref characterTranslation,
                    ref characterBody,
                    ref characterHitsBuffer,
                    ref velocityProjectionHitsBuffer,
                    ref characterDeferredImpulsesBuffer,
                    in characterPhysicsCollider,
                    characterEntity,
                    originalVelocityDirectionBeforeMove,
                    characterRotation,
                    groundingUp);
            }

#if RIVAL_PROFILING
            _decollideProfilerMarker.End();
#endif

#if RIVAL_PROFILING
            _dynamicsProfilerMarker.Begin();
#endif

            // Process moving body hit velocities
            if (characterHitsBuffer.Length > 0)
            {
                ProcessCharacterHitDynamics(
                    ref processor,
                    ref characterBody,
                    ref characterDeferredImpulsesBuffer,
                    in characterHitsBuffer,
                    characterEntity,
                    characterTranslation,
                    characterRotation,
                    groundingUp);
            }

#if RIVAL_PROFILING
            _dynamicsProfilerMarker.End();
#endif
        }

        /// <summary>
        /// handles preserving velocity momentum when getting unparented from a parent body (such as a moving platform).
        /// Momentum velocity must be added at the end of frame AFTER the character has moved, so that moving platforms have one frame to get away from our added velocity that we inherited from them.
        /// It must also be applied after deferred impulses, to solve landing velocity issues when landing on a dynamic body
        /// </summary>
        /// <param name="trackedTransformFromEntity"></param>
        /// <param name="characterBody"></param>
        /// <param name="characterTranslation"></param>
        /// <param name="deltaTime"></param>
        public static void ParentMomentumUpdate(
            ref ComponentDataFromEntity<TrackedTransform> trackedTransformFromEntity,
            ref KinematicCharacterBody characterBody, 
            in float3 characterTranslation, 
            float deltaTime,
            float3 groundingUp)
        {
            // Reset parent if parent entity doesn't exist anymore
            if (characterBody.ParentEntity != Entity.Null && !trackedTransformFromEntity.HasComponent(characterBody.ParentEntity))
            {
                characterBody.ParentEntity = Entity.Null;
            }
            if (characterBody.PreviousParentEntity != Entity.Null && !trackedTransformFromEntity.HasComponent(characterBody.PreviousParentEntity))
            {
                characterBody.PreviousParentEntity = Entity.Null;
            }

            // Handle adding parent body momentum
            if (characterBody.ParentEntity != characterBody.PreviousParentEntity)
            {
                // Handle preserving momentum from previous parent when there has been a parent change
                if (characterBody.PreviousParentEntity != Entity.Null)
                {
                    characterBody.RelativeVelocity += characterBody.ParentVelocity;
                }

                // Handle compensating momentum for new parent body
                if (characterBody.ParentEntity != Entity.Null)
                {
                    TrackedTransform parentTrackedTransform = trackedTransformFromEntity[characterBody.ParentEntity];
                    characterBody.RelativeVelocity -= parentTrackedTransform.CalculatePointVelocity(characterTranslation, deltaTime);

                    if (characterBody.IsGrounded)
                    {
                        ProjectVelocityOnGrounding(ref characterBody.RelativeVelocity, characterBody.GroundHit.Normal, groundingUp);
                    }
                }
            }
        }

        /// <summary>
        /// handles filling the `StatefulKinematicCharacterHit` buffer on the character entity, with character hits that have an Enter/Exit/Stay state associated to them
        /// </summary>
        /// <param name="statefulCharacterHitsBuffer"></param>
        /// <param name="characterHitsBuffer"></param>
        public static void ProcessStatefulCharacterHits(
            ref DynamicBuffer<StatefulKinematicCharacterHit> statefulCharacterHitsBuffer, 
            in DynamicBuffer<KinematicCharacterHit> characterHitsBuffer)
        {
            bool OldStatefulHitsContainEntity(in DynamicBuffer<StatefulKinematicCharacterHit> statefulCharacterHitsBuffer, Entity entity, int lastIndexOfOldStatefulHits, out CharacterHitState oldState)
            {
                oldState = default;

                if (lastIndexOfOldStatefulHits < 0)
                {
                    return false;
                }

                for (int i = 0; i <= lastIndexOfOldStatefulHits; i++)
                {
                    StatefulKinematicCharacterHit oldStatefulHit = statefulCharacterHitsBuffer[i];
                    if (oldStatefulHit.Hit.Entity == entity)
                    {
                        oldState = oldStatefulHit.State;
                        return true;
                    }
                }

                return false;
            }

            bool NewStatefulHitsContainEntity(in DynamicBuffer<StatefulKinematicCharacterHit> statefulCharacterHitsBuffer, Entity entity, int firstIndexOfNewStatefulHits)
            {
                if (firstIndexOfNewStatefulHits >= statefulCharacterHitsBuffer.Length)
                {
                    return false;
                }

                for (int i = firstIndexOfNewStatefulHits; i < statefulCharacterHitsBuffer.Length; i++)
                {
                    StatefulKinematicCharacterHit newStatefulHit = statefulCharacterHitsBuffer[i];
                    if (newStatefulHit.Hit.Entity == entity)
                    {
                        return true;
                    }
                }

                return false;
            }

            int lastIndexOfOldStatefulHits = statefulCharacterHitsBuffer.Length - 1;

            // Add new stateful hits
            for (int hitIndex = 0; hitIndex < characterHitsBuffer.Length; hitIndex++)
            {
                KinematicCharacterHit characterHit = characterHitsBuffer[hitIndex];
                if (!NewStatefulHitsContainEntity(in statefulCharacterHitsBuffer, characterHit.Entity, lastIndexOfOldStatefulHits + 1))
                {
                    StatefulKinematicCharacterHit newStatefulHit = new StatefulKinematicCharacterHit(characterHit);
                    bool entityWasInStatefulHitsBefore = OldStatefulHitsContainEntity(in statefulCharacterHitsBuffer, characterHit.Entity, lastIndexOfOldStatefulHits, out CharacterHitState oldHitState);

                    if (entityWasInStatefulHitsBefore)
                    {
                        switch (oldHitState)
                        {
                            case CharacterHitState.Enter:
                                newStatefulHit.State = CharacterHitState.Stay;
                                break;
                            case CharacterHitState.Stay:
                                newStatefulHit.State = CharacterHitState.Stay;
                                break;
                            case CharacterHitState.Exit:
                                newStatefulHit.State = CharacterHitState.Enter;
                                break;
                        }
                    }
                    else
                    {
                        newStatefulHit.State = CharacterHitState.Enter;
                    }

                    statefulCharacterHitsBuffer.Add(newStatefulHit);
                }
            }

            // Detect Exit states 
            for (int i = 0; i <= lastIndexOfOldStatefulHits; i++)
            {
                StatefulKinematicCharacterHit oldStatefulHit = statefulCharacterHitsBuffer[i];

                // If an old hit entity isn't in new hits, add as Exit state
                if (oldStatefulHit.State != CharacterHitState.Exit && !NewStatefulHitsContainEntity(in statefulCharacterHitsBuffer, oldStatefulHit.Hit.Entity, lastIndexOfOldStatefulHits + 1))
                {
                    oldStatefulHit.State = CharacterHitState.Exit;
                    statefulCharacterHitsBuffer.Add(oldStatefulHit);
                }
            }

            // Remove all old stateful hits
            if (lastIndexOfOldStatefulHits >= 0)
            {
                statefulCharacterHitsBuffer.RemoveRange(0, lastIndexOfOldStatefulHits + 1);
            }
        }

        public static unsafe void GroundDetection<T>(
            ref T processor,
            in KinematicCharacterBody characterBody,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            float3 characterTranslation,
            quaternion characterRotation,
            float3 groundingUp,
            float groundProbingLength,
            out bool isGrounded,
            out BasicHit groundHit,
            out float distanceToGround) where T : struct, IKinematicCharacterProcessor
        {
            isGrounded = false;
            groundHit = default;
            distanceToGround = 0f;

            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<ColliderCastHit> tmpColliderCastHits = processor.GetTmpColliderCastHits;

            ColliderCastInput input = new ColliderCastInput(characterPhysicsCollider.Value, characterTranslation, characterTranslation + (-groundingUp * groundProbingLength), characterRotation);
            tmpColliderCastHits.Clear();
            AllHitsCollector<ColliderCastHit> collector = new AllHitsCollector<ColliderCastHit>(1f, ref tmpColliderCastHits);
            collisionWorld.CastCollider(input, ref collector);

            if (FilterColliderCastHitsForGroundProbing(ref processor, ref tmpColliderCastHits, characterBody.ShouldIgnoreDynamicBodies(), characterEntity, -groundingUp, out ColliderCastHit closestHit))
            {
                // Ground hit is closest hit by default
                groundHit = new BasicHit(closestHit);
                distanceToGround = closestHit.Fraction * groundProbingLength;

                // Check grounding status
                if (characterBody.EvaluateGrounding)
                {
                    bool isGroundedOnClosestHit = processor.IsGroundedOnHit(in groundHit, (int)GroundingEvaluationType.GroundProbing);
                    if (isGroundedOnClosestHit)
                    {
                        isGrounded = true;
                    }
                    else
                    {
                        // If the closest hit wasn't grounded but other hits were detected, try to find the closest grounded hit within tolerance range
                        if (tmpColliderCastHits.Length > 1)
                        {
                            // Sort hits in ascending fraction order
                            // TODO: We are doing a sort because, presumably, it would be faster to sort & have potentially less hits to evaluate for grounding
                            tmpColliderCastHits.Sort(default(HitFractionComparer));

                            for (int i = 0; i < tmpColliderCastHits.Length; i++)
                            {
                                ColliderCastHit tmpHit = tmpColliderCastHits[i];

                                // Skip if this is our ground hit
                                if (tmpHit.RigidBodyIndex == groundHit.RigidBodyIndex && 
                                    tmpHit.ColliderKey.Equals(groundHit.ColliderKey))
                                    continue;

                                //Only accept if within tolerance distance
                                float tmpHitDistance = tmpHit.Fraction * groundProbingLength;
                                if (math.distancesq(tmpHitDistance, distanceToGround) <= Constants.GroundedHitDistanceToleranceSq)
                                {
                                    BasicHit tmpClosestGroundedHit = new BasicHit(tmpHit);
                                    bool isGroundedOnHit = processor.IsGroundedOnHit(in tmpClosestGroundedHit, (int)GroundingEvaluationType.GroundProbing);
                                    if (isGroundedOnHit)
                                    {
                                        isGrounded = true;
                                        distanceToGround = tmpHitDistance;
                                        groundHit = tmpClosestGroundedHit; 
                                        break;
                                    }
                                }
                                else
                                {
                                    // if we're starting to see hits with a distance greater than tolerance dist, give up trying to evaluate hits since the list is sorted in ascending fraction order
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        public static void ProcessCharacterHitDynamics<T>(
            ref T processor,
            ref KinematicCharacterBody characterBody,
            ref DynamicBuffer<KinematicCharacterDeferredImpulse> characterDeferredImpulsesBuffer,
            in DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            Entity characterEntity,
            float3 characterTranslation,
            quaternion characterRotation,
            float3 groundingUp) where T : struct, IKinematicCharacterProcessor
        {
            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<int> tmpRigidbodyIndexesProcessed = processor.GetTmpRigidbodyIndexesProcessed;
            ComponentDataFromEntity<StoredKinematicCharacterBodyProperties> characterBodyPropertiesFromEntity = processor.GetStoredCharacterBodyPropertiesFromEntity;
            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;
            ComponentDataFromEntity<PhysicsVelocity> physicsVelocityFromEntity = processor.GetPhysicsVelocityFromEntity;

            tmpRigidbodyIndexesProcessed.Clear();

            for (int b = 0; b < characterHitsBuffer.Length; b++)
            {
                KinematicCharacterHit characterHit = characterHitsBuffer[b];
                if (characterHit.RigidBodyIndex >= 0)
                {
                    int hitBodyIndex = characterHit.RigidBodyIndex;
                    RigidBody hitBody = collisionWorld.Bodies[hitBodyIndex];
                    Entity hitBodyEntity = hitBody.Entity;

                    if (hitBodyEntity != characterBody.ParentEntity)
                    {
                        bool bodyHasPhysicsVelocityAndMass = PhysicsUtilities.DoesBodyHavePhysicsVelocityAndMass(in collisionWorld, hitBodyIndex);
                        if (bodyHasPhysicsVelocityAndMass)
                        {
                            if (!tmpRigidbodyIndexesProcessed.Contains(characterHit.RigidBodyIndex))
                            {
                                tmpRigidbodyIndexesProcessed.Add(characterHit.RigidBodyIndex);

                                PhysicsVelocity selfPhysicsVelocity = new PhysicsVelocity { Linear = characterBody.RelativeVelocity + characterBody.ParentVelocity, Angular = default };
                                PhysicsMass selfPhysicsMass = PhysicsUtilities.GetKinematicCharacterPhysicsMass(characterBody);
                                RigidTransform selfTransform = new RigidTransform(characterRotation, characterTranslation);

                                // Compute other body's data depending on if it's a character or not
                                bool otherIsCharacter = false;
                                bool otherIsDynamic = false;
                                PhysicsVelocity otherPhysicsVelocity = new PhysicsVelocity();
                                PhysicsMass otherPhysicsMass = new PhysicsMass();
                                RigidTransform otherTransform = hitBody.WorldFromBody;
                                if (characterBodyPropertiesFromEntity.HasComponent(hitBodyEntity))
                                {
                                    StoredKinematicCharacterBodyProperties bodyProperties = characterBodyPropertiesFromEntity[hitBodyEntity];
                                    otherIsCharacter = true;
                                    otherIsDynamic = bodyProperties.SimulateDynamicBody;
                                    otherPhysicsVelocity = new PhysicsVelocity { Linear = bodyProperties.RelativeVelocity + bodyProperties.ParentVelocity, Angular = float3.zero };
                                    otherPhysicsMass = PhysicsUtilities.GetKinematicCharacterPhysicsMass(characterBodyPropertiesFromEntity[hitBodyEntity]);
                                }
                                else if (physicsMassFromEntity.HasComponent(hitBodyEntity) && physicsVelocityFromEntity.HasComponent(hitBodyEntity))
                                {
                                    otherPhysicsVelocity = physicsVelocityFromEntity[hitBodyEntity];
                                    otherPhysicsMass = physicsMassFromEntity[hitBodyEntity];
                                    otherIsDynamic = otherPhysicsMass.InverseMass > 0f;
                                }

                                // Correct the normal of the hit based on grounding considerations
                                float3 effectiveHitNormalFromOtherToSelf = characterHit.Normal;
                                if (characterHit.WasCharacterGroundedOnHitEnter && !characterHit.IsGroundedOnHit)
                                {
                                    effectiveHitNormalFromOtherToSelf = math.normalizesafe(MathUtilities.ProjectOnPlane(characterHit.Normal, groundingUp));
                                }
                                else if (characterHit.IsGroundedOnHit)
                                {
                                    effectiveHitNormalFromOtherToSelf = groundingUp;
                                }
                                // Prevent a grounding-reoriented normal for dynamic bodies
                                if (otherIsDynamic && !characterHit.IsGroundedOnHit)
                                {
                                    effectiveHitNormalFromOtherToSelf = characterHit.Normal;
                                }

                                // Mass overrides
                                if (characterBody.SimulateDynamicBody && otherIsDynamic && !otherIsCharacter)
                                {
                                    if (selfPhysicsMass.InverseMass > 0f && otherPhysicsMass.InverseMass > 0f)
                                    {
                                        processor.OverrideDynamicHitMasses(ref selfPhysicsMass, ref otherPhysicsMass, characterEntity, hitBodyEntity, hitBodyIndex);
                                    }
                                }

                                // Special cases with kinematic VS kinematic
                                if (!characterBody.SimulateDynamicBody && !otherIsDynamic)
                                {
                                    // Pretend we have a mass of 1 against a kinematic body
                                    selfPhysicsMass.InverseMass = 1f;

                                    // When other is kinematic character, cancel their velocity towards us if any, for the sake of impulse calculations. This prevents bumping
                                    if (otherIsCharacter && math.dot(otherPhysicsVelocity.Linear, effectiveHitNormalFromOtherToSelf) > 0f)
                                    {
                                        otherPhysicsVelocity.Linear = MathUtilities.ProjectOnPlane(otherPhysicsVelocity.Linear, effectiveHitNormalFromOtherToSelf);
                                    }
                                }

                                // Restore the portion of the character velocity that got lost during hit projection (so we can re-solve it with dynamics)
                                float3 velocityLostInOriginalProjection = math.projectsafe(characterHit.CharacterVelocityBeforeHit - characterHit.CharacterVelocityAfterHit, effectiveHitNormalFromOtherToSelf);
                                selfPhysicsVelocity.Linear += velocityLostInOriginalProjection;

                                // Solve impulses
                                PhysicsUtilities.SolveCollisionImpulses(
                                    selfPhysicsVelocity,
                                    otherPhysicsVelocity,
                                    selfPhysicsMass,
                                    otherPhysicsMass,
                                    selfTransform,
                                    otherTransform,
                                    characterHit.Position,
                                    effectiveHitNormalFromOtherToSelf,
                                    out float3 impulseOnSelf,
                                    out float3 impulseOnOther);

                                // Apply impulse to self
                                float3 previousCharacterLinearVel = selfPhysicsVelocity.Linear;
                                selfPhysicsVelocity.ApplyLinearImpulse(in selfPhysicsMass, impulseOnSelf);
                                float3 characterLinearVelocityChange = velocityLostInOriginalProjection + (selfPhysicsVelocity.Linear - previousCharacterLinearVel);
                                characterBody.RelativeVelocity += characterLinearVelocityChange;

                                // TODO: this ignores custom vel projection.... any alternatives?
                                // trim off any velocity that goes towards ground (prevents reoriented velocity issue)
                                if (characterHit.IsGroundedOnHit && math.dot(characterBody.RelativeVelocity, characterHit.Normal) < -Constants.DotProductSimilarityEpsilon)
                                {
                                    characterBody.RelativeVelocity = MathUtilities.ProjectOnPlane(characterBody.RelativeVelocity, groundingUp);
                                    characterBody.RelativeVelocity = MathUtilities.ReorientVectorOnPlaneAlongDirection(characterBody.RelativeVelocity, characterHit.Normal, groundingUp);
                                }

                                // if a character is moving towards is, they will also solve the collision themselves in their own update. In order to prevent solving the coll twice, we won't apply any impulse on them in that case
                                bool otherIsCharacterMovingTowardsUs = otherIsCharacter && math.dot(otherPhysicsVelocity.Linear, effectiveHitNormalFromOtherToSelf) > Constants.DotProductSimilarityEpsilon;

                                // Apply velocity change on hit body (only if dynamic and not character. Characters will solve the impulse on themselves)
                                if (!otherIsCharacterMovingTowardsUs && otherIsDynamic && math.lengthsq(impulseOnOther) > 0f)
                                {
                                    float3 previousLinearVel = otherPhysicsVelocity.Linear;
                                    float3 previousAngularVel = otherPhysicsVelocity.Angular;

                                    otherPhysicsVelocity.ApplyImpulse(otherPhysicsMass,
                                        new Translation { Value = otherTransform.pos },
                                        new Rotation { Value = otherTransform.rot },
                                        impulseOnOther,
                                        characterHit.Position);

                                    characterDeferredImpulsesBuffer.Add(new KinematicCharacterDeferredImpulse
                                    {
                                        OnEntity = hitBodyEntity,
                                        LinearVelocityChange = otherPhysicsVelocity.Linear - previousLinearVel,
                                        AngularVelocityChange = otherPhysicsVelocity.Angular - previousAngularVel,
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }

        public static unsafe void MoveWithCollisions<T>(
            ref T processor,
            ref float3 characterTranslation,
            ref KinematicCharacterBody characterBody,
            ref DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            ref DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHitsBuffer,
            in PhysicsCollider characterPhysicsCollider,
            float deltaTime,
            Entity characterEntity,
            float3 groundingUp,
            float3 originalVelocityDirection,
            quaternion characterRotation,
            out bool confirmedNoOverlapsOnLastMoveIteration) where T : struct, IKinematicCharacterProcessor
        {
            confirmedNoOverlapsOnLastMoveIteration = false;

            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<ColliderCastHit> tmpColliderCastHits = processor.GetTmpColliderCastHits;

            // Project on ground hit
            if(characterBody.IsGrounded)
            {
                ProjectVelocityOnGrounding(ref characterBody.RelativeVelocity, characterBody.GroundHit.Normal, groundingUp);
            }
            else if (characterBody.GroundHit.Entity != Entity.Null)
            {
                if (math.dot(characterBody.RelativeVelocity, characterBody.GroundHit.Normal) < -Constants.DotProductSimilarityEpsilon)
                {
                    velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit(characterBody.GroundHit, characterBody.IsGrounded));

                    processor.ProjectVelocityOnHits(
                        ref characterBody.RelativeVelocity,
                        ref characterBody.IsGrounded,
                        ref characterBody.GroundHit,
                        in velocityProjectionHitsBuffer,
                        originalVelocityDirection);
                }
            }

            // Add all close distance hits to velocity projection hits buffer
            // Helps fix some tunneling issues with rotating character colliders
            if (characterBody.ProjectVelocityOnInitialOverlaps)
            {
                if (CalculateDistanceAllCollisions(
                    ref processor,
                    in characterPhysicsCollider,
                    characterEntity,
                    characterTranslation,
                    characterRotation,
                    0f,
                    characterBody.ShouldIgnoreDynamicBodies(),
                    out NativeList<DistanceHit> overlapHits))
                {
                    for (int i = 0; i < overlapHits.Length; i++)
                    {
                        BasicHit tmpHit = new BasicHit(overlapHits[i]);

                        if (math.dot(tmpHit.Normal, characterBody.RelativeVelocity) < Constants.DotProductSimilarityEpsilon)
                        {
                            bool isGroundedOnTmpHit = false;
                            if (characterBody.EvaluateGrounding)
                            {
                                isGroundedOnTmpHit = processor.IsGroundedOnHit(in tmpHit, (int)GroundingEvaluationType.InitialOverlaps);
                            }

                            velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit
                            {
                                Entity = tmpHit.Entity,
                                RigidBodyIndex = tmpHit.RigidBodyIndex,
                                ColliderKey = tmpHit.ColliderKey,
                                Position = tmpHit.Position,
                                Normal = tmpHit.Normal,
                                IsGroundedOnHit = isGroundedOnTmpHit,
                            });

                            processor.ProjectVelocityOnHits(
                                ref characterBody.RelativeVelocity,
                                ref characterBody.IsGrounded,
                                ref characterBody.GroundHit,
                                in velocityProjectionHitsBuffer,
                                originalVelocityDirection);
                        }
                    }
                }
            }

            // Movement cast iterations
            if (characterBody.DetectMovementCollisions)
            {
                float remainingMovementLength = math.length(characterBody.RelativeVelocity) * deltaTime;
                float3 remainingMovementDirection = math.normalizesafe(characterBody.RelativeVelocity);

                int movementCastIterationsMade = 0;
                while (movementCastIterationsMade < characterBody.MaxContinuousCollisionsIterations && remainingMovementLength > 0f)
                {
                    confirmedNoOverlapsOnLastMoveIteration = false;

                    float3 castStartPosition = characterTranslation;
                    float3 castDirection = remainingMovementDirection;
                    float castLength = remainingMovementLength + Constants.CollisionOffset; // TODO: shoud we keep this offset?

                    // Cast collider for movement
                    ColliderCastInput castInput = new ColliderCastInput(characterPhysicsCollider.Value, castStartPosition, castStartPosition + (castDirection * castLength), characterRotation);
                    tmpColliderCastHits.Clear();
                    AllHitsCollector<ColliderCastHit> collector = new AllHitsCollector<ColliderCastHit>(1f, ref tmpColliderCastHits);
                    collisionWorld.CastCollider(castInput, ref collector);
                    bool foundMovementHit = FilterColliderCastHitsForMove(ref processor, ref tmpColliderCastHits, characterBody.ShouldIgnoreDynamicBodies(), !characterBody.SimulateDynamicBody, characterEntity, castDirection, Entity.Null, out ColliderCastHit closestHit, out bool foundAnyOverlaps);

                    if (!foundAnyOverlaps)
                    {
                        confirmedNoOverlapsOnLastMoveIteration = true;
                    }

                    if (foundMovementHit)
                    {
                        BasicHit movementHit = new BasicHit(closestHit);
                        float movementHitDistance = castLength * closestHit.Fraction;
                        movementHitDistance = math.max(0f, movementHitDistance - Constants.CollisionOffset);

                        bool isGroundedOnMovementHit = false;
                        if (characterBody.EvaluateGrounding)
                        {
                            // Grounding calculation
                            isGroundedOnMovementHit = processor.IsGroundedOnHit(in movementHit, (int)GroundingEvaluationType.MovementHit);
                        }

                        // Add hit to projection hits
                        KinematicCharacterHit currentCharacterHit = CreateCharacterHit(
                            in movementHit,
                            characterBody.IsGrounded,
                            characterBody.RelativeVelocity,
                            isGroundedOnMovementHit);
                        velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit(currentCharacterHit));

                        processor.OnMovementHit(
                            ref currentCharacterHit,
                            ref remainingMovementDirection,
                            ref remainingMovementLength,
                            originalVelocityDirection,
                            movementHitDistance);

                        currentCharacterHit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;
                        characterHitsBuffer.Add(currentCharacterHit);
                    }
                    // If no hits detected, just consume the rest of the movement, which will end the iterations
                    else
                    {
                        characterTranslation += (remainingMovementDirection * remainingMovementLength);
                        remainingMovementLength = 0f;
                    }

                    movementCastIterationsMade++;
                }

                // If there is still movement left after all iterations (in other words; if we were not able to solve the movement completely)....
                if (remainingMovementLength > 0f)
                {
                    if (characterBody.KillVelocityWhenExceedMaxIterations)
                    {
                        characterBody.RelativeVelocity = float3.zero;
                    }

                    if (!characterBody.DiscardMovementWhenExceedMaxIterations)
                    {
                        characterTranslation += (remainingMovementDirection * remainingMovementLength);
                    }
                }
            }
            else
            {
                characterTranslation += characterBody.RelativeVelocity * deltaTime;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsGroundedOnSlopeNormal(
            float maxGroundedSlopeDotProduct,
            float3 slopeSurfaceNormal,
            float3 groundingUp)
        {
            return math.dot(groundingUp, slopeSurfaceNormal) > maxGroundedSlopeDotProduct;
        }

        public static void ProjectVelocityOnGrounding(ref float3 velocity, float3 groundNormal, float3 groundingUp)
        {
            // Make the velocity be 100% of its magnitude when it is perfectly parallel to ground, 0% when it is towards character up,
            // and interpolated when it's in-between those
            if (math.lengthsq(velocity) > 0f)
            {
                float velocityLength = math.length(velocity);
                float3 originalDirection = math.normalizesafe(velocity);
                float3 reorientedDirection = math.normalizesafe(MathUtilities.ReorientVectorOnPlaneAlongDirection(velocity, groundNormal, groundingUp));
                float dotOriginalWithUp = math.dot(originalDirection, groundingUp);
                float dotReorientedWithUp = math.dot(reorientedDirection, groundingUp);

                float ratioFromVerticalToSlopeDirection = 0f;
                // If velocity is going towards ground, interpolate between reoriented direction and down direction (-1f ratio with up)
                if (dotOriginalWithUp < dotReorientedWithUp)
                {
                    ratioFromVerticalToSlopeDirection = math.distance(dotOriginalWithUp, -1f) / math.distance(dotReorientedWithUp, -1f);
                }
                // If velocity is going towards air, interpolate between reoriented direction and up direction (1f ratio with up)
                else
                {
                    ratioFromVerticalToSlopeDirection = math.distance(dotOriginalWithUp, 1f) / math.distance(dotReorientedWithUp, 1f);
                }
                velocity = reorientedDirection * math.lerp(0f, velocityLength, ratioFromVerticalToSlopeDirection);
            }
        }

        public static unsafe void SolveOverlaps<T>(
            ref T processor,
            ref float3 characterTranslation,
            ref KinematicCharacterBody characterBody,
            ref DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
            ref DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHitsBuffer,
            ref DynamicBuffer<KinematicCharacterDeferredImpulse> characterDeferredImpulsesBuffer,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            float3 originalVelocityDirection,
            quaternion characterRotation,
            float3 groundingUp) where T : struct, IKinematicCharacterProcessor
        {
            void RecalculateDecollisionVector(ref float3 decollisionVector, float3 originalHitNormal, float3 newDecollisionDirection, float decollisionDistance)
            {
                float overlapDistance = math.max(decollisionDistance, 0f);
                if (overlapDistance > 0f)
                {
                    decollisionVector += MathUtilities.ReverseProjectOnVector(originalHitNormal * overlapDistance, newDecollisionDirection, overlapDistance * Constants.DefaultReverseProjectionMaxLengthRatio);
                }
            }

            void DecollideFromHit(
                ref T processor,
                ref float3 characterTranslation,
                ref KinematicCharacterBody characterBody,
                ref DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
                ref DynamicBuffer<KinematicCharacterDeferredImpulse> characterDeferredImpulsesBuffer,
                ref DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHitsBuffer,
                in BasicHit hit,
                in PhysicsCollider characterPhysicsCollider,
                float decollisionDistance,
                bool characterSimulateDynamic,
                bool isGroundedOnHit,
                bool hitIsDynamic,
                bool addToCharacterHits,
                bool projectVelocityOnHit)
            {
                NativeList<int> tmpRigidbodyIndexesProcessed = processor.GetTmpRigidbodyIndexesProcessed;

                // Grounding considerations for decollision (modified decollision direction)
                float3 decollisionDirection = hit.Normal;
                float3 decollisionVector = decollisionDirection * decollisionDistance;
                if (isGroundedOnHit)
                {
                    if (isGroundedOnHit && math.dot(groundingUp, hit.Normal) > Constants.MinDotRatioForVerticalDecollision)
                    {
                        // Always decollide vertically from grounded hits
                        decollisionDirection = groundingUp;
                        RecalculateDecollisionVector(ref decollisionVector, hit.Normal, decollisionDirection, decollisionDistance);
                    }
                    else if (characterBody.IsGrounded && !hitIsDynamic)
                    {
                        // If we are grounded and hit is nongrounded, decollide horizontally on the plane of our ground normal
                        decollisionDirection = math.normalizesafe(MathUtilities.ProjectOnPlane(decollisionDirection, characterBody.GroundHit.Normal));
                        RecalculateDecollisionVector(ref decollisionVector, hit.Normal, decollisionDirection, decollisionDistance);
                    }
                }

                // In simulateDynamic mode, before we decollide from a dynamic body, check if the decollision would be obstructed by anything other than the decollided body
                if (characterSimulateDynamic &&
                    hitIsDynamic &&
                    CastColliderClosestCollisions(
                        ref processor,
                        in characterPhysicsCollider,
                        characterEntity,
                        characterTranslation,
                        characterRotation,
                        decollisionDirection,
                        decollisionDistance,
                        true,
                        true,
                        out ColliderCastHit closestHit,
                        out float closestHitDistance) &&
                    closestHit.Entity != hit.Entity)
                {
                    // Move based on how far the obstruction was
                    characterTranslation += decollisionDirection * closestHitDistance;

                    // Add a position displacement to the decollided body, to move it away from us
                    if (!tmpRigidbodyIndexesProcessed.Contains(hit.RigidBodyIndex))
                    {
                        tmpRigidbodyIndexesProcessed.Add(hit.RigidBodyIndex);

                        characterDeferredImpulsesBuffer.Add(new KinematicCharacterDeferredImpulse
                        {
                            OnEntity = hit.Entity,
                            Displacement = -hit.Normal * (decollisionDistance - closestHitDistance),
                        });
                    }
                }
                // fully decollide otherwise
                else
                {
                    characterTranslation += decollisionVector;
                }

                // Velocity projection
                float3 characterRelativeVelocityBeforeProjection = characterBody.RelativeVelocity;
                if (projectVelocityOnHit)
                {
                    velocityProjectionHitsBuffer.Add(new KinematicVelocityProjectionHit(hit, isGroundedOnHit));

                    // Project velocity on obstructing overlap
                    if (math.dot(characterBody.RelativeVelocity, hit.Normal) < 0f)
                    {
                        processor.ProjectVelocityOnHits(
                            ref characterBody.RelativeVelocity,
                            ref characterBody.IsGrounded,
                            ref characterBody.GroundHit,
                            in velocityProjectionHitsBuffer,
                            originalVelocityDirection);
                    }
                }

                // Add to character hits
                if (addToCharacterHits)
                {
                    KinematicCharacterHit ovelapCharacterHit = CreateCharacterHit(
                        in hit,
                        characterBody.IsGrounded,
                        characterRelativeVelocityBeforeProjection,
                        isGroundedOnHit);
                    ovelapCharacterHit.CharacterVelocityAfterHit = characterBody.RelativeVelocity;
                    characterHitsBuffer.Add(ovelapCharacterHit);
                }
            }

            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<int> tmpRigidbodyIndexesProcessed = processor.GetTmpRigidbodyIndexesProcessed;
            NativeList<DistanceHit> tmpDistanceHits = processor.GetTmpDistanceHits;

            tmpRigidbodyIndexesProcessed.Clear();

            int decollisionIterationsMade = 0;
            while (decollisionIterationsMade < characterBody.MaxOverlapDecollisionIterations)
            {
                decollisionIterationsMade++;

                ColliderDistanceInput distanceInput = new ColliderDistanceInput(characterPhysicsCollider.Value, math.RigidTransform(characterRotation, characterTranslation), 0f);
                tmpDistanceHits.Clear();
                AllHitsCollector<DistanceHit> collector = new AllHitsCollector<DistanceHit>(distanceInput.MaxDistance, ref tmpDistanceHits);
                collisionWorld.CalculateDistance(distanceInput, ref collector);
                FilterDistanceHitsForSolveOverlaps(ref processor, ref tmpDistanceHits, characterEntity, out DistanceHit mostPenetratingHit, out DistanceHit mostPenetratingDynamicHit, out DistanceHit mostPenetratingNonDynamicHit);

                bool foundHitForDecollision = false;

                // Dynamic mode
                if (characterBody.SimulateDynamicBody)
                {
                    DistanceHit chosenDecollisionHit = default;
                    if (mostPenetratingNonDynamicHit.Distance < 0f)
                    {
                        chosenDecollisionHit = mostPenetratingNonDynamicHit; // assume we decollide from closest nondynamic hit by default
                    }
                    bool chosenHitIsDynamic = false;
                    bool isGroundedOnChosenHit = false;
                    bool calculatedChosenHitIsGrounded = false;

                    // Remember all dynamic bodies as hits and push back those that cause an obstructed collision
                    for (int i = 0; i < tmpDistanceHits.Length; i++)
                    {
                        DistanceHit dynamicHit = tmpDistanceHits[i];
                        BasicHit basicDynamicHit = new BasicHit(dynamicHit);

                        bool isGroundedOnHit = false;
                        if (characterBody.EvaluateGrounding)
                        {
                            isGroundedOnHit = processor.IsGroundedOnHit(in basicDynamicHit, (int)GroundingEvaluationType.OverlapDecollision);
                        }

                        // is this happens to be the most penetrating hit, remember as chosen hit
                        if (dynamicHit.RigidBodyIndex == mostPenetratingHit.RigidBodyIndex &&
                           dynamicHit.ColliderKey.Value == mostPenetratingHit.ColliderKey.Value)
                        {
                            chosenDecollisionHit = dynamicHit;

                            chosenHitIsDynamic = true;
                            isGroundedOnChosenHit = isGroundedOnHit;
                            calculatedChosenHitIsGrounded = true;
                        }
                    }

                    if (chosenDecollisionHit.Entity != Entity.Null)
                    {
                        BasicHit basicChosenHit = new BasicHit(chosenDecollisionHit);

                        if (!calculatedChosenHitIsGrounded)
                        {
                            if (characterBody.EvaluateGrounding)
                            {
                                isGroundedOnChosenHit = processor.IsGroundedOnHit(in basicChosenHit, (int)GroundingEvaluationType.OverlapDecollision);
                            }
                        }

                        DecollideFromHit(
                            ref processor,
                            ref characterTranslation,
                            ref characterBody,
                            ref characterHitsBuffer,
                            ref characterDeferredImpulsesBuffer,
                            ref velocityProjectionHitsBuffer,
                            in basicChosenHit,
                            in characterPhysicsCollider,
                            -chosenDecollisionHit.Distance,
                            characterBody.SimulateDynamicBody,
                            isGroundedOnChosenHit,
                            chosenHitIsDynamic,
                            true,
                            true);

                        foundHitForDecollision = true;
                    }
                }
                // Kinematic mode
                else
                {
                    bool foundValidNonDynamicHitToDecollideFrom = mostPenetratingNonDynamicHit.Entity != Entity.Null && mostPenetratingNonDynamicHit.Distance < 0f;
                    bool isLastIteration = !foundValidNonDynamicHitToDecollideFrom || decollisionIterationsMade >= characterBody.MaxOverlapDecollisionIterations;

                    // Push back all dynamic bodies & remember as hits, but only on last iteration
                    if (isLastIteration)
                    {
                        for (int i = 0; i < tmpDistanceHits.Length; i++)
                        {
                            DistanceHit dynamicHit = tmpDistanceHits[i];
                            BasicHit basicDynamicHit = new BasicHit(dynamicHit);

                            // Add as character hit
                            KinematicCharacterHit ovelapHit = CreateCharacterHit(
                                in basicDynamicHit,
                                characterBody.IsGrounded,
                                characterBody.RelativeVelocity,
                                false);
                            characterHitsBuffer.Add(ovelapHit);

                            // Add a position displacement impulse
                            if (!tmpRigidbodyIndexesProcessed.Contains(dynamicHit.RigidBodyIndex))
                            {
                                tmpRigidbodyIndexesProcessed.Add(dynamicHit.RigidBodyIndex);

                                characterDeferredImpulsesBuffer.Add(new KinematicCharacterDeferredImpulse
                                {
                                    OnEntity = dynamicHit.Entity,
                                    Displacement = dynamicHit.SurfaceNormal * dynamicHit.Distance,
                                });
                            }
                        }
                    }

                    // Remember that we must decollide only from the closest nonDynamic hit, if any
                    if (foundValidNonDynamicHitToDecollideFrom)
                    {
                        BasicHit basicChosenHit = new BasicHit(mostPenetratingNonDynamicHit);

                        bool isGroundedOnHit = false;
                        if (characterBody.EvaluateGrounding)
                        {
                            isGroundedOnHit = processor.IsGroundedOnHit(in basicChosenHit, (int)GroundingEvaluationType.OverlapDecollision);
                        }

                        DecollideFromHit(
                            ref processor,
                            ref characterTranslation,
                            ref characterBody,
                            ref characterHitsBuffer,
                            ref characterDeferredImpulsesBuffer,
                            ref velocityProjectionHitsBuffer,
                            in basicChosenHit,
                            in characterPhysicsCollider,
                            -mostPenetratingNonDynamicHit.Distance,
                            characterBody.SimulateDynamicBody,
                            isGroundedOnHit,
                            false,
                            true,
                            true);

                        foundHitForDecollision = true;
                    }
                }

                if (!foundHitForDecollision)
                {
                    // Early exit when found no hit to decollide from
                    break;
                }
            }
        }

        public static unsafe bool CastColliderClosestCollisions<T>(
            ref T processor,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            float3 characterTranslation,
            quaternion characterRotation,
            float3 direction,
            float length,
            bool onlyObstructingHits,
            bool ignoreDynamicBodies,
            out ColliderCastHit hit,
            out float hitDistance) where T : struct, IKinematicCharacterProcessor
        {
            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<ColliderCastHit> tmpColliderCastHits = processor.GetTmpColliderCastHits;

            ColliderCastInput castInput = new ColliderCastInput(characterPhysicsCollider.Value, characterTranslation, characterTranslation + (direction * length), characterRotation);
            tmpColliderCastHits.Clear();
            AllHitsCollector<ColliderCastHit> collector = new AllHitsCollector<ColliderCastHit>(1f, ref tmpColliderCastHits);
            collisionWorld.CastCollider(castInput, ref collector);
            if (FilterColliderCastHitsForClosestCollisions(ref processor, ref tmpColliderCastHits, ignoreDynamicBodies, characterEntity, onlyObstructingHits, direction, out ColliderCastHit closestHit))
            {
                hit = closestHit;
                hitDistance = length * hit.Fraction;
                return true;
            }

            hit = default;
            hitDistance = default;
            return false;
        }

        public static unsafe bool CastColliderAllCollisions<T>(
            ref T processor,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            float3 characterTranslation,
            quaternion characterRotation,
            float3 direction,
            float length,
            bool onlyObstructingHits,
            bool ignoreDynamicBodies,
            out NativeList<ColliderCastHit> hits) where T : struct, IKinematicCharacterProcessor
        {
            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<ColliderCastHit> tmpColliderCastHits = processor.GetTmpColliderCastHits;

            hits = tmpColliderCastHits;

            ColliderCastInput castInput = new ColliderCastInput(characterPhysicsCollider.Value, characterTranslation, characterTranslation + (direction * length), characterRotation);
            tmpColliderCastHits.Clear();
            AllHitsCollector<ColliderCastHit> collector = new AllHitsCollector<ColliderCastHit>(1f, ref tmpColliderCastHits);
            collisionWorld.CastCollider(castInput, ref collector);
            if (FilterColliderCastHitsForAllCollisions(ref processor, ref tmpColliderCastHits, ignoreDynamicBodies, characterEntity, onlyObstructingHits, direction))
            {
                return true;
            }

            return false;
        }

        public static bool RaycastClosestCollisions<T>(
            ref T processor,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            float3 startPoint,
            float3 direction,
            float length,
            bool ignoreDynamicBodies,
            out RaycastHit hit,
            out float hitDistance) where T : struct, IKinematicCharacterProcessor
        {
            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<RaycastHit> tmpRaycastHits = processor.GetTmpRaycastHits;

            RaycastInput castInput = new RaycastInput
            {
                Start = startPoint,
                End = startPoint + (direction * length),
                Filter = characterPhysicsCollider.Value.Value.Filter,
            };
            tmpRaycastHits.Clear();
            AllHitsCollector<RaycastHit> collector = new AllHitsCollector<RaycastHit>(1f, ref tmpRaycastHits);
            collisionWorld.CastRay(castInput, ref collector);
            if (FilterRaycastHitsForClosestCollisions(ref processor, ref tmpRaycastHits, ignoreDynamicBodies, characterEntity, out RaycastHit closestHit))
            {
                hit = closestHit;
                hitDistance = length * hit.Fraction;
                return true;
            }

            hit = default;
            hitDistance = default;
            return false;
        }

        public static bool RaycastAllCollisions<T>(
            ref T processor,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            float3 startPoint,
            float3 direction,
            float length,
            bool ignoreDynamicBodies,
            out NativeList<RaycastHit> hits) where T : struct, IKinematicCharacterProcessor
        {
            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<RaycastHit> tmpRaycastHits = processor.GetTmpRaycastHits;

            hits = tmpRaycastHits;

            RaycastInput castInput = new RaycastInput
            {
                Start = startPoint,
                End = startPoint + (direction * length),
                Filter = characterPhysicsCollider.Value.Value.Filter,
            };
            tmpRaycastHits.Clear();
            AllHitsCollector<RaycastHit> collector = new AllHitsCollector<RaycastHit>(1f, ref tmpRaycastHits);
            collisionWorld.CastRay(castInput, ref collector);
            if (FilterRaycastHitsForAllCollisions(ref processor, ref tmpRaycastHits, ignoreDynamicBodies, characterEntity))
            {
                return true;
            }

            return false;
        }

        public static unsafe bool CalculateDistanceClosestCollisions<T>(
            ref T processor,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            float3 characterTranslation,
            quaternion characterRotation,
            float maxDistance,
            bool ignoreDynamicBodies,
            out DistanceHit hit) where T : struct, IKinematicCharacterProcessor
        {
            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<DistanceHit> tmpDistanceHits = processor.GetTmpDistanceHits;

            ColliderDistanceInput distanceInput = new ColliderDistanceInput(characterPhysicsCollider.Value, math.RigidTransform(characterRotation, characterTranslation), maxDistance);
            tmpDistanceHits.Clear();
            AllHitsCollector<DistanceHit> collector = new AllHitsCollector<DistanceHit>(distanceInput.MaxDistance, ref tmpDistanceHits);
            collisionWorld.CalculateDistance(distanceInput, ref collector);
            if (FilterDistanceHitsForClosestCollisions(ref processor, ref tmpDistanceHits, ignoreDynamicBodies, characterEntity, out DistanceHit closestHit))
            {
                hit = closestHit;
                return true;
            }

            hit = default;
            return false;
        }

        public static unsafe bool CalculateDistanceAllCollisions<T>(
            ref T processor,
            in PhysicsCollider characterPhysicsCollider,
            Entity characterEntity,
            float3 characterTranslation,
            quaternion characterRotation,
            float maxDistance,
            bool ignoreDynamicBodies,
            out NativeList<DistanceHit> hits) where T : struct, IKinematicCharacterProcessor
        {
            CollisionWorld collisionWorld = processor.GetCollisionWorld;
            NativeList<DistanceHit> tmpDistanceHits = processor.GetTmpDistanceHits;

            hits = tmpDistanceHits;

            ColliderDistanceInput distanceInput = new ColliderDistanceInput(characterPhysicsCollider.Value, math.RigidTransform(characterRotation, characterTranslation), maxDistance);
            tmpDistanceHits.Clear();
            AllHitsCollector<DistanceHit> collector = new AllHitsCollector<DistanceHit>(distanceInput.MaxDistance, ref tmpDistanceHits);
            collisionWorld.CalculateDistance(distanceInput, ref collector);
            if (FilterDistanceHitsForAllCollisions(ref processor, ref tmpDistanceHits, ignoreDynamicBodies, characterEntity))
            {
                return true;
            }

            return false;
        }

        public static bool MovementWouldHitNonGroundedObstruction<T>(
            ref T processor,
            in KinematicCharacterBody characterBody,
            in PhysicsCollider characterPhysicsCollider,
            float3 movement,
            Entity characterEntity,
            float3 characterTranslation,
            quaternion characterRotation) where T : struct, IKinematicCharacterProcessor
        {
            if (CastColliderClosestCollisions(
                ref processor,
                in characterPhysicsCollider,
                characterEntity,
                characterTranslation,
                characterRotation,
                math.normalizesafe(movement),
                math.length(movement),
                true,
                characterBody.ShouldIgnoreDynamicBodies(),
                out ColliderCastHit hit,
                out float hitDistance))
            {
                if (characterBody.EvaluateGrounding)
                {
                    if (!processor.IsGroundedOnHit(new BasicHit(hit), (int)GroundingEvaluationType.Default))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        public static void ApplyParentRotationToTargetRotation(ref quaternion targetRotation, in KinematicCharacterBody characterBody, float fixedDeltaTime, float deltaTime)
        {
            float rotationRatio = math.clamp(deltaTime / fixedDeltaTime, 0f, 1f);
            quaternion rotationFromCharacterParent = math.slerp(quaternion.identity, characterBody.RotationFromParent, rotationRatio);
            targetRotation = math.mul(targetRotation, rotationFromCharacterParent);
        }

        public static KinematicCharacterHit CreateCharacterHit(
            in BasicHit newHit,
            bool characterIsGrounded,
            float3 characterRelativeVelocity,
            bool isGroundedOnHit)
        {
            KinematicCharacterHit newCharacterHit = new KinematicCharacterHit
            {
                Entity = newHit.Entity,
                RigidBodyIndex = newHit.RigidBodyIndex,
                ColliderKey = newHit.ColliderKey,
                Normal = newHit.Normal,
                Position = newHit.Position,
                WasCharacterGroundedOnHitEnter = characterIsGrounded,
                IsGroundedOnHit = isGroundedOnHit,
                CharacterVelocityBeforeHit = characterRelativeVelocity,
                CharacterVelocityAfterHit = characterRelativeVelocity,
            };

            return newCharacterHit;
        }

        public static bool FilterColliderCastHitsForGroundProbing<T>(
            ref T processor,
            ref NativeList<ColliderCastHit> hits,
            bool ignoreDynamicBodies,
            Entity characterEntity,
            float3 castDirection,
            out ColliderCastHit closestHit) where T : struct, IKinematicCharacterProcessor
        {
            closestHit = default;
            closestHit.Fraction = float.MaxValue;

            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = hits.Length - 1; i >= 0; i--)
            {
                bool hitAccepted = false;
                var hit = hits[i];

                if (hit.Entity != characterEntity)
                {
                    // ignore hits if we're going away from them
                    float dotRatio = math.dot(hit.SurfaceNormal, castDirection);
                    if (dotRatio < -Constants.DotProductSimilarityEpsilon)
                    {
                        if (!ignoreDynamicBodies || !PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity))
                        {
                            if (processor.CanCollideWithHit(new BasicHit(hit)))
                            {
                                hitAccepted = true;

                                if (hit.Fraction < closestHit.Fraction)
                                {
                                    closestHit = hit;
                                }
                            }
                        }
                    }
                }

                if (!hitAccepted)
                {
                    hits.RemoveAtSwapBack(i);
                }
            }

            return closestHit.Entity != Entity.Null;
        }

        public static bool FilterColliderCastHitsForClosestGroundedOnSlope<T>(
            ref T processor,
            ref NativeList<ColliderCastHit> hits,
            bool ignoreDynamicBodies,
            Entity characterEntity,
            float3 castDirection,
            float3 groundingUp,
            float maxGroundedSlopeDotProduct,
            out ColliderCastHit closestHit) where T : struct, IKinematicCharacterProcessor
        {
            closestHit = default;
            closestHit.Fraction = float.MaxValue;

            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = hits.Length - 1; i >= 0; i--)
            {
                var hit = hits[i];
                if (hit.Entity != characterEntity)
                {
                    // ignore hits if we're going away from them
                    float dotRatio = math.dot(hit.SurfaceNormal, castDirection);
                    if (dotRatio < -Constants.DotProductSimilarityEpsilon)
                    {
                        if (!ignoreDynamicBodies || !PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity))
                        {
                            if (processor.CanCollideWithHit(new BasicHit(hit)))
                            {
                                if (hit.Fraction < closestHit.Fraction)
                                {
                                    if (IsGroundedOnSlopeNormal(maxGroundedSlopeDotProduct, hit.SurfaceNormal, groundingUp))
                                    {
                                        closestHit = hit;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return closestHit.Entity != Entity.Null;
        }

        public static bool FilterColliderCastHitsForMove<T>(
            ref T processor,
            ref NativeList<ColliderCastHit> hits,
            bool ignoreDynamicBodies,
            bool characterIsKinematic,
            Entity characterEntity,
            float3 castDirection,
            Entity ignoredEntity,
            out ColliderCastHit closestHit,
            out bool foundAnyOverlaps) where T : struct, IKinematicCharacterProcessor
        {
            foundAnyOverlaps = false;
            closestHit = default;
            closestHit.Fraction = float.MaxValue;
            float dotRatioOfSelectedHit = float.MaxValue;

            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = hits.Length - 1; i >= 0; i--)
            {
                var hit = hits[i];
                if (hit.Entity != ignoredEntity)
                {
                    if (hit.Entity != characterEntity)
                    {
                        if (processor.CanCollideWithHit(new BasicHit(hit)))
                        {
                            bool hitBodyIsDynamic = PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity);

                            // Remember overlaps (must always include dynamic hits or hits we move away from)
                            if (hit.Fraction <= 0f || (characterIsKinematic && hitBodyIsDynamic))
                            {
                                foundAnyOverlaps = true;
                            }

                            if (!ignoreDynamicBodies || !hitBodyIsDynamic)
                            {
                                // ignore hits if we're going away from them
                                float dotRatio = math.dot(hit.SurfaceNormal, castDirection);
                                if (dotRatio < -Constants.DotProductSimilarityEpsilon)
                                {
                                    // only accept closest hit so far
                                    if (hit.Fraction <= closestHit.Fraction)
                                    {
                                        // Accept hit if it's the new closest one, or if equal distance but more obstructing
                                        bool isCloserThanPreviousSelectedHit = hit.Fraction < closestHit.Fraction;
                                        if (isCloserThanPreviousSelectedHit || dotRatio < dotRatioOfSelectedHit)
                                        {
                                            closestHit = hit;
                                            dotRatioOfSelectedHit = dotRatio;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return closestHit.Entity != Entity.Null;
        }

        public static bool FilterColliderCastHitsForClosestCollisions<T>(
            ref T processor,
            ref NativeList<ColliderCastHit> hits,
            bool ignoreDynamicBodies,
            Entity characterEntity,
            bool onlyObstructingHits,
            float3 castDirection,
            out ColliderCastHit closestHit) where T : struct, IKinematicCharacterProcessor
        {
            closestHit = default;
            closestHit.Fraction = float.MaxValue;

            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = 0; i < hits.Length; i++)
            {
                var hit = hits[i];
                if (hit.Fraction <= closestHit.Fraction)
                {
                    if (hit.Entity != characterEntity)
                    {
                        if (!ignoreDynamicBodies || !PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity))
                        {
                            // ignore hits if we're going away from them
                            if (!onlyObstructingHits || math.dot(hit.SurfaceNormal, castDirection) < -Constants.DotProductSimilarityEpsilon)
                            {
                                if (processor.CanCollideWithHit(new BasicHit(hit)))
                                {
                                    closestHit = hit;
                                }
                            }
                        }
                    }
                }
            }

            return closestHit.Entity != Entity.Null;
        }

        public static bool FilterColliderCastHitsForAllCollisions<T>(
            ref T processor,
            ref NativeList<ColliderCastHit> hits,
            bool ignoreDynamicBodies,
            Entity characterEntity,
            bool onlyObstructingHits,
            float3 castDirection) where T : struct, IKinematicCharacterProcessor
        {
            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = hits.Length - 1; i >= 0; i--)
            {
                bool hitAccepted = false;
                var hit = hits[i];
                if (hit.Entity != characterEntity)
                {
                    if (!ignoreDynamicBodies || !PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity))
                    {
                        // ignore hits if we're going away from them
                        if (!onlyObstructingHits || math.dot(hit.SurfaceNormal, castDirection) < -Constants.DotProductSimilarityEpsilon)
                        {
                            if (processor.CanCollideWithHit(new BasicHit(hit)))
                            {
                                hitAccepted = true;
                            }
                        }
                    }
                }

                if (!hitAccepted)
                {
                    hits.RemoveAtSwapBack(i);
                }
            }

            return hits.Length > 0;
        }

        public static void FilterDistanceHitsForSolveOverlaps<T>(
            ref T processor,
            ref NativeList<DistanceHit> hits,
            Entity characterEntity,
            out DistanceHit closestHit,
            out DistanceHit closestDynamicHit,
            out DistanceHit closestNonDynamicHit) where T : struct, IKinematicCharacterProcessor
        {
            closestHit = default;
            closestHit.Fraction = float.MaxValue;
            closestDynamicHit = default;
            closestDynamicHit.Fraction = float.MaxValue;
            closestNonDynamicHit = default;
            closestNonDynamicHit.Fraction = float.MaxValue;

            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = hits.Length - 1; i >= 0; i--)
            {
                bool hitAccepted = false;
                var hit = hits[i];
                if (hit.Entity != characterEntity)
                {
                    if (processor.CanCollideWithHit(new BasicHit(hit)))
                    {
                        bool isBodyDynamic = PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity);

                        if (hit.Distance < closestHit.Distance)
                        {
                            closestHit = hit;
                        }
                        if (isBodyDynamic && hit.Distance < closestDynamicHit.Distance)
                        {
                            closestDynamicHit = hit;
                        }
                        if (!isBodyDynamic && hit.Distance < closestNonDynamicHit.Distance)
                        {
                            closestNonDynamicHit = hit;
                        }

                        // Keep all dynamic hits in the list (and only those)
                        if (isBodyDynamic)
                        {
                            hitAccepted = true;
                        }
                    }
                }

                if (!hitAccepted)
                {
                    hits.RemoveAtSwapBack(i);
                }
            }
        }

        public static bool FilterDistanceHitsForClosestCollisions<T>(
            ref T processor,
            ref NativeList<DistanceHit> hits,
            bool ignoreDynamicBodies,
            Entity characterEntity,
            out DistanceHit closestHit) where T : struct, IKinematicCharacterProcessor
        {
            closestHit = default;
            closestHit.Fraction = float.MaxValue;

            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = 0; i < hits.Length; i++)
            {
                var hit = hits[i];
                if (hit.Distance < closestHit.Distance)
                {
                    if (hit.Entity != characterEntity)
                    {
                        if (!ignoreDynamicBodies || !PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity))
                        {
                            if (processor.CanCollideWithHit(new BasicHit(hit)))
                            {
                                closestHit = hit;
                            }
                        }
                    }
                }
            }

            return closestHit.Entity != Entity.Null;
        }

        public static bool FilterDistanceHitsForAllCollisions<T>(
            ref T processor,
            ref NativeList<DistanceHit> hits,
            bool ignoreDynamicBodies,
            Entity characterEntity) where T : struct, IKinematicCharacterProcessor
        {
            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = hits.Length - 1; i >= 0; i--)
            {
                bool hitAccepted = false;
                var hit = hits[i];
                if (hit.Entity != characterEntity)
                {
                    if (!ignoreDynamicBodies || !PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity))
                    {
                        if (processor.CanCollideWithHit(new BasicHit(hit)))
                        {
                            hitAccepted = true;
                        }
                    }
                }

                if (!hitAccepted)
                {
                    hits.RemoveAtSwapBack(i);
                }
            }

            return hits.Length > 0;
        }

        public static bool FilterRaycastHitsForClosestCollisions<T>(
            ref T processor,
            ref NativeList<RaycastHit> hits,
            bool ignoreDynamicBodies,
            Entity characterEntity,
            out RaycastHit closestHit) where T : struct, IKinematicCharacterProcessor
        {
            closestHit = default;
            closestHit.Fraction = float.MaxValue;

            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = 0; i < hits.Length; i++)
            {
                var hit = hits[i];
                if (hit.Fraction < closestHit.Fraction)
                {
                    if (hit.Entity != characterEntity)
                    {
                        if (!ignoreDynamicBodies || !PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity))
                        {
                            if (processor.CanCollideWithHit(new BasicHit(hit)))
                            {
                                closestHit = hit;
                            }
                        }
                    }
                }
            }

            return closestHit.Entity != Entity.Null;
        }

        public static bool FilterRaycastHitsForAllCollisions<T>(
            ref T processor,
            ref NativeList<RaycastHit> hits,
            bool ignoreDynamicBodies,
            Entity characterEntity) where T : struct, IKinematicCharacterProcessor
        {
            ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;

            for (int i = hits.Length - 1; i >= 0; i--)
            {
                bool hitAccepted = false;
                var hit = hits[i];
                if (hit.Entity != characterEntity)
                {
                    if (!ignoreDynamicBodies || !PhysicsUtilities.IsBodyDynamic(in physicsMassFromEntity, hit.Entity))
                    {
                        if (processor.CanCollideWithHit(new BasicHit(hit)))
                        {
                            hitAccepted = true;
                        }
                    }
                }

                if (!hitAccepted)
                {
                    hits.RemoveAtSwapBack(i);
                }
            }

            return hits.Length > 0;
        }

        public static JobHandle ScheduleDeferredImpulsesJob(SystemBase forSystem, EntityQuery characterQuery, JobHandle dependency, bool run = false)
        {
            KinematicCharacterDeferredImpulsesJob deferredImpulsesJob = new KinematicCharacterDeferredImpulsesJob
            {
                CharacterDeferredImpulsesBufferType = forSystem.GetBufferTypeHandle<KinematicCharacterDeferredImpulse>(false),
                CharacterBodyFromEntity = forSystem.GetComponentDataFromEntity<KinematicCharacterBody>(false),
                PhysicsVelocityFromEntity = forSystem.GetComponentDataFromEntity<PhysicsVelocity>(false),
                TranslationFromEntity = forSystem.GetComponentDataFromEntity<Translation>(false),
            };

            if (run)
            {
                deferredImpulsesJob.Run(characterQuery);
            }
            else
            {
                dependency = deferredImpulsesJob.ScheduleSingle(characterQuery, dependency); // Must not be parallel due to possibility of writing to the same entity from multiple different places
            }

            return dependency;
        }

        public static void ProcessDeferredImpulses(
            ref ComponentDataFromEntity<Translation> translationFromEntity,
            ref ComponentDataFromEntity<PhysicsVelocity> physicsVelocityFromEntity,
            ref ComponentDataFromEntity<KinematicCharacterBody> characterBodyFromEntity,
            in DynamicBuffer<KinematicCharacterDeferredImpulse> characterDeferredImpulsesBuffer)
        {
            for (int deferredImpulseIndex = 0; deferredImpulseIndex < characterDeferredImpulsesBuffer.Length; deferredImpulseIndex++)
            {
                KinematicCharacterDeferredImpulse deferredImpulse = characterDeferredImpulsesBuffer[deferredImpulseIndex];

                // Impulse
                bool isImpulseOnCharacter = characterBodyFromEntity.HasComponent(deferredImpulse.OnEntity);
                if (isImpulseOnCharacter)
                {
                    KinematicCharacterBody hitCharacterBody = characterBodyFromEntity[deferredImpulse.OnEntity];
                    if (hitCharacterBody.SimulateDynamicBody)
                    {
                        hitCharacterBody.RelativeVelocity += deferredImpulse.LinearVelocityChange;
                        characterBodyFromEntity[deferredImpulse.OnEntity] = hitCharacterBody;
                    }
                }
                else
                {
                    PhysicsVelocity bodyPhysicsVelocity = physicsVelocityFromEntity[deferredImpulse.OnEntity];

                    bodyPhysicsVelocity.Linear += deferredImpulse.LinearVelocityChange;
                    bodyPhysicsVelocity.Angular += deferredImpulse.AngularVelocityChange;

                    physicsVelocityFromEntity[deferredImpulse.OnEntity] = bodyPhysicsVelocity;
                }

                // Displacement
                if (math.lengthsq(deferredImpulse.Displacement) > 0f)
                {
                    Translation bodyTranslation = translationFromEntity[deferredImpulse.OnEntity];
                    bodyTranslation.Value += deferredImpulse.Displacement;
                    translationFromEntity[deferredImpulse.OnEntity] = bodyTranslation;
                }
            }
        }

        public static void SetOrUpdateParentBody(
            ref KinematicCharacterBody characterBody,
            in ComponentDataFromEntity<TrackedTransform> trackedTransformFromEntity,
            Entity parentEntity,
            float3 anchorPointWorldSpace)
        {
            if (parentEntity != Entity.Null && trackedTransformFromEntity.HasComponent(parentEntity))
            {
                characterBody.ParentEntity = parentEntity;
                characterBody.ParentAnchorPoint = anchorPointWorldSpace;
            }
            else
            {
                characterBody.ParentEntity = Entity.Null;
                characterBody.ParentAnchorPoint = default;
            }
        }

        public static class DefaultMethods
        {
            public static bool IsGroundedOnHit_Simple<T>(
                in T processor,
                in BasicHit hit,
                in KinematicCharacterBody characterBody,
                float3 groundingUp) where T : struct, IKinematicCharacterProcessor
            {
                if(ShouldPreventGroundingBasedOnVelocity(in processor, in hit, in characterBody))
                {
                    return false;
                }

                return IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, hit.Normal, groundingUp);
            }

            public unsafe static bool IsGroundedOnHit<T>(
                ref T processor,
                in BasicHit hit,
                in KinematicCharacterBody characterBody,
                in PhysicsCollider characterPhysicsCollider,
                Entity characterEntity,
                float3 groundingUp,
                int groundingEvaluationType,
                bool stepHandling,
                float maxStepHeight,
                float extraStepChecksDistance) where T : struct, IKinematicCharacterProcessor
            {
                if (ShouldPreventGroundingBasedOnVelocity(in processor, in hit, in characterBody))
                {
                    return false;
                }

                bool isGroundedOnSlope = IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, hit.Normal, groundingUp);

                // Handle detecting grounding on step edges if not grounded on slope
                bool isGroundedOnSteps = false;
                if (!isGroundedOnSlope && stepHandling && maxStepHeight > 0f)
                {
                    bool hitIsOnCharacterBottom = math.dot(groundingUp, hit.Normal) > Constants.DotProductSimilarityEpsilon;
                    if (hitIsOnCharacterBottom ||
                        (groundingEvaluationType != (int)GroundingEvaluationType.MovementHit && groundingEvaluationType != (int)GroundingEvaluationType.InitialOverlaps))
                    {
                        isGroundedOnSteps = IsGroundedOnSteps(
                            ref processor,
                            in hit,
                            in characterBody,
                            in characterPhysicsCollider,
                            characterEntity,
                            groundingUp,
                            maxStepHeight,
                            extraStepChecksDistance);
                    }
                }

                return isGroundedOnSlope || isGroundedOnSteps;
            }

            public static bool ShouldPreventGroundingBasedOnVelocity<T>(
                in T processor,
                in BasicHit hit,
                in KinematicCharacterBody characterBody) where T : struct, IKinematicCharacterProcessor
            {
                // Prevent grounding if nongrounded and going away from ground normal
                // (this prevents snapping to ground when you are in air, going upwards, and hopping onto the side of a platform)
                if (!characterBody.WasGroundedBeforeCharacterUpdate &&
                    math.dot(characterBody.RelativeVelocity, hit.Normal) > Constants.DotProductSimilarityEpsilon &&
                    math.lengthsq(characterBody.RelativeVelocity) > Constants.MinVelocityLengthSqForGroundingIgnoreCheck)
                {
                    ComponentDataFromEntity<PhysicsVelocity> physicsVelocityFromEntity = processor.GetPhysicsVelocityFromEntity;
                    ComponentDataFromEntity<PhysicsMass> physicsMassFromEntity = processor.GetPhysicsMassFromEntity;
                    if (physicsVelocityFromEntity.HasComponent(hit.Entity) && physicsMassFromEntity.HasComponent(hit.Entity))
                    {
                        RigidBody hitBody = processor.GetCollisionWorld.Bodies[hit.RigidBodyIndex];
                        PhysicsVelocity hitPhysicsVelocity = physicsVelocityFromEntity[hit.Entity];
                        PhysicsMass hitPhysicsMass = physicsMassFromEntity[hit.Entity];
                        Translation hitTranslation = new Translation { Value = hitBody.WorldFromBody.pos };
                        Rotation hitRotation = new Rotation { Value = hitBody.WorldFromBody.rot };
                        float3 groundVelocityAtPoint = hitPhysicsVelocity.GetLinearVelocity(hitPhysicsMass, hitTranslation, hitRotation, hit.Position);

                        float characterVelocityOnNormal = math.dot(characterBody.RelativeVelocity, hit.Normal);
                        float groundVelocityOnNormal = math.dot(groundVelocityAtPoint, hit.Normal);

                        // Ignore grounding if our velocity is escaping the ground velocity
                        if (characterVelocityOnNormal > groundVelocityOnNormal)
                        {
                            return true;
                        }
                    }
                    else
                    {
                        // If the ground has no velocity and our velocity is going away from it
                        return true;
                    }
                }

                return false;
            }

            public static bool IsGroundedOnSteps<T>(
                ref T processor,
                in BasicHit hit,
                in KinematicCharacterBody characterBody,
                in PhysicsCollider characterPhysicsCollider,
                Entity characterEntity,
                float3 groundingUp,
                float maxStepHeight,
                float extraStepChecksDistance) where T : struct, IKinematicCharacterProcessor
            {
                if (maxStepHeight > 0f)
                {
                    bool isGroundedOnBackStep = false;
                    bool isGroundedOnForwardStep = false;
                    float3 backCheckDirection = math.normalizesafe(MathUtilities.ProjectOnPlane(hit.Normal, groundingUp));

                    // Close back step hit
                    bool backStepHitFound = RaycastClosestCollisions(
                        ref processor,
                        in characterPhysicsCollider,
                        characterEntity,
                        hit.Position + (backCheckDirection * Constants.StepGroundingDetectionHorizontalOffset),
                        -groundingUp,
                        maxStepHeight,
                        characterBody.ShouldIgnoreDynamicBodies(),
                        out RaycastHit backStepHit,
                        out float backHitDistance);
                    if (backStepHitFound && backHitDistance > 0f)
                    {
                        isGroundedOnBackStep = IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, backStepHit.SurfaceNormal, groundingUp);
                    }

                    if (!isGroundedOnBackStep && extraStepChecksDistance > Constants.StepGroundingDetectionHorizontalOffset)
                    {
                        // Extra back step hit
                        backStepHitFound = RaycastClosestCollisions(
                            ref processor,
                            in characterPhysicsCollider,
                            characterEntity,
                            hit.Position + (backCheckDirection * extraStepChecksDistance),
                            -groundingUp,
                            maxStepHeight,
                            characterBody.ShouldIgnoreDynamicBodies(),
                            out backStepHit,
                            out backHitDistance);
                        if (backStepHitFound && backHitDistance > 0f)
                        {
                            isGroundedOnBackStep = IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, backStepHit.SurfaceNormal, groundingUp);
                        }
                    }

                    if (isGroundedOnBackStep)
                    {
                        float forwardCheckHeight = maxStepHeight - backHitDistance;

                        // Close forward step hit
                        bool forwardStepHitFound = RaycastClosestCollisions(
                            ref processor,
                            in characterPhysicsCollider,
                            characterEntity,
                            hit.Position + (groundingUp * forwardCheckHeight) + (-backCheckDirection * Constants.StepGroundingDetectionHorizontalOffset),
                            -groundingUp,
                            maxStepHeight,
                            characterBody.ShouldIgnoreDynamicBodies(),
                            out RaycastHit forwardStepHit,
                            out float forwardHitDistance);
                        if (forwardStepHitFound && forwardHitDistance > 0f)
                        {
                            isGroundedOnForwardStep = IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, forwardStepHit.SurfaceNormal, groundingUp);
                        }

                        if (!isGroundedOnForwardStep && extraStepChecksDistance > Constants.StepGroundingDetectionHorizontalOffset)
                        {
                            // Extra forward step hit
                            forwardStepHitFound = RaycastClosestCollisions(
                                ref processor,
                                in characterPhysicsCollider,
                                characterEntity,
                                hit.Position + (groundingUp * forwardCheckHeight) + (-backCheckDirection * extraStepChecksDistance),
                                -groundingUp,
                                maxStepHeight,
                                characterBody.ShouldIgnoreDynamicBodies(),
                                out forwardStepHit,
                                out forwardHitDistance);
                            if (forwardStepHitFound && forwardHitDistance > 0f)
                            {
                                isGroundedOnForwardStep = IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, forwardStepHit.SurfaceNormal, groundingUp);
                            }
                        }
                    }

                    return isGroundedOnBackStep && isGroundedOnForwardStep;
                }

                return false;
            }

            public static bool CanCollideWithHit(in BasicHit hit, in ComponentDataFromEntity<StoredKinematicCharacterBodyProperties> storedCharacterBodyFromEntity)
            {
                // Only collide with collidable colliders
                if (hit.Material.CollisionResponse == CollisionResponsePolicy.Collide ||
                    hit.Material.CollisionResponse == CollisionResponsePolicy.CollideRaiseCollisionEvents)
                {
                    return true;
                }

                // If collider's collision response is Trigger or None, it could potentially be a Character. So make a special exception in that case
                if (storedCharacterBodyFromEntity.HasComponent(hit.Entity))
                {
                    return true;
                }

                return false;
            }

            public static void ProjectVelocityOnHits(
                ref float3 velocity,
                ref bool characterIsGrounded,
                ref BasicHit characterGroundHit,
                in DynamicBuffer<KinematicVelocityProjectionHit> hits,
                float3 originalVelocityDirection,
                float3 groundingUp,
                bool constrainToGoundPlane = true)
            {
                bool IsSamePlane(float3 planeA, float3 planeB)
                {
                    return math.dot(planeA, planeB) > (1f - Constants.DotProductSimilarityEpsilon);
                }

                void ProjectVelocityOnSingleHit(ref float3 velocity, ref bool characterIsGrounded, ref BasicHit characterGroundHit, in KinematicVelocityProjectionHit hit)
                {
                    if (characterIsGrounded)
                    {
                        if (hit.IsGroundedOnHit)
                        {
                            // Simply reorient velocity
                            velocity = MathUtilities.ReorientVectorOnPlaneAlongDirection(velocity, hit.Normal, groundingUp);
                        }
                        else
                        {
                            if (constrainToGoundPlane)
                            {
                                // Project velocity on crease formed between ground normal and obstruction
                                float3 groundedCreaseDirection = math.normalizesafe(math.cross(characterGroundHit.Normal, hit.Normal));
                                velocity = math.projectsafe(velocity, groundedCreaseDirection);
                            }
                            else
                            {
                                // Regular projection
                                velocity = MathUtilities.ProjectOnPlane(velocity, hit.Normal);
                            }
                        }
                    }
                    else
                    {
                        if (hit.IsGroundedOnHit)
                        {
                            // Handle grounded landing
                            velocity = MathUtilities.ProjectOnPlane(velocity, groundingUp);
                            velocity = MathUtilities.ReorientVectorOnPlaneAlongDirection(velocity, hit.Normal, groundingUp);
                        }
                        else
                        {
                            // Regular projection
                            velocity = MathUtilities.ProjectOnPlane(velocity, hit.Normal);
                        }
                    }

                    // Replace grounding when the hit is grounded (or when not trying to constrain movement to ground plane
                    if (hit.IsGroundedOnHit || !constrainToGoundPlane)
                    {
                        // This could be a virtual hit, so make sure to only count it if it has a valid rigidbody
                        if (hit.RigidBodyIndex >= 0)
                        {
                            // make sure to only count as ground if the normal is pointing up
                            if (math.dot(groundingUp, hit.Normal) > Constants.DotProductSimilarityEpsilon)
                            {
                                characterIsGrounded = hit.IsGroundedOnHit;
                                characterGroundHit = new BasicHit(hit);
                            }
                        }
                    }
                }

                if (math.lengthsq(velocity) <= 0f || math.lengthsq(originalVelocityDirection) <= 0f)
                {
                    return;
                }

                int hitsCount = hits.Length;
                int firstHitIndex = hits.Length - 1;
                KinematicVelocityProjectionHit firstHit = hits[firstHitIndex];
                float3 velocityDirection = math.normalizesafe(velocity);

                if (math.dot(velocityDirection, firstHit.Normal) < 0f)
                {
                    // Project on first plane
                    ProjectVelocityOnSingleHit(ref velocity, ref characterIsGrounded, ref characterGroundHit, in firstHit);
                    velocityDirection = math.normalizesafe(velocity);

                    // Original velocity direction will act as a plane constaint just like other hits, to prevent our velocity from going back the way it came from. Hit index -1 represents original velocity
                    KinematicVelocityProjectionHit originalVelocityHit = default;
                    originalVelocityHit.Normal = characterIsGrounded ? math.normalizesafe(MathUtilities.ProjectOnPlane(originalVelocityDirection, groundingUp)) : originalVelocityDirection;

                    // Detect creases and corners by observing how the projected velocity would interact with previously-detected planes
                    for (int secondHitIndex = -1; secondHitIndex < hitsCount; secondHitIndex++)
                    {
                        if (secondHitIndex == firstHitIndex)
                            continue;

                        KinematicVelocityProjectionHit secondHit = originalVelocityHit;
                        if (secondHitIndex >= 0)
                        {
                            secondHit = hits[secondHitIndex];
                        }

                        if (IsSamePlane(firstHit.Normal, secondHit.Normal))
                            continue;

                        if (math.dot(velocityDirection, secondHit.Normal) > -Constants.DotProductSimilarityEpsilon)
                            continue;

                        // Project on second plane
                        ProjectVelocityOnSingleHit(ref velocity, ref characterIsGrounded, ref characterGroundHit, in secondHit);
                        velocityDirection = math.normalizesafe(velocity);

                        // If the velocity projected on second plane goes back in first plane, it's a crease
                        if (math.dot(velocityDirection, firstHit.Normal) > -Constants.DotProductSimilarityEpsilon)
                            continue;

                        // Special case corner detection when grounded: if crease is made out of 2 non-grounded planes; it's a corner
                        if (characterIsGrounded && !firstHit.IsGroundedOnHit && !secondHit.IsGroundedOnHit)
                        {
                            velocity = default;
                            break;
                        }
                        else
                        {
                            // Velocity projection on crease
                            float3 creaseDirection = math.normalizesafe(math.cross(firstHit.Normal, secondHit.Normal));
                            if (secondHit.IsGroundedOnHit)
                            {
                                velocity = MathUtilities.ReorientVectorOnPlaneAlongDirection(velocity, secondHit.Normal, groundingUp);
                            }
                            velocity = math.projectsafe(velocity, creaseDirection);
                            velocityDirection = math.normalizesafe(velocity);
                        }

                        // Corner detection: see if projected velocity would enter back a third plane we already detected
                        for (int thirdHitIndex = -1; thirdHitIndex < hitsCount; thirdHitIndex++)
                        {
                            if (thirdHitIndex == firstHitIndex && thirdHitIndex == secondHitIndex)
                                continue;

                            KinematicVelocityProjectionHit thirdHit = originalVelocityHit;
                            if (thirdHitIndex >= 0)
                            {
                                thirdHit = hits[thirdHitIndex];
                            }

                            if (IsSamePlane(firstHit.Normal, thirdHit.Normal) || IsSamePlane(secondHit.Normal, thirdHit.Normal))
                                continue;

                            if (math.dot(velocityDirection, thirdHit.Normal) < -Constants.DotProductSimilarityEpsilon)
                            {
                                // Velocity projection on corner
                                velocity = default;
                                break;
                            }
                        }

                        if (math.lengthsq(velocity) <= math.EPSILON)
                        {
                            break;
                        }
                    }
                }
            }

            public static void MovingPlatformDetection(
                ref ComponentDataFromEntity<TrackedTransform> trackedTransformFromEntity,
                ref ComponentDataFromEntity<StoredKinematicCharacterBodyProperties> storedCharacterBodyFromEntity,
                ref KinematicCharacterBody characterBody)
            {
                if (characterBody.IsGrounded && !storedCharacterBodyFromEntity.HasComponent(characterBody.GroundHit.Entity))
                {
                    SetOrUpdateParentBody(ref characterBody, in trackedTransformFromEntity, characterBody.GroundHit.Entity, characterBody.GroundHit.Position);
                }
                else
                {
                    SetOrUpdateParentBody(ref characterBody, in trackedTransformFromEntity, Entity.Null, default);
                }
            }

            public static void UpdateGroundPushing<T>(
                ref T processor,
                ref DynamicBuffer<KinematicCharacterDeferredImpulse> characterDeferredImpulsesBuffer,
                ref KinematicCharacterBody characterBody,
                float deltaTime,
                Entity characterEntity,
                float3 gravity,
                float3 characterTranslation,
                quaternion characterRotation,
                float forceMultiplier = 1f) where T : struct, IKinematicCharacterProcessor
            {
                if (characterBody.IsGrounded)
                {
                    Entity groundEntity = characterBody.GroundHit.Entity;

                    if (groundEntity != Entity.Null &&
                        processor.GetPhysicsMassFromEntity.TryGetComponent(groundEntity, out PhysicsMass groundPhysicsMass) &&
                        processor.GetPhysicsVelocityFromEntity.TryGetComponent(groundEntity, out PhysicsVelocity groundPhysicsVelocity))
                    {
                        if (groundPhysicsMass.InverseMass > 0f) // if dynamic
                        {
                            bool groundIsCharacter = processor.GetStoredCharacterBodyPropertiesFromEntity.HasComponent(groundEntity);
                            if (!groundIsCharacter)
                            {
                                PhysicsMass selfPhysicsMass = PhysicsUtilities.GetKinematicCharacterPhysicsMass(characterBody);
                                RigidTransform selfTransform = new RigidTransform(characterRotation, characterTranslation);
                                RigidTransform groundTransform = processor.GetCollisionWorld.Bodies[characterBody.GroundHit.RigidBodyIndex].WorldFromBody;

                                selfPhysicsMass.InverseMass = 1f / characterBody.Mass;
                                processor.OverrideDynamicHitMasses(ref selfPhysicsMass, ref groundPhysicsMass, characterEntity, groundEntity, characterBody.GroundHit.RigidBodyIndex);

                                float3 groundPointVelocity = groundPhysicsVelocity.GetLinearVelocity(
                                    groundPhysicsMass,
                                    new Translation { Value = groundTransform.pos },
                                    new Rotation { Value = groundTransform.rot },
                                    characterBody.GroundHit.Position);

                                // Solve impulses
                                PhysicsUtilities.SolveCollisionImpulses(
                                    new PhysicsVelocity { Linear = groundPointVelocity + (gravity * deltaTime), Angular = default },
                                    groundPhysicsVelocity,
                                    selfPhysicsMass,
                                    groundPhysicsMass,
                                    selfTransform,
                                    groundTransform,
                                    characterBody.GroundHit.Position,
                                    -math.normalizesafe(gravity),
                                    out float3 impulseOnSelf,
                                    out float3 impulseOnOther);

                                float3 previousLinearVel = groundPhysicsVelocity.Linear;
                                float3 previousAngularVel = groundPhysicsVelocity.Angular;

                                groundPhysicsVelocity.ApplyImpulse(groundPhysicsMass,
                                    new Translation { Value = groundTransform.pos },
                                    new Rotation { Value = groundTransform.rot },
                                    impulseOnOther * forceMultiplier,
                                    characterBody.GroundHit.Position);

                                characterDeferredImpulsesBuffer.Add(new KinematicCharacterDeferredImpulse
                                {
                                    OnEntity = groundEntity,
                                    LinearVelocityChange = groundPhysicsVelocity.Linear - previousLinearVel,
                                    AngularVelocityChange = groundPhysicsVelocity.Angular - previousAngularVel,
                                });
                            }
                        }
                    }
                }
            }

            public static void OnMovementHit<T>(
                ref T processor,
                ref KinematicCharacterHit hit,
                ref KinematicCharacterBody characterBody,
                ref DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHitsBuffer,
                ref float3 characterTranslation,
                ref float3 remainingMovementDirection,
                ref float remainingMovementLength,
                in PhysicsCollider characterPhysicsCollider,
                Entity characterEntity,
                quaternion characterRotation,
                float3 groundingUp,
                float3 originalVelocityDirection,
                float movementHitDistance,
                bool stepHandling,
                float maxStepHeight) where T : struct, IKinematicCharacterProcessor
            {
                bool hasSteppedUp = false;

                if (stepHandling && !hit.IsGroundedOnHit)
                {
                    CheckForSteppingUpHit(
                        ref processor,
                        ref hit,
                        ref characterBody,
                        ref characterTranslation,
                        ref remainingMovementDirection,
                        ref remainingMovementLength,
                        in characterPhysicsCollider,
                        characterEntity,
                        characterRotation,
                        groundingUp,
                        movementHitDistance,
                        stepHandling,
                        maxStepHeight,
                        out hasSteppedUp);
                }

                if (!hasSteppedUp)
                {
                    // Advance position to closest hit
                    characterTranslation += remainingMovementDirection * movementHitDistance;
                    remainingMovementLength -= movementHitDistance;

                    // Project velocity
                    float3 velocityBeforeProjection = characterBody.RelativeVelocity;

                    processor.ProjectVelocityOnHits(
                        ref characterBody.RelativeVelocity,
                        ref characterBody.IsGrounded,
                        ref characterBody.GroundHit,
                        in velocityProjectionHitsBuffer,
                        originalVelocityDirection);

                    // Recalculate remaining movement after projection
                    float projectedVelocityLengthFactor = math.length(characterBody.RelativeVelocity) / math.length(velocityBeforeProjection);
                    remainingMovementLength *= projectedVelocityLengthFactor;
                    remainingMovementDirection = math.normalizesafe(characterBody.RelativeVelocity);
                }
            }

            public static void CheckForSteppingUpHit<T>(
                ref T processor,
                ref KinematicCharacterHit hit,
                ref KinematicCharacterBody characterBody,
                ref float3 characterTranslation,
                ref float3 remainingMovementDirection,
                ref float remainingMovementLength,
                in PhysicsCollider characterPhysicsCollider,
                Entity characterEntity,
                quaternion characterRotation,
                float3 groundingUp,
                float hitDistance,
                bool stepHandling,
                float maxStepHeight,
                out bool hasSteppedUp) where T : struct, IKinematicCharacterProcessor
            {
                hasSteppedUp = false;

                // Step up hits (only needed if not grounded on that hit)
                if (characterBody.EvaluateGrounding &&
                    stepHandling &&
                    !hit.IsGroundedOnHit &&
                    maxStepHeight > 0f)
                {
                    float3 startPositionOfUpCheck = characterTranslation;
                    float3 upCheckDirection = groundingUp;
                    float upCheckDistance = maxStepHeight;

                    // Up cast
                    bool foundUpStepHit = CastColliderClosestCollisions(
                        ref processor,
                        in characterPhysicsCollider,
                        characterEntity,
                        startPositionOfUpCheck,
                        characterRotation,
                        upCheckDirection,
                        upCheckDistance,
                        true,
                        characterBody.ShouldIgnoreDynamicBodies(),
                        out ColliderCastHit upStepHit,
                        out float upStepHitDistance);

                    if (foundUpStepHit)
                    {
                        upStepHitDistance = math.max(0f, upStepHitDistance - Constants.CollisionOffset);
                    }
                    else
                    {
                        upStepHitDistance = upCheckDistance;
                    }

                    if (upStepHitDistance > 0f)
                    {
                        float3 startPositionOfForwardCheck = startPositionOfUpCheck + (upCheckDirection * upStepHitDistance);
                        float distanceOverStep = math.length(math.projectsafe(remainingMovementDirection * (remainingMovementLength - hitDistance), hit.Normal));
                        float3 endPositionOfForwardCheck = startPositionOfForwardCheck + (remainingMovementDirection * remainingMovementLength);
                        float minimumDistanceOverStep = Constants.CollisionOffset * 3f;
                        if (distanceOverStep < minimumDistanceOverStep)
                        {
                            endPositionOfForwardCheck += -hit.Normal * (minimumDistanceOverStep - distanceOverStep);
                        }
                        float3 forwardCheckDirection = math.normalizesafe(endPositionOfForwardCheck - startPositionOfForwardCheck);
                        float forwardCheckDistance = math.length(endPositionOfForwardCheck - startPositionOfForwardCheck);

                        // Forward cast
                        bool foundForwardStepHit = CastColliderClosestCollisions(
                            ref processor,
                            in characterPhysicsCollider,
                            characterEntity,
                            startPositionOfForwardCheck,
                            characterRotation,
                            forwardCheckDirection,
                            forwardCheckDistance,
                            true,
                            characterBody.ShouldIgnoreDynamicBodies(),
                            out ColliderCastHit forwardStepHit,
                            out float forwardStepHitDistance);

                        if (foundForwardStepHit)
                        {
                            forwardStepHitDistance = math.max(0f, forwardStepHitDistance - Constants.CollisionOffset);
                        }
                        else
                        {
                            forwardStepHitDistance = forwardCheckDistance;
                        }

                        if (forwardStepHitDistance > 0f)
                        {
                            float3 startPositionOfDownCheck = startPositionOfForwardCheck + (forwardCheckDirection * forwardStepHitDistance);
                            float3 downCheckDirection = -groundingUp;
                            float downCheckDistance = upStepHitDistance;

                            // Down cast
                            bool foundDownStepHit = CastColliderClosestCollisions(
                                ref processor,
                                in characterPhysicsCollider,
                                characterEntity,
                                startPositionOfDownCheck,
                                characterRotation,
                                downCheckDirection,
                                downCheckDistance,
                                true,
                                characterBody.ShouldIgnoreDynamicBodies(),
                                out ColliderCastHit downStepHit,
                                out float downStepHitDistance);

                            if (foundDownStepHit && downStepHitDistance > 0f)
                            {
                                BasicHit stepHit = new BasicHit(downStepHit);
                                bool isGroundedOnStepHit = false;
                                if (characterBody.EvaluateGrounding)
                                {
                                    isGroundedOnStepHit = processor.IsGroundedOnHit(in stepHit, (int)GroundingEvaluationType.StepUpHit);
                                }

                                if (isGroundedOnStepHit)
                                {
                                    float steppedHeight = upStepHitDistance - downStepHitDistance;

                                    if (steppedHeight > Constants.CollisionOffset)
                                    {
                                        // Step up
                                        characterTranslation += groundingUp * steppedHeight;
                                        characterTranslation += forwardCheckDirection * forwardStepHitDistance;

                                        characterBody.IsGrounded = true;
                                        characterBody.GroundHit = stepHit;

                                        // Project vel
                                        characterBody.RelativeVelocity = MathUtilities.ProjectOnPlane(characterBody.RelativeVelocity, groundingUp);
                                        remainingMovementDirection = math.normalizesafe(characterBody.RelativeVelocity);
                                        remainingMovementLength -= forwardStepHitDistance;

                                        hasSteppedUp = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            public static void DetectFutureSlopeChange<T>(
                ref T processor,
                in BasicHit groundHit,
                in KinematicCharacterBody characterBody,
                in PhysicsCollider characterPhysicsCollider,
                Entity characterEntity,
                float3 detectionVelocity,
                float3 groundingUp,
                float verticalOffset,
                float downDetectionDepth,
                float deltaTimeIntoFuture,
                float secondaryNoGroundingCheckDistance,
                bool stepHandling, 
                float maxStepHeight,
                out bool isMovingTowardsNoGrounding,
                out bool foundSlopeHit,
                out float futureSlopeChangeAnglesRadians,
                out RaycastHit futureSlopeHit) where T : struct, IKinematicCharacterProcessor
            {
                float CalculateAngleOfHitWithCurrentGroundUp(float3 currentGroundUp, float3 hitNormal, float3 velocityDirection)
                {
                    float3 velocityRight = math.normalizesafe(math.cross(velocityDirection, -groundingUp));
                    float3 currentGroundNormalOnPlane = MathUtilities.ProjectOnPlane(currentGroundUp, velocityRight);
                    float3 downHitNormalOnPlane = MathUtilities.ProjectOnPlane(hitNormal, velocityRight);
                    float slopeChangeAnglesRadians = MathUtilities.AngleRadians(currentGroundNormalOnPlane, downHitNormalOnPlane);

                    // invert angle sign if it's a downward slope change
                    if(math.dot(currentGroundNormalOnPlane, velocityDirection) < math.dot(downHitNormalOnPlane, velocityDirection))
                    {
                        slopeChangeAnglesRadians *= -1;
                    }

                    return slopeChangeAnglesRadians;
                }

                isMovingTowardsNoGrounding = false;
                foundSlopeHit = false;
                futureSlopeChangeAnglesRadians = 0f;
                futureSlopeHit = default;

                if (IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, groundHit.Normal, groundingUp))
                {
                    if(stepHandling)
                    {
                        downDetectionDepth = math.max(maxStepHeight, downDetectionDepth) + verticalOffset;
                    }
                    else
                    {
                        downDetectionDepth = downDetectionDepth + verticalOffset;
                    }

                    float3 velocityDirection = math.normalizesafe(detectionVelocity);
                    float3 rayStartPoint = groundHit.Position + (groundingUp * verticalOffset);
                    float3 rayDirection = velocityDirection;
                    float rayLength = math.length(detectionVelocity * deltaTimeIntoFuture);

                    if (rayLength > math.EPSILON)
                    {
                        // Raycast forward 
                        bool forwardHitFound = RaycastClosestCollisions(
                            ref processor,
                            in characterPhysicsCollider,
                            characterEntity,
                            rayStartPoint,
                            rayDirection,
                            rayLength,
                            characterBody.ShouldIgnoreDynamicBodies(),
                            out RaycastHit forwardHit,
                            out float forwardHitDistance);

                        if (forwardHitFound)
                        {
                            foundSlopeHit = true;
                            futureSlopeChangeAnglesRadians = CalculateAngleOfHitWithCurrentGroundUp(groundHit.Normal, forwardHit.SurfaceNormal, velocityDirection);
                            futureSlopeHit = forwardHit;
                        }
                        else
                        {
                            rayStartPoint = rayStartPoint + (rayDirection * rayLength);
                            rayDirection = -groundingUp;
                            rayLength = downDetectionDepth;

                            // Raycast down 
                            bool downHitFound = RaycastClosestCollisions(
                                ref processor,
                                in characterPhysicsCollider,
                                characterEntity,
                                rayStartPoint,
                                rayDirection,
                                rayLength,
                                characterBody.ShouldIgnoreDynamicBodies(),
                                out RaycastHit downHit,
                                out float downHitDistance);

                            if (downHitFound)
                            {
                                foundSlopeHit = true;
                                futureSlopeChangeAnglesRadians = CalculateAngleOfHitWithCurrentGroundUp(groundHit.Normal, downHit.SurfaceNormal, velocityDirection);
                                futureSlopeHit = downHit;

                                if (!IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, downHit.SurfaceNormal, groundingUp))
                                {
                                    isMovingTowardsNoGrounding = true;
                                }
                            }
                            else
                            {
                                isMovingTowardsNoGrounding = true;
                            }

                            if(isMovingTowardsNoGrounding)
                            {
                                rayStartPoint += velocityDirection * secondaryNoGroundingCheckDistance;

                                // Raycast down (secondary)
                                bool secondDownHitFound = RaycastClosestCollisions(
                                    ref processor,
                                    in characterPhysicsCollider,
                                    characterEntity,
                                    rayStartPoint,
                                    rayDirection,
                                    rayLength,
                                    characterBody.ShouldIgnoreDynamicBodies(),
                                    out RaycastHit secondDownHit,
                                    out float secondDownHitDistance);

                                if (secondDownHitFound)
                                {
                                    if (!foundSlopeHit)
                                    {
                                        foundSlopeHit = true;
                                        futureSlopeChangeAnglesRadians = CalculateAngleOfHitWithCurrentGroundUp(groundHit.Normal, secondDownHit.SurfaceNormal, velocityDirection);
                                        futureSlopeHit = secondDownHit;
                                    }

                                    if (IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, secondDownHit.SurfaceNormal, groundingUp))
                                    {
                                        isMovingTowardsNoGrounding = false;
                                    }
                                }
                                else
                                {
                                    rayStartPoint += rayDirection * rayLength;
                                    rayDirection = -velocityDirection;
                                    rayLength = math.length(detectionVelocity * deltaTimeIntoFuture) + secondaryNoGroundingCheckDistance;

                                    // Raycast backward
                                    bool backHitFound = RaycastClosestCollisions(
                                        ref processor,
                                        in characterPhysicsCollider,
                                        characterEntity,
                                        rayStartPoint,
                                        rayDirection,
                                        rayLength,
                                        characterBody.ShouldIgnoreDynamicBodies(),
                                        out RaycastHit backHit,
                                        out float backHitDistance); 
                                    
                                    if (backHitFound)
                                    {
                                        foundSlopeHit = true;
                                        futureSlopeChangeAnglesRadians = CalculateAngleOfHitWithCurrentGroundUp(groundHit.Normal, backHit.SurfaceNormal, velocityDirection);
                                        futureSlopeHit = backHit;

                                        if (IsGroundedOnSlopeNormal(characterBody.MaxGroundedSlopeDotProduct, backHit.SurfaceNormal, groundingUp))
                                        {
                                            isMovingTowardsNoGrounding = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}