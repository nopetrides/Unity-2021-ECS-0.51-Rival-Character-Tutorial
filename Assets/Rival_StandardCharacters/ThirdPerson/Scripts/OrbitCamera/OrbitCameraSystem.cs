using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;
using Rival;

[UpdateAfter(typeof(TransformSystemGroup))]
[UpdateBefore(typeof(EndSimulationEntityCommandBufferSystem))]
public partial class OrbitCameraSystem : SystemBase
{
    public BuildPhysicsWorld BuildPhysicsWorldSystem;

    protected override void OnCreate()
    {
        base.OnCreate();

        BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
    }

    protected unsafe override void OnUpdate()
    {
        float deltaTime = Time.DeltaTime;
        float fixedDeltaTime = World.GetOrCreateSystem<FixedStepSimulationSystemGroup>().RateManager.Timestep;
        CollisionWorld collisionWorld = BuildPhysicsWorldSystem.PhysicsWorld.CollisionWorld;

        // Update
        Dependency = Entities
            .WithReadOnly(collisionWorld)
            .ForEach((
                Entity entity,
                ref Translation translation,
                ref OrbitCamera orbitCamera,
                in OrbitCameraInputs inputs,
                in DynamicBuffer<OrbitCameraIgnoredEntityBufferElement> ignoredEntitiesBuffer) =>
        {
            // if there is a followed entity, place the camera relatively to it
            if (orbitCamera.FollowedCharacterEntity != Entity.Null)
            {
                Rotation selfRotation = GetComponent<Rotation>(entity);

                // Select the real camera target
                LocalToWorld targetEntityLocalToWorld = default;
                if(HasComponent<CameraTarget>(orbitCamera.FollowedCharacterEntity))
                {
                    CameraTarget cameraTarget = GetComponent<CameraTarget>(orbitCamera.FollowedCharacterEntity);
                    targetEntityLocalToWorld = GetComponent<LocalToWorld>(cameraTarget.TargetEntity);
                }
                else
                {
                    targetEntityLocalToWorld = GetComponent<LocalToWorld>(orbitCamera.FollowedCharacterEntity);
                }

                // Rotation
                {
                    selfRotation.Value = quaternion.LookRotationSafe(orbitCamera.PlanarForward, targetEntityLocalToWorld.Up);

                    // Handle rotating the camera along with character's parent entity (moving platform)
                    if (orbitCamera.RotateWithCharacterParent && HasComponent<KinematicCharacterBody>(orbitCamera.FollowedCharacterEntity))
                    {
                        KinematicCharacterBody characterBody = GetComponent<KinematicCharacterBody>(orbitCamera.FollowedCharacterEntity);
                        KinematicCharacterUtilities.ApplyParentRotationToTargetRotation(ref selfRotation.Value, in characterBody, fixedDeltaTime, deltaTime);
                        orbitCamera.PlanarForward = math.normalizesafe(MathUtilities.ProjectOnPlane(MathUtilities.GetForwardFromRotation(selfRotation.Value), targetEntityLocalToWorld.Up));
                    }

                    // Yaw
                    float yawAngleChange = inputs.Look.x * orbitCamera.RotationSpeed;
                    quaternion yawRotation = quaternion.Euler(targetEntityLocalToWorld.Up * math.radians(yawAngleChange));
                    orbitCamera.PlanarForward = math.rotate(yawRotation, orbitCamera.PlanarForward);

                    // Pitch
                    orbitCamera.PitchAngle += -inputs.Look.y * orbitCamera.RotationSpeed;
                    orbitCamera.PitchAngle = math.clamp(orbitCamera.PitchAngle, orbitCamera.MinVAngle, orbitCamera.MaxVAngle);
                    quaternion pitchRotation = quaternion.Euler(math.right() * math.radians(orbitCamera.PitchAngle));

                    // Final rotation
                    selfRotation.Value = quaternion.LookRotationSafe(orbitCamera.PlanarForward, targetEntityLocalToWorld.Up);
                    selfRotation.Value = math.mul(selfRotation.Value, pitchRotation);
                }

                float3 cameraForward = MathUtilities.GetForwardFromRotation(selfRotation.Value);

                // Distance input
                float desiredDistanceMovementFromInput = inputs.Zoom * orbitCamera.DistanceMovementSpeed * deltaTime;
                orbitCamera.TargetDistance = math.clamp(orbitCamera.TargetDistance + desiredDistanceMovementFromInput, orbitCamera.MinDistance, orbitCamera.MaxDistance);
                orbitCamera.CurrentDistanceFromMovement = math.lerp(orbitCamera.CurrentDistanceFromMovement, orbitCamera.TargetDistance, MathUtilities.GetSharpnessInterpolant(orbitCamera.DistanceMovementSharpness, deltaTime));

                // Obstructions
                if (orbitCamera.ObstructionRadius > 0f)
                {
                    float obstructionCheckDistance = orbitCamera.CurrentDistanceFromMovement;

                    CameraObstructionHitsCollector collector = new CameraObstructionHitsCollector(ignoredEntitiesBuffer, cameraForward);
                    collisionWorld.SphereCastCustom<CameraObstructionHitsCollector>(
                        targetEntityLocalToWorld.Position,
                        orbitCamera.ObstructionRadius,
                        -cameraForward,
                        obstructionCheckDistance,
                        ref collector,
                        CollisionFilter.Default,
                        QueryInteraction.IgnoreTriggers);

                    float newObstructedDistance = obstructionCheckDistance;
                    if (collector.NumHits > 0)
                    {
                        newObstructedDistance = obstructionCheckDistance * collector.ClosestHit.Fraction;

                        // Redo cast with the interpolated body transform to prevent FixedUpdate jitter in obstruction detection
                        if (orbitCamera.PreventFixedUpdateJitter)
                        {
                            RigidBody hitBody = collisionWorld.Bodies[collector.ClosestHit.RigidBodyIndex];
                            LocalToWorld hitBodyLocalToWorld = GetComponent<LocalToWorld>(hitBody.Entity);

                            hitBody.WorldFromBody = new RigidTransform(quaternion.LookRotationSafe(hitBodyLocalToWorld.Forward, hitBodyLocalToWorld.Up), hitBodyLocalToWorld.Position);

                            collector = new CameraObstructionHitsCollector(ignoredEntitiesBuffer, cameraForward);
                            hitBody.SphereCastCustom<CameraObstructionHitsCollector>(
                                targetEntityLocalToWorld.Position,
                                orbitCamera.ObstructionRadius,
                                -cameraForward,
                                obstructionCheckDistance,
                                ref collector,
                                CollisionFilter.Default,
                                QueryInteraction.IgnoreTriggers);

                            if (collector.NumHits > 0)
                            {
                                newObstructedDistance = obstructionCheckDistance * collector.ClosestHit.Fraction;
                            }
                        }
                    }

                    // Update current distance based on obstructed distance
                    if (orbitCamera.CurrentDistanceFromObstruction < newObstructedDistance)
                    {
                        // Move outer
                        orbitCamera.CurrentDistanceFromObstruction = math.lerp(orbitCamera.CurrentDistanceFromObstruction, newObstructedDistance, MathUtilities.GetSharpnessInterpolant(orbitCamera.ObstructionOuterSmoothingSharpness, deltaTime));
                    }
                    else if (orbitCamera.CurrentDistanceFromObstruction > newObstructedDistance)
                    {
                        // Move inner
                        orbitCamera.CurrentDistanceFromObstruction = math.lerp(orbitCamera.CurrentDistanceFromObstruction, newObstructedDistance, MathUtilities.GetSharpnessInterpolant(orbitCamera.ObstructionInnerSmoothingSharpness, deltaTime));
                    }
                }
                else
                {
                    orbitCamera.CurrentDistanceFromObstruction = orbitCamera.CurrentDistanceFromMovement;
                }

                // Calculate final camera position from targetposition + rotation + distance
                translation.Value = targetEntityLocalToWorld.Position + (-cameraForward * orbitCamera.CurrentDistanceFromObstruction);

                // Manually calculate the LocalToWorld since this is updating after the Transform systems, and the LtW is what rendering uses
                LocalToWorld cameraLocalToWorld = new LocalToWorld();
                cameraLocalToWorld.Value = new float4x4(selfRotation.Value, translation.Value);
                SetComponent(entity, cameraLocalToWorld);
                SetComponent(entity, selfRotation);
            }
        }).Schedule(Dependency);
    }
}

public struct CameraObstructionHitsCollector : ICollector<ColliderCastHit>
{
    public bool EarlyOutOnFirstHit => false;
    public float MaxFraction => 1f;
    public int NumHits { get; private set; }

    public ColliderCastHit ClosestHit;

    private float _closestHitFraction;
    private float3 _cameraDirection;
    private DynamicBuffer<OrbitCameraIgnoredEntityBufferElement> _ignoredEntitiesBuffer;

    public CameraObstructionHitsCollector(DynamicBuffer<OrbitCameraIgnoredEntityBufferElement> ignoredEntitiesBuffer, float3 cameraDirection)
    {
        NumHits = 0;
        ClosestHit = default;

        _closestHitFraction = float.MaxValue;
        _cameraDirection = cameraDirection;
        _ignoredEntitiesBuffer = ignoredEntitiesBuffer;
    }

    public bool AddHit(ColliderCastHit hit)
    {
        if (math.dot(hit.SurfaceNormal, _cameraDirection) < 0f || !PhysicsUtilities.IsCollidable(hit.Material))
        {
            return false;
        }

        for (int i = 0; i < _ignoredEntitiesBuffer.Length; i++)
        {
            if (_ignoredEntitiesBuffer[i].Entity == hit.Entity)
            {
                return false;
            }
        }
        
        // Add ignoring collisions to the camera to match the character physics tag ignoring system
        /*if (_ignoredPhysicsTags.Value > CustomPhysicsBodyTags.Nothing.Value)
        {
            if ((CollisionWorld.Bodies[hit.RigidBodyIndex].CustomTags & ThirdPersonCharacter.IgnoredPhysicsTags.Value) 
                > 0)
            {
                return false;
            }
        }*/

        // Process valid hit
        if (hit.Fraction < _closestHitFraction)
        {
            _closestHitFraction = hit.Fraction;
            ClosestHit = hit;
        }
        NumHits++;

        return true;
    }
}