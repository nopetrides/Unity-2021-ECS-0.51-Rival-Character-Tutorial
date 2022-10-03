using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace ECS.Systems
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(BuildPhysicsWorld))]
    public partial class MovingPlatformSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            float deltaTime = Time.DeltaTime;
            float invDelta = 1f / deltaTime;
            float time = (float)World.Time.ElapsedTime;
            Entities.ForEach((Entity entity, ref MovingPlatform movingPlatform, ref PhysicsVelocity physicsVelocity,
            in PhysicsMass physicsMass, in Translation translation, in Rotation rotation) => {
                if (!movingPlatform.IsInitialized)
                {
                    // Save the initial values, because we move in reference to them
                    movingPlatform.OriginalPosition = translation.Value;
                    movingPlatform.OriginalRotation = rotation.Value;
                    movingPlatform.IsInitialized = true;
                }

                float3 targetPos =
                    movingPlatform.OriginalPosition +
                    (math.normalizesafe(movingPlatform.TranslationAxis) *
                     math.sin(time * movingPlatform.TranslationSpeed) * movingPlatform.TranslationAmplitude);
                quaternion rotationFromMovement = quaternion.Euler(math.normalizesafe(movingPlatform.RotationAxis) *
                                                                   movingPlatform.RotationSpeed * time);
                quaternion targetRot = math.mul(rotationFromMovement, movingPlatform.OriginalRotation);
                
                // Final update
                physicsVelocity = PhysicsVelocity.CalculateVelocityToTarget(in physicsMass, in translation,
                    in rotation, new RigidTransform(targetRot, targetPos), invDelta);
            }).Schedule();
        }
    }
}
