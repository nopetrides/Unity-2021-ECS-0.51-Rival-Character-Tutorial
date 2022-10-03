using ECS.Components;
using Rival;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Stateful;
using Unity.Transforms;

namespace ECS.Systems
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StatefulTriggerEventBufferSystem))]
    public partial class JumpPadSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            Entities.WithoutBurst().ForEach((Entity entity, in Rotation rotation, in JumpPad jumpPad, in DynamicBuffer<StatefulTriggerEvent> triggerEventsBuffer) =>
            {
                for (int i = 0; i < triggerEventsBuffer.Length; i++)
                {
                    StatefulTriggerEvent triggerEvent = triggerEventsBuffer[i];
                    Entity otherEntity = triggerEvent.GetOtherEntity(entity);

                    if (triggerEvent.State == StatefulEventState.Enter &&
                        HasComponent<KinematicCharacterBody>(otherEntity))
                    {
                        KinematicCharacterBody characterBody = GetComponent<KinematicCharacterBody>(otherEntity);

                        characterBody.RelativeVelocity = MathUtilities.ProjectOnPlane(characterBody.RelativeVelocity,
                            math.normalizesafe(jumpPad.JumpForce));

                        characterBody.RelativeVelocity += jumpPad.JumpForce;
                        
                        characterBody.Unground();

                        SetComponent(otherEntity, characterBody);
                    }
                }
            }).Run();
        }
    }
}
