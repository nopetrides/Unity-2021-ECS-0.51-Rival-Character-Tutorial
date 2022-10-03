using Rival;
using Unity.Entities;
using UnityEngine;

namespace ECS.Systems
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(KinematicCharacterUpdateGroup))]
    public partial class CharacterHitsDetectionSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            Entities.ForEach((Entity entity, 
                ref DynamicBuffer<KinematicCharacterHit> characterHitsBuffer) => 
            {
                for (int i = 0; i < characterHitsBuffer.Length; i++)
                {
                    KinematicCharacterHit hit = characterHitsBuffer[i];
                    if (!hit.IsGroundedOnHit)
                    {
                        //Debug.Log("Detected an ungrounded hit");
                    }
                }
            }).Run();

            Entities.ForEach((Entity entity,
                ref DynamicBuffer<StatefulKinematicCharacterHit> statefulKinematicCharacterHitsBuffer) =>
            {
                for (int i = 0; i < statefulKinematicCharacterHitsBuffer.Length; i++)
                {
                    StatefulKinematicCharacterHit hit = statefulKinematicCharacterHitsBuffer[i];
                    if (hit.State == CharacterHitState.Enter)
                    {
                        //Debug.Log("Entered new hit");
                    }
                    if (hit.State == CharacterHitState.Exit)
                    {
                        //Debug.Log("Left existing hit");
                    }
                }
            }).Run();
        }
    }
}
