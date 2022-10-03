using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Rival
{
    [UpdateInGroup(typeof(KinematicCharacterUpdateGroup), OrderFirst = true)]
    public partial class StoreKinematicCharacterBodyPropertiesSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            Dependency = Entities.ForEach((ref StoredKinematicCharacterBodyProperties storedProperties, in KinematicCharacterBody characterBody) =>
            {
                storedProperties.FromCharacterBody(in characterBody);
            }).ScheduleParallel(Dependency);
        }
    }
}