using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Rival
{
    [DisallowMultipleComponent]
    public class TrackedTransformAuthoring : MonoBehaviour, IConvertGameObjectToEntity
    {
        public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
        {
            RigidTransform currentTransform = new RigidTransform(transform.rotation, transform.position);
            TrackedTransform trackedTransform = new TrackedTransform
            {
                CurrentFixedRateTransform = currentTransform,
                PreviousFixedRateTransform = currentTransform,
            };

            dstManager.AddComponentData(entity, trackedTransform);
        }
    }
}
