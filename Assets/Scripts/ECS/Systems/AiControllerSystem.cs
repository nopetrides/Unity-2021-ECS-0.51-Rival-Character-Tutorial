using ECS.Components;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace ECS.Systems
{
    public partial class AiControllerSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            PhysicsWorld physicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>().PhysicsWorld;
            NativeList<DistanceHit> distanceHits = new NativeList<DistanceHit>(Allocator.TempJob);

            Entities.
                WithDisposeOnCompletion(distanceHits).
                ForEach((ref ThirdPersonCharacterInputs characterInputs, in AiController aiController, 
                    in ThirdPersonCharacterComponent character, in Translation translation) => 
                {
                    distanceHits.Clear();

                    AllHitsCollector<DistanceHit> hitsCollector = new AllHitsCollector<DistanceHit>(aiController.DetectionDistance, ref distanceHits);

                    PointDistanceInput distanceInput = new PointDistanceInput
                    {
                        Position = translation.Value,
                        MaxDistance = aiController.DetectionDistance,
                        Filter = new CollisionFilter()
                        {
                            BelongsTo = CollisionFilter.Default.BelongsTo,
                            CollidesWith = aiController.DetectionFilter.Value
                        }
                    };

                    physicsWorld.CalculateDistance(distanceInput, ref hitsCollector);

                    Entity selectedTarget = Entity.Null;
                    for (int i = 0; i < hitsCollector.NumHits; i++)
                    {
                        Entity hitEntity = distanceHits[i].Entity;

                        if (HasComponent<ThirdPersonCharacterComponent>(hitEntity) &&
                            !HasComponent<AiController>(hitEntity))
                        {
                            selectedTarget = hitEntity;
                            break;
                        }
                    }

                    if (selectedTarget != Entity.Null)
                    {
                        characterInputs.MoveVector = math.normalizesafe(GetComponent<Translation>(selectedTarget).Value - translation.Value);
                    }
                    else
                    {
                        characterInputs.MoveVector = float3.zero;
                    }
                }).Schedule();
        }
    }
}
