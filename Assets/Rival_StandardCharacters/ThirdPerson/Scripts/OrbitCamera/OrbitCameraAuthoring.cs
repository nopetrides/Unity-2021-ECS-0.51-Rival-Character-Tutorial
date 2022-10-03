using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[DisallowMultipleComponent]
public class OrbitCameraAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
    public GameObject FollowedCharacter;
    public List<GameObject> IgnoredEntities = new List<GameObject>();
    public OrbitCamera OrbitCamera = OrbitCamera.GetDefault();

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        OrbitCamera.CurrentDistanceFromMovement = OrbitCamera.TargetDistance;
        OrbitCamera.CurrentDistanceFromObstruction = OrbitCamera.TargetDistance;
        OrbitCamera.PlanarForward = -math.forward();

        if (FollowedCharacter)
        {
            OrbitCamera.FollowedCharacterEntity = conversionSystem.GetPrimaryEntity(FollowedCharacter);
        }

        dstManager.AddComponentData(entity, OrbitCamera);
        dstManager.AddComponentData(entity, new OrbitCameraInputs());
        DynamicBuffer<OrbitCameraIgnoredEntityBufferElement> ignoredEntitiesBuffer = dstManager.AddBuffer<OrbitCameraIgnoredEntityBufferElement>(entity);

        if (OrbitCamera.FollowedCharacterEntity != Entity.Null)
        {
            ignoredEntitiesBuffer.Add(new OrbitCameraIgnoredEntityBufferElement
            {
                Entity = OrbitCamera.FollowedCharacterEntity,
            });
        }
        for (int i = 0; i < IgnoredEntities.Count; i++)
        {
            ignoredEntitiesBuffer.Add(new OrbitCameraIgnoredEntityBufferElement
            {
                Entity = conversionSystem.GetPrimaryEntity(IgnoredEntities[i]),
            });
        }
    }
}