using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[Serializable]
[GenerateAuthoringComponent]
public struct CameraTarget : IComponentData
{
    public Entity TargetEntity;
}
