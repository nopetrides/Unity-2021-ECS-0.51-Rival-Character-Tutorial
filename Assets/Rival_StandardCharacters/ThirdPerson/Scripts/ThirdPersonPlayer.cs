using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[Serializable]
[GenerateAuthoringComponent]
public struct ThirdPersonPlayer : IComponentData
{
    public Entity ControlledCharacter;
    public Entity ControlledCamera;

    [NonSerialized]
    public uint LastInputsProcessingTick;
}
