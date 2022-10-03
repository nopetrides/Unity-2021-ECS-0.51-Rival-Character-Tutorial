using System;
using Unity.Entities;
using Unity.Mathematics;

namespace ECS.Components
{
    [Serializable]
    [GenerateAuthoringComponent]
    public struct JumpPad : IComponentData
    {
        public float3 JumpForce;
    }
}
