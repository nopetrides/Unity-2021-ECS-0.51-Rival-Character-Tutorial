using System;
using Unity.Entities;

namespace ECS.Components
{
    [Serializable]
    [GenerateAuthoringComponent]
    public struct CharacterFrictionSurface : IComponentData
    {
        public float VelocityFactor;
    }
}
