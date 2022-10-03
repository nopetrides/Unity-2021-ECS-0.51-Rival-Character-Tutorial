using System;
using Unity.Entities;
using Unity.Physics.Authoring;

namespace ECS.Components
{
    [Serializable]
    [GenerateAuthoringComponent]
    public struct AiController : IComponentData
    {
        public float DetectionDistance;
        public PhysicsCategoryTags DetectionFilter;
    }
}
