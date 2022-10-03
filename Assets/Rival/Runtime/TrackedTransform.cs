using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Rival
{
    [Serializable]
    public struct TrackedTransform : IComponentData
    {
        [HideInInspector]
        public RigidTransform CurrentFixedRateTransform;
        [HideInInspector]
        public RigidTransform PreviousFixedRateTransform;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3 CalculatePointDisplacement(float3 point)
        {
            float3 characterLocalPositionToPreviousParentTransform = math.transform(math.inverse(PreviousFixedRateTransform), point);
            float3 characterTargetTranslation = math.transform(CurrentFixedRateTransform, characterLocalPositionToPreviousParentTransform);
            return characterTargetTranslation - point;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float3 CalculatePointVelocity(float3 point, float deltaTime)
        {
            return CalculatePointDisplacement(point) / deltaTime;
        }
    }
}