using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace Rival
{
    public static class MathUtilities
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion FromToRotation(quaternion from, quaternion to)
        {
            return math.mul(math.inverse(from), to);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float AngleRadians(float3 from, float3 to)
        {
            float denominator = (float)math.sqrt(math.lengthsq(from) * math.lengthsq(to));
            if (denominator < math.EPSILON)
                return 0F;

            float dot = math.clamp(math.dot(from, to) / denominator, -1F, 1F);
            return math.acos(dot);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float AngleRadiansToDotRatio(float angleRadians)
        {
            return math.cos(angleRadians);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float DotRatioToAngleRadians(float dotRatio)
        {
            return math.acos(dotRatio);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ProjectOnPlane(float3 vector, float3 onPlaneNormal)
        {
            return vector - math.projectsafe(vector, onPlaneNormal);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ReverseProjectOnVector(float3 projectedVector, float3 onNormalizedVector, float maxLength)
        {
            float projectionRatio = math.dot(math.normalizesafe(projectedVector), onNormalizedVector);
            if (projectionRatio == 0f)
            {
                return projectedVector;
            }

            float deprojectedLength = math.clamp(math.length(projectedVector) / projectionRatio, 0f, maxLength);
            return onNormalizedVector * deprojectedLength;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ClampToMaxLength(float3 vector, float maxLength)
        {
            float sqrmag = math.lengthsq(vector);
            if (sqrmag > maxLength * maxLength)
            {
                float mag = math.sqrt(sqrmag);
                float normalized_x = vector.x / mag;
                float normalized_y = vector.y / mag;
                float normalized_z = vector.z / mag;
                return new float3(normalized_x * maxLength,
                    normalized_y * maxLength,
                    normalized_z * maxLength);
            }

            return vector;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetUpFromRotation(quaternion rot)
        {
            return math.mul(rot, math.up());
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetRightFromRotation(quaternion rot)
        {
            return math.mul(rot, math.right());
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetForwardFromRotation(quaternion rot)
        {
            return math.mul(rot, math.forward());
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float GetSharpnessInterpolant(float sharpness, float dt)
        {
            return math.saturate(1f - math.exp(-sharpness * dt));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 ReorientVectorOnPlaneAlongDirection(float3 vector, float3 onPlaneNormal, float3 alongDirection)
        {
            float length = math.length(vector);

            if (length <= math.EPSILON)
                return float3.zero;

            float3 reorientAxis = math.cross(vector, alongDirection);
            float3 reorientedVector = math.normalizesafe(math.cross(onPlaneNormal, reorientAxis)) * length;

            return reorientedVector;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static quaternion CreateRotationWithUpPriority(float3 up, float3 forward)
        {
            if (math.abs(math.dot(forward, up)) == 1f)
            {
                forward = math.forward();
            }
            forward = math.normalizesafe(MathUtilities.ProjectOnPlane(forward, up));

            return quaternion.LookRotationSafe(forward, up);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetAxisSystemFromForward(float3 fwd, out float3 right, out float3 up)
        {
            float3 initialVector = math.up();
            if (math.dot(fwd, initialVector) > 0.9f)
            {
                initialVector = math.right();
            }

            right = math.normalizesafe(math.cross(initialVector, fwd));
            up = math.normalizesafe(math.cross(fwd, right));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float2 GetClosestPointOnSegment(float2 a, float2 b, float2 p, out float ratio)
        {
            ratio = 0f;
            float2 ap = p - a;
            float2 ab = b - a;
            float magnitudeSqAB = math.lengthsq(ab);

            if (magnitudeSqAB <= 0f)
            {
                return a;
            }

            float ABAPproduct = math.dot(ap, ab);
            ratio = ABAPproduct / magnitudeSqAB;

            if (ratio < 0f)
            {
                return a;
            }
            else if (ratio > 1f)
            {
                return b;
            }
            else
            {
                return a + ((b - a) * ratio);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsPointInsideTriangle(float2 p, float2 p1, float2 p2, float2 p3)
        {
            float a = ((p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y)) / ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));
            float b = ((p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y)) / ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));
            float c = 1.0f - a - b;

            return a > 0f && b > 0f && c > 0f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 CalculatePointDisplacement(float3 pointWorldSpace, RigidTransform fromTransform, RigidTransform toTransform)
        {
            float3 pointLocalPositionRelativeToPreviousTransform = math.transform(math.inverse(fromTransform), pointWorldSpace);
            float3 pointNewWorldPosition = math.transform(toTransform, pointLocalPositionRelativeToPreviousTransform);
            return pointNewWorldPosition - pointWorldSpace;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 CalculatePointDisplacementFromVelocity(float deltaTime, RigidTransform bodyRigidTransform, float3 linearVelocity, float3 angularVelocity, float3 pointWorldSpace)
        {
            RigidTransform targetBodyRigidTransform = new RigidTransform();
            targetBodyRigidTransform.pos = bodyRigidTransform.pos + (linearVelocity * deltaTime);
            targetBodyRigidTransform.rot = math.mul(bodyRigidTransform.rot, quaternion.Euler(angularVelocity * deltaTime));

            return CalculatePointDisplacement(pointWorldSpace, bodyRigidTransform, targetBodyRigidTransform);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetRotationAroundPoint(ref quaternion rotation, ref float3 translation, float3 aroundPoint, quaternion targetRotation)
        {
            float3 localPointToTranslation = math.mul(math.inverse(rotation), translation - aroundPoint);
            translation = aroundPoint + math.mul(targetRotation, localPointToTranslation);
            rotation = targetRotation;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void RotateAroundPoint(ref quaternion rotation, ref float3 translation, float3 aroundPoint, quaternion addedRotation)
        {
            float3 localPointToTranslation = math.mul(math.inverse(rotation), translation - aroundPoint);
            rotation = math.mul(rotation, addedRotation);
            translation = aroundPoint + math.mul(rotation, localPointToTranslation);
        }
    }
}