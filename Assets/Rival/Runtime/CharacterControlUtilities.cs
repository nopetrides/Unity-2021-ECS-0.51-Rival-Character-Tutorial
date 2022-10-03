using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;

namespace Rival
{
    public static class CharacterControlUtilities
    {
        // calculates the signed slope angle (radians) in a given movement direction.
        // The resulting angle will be positive if the slope goes up, and negative if the slope goes down.
        public static float GetSlopeAngleTowardsDirection(bool useDegrees, float3 moveDirection, float3 slopeNormal, float3 groundingUp)
        {
            float3 moveDirectionOnSlopePlane = math.normalizesafe(MathUtilities.ProjectOnPlane(moveDirection, slopeNormal));
            float angleRadiansWithUp = MathUtilities.AngleRadians(moveDirectionOnSlopePlane, groundingUp);

            if (useDegrees)
            {
                return 90f - math.degrees(angleRadiansWithUp);
            }
            else
            {
                return (math.PI * 0.5f) - angleRadiansWithUp;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void StandardGroundMove_Interpolated(ref float3 velocity, float3 targetVelocity, float sharpness, float deltaTime, float3 groundingUp, float3 groundedHitNormal)
        {
            velocity = MathUtilities.ReorientVectorOnPlaneAlongDirection(velocity, groundedHitNormal, groundingUp);
            targetVelocity = MathUtilities.ReorientVectorOnPlaneAlongDirection(targetVelocity, groundedHitNormal, groundingUp);
            InterpolateVelocityTowardsTarget(ref velocity, targetVelocity, deltaTime, sharpness);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void StandardGroundMove_Accelerated(ref float3 velocity, float3 acceleration, float maxSpeed, float deltaTime, float3 movementPlaneUp, float3 groundedHitNormal, bool forceNoSpeedExcess)
        {
            float3 addedVelocityFromAcceleration = float3.zero;
            AccelerateVelocity(ref addedVelocityFromAcceleration, acceleration, deltaTime);

            velocity = MathUtilities.ReorientVectorOnPlaneAlongDirection(velocity, groundedHitNormal, movementPlaneUp);
            addedVelocityFromAcceleration = MathUtilities.ReorientVectorOnPlaneAlongDirection(addedVelocityFromAcceleration, groundedHitNormal, movementPlaneUp);
            ClampAdditiveVelocityToMaxSpeedOnPlane(ref addedVelocityFromAcceleration, velocity, maxSpeed, groundedHitNormal, forceNoSpeedExcess);
            velocity += addedVelocityFromAcceleration;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void StandardAirMove(ref float3 velocity, float3 acceleration, float maxSpeed, float3 movementPlaneUp, float deltaTime, bool forceNoMaxSpeedExcess)
        {
            float3 addedVelocityFromAcceleration = float3.zero;
            AccelerateVelocity(ref addedVelocityFromAcceleration, acceleration, deltaTime);
            ClampAdditiveVelocityToMaxSpeedOnPlane(ref addedVelocityFromAcceleration, velocity, maxSpeed, movementPlaneUp, forceNoMaxSpeedExcess);
            velocity += addedVelocityFromAcceleration;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InterpolateVelocityTowardsTarget(ref float3 velocity, float3 targetVelocity, float deltaTime, float interpolationSharpness)
        {
            velocity = math.lerp(velocity, targetVelocity, MathUtilities.GetSharpnessInterpolant(interpolationSharpness, deltaTime));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void AccelerateVelocity(ref float3 velocity, float3 acceleration, float deltaTime)
        {
            velocity += acceleration * deltaTime;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClampAdditiveVelocityToMaxSpeedOnPlane(ref float3 additiveVelocity, float3 originalVelocity, float maxSpeed, float3 movementPlaneUp, bool forceNoMaxSpeedExcess)
        {
            if (forceNoMaxSpeedExcess)
            {
                float3 totalVelocity = originalVelocity + additiveVelocity;
                float3 velocityUp = math.projectsafe(totalVelocity, movementPlaneUp);
                float3 velocityHorizontal = MathUtilities.ProjectOnPlane(totalVelocity, movementPlaneUp);
                velocityHorizontal = MathUtilities.ClampToMaxLength(velocityHorizontal, maxSpeed);
                additiveVelocity = (velocityHorizontal + velocityUp) - originalVelocity;
            }
            else
            {
                float maxSpeedSq = maxSpeed * maxSpeed;

                float3 additiveVelocityOnPlaneUp = math.projectsafe(additiveVelocity, movementPlaneUp);
                float3 additiveVelocityOnPlane = additiveVelocity - additiveVelocityOnPlaneUp;

                float3 originalVelocityOnPlaneUp = math.projectsafe(originalVelocity, movementPlaneUp);
                float3 originalVelocityOnPlane = originalVelocity - originalVelocityOnPlaneUp;

                float3 totalVelocityOnPlane = originalVelocityOnPlane + additiveVelocityOnPlane;

                if (math.lengthsq(totalVelocityOnPlane) > maxSpeedSq)
                {
                    float3 originalVelocityForwardOnPlane = math.normalizesafe(originalVelocityOnPlane);
                    float3 totalVelocityDirectionOnPlane = math.normalizesafe(totalVelocityOnPlane);

                    float3 totalClampedVelocityOnPlane = float3.zero;
                    if (math.dot(totalVelocityDirectionOnPlane, originalVelocityForwardOnPlane) > 0f)
                    {
                        float3 originalVelocityRightOnPlane = math.normalizesafe(math.cross(originalVelocityForwardOnPlane, movementPlaneUp));

                        // trim additive velocity excess in original velocity direction
                        float3 trimmedTotalVelocityForwardComponent = MathUtilities.ClampToMaxLength(math.projectsafe(totalVelocityOnPlane, originalVelocityForwardOnPlane), math.max(maxSpeed, math.length(originalVelocityOnPlane)));
                        float3 trimmedTotalVelocityRightComponent = MathUtilities.ClampToMaxLength(math.projectsafe(totalVelocityOnPlane, originalVelocityRightOnPlane), maxSpeed);
                        totalClampedVelocityOnPlane = trimmedTotalVelocityForwardComponent + trimmedTotalVelocityRightComponent;
                    }
                    else
                    {
                        // clamp totalvelocity to circle
                        totalClampedVelocityOnPlane = MathUtilities.ClampToMaxLength(totalVelocityOnPlane, maxSpeed);
                    }

                    float3 clampedAdditiveVelocityOnPlane = totalClampedVelocityOnPlane - originalVelocityOnPlane;
                    additiveVelocity = clampedAdditiveVelocityOnPlane + additiveVelocityOnPlaneUp;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void StandardJump(ref KinematicCharacterBody characterBody, float3 jumpVelocity, bool cancelVelocityBeforeJump = false, float3 velocityCancelingUpDirection = default)
        {
            // Without this, the ground snapping mecanism would prevent you from jumping
            characterBody.Unground();

            if (cancelVelocityBeforeJump)
            {
                characterBody.RelativeVelocity = MathUtilities.ProjectOnPlane(characterBody.RelativeVelocity, velocityCancelingUpDirection);
            }

            characterBody.RelativeVelocity += jumpVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyDragToVelocity(ref float3 velocity, float deltaTime, float drag)
        {
            velocity *= (1f / (1f + (drag * deltaTime)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 GetLinearVelocityForMovePosition(float deltaTime, float3 positionDelta)
        {
            if (deltaTime > 0f)
            {
                return positionDelta / deltaTime;
            }

            return default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SlerpRotationTowardsDirectionAroundUp(ref quaternion rotation, float deltaTime, float3 worldTargetDirection, float3 upDirection, float orientationSharpness)
        {
            if (math.lengthsq(worldTargetDirection) > 0f)
            {
                rotation = math.slerp(rotation, MathUtilities.CreateRotationWithUpPriority(upDirection, worldTargetDirection), MathUtilities.GetSharpnessInterpolant(orientationSharpness, deltaTime));
            }
        }
    }
}
