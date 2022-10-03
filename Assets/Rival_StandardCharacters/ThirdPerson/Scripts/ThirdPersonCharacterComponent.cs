using System;
using Unity.Physics.Authoring;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

[Serializable]
public struct ThirdPersonCharacterComponent : IComponentData
{
    [Header("Physics")] 
    public CustomPhysicsBodyTags IgnoredPhysicsTags;
    
    [Header("Movement")]
    public float RotationSharpness;
    public float GroundMaxSpeed;
    public float GroundedMovementSharpness;
    public float AirAcceleration;
    public float AirMaxSpeed;
    public float AirDrag;
    public float JumpSpeed;
    public int TotalJumps;
    public float SprintSpeedMultiplier;
    public float3 Gravity;

    [Header("Step Handling")]
    public bool StepHandling;
    public float MaxStepHeight;
    public float ExtraStepChecksDistance;

    [Header("Slope Changes")]
    public bool PreventGroundingWhenMovingTowardsNoGrounding;
    public bool HasMaxDownwardSlopeChangeAngle;
    [Range(0f, 180f)]
    public float MaxDownwardSlopeChangeAngle;

    [Header("Misc")]
    public bool ConstrainVelocityToGroundPlane;

    [HideInInspector]
    public float3 GroundingUp;

    [HideInInspector] 
    public int CurrentJumpsUsed;

    public static ThirdPersonCharacterComponent GetDefault()
    {
        return new ThirdPersonCharacterComponent
        {
            RotationSharpness = 25f,
            GroundMaxSpeed = 10f,
            GroundedMovementSharpness = 15f,
            AirAcceleration = 50f,
            AirMaxSpeed = 10f,
            AirDrag = 0f,
            JumpSpeed = 10f,
            Gravity = math.up() * -30f,

            StepHandling = false,
            MaxStepHeight = 0.5f,
            ExtraStepChecksDistance = 0.1f,

            PreventGroundingWhenMovingTowardsNoGrounding = true,
            HasMaxDownwardSlopeChangeAngle = false,
            MaxDownwardSlopeChangeAngle = 90f,

            ConstrainVelocityToGroundPlane = true,

            GroundingUp = math.up(),
        };
    }
}

[Serializable]
public struct ThirdPersonCharacterInputs : IComponentData
{
    public float3 MoveVector;
    public bool JumpRequested;
    public bool Spriting;
}
