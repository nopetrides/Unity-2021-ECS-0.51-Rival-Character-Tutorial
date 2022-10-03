using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
[UpdateBefore(typeof(FixedStepSimulationSystemGroup))]
public partial class ThirdPersonPlayerSystem : SystemBase
{
    public FixedUpdateTickSystem FixedUpdateTickSystem;

    protected override void OnCreate()
    {
        base.OnCreate();

        FixedUpdateTickSystem = World.GetOrCreateSystem<FixedUpdateTickSystem>();
    }

    protected override void OnUpdate()
    {
        uint fixedTick = FixedUpdateTickSystem.FixedTick;

        // Gather raw input
        float2 moveInput = float2.zero;
        moveInput.y += Input.GetKey(KeyCode.W) ? 1f : 0f;
        moveInput.y += Input.GetKey(KeyCode.S) ? -1f : 0f;
        moveInput.x += Input.GetKey(KeyCode.D) ? 1f : 0f;
        moveInput.x += Input.GetKey(KeyCode.A) ? -1f : 0f;
        bool jumpInput = Input.GetKeyDown(KeyCode.Space);
        bool sprintInput = Input.GetKey(KeyCode.LeftShift);
        float2 cameraLookInput = new float2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
        float cameraZoomInput = -Input.mouseScrollDelta.y;

        Entities
            .ForEach((ref ThirdPersonPlayer player) =>
            {
                if (HasComponent<ThirdPersonCharacterInputs>(player.ControlledCharacter))
                {
                    ThirdPersonCharacterInputs characterInputs = GetComponent<ThirdPersonCharacterInputs>(player.ControlledCharacter);

                    quaternion cameraRotation = GetComponent<Rotation>(player.ControlledCamera).Value;
                    float3 cameraForwardOnUpPlane = math.normalizesafe(Rival.MathUtilities.ProjectOnPlane(Rival.MathUtilities.GetForwardFromRotation(cameraRotation), math.up()));
                    float3 cameraRight = Rival.MathUtilities.GetRightFromRotation(cameraRotation);

                    // Move
                    characterInputs.MoveVector = (moveInput.y * cameraForwardOnUpPlane) + (moveInput.x * cameraRight);
                    characterInputs.MoveVector = Rival.MathUtilities.ClampToMaxLength(characterInputs.MoveVector, 1f);

                    // Jump
                    // Punctual input presses need special handling when they will be used in a fixed step system.
                    // We essentially need to remember if the button was pressed at any point over the last fixed update
                    if (player.LastInputsProcessingTick == fixedTick)
                    {
                        characterInputs.JumpRequested = jumpInput || characterInputs.JumpRequested; 
                    }
                    else
                    {
                        characterInputs.JumpRequested = jumpInput;
                    }

                    characterInputs.Spriting = sprintInput;

                    SetComponent(player.ControlledCharacter, characterInputs);
                }

                // Camera control
                if (HasComponent<OrbitCameraInputs>(player.ControlledCamera))
                {
                    OrbitCameraInputs cameraInputs = GetComponent<OrbitCameraInputs>(player.ControlledCamera);
                    cameraInputs.Look = cameraLookInput; 
                    cameraInputs.Zoom = cameraZoomInput; 

                    SetComponent(player.ControlledCamera, cameraInputs);
                }

                player.LastInputsProcessingTick = fixedTick;
            }).Schedule();
    }
}
