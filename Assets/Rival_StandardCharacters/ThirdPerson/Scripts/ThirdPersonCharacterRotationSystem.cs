using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using Rival;

[UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
[UpdateAfter(typeof(ThirdPersonPlayerSystem))]
[UpdateBefore(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(TransformSystemGroup))]
public partial class ThirdPersonCharacterRotationSystem : SystemBase
{
    public FixedStepSimulationSystemGroup FixedStepSimulationSystemGroup;
    public EntityQuery CharacterQuery;

    protected override void OnCreate()
    {
        base.OnCreate();

        FixedStepSimulationSystemGroup = World.GetOrCreateSystem<FixedStepSimulationSystemGroup>();

        CharacterQuery = GetEntityQuery(new EntityQueryDesc
        {
            All = MiscUtilities.CombineArrays(
                KinematicCharacterUtilities.GetCoreCharacterComponentTypes(),
                new ComponentType[]
                {
                    typeof(ThirdPersonCharacterComponent),
                    typeof(ThirdPersonCharacterInputs),
                }),
        });

        RequireForUpdate(CharacterQuery);
    }

    protected unsafe override void OnUpdate()
    {
        float deltaTime = Time.DeltaTime;
        float fixedDeltaTime = FixedStepSimulationSystemGroup.RateManager.Timestep;

        Entities.ForEach((
            Entity entity,
            ref Rotation characterRotation,
            ref ThirdPersonCharacterComponent character,
            in ThirdPersonCharacterInputs characterInputs,
            in KinematicCharacterBody characterBody) =>
        {
            // Rotate towards move direction
            if (math.lengthsq(characterInputs.MoveVector) > 0f)
            {
                CharacterControlUtilities.SlerpRotationTowardsDirectionAroundUp(ref characterRotation.Value, deltaTime, math.normalizesafe(characterInputs.MoveVector), MathUtilities.GetUpFromRotation(characterRotation.Value), character.RotationSharpness);
            }

            // Add rotation from parent body to the character rotation
            // (this is for allowing a rotating moving platform to rotate your character as well, and handle interpolation properly)
            KinematicCharacterUtilities.ApplyParentRotationToTargetRotation(ref characterRotation.Value, in characterBody, fixedDeltaTime, deltaTime);
        }).Schedule();
    }
}
