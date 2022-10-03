using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

[UpdateInGroup(typeof(FixedStepSimulationSystemGroup), OrderFirst = true)]
public partial class FixedUpdateTickSystem : SystemBase
{
    public uint FixedTick;

    protected override void OnUpdate()
    {
        FixedTick++;
    }
}
