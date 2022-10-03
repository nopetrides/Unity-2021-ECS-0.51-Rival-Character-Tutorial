

using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;

public class MainGameObjectCamera : MonoBehaviour
{
    private void Update()
    {
        foreach (World world in World.All)
        {
            MainCameraSystem mainCameraSystem = world.GetExistingSystem<MainCameraSystem>();
            if (mainCameraSystem != null && mainCameraSystem.Enabled)
            {
                mainCameraSystem.CameraGameObjectTransform = this.transform;
                Destroy(this);
            }
        }
    }
}