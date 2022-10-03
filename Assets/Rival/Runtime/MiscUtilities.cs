using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;
using System.Linq;

namespace Rival
{
    public static class MiscUtilities
    {
        public static T[] CombineArrays<T>(T[] a, T[] b)
        {
            var list = a.ToList<T>();
            list.AddRange(b);
            return list.ToArray<T>();
        }
    }
}
