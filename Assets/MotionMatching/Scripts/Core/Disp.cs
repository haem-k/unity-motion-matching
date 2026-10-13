using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

public class Disp : ICloneable
{
    public Quaternion[] dLocalRs;
    public Vector3[] dLocalTs;

    public Disp(int jointNum, Pose from, Pose target)
    {
        dLocalRs = Enumerable.Repeat(Quaternion.identity, jointNum).ToArray();
        dLocalTs = Enumerable.Repeat(Vector3.zero, jointNum).ToArray();
        
        for (int i = 0; i < jointNum; i++)
        {
            Matrix4x4 dispTrf = target.locals[i] * from.locals[i].inverse;
            dLocalRs[i] = dispTrf.rotation;
            dLocalTs[i] = dispTrf.GetColumn(3);
        }

    }

    public Disp(Disp other)
    {
        dLocalRs = other.dLocalRs.Clone() as Quaternion[];
        dLocalTs = other.dLocalTs.Clone() as Vector3[];
    }

    public object Clone()
    {
        return new Disp(this);
    }

    public Disp DeepCopy()
    {
        return new Disp(this);
    }

    public void Scale(float blendWeight)
    {
        for (int i = 0; i < dLocalRs.Length; i++)
        {
            dLocalRs[i] = Quaternion.Slerp(Quaternion.identity, dLocalRs[i], blendWeight);
        }
        for (int i = 0; i < dLocalTs.Length; i++)
        {
            dLocalTs[i] = blendWeight * dLocalTs[i];
        }
    }
}