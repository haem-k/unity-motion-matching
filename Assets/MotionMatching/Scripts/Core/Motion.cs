using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class Motion : ScriptableObject
{
    public string motionName;
    public Pose[] poses;

    public int FrameNum()
    {
        return poses.Length;
    }
}
