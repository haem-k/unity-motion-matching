using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Character: MonoBehaviour
{
    public GameObject[] joints;

    [NonSerialized]
    public int[] parentIndex;
    public Dictionary<string, int> jointName2Idx;

    void OnEnable()
    {
        // Initialize parentIndex, jointNameToIndex
        int jointNum = joints.Length;
        parentIndex = new int[jointNum];
        jointName2Idx = new Dictionary<string, int>();
        for (int i=0; i<jointNum; i++)
        {
            if(i==0)
            {
                parentIndex[i] = -1;
            }
            else
            {
                GameObject parent = joints[i].transform.parent.gameObject;
                int idx = Array.FindIndex(joints, val => val.Equals(parent));
                parentIndex[i] = idx;
            }

            jointName2Idx.Add(joints[i].name, i);
        }
    }

    public void SetPose(Pose pose)
    {
        for (int i = 0; i < pose.JointNum(); i++)
        {
            Matrix4x4 jointLocal = pose.locals[i];
            if (i == 0)
            {
                Matrix4x4 firstGlobal = pose.root * jointLocal;
                joints[i].transform.position = firstGlobal.GetColumn(3);
                joints[i].transform.rotation = firstGlobal.rotation;
            }
            else
            {
                joints[i].transform.localRotation = jointLocal.rotation;
            }
        }
    }
}