using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class MotionPlayer : MonoBehaviour
{
    [Header("Data Setting")]
    public Character character;
    public Motion[] motionList;
    public GameObject root;

    int motionIdx  = 0;
    int frame = 0;
    int checkDataChange = 0;

    // Start is called before the first frame update
    void Start()
    {
        Application.targetFrameRate = 30;
        checkDataChange = motionIdx;
    }

    // Update is called once per frame
    void Update()
    {
        // Check if the current data is valid
        motionIdx = Mathf.Clamp(motionIdx,  0, motionList.Length - 1);
        if (motionIdx != checkDataChange) {
            frame = 0;
            checkDataChange = motionIdx;
        }

        // Set pose of the character
        Pose pose = motionList[motionIdx].poses[frame];
        character.SetPose(pose);
        root.transform.position = pose.root.GetColumn(3);
        root.transform.rotation = pose.root.rotation;

        // Increase frame
        if (frame + 1 >= motionList[motionIdx].FrameNum())
            frame = 0;
        else
            frame++;
    }
    
    // Count starts from 1
    void OnDrawGizmos() 
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 40;
        style.normal.textColor = Color.black;
        Handles.Label(new Vector3(1.0f, 0, 1.0f), "Motion " + (motionIdx + 1).ToString() + "\nframe: " + frame.ToString(), style);
    }
}
