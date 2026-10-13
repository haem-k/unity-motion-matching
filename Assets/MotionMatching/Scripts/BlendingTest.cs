using UnityEditor;
using UnityEngine;

public class BlendingTest : MonoBehaviour
{
    public Character character;
    public GameObject rootVis;

    public Motion motion;
    Pose[] poses1;
    Pose[] poses2;
    Pose[] blended;

    int frame = 0;
    int segmentLen;
    int frameNum;
    public int p1StartIdx;
    public int p2StartIdx;

    void Start()
    {
        Application.targetFrameRate = 30;
        segmentLen = 60;
        frameNum = segmentLen * 2 - 1;

        p1StartIdx = Mathf.Clamp(p1StartIdx, 0, motion.FrameNum() - segmentLen - 1);
        p2StartIdx = Mathf.Clamp(p2StartIdx, 0, motion.FrameNum() - segmentLen - 1);

        // Setup
        // Obtain motion segments from given sequence
        poses1 = new Pose[segmentLen];
        poses2 = new Pose[segmentLen];
        blended = new Pose[frameNum];

        for (int i=0; i<segmentLen; i++)
        {
            poses1[i] = new Pose(motion.poses[p1StartIdx + i]);
            poses2[i] = new Pose(motion.poses[p2StartIdx + i]);
        }

        // Inertialization blending
        Pose[] alignedPoses2 = new Pose[segmentLen];
        // TASK3-1: Align the latter motion to end of first motion
        // ------------------- Implement here ------------------- //
        Matrix4x4 p1LastRoot = poses1[segmentLen - 1].root;
        Matrix4x4 p2FirstRoot = poses2[0].root;
        for (int i = 0; i < segmentLen; i++)
        {
            alignedPoses2[i] = poses2[i].MoveRootAnchor(p2FirstRoot, p1LastRoot);
        }
        // ------------------------------------------------------ //

        poses1.CopyTo(blended, 0);


        // TASK3-2: Blend the displacement and add to aligned poses
        // Hint: Use Disp class, Disp.Scale function, and Pose.Add function

        // ------------------- Implement here ------------------- //
        Pose p1LastPose = poses1[segmentLen - 1];
        Pose p2FirstPose = alignedPoses2[0];
        Disp disp = new Disp(character.joints.Length, p2FirstPose, p1LastPose);
        int blendNum = 10; // number of frames to blend

        for (int i = segmentLen; i < segmentLen + blendNum; i++)
        {
            int idx = i - segmentLen + 1;
            Pose addedPose = new Pose(alignedPoses2[idx]);
            Disp scaledDisp = new Disp(disp);
            float blendWeight = 1 - (float)idx / blendNum;
            scaledDisp.Scale(blendWeight);
            addedPose.Add(scaledDisp);
            blended[i] = addedPose;
        }
        // ------------------------------------------------------ //


        // Fill in last part of the motion after blended frames
        for (int i=segmentLen + blendNum; i<frameNum; i++)
        {
            int idx = i - segmentLen + 1;
            blended[i] = new Pose(alignedPoses2[idx]);
        }
    }

    void Update()
    {
        character.SetPose(blended[frame]);
        rootVis.transform.position = blended[frame].root.GetColumn(3);
        rootVis.transform.rotation = blended[frame].root.rotation;

        if (frame + 1 == frameNum)
            frame = 0;
        else
            frame++;
    }

    // Frame count starts from 1
    void OnDrawGizmos()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 40;
        style.normal.textColor = Color.black;
        Handles.Label(new Vector3(1.0f, 0, 1.0f), "Frame: " + frame.ToString(), style);
    }
}

