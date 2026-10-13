using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Feature
{
    public float[] poseFeature;
    public float[] controlFeature;

    public Feature(float[] poseFeature, float[] controlFeature)
    {
        this.poseFeature = poseFeature;
        this.controlFeature = controlFeature;
    }

    public Feature(Feature other)
    {
        poseFeature = other.poseFeature.Clone() as float[];
        controlFeature = other.controlFeature.Clone() as float[];
    }

    public float[] GetTreeNode()
    {
        // Get query vector using current, previous pose
        int poseFeatureDim = poseFeature.Length;
        int controlFeatureDim = controlFeature.Length;
        int featureDim = poseFeatureDim + controlFeatureDim;

        float[] node = new float[featureDim];
        poseFeature.CopyTo(node, 0);
        controlFeature.CopyTo(node, poseFeatureDim);

        return node;
    }

    public static Matrix4x4 ControlToMatrix(float[] control)
    {
        Matrix4x4 mat = Matrix4x4.identity;
        Vector4 pos = new Vector4(control[0], 0, control[1], 1);
        Vector4 forward = new Vector4(control[2], 0, control[3], 0);
        Vector4 up = Vector3.up;
        Vector4 xAxis = Vector3.Cross(up, forward);
        mat.SetColumn(0, xAxis);
        mat.SetColumn(1, up);
        mat.SetColumn(2, forward);
        mat.SetColumn(3, pos);
        return mat;
    }
}

public class FeatureOptions
{
    public int featureDim;
    public int poseDim;     // foot position, foot velocity, hip velocity
    public int controlDim;  // root xz position, xz of forward vector
    public int posJointNum;
    public int velJointNum;

    public string[] posJointNames = { "mixamorig:LeftFoot", "mixamorig:RightFoot" };
    public string[] velJointNames = { "mixamorig:LeftFoot", "mixamorig:RightFoot", "mixamorig:Hips" };
    public int[] trajIdx = new int[] { 10, 20, 30 };
    public int lastTidx;

    public float poseWeight = 1.0f;
    //public float controlWeight = 1.0f; // TASK1, 2, 3) for debug purpose. Recommended to use 0.6 for 4_MotionMatching
    public float controlWeight = 0.6f;

    // Default options
    public FeatureOptions()
    {
        posJointNum = posJointNames.Length;
        velJointNum = velJointNames.Length;
        poseDim = posJointNum * 3 + velJointNum * 3;

        int numTraj = trajIdx.Length;
        lastTidx = trajIdx[numTraj - 1];
        controlDim = 4 * numTraj; 

        featureDim = poseDim + controlDim;
    }
}

public class FeatureVis
{
    Dictionary<string, int> jointName2Idx;
    FeatureOptions op;

    public FeatureVis(FeatureOptions op, Dictionary<string, int> jointName2Idx)
    {
        this.op = op;
        this.jointName2Idx = jointName2Idx;
    }

    public Vector3[] GetJointPosFromFeature(float[] poseFeature, Matrix4x4 root)
    {
        Vector3[] jointPos = new Vector3[op.posJointNum];
        for (int i = 0; i < op.posJointNum; i++)
        {
            Vector4 pos = new Vector4(poseFeature[3 * i], poseFeature[3 * i + 1], poseFeature[3 * i + 2], 1);
            jointPos[i] = root * (pos / op.poseWeight);
        }
        return jointPos;
    }

    public Vector3[] GetJointVelStart(MatchingState state)
    {
        Vector3[] jointVelStart = new Vector3[op.velJointNum];
        for (int i = 0; i < op.velJointNum; i++)
        {
            string name = op.velJointNames[i];
            int idx = jointName2Idx[name];
            Vector3 start;
            if (state.prevPose != null)
                start = state.prevPose.globals[idx].GetColumn(3);
            else
                start = Vector3.zero;
            jointVelStart[i] = start;
        }
        return jointVelStart;
    }

    public Vector3[] GetJointVelEnd(float[] poseFeature, MatchingState state)
    {
        Vector3[] jointVelEnd = new Vector3[op.velJointNum];
        Vector3[] jointVelStart = GetJointVelStart(state);
        int velStartIdx = op.posJointNum * 3;
        for (int i = 0; i < op.velJointNum; i++)
        {
            Vector3 vel = new Vector3(poseFeature[velStartIdx + 3 * i], poseFeature[velStartIdx + 3 * i + 1], poseFeature[velStartIdx + 3 * i + 2]);
            if (state.prevPose != null)
                vel = state.prevPose.root * vel;
            else
                vel = Vector3.zero;
            jointVelEnd[i] = jointVelStart[i] + vel / op.poseWeight;
        }
        return jointVelEnd;
    }

    public Matrix4x4[] GetTrajAxesFromFeature(float[] controlFeature)
    {
        Matrix4x4[] trajAxes = new Matrix4x4[op.trajIdx.Length];
        for (int i = 0; i < op.trajIdx.Length; i++)
        {
            float[] control = new float[4];
            for (int j = 0; j < 4; j++)
                control[j] = controlFeature[4 * i + j] / op.controlWeight;
            Matrix4x4 controlMat = Feature.ControlToMatrix(control);
            trajAxes[i] = controlMat;
        }
        return trajAxes;
    }
}