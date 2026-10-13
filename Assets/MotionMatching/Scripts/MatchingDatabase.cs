using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using KdTree;

public class MatchingDatabase
{
    Character character;

    List<Pose> motionDB;
    List<Feature> featureDB;
    KdTree<float, int> tree;
    FeatureOptions op;
    
    List<int> motionIdx;
    List<int> nidx2pidx;
    List<int> pidx2nidx;

    int blendNum = 10;
    bool blend = true;

    public MatchingDatabase(Character character, Motion[] motions, FeatureOptions op)
    {
        this.character = character;
        this.op = op;

        // Initialize motion database
        motionDB = new List<Pose>();
        motionIdx = new List<int>();
        for (int midx = 0; midx < motions.Length; midx++)
        {
            Motion m = motions[midx];
            for (int pidx = 0; pidx < m.poses.Length; pidx++)
            {
                Pose p = new Pose(m.poses[pidx]);
                motionDB.Add(p);
            }
            int[] indices = Enumerable.Repeat(midx, m.FrameNum()).ToArray();

            motionIdx.AddRange(indices);
        }

        // Initialize mapping index 
        pidx2nidx = new List<int>();
        nidx2pidx = new List<int>();
        int totalPoseNum = motionDB.Count;
        for (int pidx = 0; pidx < totalPoseNum; pidx++)
        {
            bool is_not_end = (pidx + op.lastTidx) < totalPoseNum;
            if (is_not_end && IsSameMotionPidx(pidx, pidx + op.lastTidx)) // valid pidx
            {
                pidx2nidx.Add(nidx2pidx.Count);
                nidx2pidx.Add(pidx);
            }
            else // invalid pidx
            {
                pidx2nidx.Add(-1);
            }
        }

        // Initialize featureDB
        featureDB = new List<Feature>();
        tree = new KdTree<float, int>(op.featureDim, new KdTree.Math.FloatMath());
        int featureNum = nidx2pidx.Count;
        for (int i = 0; i < featureNum; i++)
        {
            int pidx = nidx2pidx[i];
            float[] poseFeature = GetPoseFeature(pidx, op);
            float[] controlFeature = GetControlFeature(pidx, op);
            Feature feature = new Feature(poseFeature, controlFeature);
            featureDB.Add(feature);
            tree.Add(feature.GetTreeNode(), i);
        }

    }

    public MatchingState GetRandomState()
    {
        int nidx = Random.Range(0, featureDB.Count);
        int pidx = nidx2pidx[nidx];
        Pose pose = new Pose(motionDB[pidx]);
        pose.MoveRootToIdentity();

        MatchingState state = new MatchingState(nidx, pidx, pose);
        return state;
    }

    public MatchingState Search(MatchingState state, float[] inputControl)
    {
        // TASK4-1: Search for nearest neighbor feature
        //        & retrieve pose from DB accrodingly
        // Hint: Use tree.GetNearestNeighbors()

        // ------------------- Implement here ------------------- //
        float[] poseFeature = GetPoseFeature(state.pose, state.prevPose, op);
        float[] controlFeature = GetInputControlFeature(inputControl, state);
        Feature query = new Feature(poseFeature, controlFeature);

        var nn = tree.GetNearestNeighbours(query.GetTreeNode(), 1)[0];
        int nNidx = nn.Value;
        int nPidx = nidx2pidx[nNidx];
        Debug.Log("searched nidx: " + nPidx);
        // ------------------------------------------------------ //

        Pose pose = new Pose(motionDB[nPidx]);
        MatchingState searchedState = new MatchingState(nNidx, nPidx, pose);
        return searchedState;

    }

    public MatchingState Next(int nextNidx, MatchingState state)
    {
        int nextPidx = nidx2pidx[nextNidx]; // assume there exists a feature for nextPidx-1
        // TASK4-2: Retrieve next pose
        //          & concat and blend to current motion
        // ------------------- Implement here ------------------- //
        Pose nextPose = new Pose(motionDB[nextPidx]);
        nextPose.MoveRootAnchor(motionDB[nextPidx - 1].root, state.pose.root);
        int blendCount = state.blendCount;
        if (blend)
        {
            if (state.blendCount > 0 && state.blendCount <= blendNum)
            {
                float blendWeight = 1 - (float)(blendCount) / blendNum;
                Disp scaledDisp = new Disp(state.disp);
                scaledDisp.Scale(blendWeight);
                nextPose.Add(scaledDisp);
                blendCount++;
            }
        }
        // ------------------------------------------------------ //

        MatchingState nextState = new MatchingState(nextNidx, nextPidx, nextPose, state.pose, state.disp, blendCount);
        return nextState;
    }

    public Pose LookUpPose(int nidx)
    {
        int pidx = nidx2pidx[nidx];
        Pose pose = new Pose(motionDB[pidx]);
        pose.MoveRootToIdentity(); 
        return pose;
    }

    public Feature LookUpFeature(int pidx)
    {
        int nidx = pidx2nidx[pidx];
        Feature feature = new Feature(featureDB[nidx]);
        return feature;
    }
    
    public int NidxToPidx(int nidx)
    {
        return nidx2pidx[nidx];
    }

    public int PidxToNidx(int pidx)
    {
        return pidx2nidx[pidx];
    }

    public Feature GetFeature(int nidx)
    {
        return featureDB[nidx];
    }

    public Pose GetPose(int pidx)
    {
        return motionDB[pidx];
    }

    public int CountMotionDB()
    {
        return motionDB.Count;
    }

    public int CountFeatureDB()
    {
        return featureDB.Count;
    }

    float[] GetPoseFeature(int pidx, FeatureOptions op)
    {
        Pose pose = new Pose(motionDB[pidx]);
        Pose prevPose;
        if (pidx != 0 && IsSameMotionPidx(pidx, pidx - 1)) // if there is previous pose
        {
            prevPose = new Pose(motionDB[pidx - 1]);
        }
        else // first pose of a motion
        {
            prevPose = null;
        }

        return GetPoseFeature(pose, prevPose, op);
    }

    float[] GetPoseFeature(Pose pose, Pose prevPose, FeatureOptions op)
    {
        // Get root relative joint positions
        Vector3[] jointPos = new Vector3[op.posJointNum];
        for(int i=0; i<op.posJointNum; i++)
        {
            int idx = character.jointName2Idx[op.posJointNames[i]];
            Vector3 pos = pose.GetRootRelPosition(idx);
            jointPos[i] = pos;
        }

        // Get joint velocity
        Vector3[] jointVel = new Vector3[op.velJointNum];
        for(int i=0; i<op.velJointNum; i++)
        {
            int idx = character.jointName2Idx[op.velJointNames[i]];
            Vector3 vel = Vector3.zero;
            if(prevPose != null)
            {
                // TASK2-2: Extract translational velocity pose features
                // ------------------- Implement here ------------------- //
                Matrix4x4 prevRoot = prevPose.root;
                Vector3 prevRelPos = prevPose.GetRootRelPosition(idx);
                Vector3 prevRelCurPos = (prevRoot.inverse * pose.globals[idx]).GetColumn(3);
                vel = (prevRelCurPos - prevRelPos);
                // ------------------------------------------------------ //

            }
            jointVel[i] = vel;
        }

        return ConvertPoseFeatureToArray(jointPos, jointVel);
    }

    float[] ConvertPoseFeatureToArray(Vector3[] jointPos, Vector3[] jointVel)
    {
        float[] poseFeature = Enumerable.Repeat(0.0f, op.poseDim).ToArray();
        
        // Fill in joint position
        for(int i=0; i<op.posJointNum; i++)
        {
            poseFeature[3 * i] = jointPos[i].x * op.poseWeight;
            poseFeature[3 * i + 1] = jointPos[i].y * op.poseWeight;
            poseFeature[3 * i + 2] = jointPos[i].z * op.poseWeight;
        }

        // Fill in joint velocity
        int velStartIdx = op.posJointNum * 3;
        for(int i=0; i<op.velJointNum; i++)
        {
            poseFeature[velStartIdx + 3 * i] = jointVel[i].x * op.poseWeight;
            poseFeature[velStartIdx + 3 * i + 1] = jointVel[i].y * op.poseWeight;
            poseFeature[velStartIdx + 3 * i + 2] = jointVel[i].z * op.poseWeight;
        }

        return poseFeature ;
    }

    float[] GetControlFeature(int pidx, FeatureOptions op)
    {
        int trajNum = op.trajIdx.Length;
        float[] controlFeature = Enumerable.Repeat(0.0f, trajNum * 4).ToArray(); ;
        Matrix4x4 firstRoot = motionDB[pidx].root;
        for (int i = 0; i < trajNum; i++)
        {
            int tidx = op.trajIdx[i];

            // TASK2-3: Obtain control(trajectory) features from motion
            //          Coordinates should be relative to Pose at pidx
            // ------------------- Implement here ------------------- //
            Pose pose = new Pose(motionDB[pidx + tidx]);
            pose.MoveRootAnchor(firstRoot, Matrix4x4.identity);

            controlFeature[4 * i] = pose.root.GetColumn(3).x * op.controlWeight;
            controlFeature[4 * i + 1] = pose.root.GetColumn(3).z * op.controlWeight;
            controlFeature[4 * i + 2] = pose.root.GetColumn(2).x * op.controlWeight;
            controlFeature[4 * i + 3] = pose.root.GetColumn(2).z * op.controlWeight;

            // ------------------------------------------------------ //

        }

        return controlFeature;
    }

    float[] GetInputControlFeature(float[] inputControl, MatchingState state)
    {
        int trajNum = op.trajIdx.Length;
        float[] inputControlFeature = Enumerable.Repeat(0.0f, trajNum * 4).ToArray();

        // Access current control feature and interpolate with input control
        int nidx = pidx2nidx[state.pidx];
        if (nidx == -1)
            return inputControlFeature;
        float[] controlFeature = featureDB[nidx].controlFeature;
        
        
        for(int i=0; i<trajNum; i++)
        {
            
            // interpolate position
            float inputX = inputControl[4 * i];
            float inputZ = inputControl[4 * i + 1];

            float xPos = (controlFeature[4 * i] + inputX) / 2;
            float zPos = (controlFeature[4 * i + 1] + inputZ) / 2;
            //float xPos = inputX;
            //float zPos = inputZ;

            // interpolate rotation
            Vector3 input = new Vector3(inputX, 0.0f, inputZ);
            Matrix4x4 inputMat = ForwardVecToMatrix(input);
            Quaternion inputRot = inputMat.rotation;

            Vector3 featureForward = new Vector3(controlFeature[4 * i + 2], 0, controlFeature[4 * i + 3]);
            Matrix4x4 featureMat = ForwardVecToMatrix(featureForward);
            Quaternion featureRot = featureMat.rotation;

            Quaternion interpRot = Quaternion.Slerp(inputRot, featureRot, 0.5f);
            Vector3 interpVec = interpRot * Vector3.forward;
            //Vector3 interpVec = inputRot * Vector3.forward;

            float xDir = interpVec.x;
            float zDir = interpVec.z;
            //float xDir = 0;
            //float zDir = 0;

            inputControlFeature[4 * i] = xPos * op.controlWeight;
            inputControlFeature[4 * i + 1] = zPos * op.controlWeight;
            inputControlFeature[4 * i + 2] = xDir * op.controlWeight;
            inputControlFeature[4 * i + 3] = zDir * op.controlWeight;
            
        }
        
        return inputControlFeature;
    }

    public bool IsSameMotionPidx(int firstPidx, int secondPidx)
    {
        if (motionIdx[firstPidx] == motionIdx[secondPidx])
        {
            return true;
        }
        return false;
    }

    public bool IsSameMotionNidx(int firstNIdx, int secondNIdx)
    {
        int firstPidx = nidx2pidx[firstNIdx];
        int secondPidx = nidx2pidx[secondNIdx];
        if (motionIdx[firstPidx] == motionIdx[secondPidx])
        {
            return true;
        }
        return false;
    }

    public Matrix4x4 ForwardVecToMatrix(Vector3 forward)
    {
        forward = forward.normalized;
        Vector3 up = Vector3.up;
        Vector3 xAxis = Vector3.Cross(up, forward);
        Matrix4x4 forwardMat = Matrix4x4.identity;
        forwardMat.SetColumn(0, xAxis);
        forwardMat.SetColumn(1, up);
        forwardMat.SetColumn(2, forward);
        return forwardMat;
    }
}
