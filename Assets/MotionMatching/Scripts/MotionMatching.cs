using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using System.Linq;


public class MotionMatching : MonoBehaviour
{
    public Character character;
    public GameObject hip;
    public GameObject trajPrefab;

    public Motion[] motions;
    int searchRate = 10;

    MatchingDatabase db;
    FeatureOptions op;
    FeatureVis vis;
    MatchingState state;

    // Visualize feature
    List<GameObject> trajAxes;

    // User input
    public bool showInput;
    float forwardInput;
    float horizontalInput;
    int upMaxCount = 0;
    int downMaxCount = 0;
    int leftMaxCount = 0;
    int rightMaxCount = 0;

    public float[] inputControl;
    float controlScale = 2.8f;
    float speed = 0.2f;

    int frame = 0;
    void Start()
    {
        Application.targetFrameRate = 30;

        // Set up 
        op = new FeatureOptions();
        db = new MatchingDatabase(character, motions, op);
        vis = new FeatureVis(op, character.jointName2Idx);
        inputControl = Enumerable.Repeat(0.0f, op.controlDim).ToArray();
        controlScale /= op.controlWeight;

        // Prepare GameObjects for control feature visualization
        trajAxes = new List<GameObject>();
        for (int i = 0; i < op.trajIdx.Length; i++)
        {
            GameObject newTraj = Instantiate(trajPrefab, Vector3.zero, Quaternion.identity);
            trajAxes.Add(newTraj);
        }

        // Init state
        state = db.GetRandomState();
    }


    void Update()
    {
        // Render current information
        // Apply pose to the character
        character.SetPose(state.pose);

        // Receive input from user
        ReceiveInput();
        if (showInput) // show input for debugging purpose
        {
            Vector4 input = new Vector4(inputControl[op.controlDim - 4], 0, inputControl[op.controlDim - 3], 1);
            input = state.pose.root * input;
            Vector3 chPos = character.joints[0].transform.position;
            chPos.y = 0;
            Debug.DrawLine(input, chPos, Color.blue);
        }

        // Visualize current feature
        Feature feature = db.LookUpFeature(state.pidx);
        VisualizeFeature(feature, state);

        // Visualize hip transformation
        hip.transform.position = state.pose.globals[0].GetColumn(3);
        hip.transform.rotation = state.pose.globals[0].rotation;

        // Update information for next frame
        frame++;
        if (frame % searchRate == 0 && frame != 0)  // need to search
        {
            // Search for nearest neighbor
            inputControl = PreprocessInput(horizontalInput, forwardInput);
            MatchingState searchedState = db.Search(state, inputControl);

            int nextNidx = searchedState.nidx + 1;
            if (nextNidx >= db.CountFeatureDB() || !db.IsSameMotionNidx(searchedState.nidx, nextNidx))
                state = SetToPresetFeature(searchedState.nidx);
            else
            {
                Pose searchedPose = searchedState.pose.MoveRootTo(state.pose.root);
                state.disp = new Disp(character.joints.Length, searchedPose, state.pose);
                state.blendCount = 1;
                state = db.Next(nextNidx, state);
            }
        }
        else // no need to search
        {
            int nextNidx = state.nidx + 1;
            if (nextNidx >= db.CountFeatureDB() || !db.IsSameMotionNidx(state.nidx, nextNidx))
                state = SetToPresetFeature(state.nidx);
            else
                state = db.Next(nextNidx, state);
        }
    }

    void OnDrawGizmos()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 40;
        style.normal.textColor = Color.black;
        Handles.Label(Vector3.zero, "Frame: " + frame.ToString(), style); // function runs after update
    }

    MatchingState SetToPresetFeature(int nidx)
    {
        int subtractIdx = Random.Range(10, 20);
        int newNidx = nidx - subtractIdx;
        Debug.Log("SetToPresetFeature) newNidx: " + newNidx);

        Pose newPose = db.LookUpPose(newNidx);
        newPose = newPose.MoveRootTo(state.pose.root);
        Disp disp = new Disp(character.joints.Length, newPose, state.pose);
        state.blendCount = 1;
        state.disp = disp;

        MatchingState newState = db.Next(newNidx + 1, state);
        return newState;
    }

    void VisualizeFeature(Feature feature, MatchingState state)
    {
        Matrix4x4 root = state.pose.root;

        // Joint position not visualized since the poses are blended
        // Visualize control feature
        Matrix4x4[] trajAxesMat = vis.GetTrajAxesFromFeature(feature.controlFeature);
        for (int i = 0; i < trajAxesMat.Length; i++)
        {
            Matrix4x4 controlMat = trajAxesMat[i];
            controlMat = root * controlMat;
            trajAxes[i].transform.position = controlMat.GetColumn(3);
            trajAxes[i].transform.rotation = controlMat.rotation;
        }

        // Visualize joint velocity
        Vector3[] jointStart = vis.GetJointVelStart(state);
        Vector3[] jointEnd = vis.GetJointVelEnd(feature.poseFeature, state);
        for (int i = 0; i < jointStart.Length; i++)
        {
            Debug.DrawLine(jointStart[i], jointEnd[i], Color.red);
        }
    }

    public void ReceiveInput()
    {
        forwardInput = Input.GetAxis("Vertical");
        horizontalInput = Input.GetAxis("Horizontal");

        if (forwardInput == 1)
            upMaxCount++;
        else
            upMaxCount = 0;

        if (forwardInput == -1)
            downMaxCount++;
        else
            downMaxCount = 0;

        if (horizontalInput == 1)
            rightMaxCount++;
        else
            rightMaxCount = 0;

        if (horizontalInput == -1)
            leftMaxCount++;
        else
            leftMaxCount = 0;

        if (upMaxCount >= 2)
        {
            float scale = speed * upMaxCount;
            scale = Mathf.Clamp(scale, 1, controlScale);
            forwardInput *= scale;
        }
        if (downMaxCount >= 2)
        {
            float scale = speed * downMaxCount;
            scale = Mathf.Clamp(scale, 1, controlScale);
            forwardInput *= scale;
        }
        if (leftMaxCount >= 2)
        {
            float scale = speed * leftMaxCount;
            scale = Mathf.Clamp(scale, 1, controlScale);
            horizontalInput *= scale;
        }
        if (rightMaxCount >= 2)
        {
            float scale = speed * rightMaxCount;
            scale = Mathf.Clamp(scale, 1, controlScale);
            horizontalInput *= scale;
        }

    }

    public float[] PreprocessInput(float horizontal, float forward)
    {
        int trajNum = op.trajIdx.Length;

        Vector3 input = Vector3.zero;
        input.x = horizontal;
        input.z = forward;

        for (int i = 0; i < trajNum; i++)
        {
            //inputControl[4 * i] = input.x / trajNum * i;
            //inputControl[4 * i + 1] = input.z / trajNum * i;
            inputControl[4 * i] = input.x / trajNum * (i + 1);
            inputControl[4 * i + 1] = input.z / trajNum * (i + 1);
            /*
            Debug.LogWarning("control feature index :" + i);
            Debug.Log(inputControl[4 * i]);
            Debug.Log(inputControl[4 * i + 1]);
            Debug.Log(input.x);
            Debug.Log(input.z);
            */

        }
        return inputControl;
    }
}

public class MatchingState
{
    public int nidx;
    public int pidx;
    public Pose pose;
    public Pose prevPose;

    public int blendCount = 0;
    public Disp disp = null;

    public MatchingState(int nidx, int pidx, Pose pose, Pose prevPose=null)
    {
        this.nidx = nidx;
        this.pidx = pidx;
        this.pose = new Pose(pose);
        this.prevPose = prevPose;
    }

    public MatchingState(int nidx, int pidx, Pose pose, Pose prevPose, Disp disp, int blendCount)
    {
        this.nidx = nidx;
        this.pidx = pidx;
        this.pose = new Pose(pose);
        this.prevPose = new Pose(prevPose);
        this.disp = disp;
        this.blendCount = blendCount;
    }
}