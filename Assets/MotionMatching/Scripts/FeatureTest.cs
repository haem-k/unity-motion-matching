using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class FeatureTest : MonoBehaviour
{
    public Character character;
    public Motion[] motions;
    public GameObject trajPrefab;

    FeatureOptions op;
    FeatureVis vis;
    MatchingDatabase db;
    MatchingState state;

    List<GameObject> jointSpheres;      // Spheres to visualize joint position
    List<GameObject> trajAxes;          // Axes to visualize control feature
    float sphereScale = 0.08f;

    void Start()
    {
        // Set up
        Application.targetFrameRate = 30;
        op = new FeatureOptions();
        db = new MatchingDatabase(character, motions, op);
        vis = new FeatureVis(op, character.jointName2Idx);

        // Initialize GameObjects
        jointSpheres = new List<GameObject>();
        for (int i = 0; i < op.posJointNum; i++)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.localScale = new Vector3(sphereScale, sphereScale, sphereScale);
            sphere.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
            jointSpheres.Add(sphere);
        }
        trajAxes = new List<GameObject>();
        for (int i = 0; i < op.trajIdx.Length; i++)
        {
            GameObject newTraj = Instantiate(trajPrefab, Vector3.zero, Quaternion.identity);
            trajAxes.Add(newTraj);
        }
    }

    // Go through all the features, visualize them
    int nidx = 0; 
    void Update()
    {
        int pidx = db.NidxToPidx(nidx);

        // Set up state
        Pose renderPose = db.LookUpPose(nidx);
        if (nidx == 0)
            state = new MatchingState(nidx, pidx, renderPose);
        else
        {
            Pose prevPose = null;
            if (pidx != 0 && db.IsSameMotionPidx(pidx - 1, pidx))
            {
                prevPose = new Pose(db.GetPose(pidx - 1));
                prevPose.MoveRootAnchor(db.GetPose(pidx).root, renderPose.root);
            }
            state = new MatchingState(nidx, pidx, renderPose, prevPose);
        } 


        // Visualize features
        Feature feature = db.GetFeature(nidx);
        
        // Visualize trajectory
        Matrix4x4[] trajAxesMat = vis.GetTrajAxesFromFeature(feature.controlFeature);
        for (int i = 0; i < trajAxesMat.Length; i++)
        {
            Matrix4x4 controlMat = trajAxesMat[i];
            trajAxes[i].transform.position = controlMat.GetColumn(3);
            trajAxes[i].transform.rotation = controlMat.rotation;
        }
        character.SetPose(renderPose);

        // Visualize joint position
        Vector3[] jointPos = vis.GetJointPosFromFeature(feature.poseFeature, renderPose.root);
        for(int i=0; i<jointPos.Length; i++)
        {
            jointSpheres[i].transform.position = jointPos[i];
            jointSpheres[i].transform.rotation = Quaternion.identity;
        }

        // Visualize joint velocity
        Vector3[] jointStart = vis.GetJointVelStart(state);
        Vector3[] jointEnd = vis.GetJointVelEnd(feature.poseFeature, state);
        for(int i=0; i<jointStart.Length; i++)
        {
            Debug.DrawLine(jointStart[i], jointEnd[i], Color.red);
        }

        if (nidx + 1 == db.CountFeatureDB())
            nidx = 0;
        else
            nidx++;
    }

    void OnDrawGizmos()
    {
        GUIStyle style = new GUIStyle();
        style.fontSize = 40;
        style.normal.textColor = Color.black;
        Handles.Label(new Vector3(1.0f, 0, 1.0f), "nidx: " + (nidx - 1).ToString(), style);
    }
}