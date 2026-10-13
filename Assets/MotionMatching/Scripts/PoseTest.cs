using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class PoseTest : MonoBehaviour
{
    public bool debugA;
    public bool debugB;

    public GameObject newOrigin;
    public Character resultA;
    public Character resultB;

    public Character originalA;
    public Character originalB;

    public Motion motion;
    public int idxA = 0;
    public int idxB = 0;

    Pose poseA;
    Pose poseB;

    Pose originalPoseA;
    Pose originalPoseB;

    void Start()
    {
        // Disable debugging
        originalA.transform.localScale = new Vector3(0, 0, 0);
        originalB.transform.localScale = new Vector3(0, 0, 0);
    }

    void Update()
    {
        idxA = Mathf.Clamp(idxA, 0, motion.FrameNum() - 1);
        idxB = Mathf.Clamp(idxB, 0, motion.FrameNum() - 1);

        Matrix4x4 newOriginMat = Matrix4x4.TRS(newOrigin.transform.position, newOrigin.transform.rotation, Vector3.one);
        poseA = new Pose(motion.poses[idxA]);
        poseB = new Pose(motion.poses[idxB]);
        originalPoseA = new Pose(motion.poses[idxA]);
        originalPoseB = new Pose(motion.poses[idxB]);
        
        // TASK1-2: Use MoveRootAnchor to manipulate the poses
        // Hint: Matrix4x4.identity is the identity matrix of global frame
        // ------------------- Implement here ------------------- //
        poseA.MoveRootAnchor(Matrix4x4.identity, newOriginMat);
        poseB.MoveRootAnchor(newOriginMat, poseA.root);
        // ------------------------------------------------------ //

        resultA.SetPose(poseA);
        resultB.SetPose(poseB);

        originalA.SetPose(originalPoseA);
        originalB.SetPose(originalPoseB);

        // Debugging utilities
        if (debugA)
        {
            originalA.transform.localScale = new Vector3(1, 1, 1);
            Debug.DrawLine(poseA.root.GetColumn(3), newOriginMat.GetColumn(3), Color.red);
            Debug.DrawLine(originalPoseA.root.GetColumn(3), Vector3.zero, Color.black);
        }
        else
            originalA.transform.localScale = new Vector3(0, 0, 0);

        if (debugB)
        {
            originalB.transform.localScale = new Vector3(1, 1, 1);
            Debug.DrawLine(poseA.root.GetColumn(3), poseB.root.GetColumn(3), Color.green);
            Debug.DrawLine(originalPoseB.root.GetColumn(3), newOriginMat.GetColumn(3), Color.black);
        }
        else
            originalB.transform.localScale = new Vector3(0, 0, 0);
    }
}
