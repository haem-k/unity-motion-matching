using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

[Serializable]
public class Pose: ICloneable
{
    public Matrix4x4 root;
    public Matrix4x4[] locals;
    public Matrix4x4[] globals;
    public int[] parentIndex;
    
    public Pose(Pose other)
    {
        root = other.root;
        locals = other.locals.Clone() as Matrix4x4[];
        globals = (other.globals == null) ? null : other.globals.Clone() as Matrix4x4[];
        parentIndex = other.parentIndex.Clone() as int[];
    }

    public Pose(Matrix4x4 root, ref Matrix4x4[] locals, int[] parentIndex)
    {
        Debug.Assert(locals != null);

        this.root = root;
        this.locals = locals;
        this.parentIndex = parentIndex;
        globals = SetGlobal();
    }

    public object Clone()
    {
        return new Pose(this);
    }

    public Pose DeepCopy()
    {
        return new Pose(this);
    }
    
    public int JointNum()
    {
        return (locals == null) ? 0 : locals.Length;
    }

    public Matrix4x4[] SetGlobal()
    {
        if (locals == null)
            return globals;

        int jointNum = this.JointNum();
        globals = new Matrix4x4[jointNum];

        for(int i=0; i<jointNum; i++)
        {
            int parIdx = parentIndex[i];
            if(parIdx < 0)
            {
                globals[i] = root * locals[i];
            }
            else
            {
                globals[i] = globals[parIdx] * locals[i];
            }
        }

        return globals;
    }

    public Pose MoveRootAnchor(Matrix4x4 from, Matrix4x4 to)
    {
        //Pose movedPose = new Pose(this);

        // TASK1-1: Move relative position & orientation of Pose w.r.t 'from' reference frame
        //          and make 'to' transformation a new reference frame

        // ------------------- Implement here ------------------- //
        Matrix4x4 dtrf = to * from.inverse;

        root = dtrf * root;
        for (int i = 0; i < globals.Length; i++)
        {
            globals[i] = dtrf * globals[i];
        }
        // ------------------------------------------------------ //


        return this;
    }

    public Pose MoveRootToIdentity()
    {
        return MoveRootAnchor(root, Matrix4x4.identity);
    }

    public Pose MoveRootTo(Matrix4x4 target)
    {
        
        return MoveRootAnchor(root, target);
    }

    public Vector3 GetRootRelPosition(int jointIdx)
    {
        if(globals == null)
        {
            SetGlobal();
        }

        // TASK2-1: Obtain root relative position for given joint that corresponds to jointIdx
        // ------------------- Implement here ------------------- //
        Matrix4x4 jointGlobal = globals[jointIdx];
        Vector3 rootRelPosition = (root.inverse * jointGlobal).GetColumn(3);        
        // ------------------------------------------------------ //


        return rootRelPosition;
    }

    public void Add(Disp disp)
    {
        int jointNum = disp.dLocalRs.Length;
        for(int i=0; i<jointNum; i++)
        {
            Vector3 dispT = disp.dLocalTs[i];
            Quaternion dispR = disp.dLocalRs[i];
            Matrix4x4 dispTrf = Matrix4x4.TRS(dispT, dispR, Vector3.one);
            locals[i] = dispTrf * locals[i];
        }

        // Recompute global
        globals = SetGlobal();
    }

}

