using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class MotionExporter : MonoBehaviour
{
    public Character character;
    public List<AnimationClip> animClips;

    void Start()
    {
        // Export each clips to Motion 
        Export(animClips, "Assets/MotionMatching/Data/");
    }

    void Export(List<AnimationClip> animClips, string savePath)
    {
        foreach(var clip in animClips)
        {
            Motion motion = ScriptableObject.CreateInstance<Motion>();
            motion.motionName = clip.name;
            //motion.motionName = "sample";   // for sample data
            AssetDatabase.CreateAsset(motion, savePath + motion.motionName + ".asset");

            int frameRate = (int)clip.frameRate;
            int totalFrames = (int)Mathf.Round(clip.length * frameRate);
            int poseNum = totalFrames;
            //int poseNum = 100; // for sample data

            motion.poses = new Pose[poseNum];
            for (int i = 0; i < poseNum; i++)
            {
                float playTime = (float)(i * (1.0f / frameRate));
                clip.SampleAnimation(character.transform.gameObject, playTime);

                Matrix4x4[] locals = new Matrix4x4[character.joints.Length];
                
                // Get root transformation
                Matrix4x4 firstJoint = Matrix4x4.TRS(character.joints[0].transform.position, character.joints[0].transform.rotation, Vector3.one);
                Matrix4x4 root = ProjectToGround(firstJoint);

                // Get parent relative transformation
                for (int j = 0; j < character.joints.Length; j++)
                {
                    if (j == 0)
                    {
                        Matrix4x4 jointGlobal = Matrix4x4.TRS(character.joints[j].transform.position, character.joints[j].transform.rotation, Vector3.one);
                        locals[j] = root.inverse * jointGlobal;
                    }
                    else
                    {
                        locals[j] = Matrix4x4.TRS(character.joints[j].transform.localPosition, character.joints[j].transform.localRotation, Vector3.one);
                    }
                }
                motion.poses[i] = new Pose(root, ref locals, character.parentIndex);
            }

            // Save asset
            EditorUtility.SetDirty(motion);
            AssetDatabase.SaveAssets();
        }
    }

    Matrix4x4 ProjectToGround(Matrix4x4 transformation)
    {
        Matrix4x4 projected = transformation;
        projected.m13 = 0;
        Vector3 z = projected.GetColumn(2);
        z.y = 0;
        z = z.normalized;
        Vector3 y = new Vector3(0, 1, 0);
        Vector3 x = Vector3.Cross(y, z);

        // TODO: check if this is correctly initialized
        projected.SetColumn(0, x);
        projected.SetColumn(1, y);
        projected.SetColumn(2, z);
        return projected;
    }
}
