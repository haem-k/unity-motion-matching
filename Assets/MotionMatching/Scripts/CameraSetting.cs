using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraSetting : MonoBehaviour
{
    public Character character;
    public Vector3 offset = new Vector3(0, 1.0f, 4.0f);
    
    Vector3 camPos;
    Quaternion camRot = Quaternion.identity;

    Vector3 camLookAt;
    void LateUpdate()
    {
        camLookAt = character.joints[0].transform.position;
        camPos = camLookAt + offset;
        camPos.y = 2.0f;

        // set position
        Vector3 camFocus = camLookAt;
        camFocus.y = 1.0f;

        // set focus
        Vector3 camForward = (camFocus - camPos).normalized;
        Vector3 camX = Vector3.Cross(Vector3.up, camForward);
        Vector3 camY = Vector3.Cross(camForward, camX).normalized;
        Matrix4x4 camMat = Matrix4x4.identity;
        camMat.SetColumn(0, camX);
        camMat.SetColumn(1, camY);
        camMat.SetColumn(2, camForward);
        camRot = camMat.rotation;

        // update camera
        transform.position = camPos;
        transform.rotation = camRot;
    }
}
