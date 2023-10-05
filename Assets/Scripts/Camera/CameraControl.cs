using UnityEngine;
using System.Collections;

namespace RVP
{
    [RequireComponent(typeof(Camera))]
    [RequireComponent(typeof(AudioListener))]
    [DisallowMultipleComponent]
    [AddComponentMenu("RVP/Camera/Camera Control", 0)]

    // Class for controlling the camera
    public class CameraControl : MonoBehaviour
    {
        Transform tr;
        Camera cam;
        VehicleParent vp;
        public Transform target; // Target Vehicle
        Rigidbody targetBody;

        public float height;
        public float distance;

        float xInput;
        float yInput;

        Vector3 lookDir;
        float smoothYRot;
        Transform lookObj;
        Vector3 forwardLook;
        Vector3 upLook;
        Vector3 targetForward;
        Vector3 targetUp;
        [Tooltip("Should the camera stay flat? (Local y-axis always points up)")]
        public bool stayFlat;

        [Tooltip("Mask for which objects will be checked in between the camera and target vehicle")]
        public LayerMask castMask;

        void Start() {
            tr = transform;
            cam = GetComponent<Camera>();
            Initialize();
        }

        public void Initialize() {
            // lookObj adalah objek yang digunakan untuk membantu memposisikan dan memutar kamera.
            if (!lookObj) {
                GameObject lookTemp = new GameObject("Camera Looker");
                lookObj = lookTemp.transform;
            }

            // Untuk menetapkan variabel berdasarkan properti kendaraan target
            if (target) {
                vp = target.GetComponent<VehicleParent>();
                distance += vp.cameraDistanceChange;
                height += vp.cameraHeightChange;
                forwardLook = target.forward;
                upLook = target.up;
                targetBody = target.GetComponent<Rigidbody>();
            }

            // Untuk menetapkan mode pembaruan pendengar audio ke fixed, karena kamera bergerak dalam FixedUpdate
            // Hal ini diperlukan agar efek doppler (suara) terdengar benar
            GetComponent<AudioListener>().velocityUpdateMode = AudioVelocityUpdateMode.Fixed;
        }

        void FixedUpdate() {
            if (target && targetBody && target.gameObject.activeSelf) {
                if (vp.groundedWheels > 0) {
                    targetForward = stayFlat ? new Vector3(vp.norm.up.x, 0, vp.norm.up.z) : vp.norm.up;
                }
                // Kasus alternatif agar arah ke depan di udara sesuai dengan kecepatan kendaraan
                /*else {
                    targetForward = targetBody.velocity.normalized;
                }*/

                targetUp = stayFlat ? GlobalControl.worldUpDir : vp.norm.forward;
                lookDir = Vector3.Slerp(lookDir, (xInput == 0 && yInput == 0 ? Vector3.forward : new Vector3(xInput, 0, yInput).normalized), 0.1f * TimeMaster.inverseFixedTimeFactor);
                smoothYRot = Mathf.Lerp(smoothYRot, targetBody.angularVelocity.y, 0.02f * TimeMaster.inverseFixedTimeFactor);

                // Menentukan arah ke atas kamera
                RaycastHit hit;
                if (Physics.Raycast(target.position, -targetUp, out hit, 1, castMask) && !stayFlat) {
                    upLook = Vector3.Lerp(upLook, (Vector3.Dot(hit.normal, targetUp) > 0.5 ? hit.normal : targetUp), 0.05f * TimeMaster.inverseFixedTimeFactor);
                }
                else {
                    upLook = Vector3.Lerp(upLook, targetUp, 0.05f * TimeMaster.inverseFixedTimeFactor);
                }

                // Menghitung variabel rotasi dan posisi
                forwardLook = Vector3.Lerp(forwardLook, targetForward, 0.05f * TimeMaster.inverseFixedTimeFactor);
                lookObj.rotation = Quaternion.LookRotation(forwardLook, upLook);
                lookObj.position = target.position;
                Vector3 lookDirActual = (lookDir - new Vector3(Mathf.Sin(smoothYRot), 0, Mathf.Cos(smoothYRot)) * Mathf.Abs(smoothYRot) * 0.2f).normalized;
                Vector3 forwardDir = lookObj.TransformDirection(lookDirActual);
                Vector3 localOffset = lookObj.TransformPoint(-lookDirActual * distance - lookDirActual * Mathf.Min(targetBody.velocity.magnitude * 0.05f, 2) + Vector3.up * height);

                // Periksa apakah ada objek di antara kamera dan kendaraan target dan pindahkan kamera ke depannya
                if (Physics.Linecast(target.position, localOffset, out hit, castMask)) {
                    tr.position = hit.point + (target.position - localOffset).normalized * (cam.nearClipPlane + 0.1f);
                }
                else {
                    tr.position = localOffset;
                }

                tr.rotation = Quaternion.LookRotation(forwardDir, lookObj.up);
            }
        }

        // Fungsi untuk mengatur input rotasi kamera
        public void SetInput(float x, float y) {
            xInput = x;
            yInput = y;
        }

        // Untuk menghancurkan lookObj
        void OnDestroy() {
            if (lookObj) {
                Destroy(lookObj.gameObject);
            }
        }
    }
}