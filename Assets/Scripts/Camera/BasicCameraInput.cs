using UnityEngine;
using System.Collections;

namespace RVP
{
    [DisallowMultipleComponent]
    [AddComponentMenu("RVP/Camera/Basic Camera Input", 1)]

    // Class for mengatur input kamera dengan manajer input
    public class BasicCameraInput : MonoBehaviour
    {
        CameraControl cam;
        public string xInputAxis;
        public string yInputAxis;

        void Start() {
            // Untuk mendapatkan pengontrol kamera
            cam = GetComponent<CameraControl>();
        }

        void FixedUpdate() {
            // Untuk mengatur input rotasi kamera jika axis input valid
            if (cam && !string.IsNullOrEmpty(xInputAxis) && !string.IsNullOrEmpty(yInputAxis)) {
                cam.SetInput(Input.GetAxis(xInputAxis), Input.GetAxis(yInputAxis));
            }
        }
    }
}