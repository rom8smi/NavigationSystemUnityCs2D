using UnityEngine;

namespace TriangulationNavigation
{
    public class CameraController : MonoBehaviour
    {
        Camera cam;
        NavigationSystemComponent navigationSystemComponent;

        void Start()
        {
            cam = GetComponent<Camera>();
            navigationSystemComponent = FindObjectOfType<NavigationSystemComponent>();
        }

        void Update()
        {
            float cameraSize = cam.orthographicSize;

            if (Input.GetMouseButton(0))
            {
                float dx = -Input.GetAxis("Mouse X") * cameraSize * 0.1f;
                float dz = -Input.GetAxis("Mouse Y") * cameraSize * 0.1f;

                transform.position += new Vector3(dx, 0.0f, dz);
            }

            float msw = Input.GetAxis("Mouse ScrollWheel");
            if (msw != 0.0f)
            {
                cameraSize += cameraSize * msw * 0.1f;
                cam.orthographicSize = cameraSize;
            }

            if (navigationSystemComponent != null)
            {
                navigationSystemComponent.cameraScaleMultiplier = cameraSize / 50.5f;
            }
        }
    }
}
