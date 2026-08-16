using System;
using System.IO;
using BowyerWatsonTriangulationNamespace;
using UnityEngine;
using GenericCode;

public class PerformanceMeasurerBowyerWatson : MonoBehaviour
{
    string fileName = "BowyerWatson";
    int currentCount = 10;
    int maxCount = 1000;
    double lastTime;
    int iUpdate = 0;

    void Start()
    {

    }

    void Update()
    {
        iUpdate++;

        if (iUpdate > 10)
        {
            iUpdate = 0;
            var points = PointsGenerator.GetRandomPointsInsideCircle(currentCount, 2, 40.0f);

            var sw = new System.Diagnostics.Stopwatch();
            sw.Start();
            var triangulation = new BowyerWatsonTriangulation();
            triangulation.Triangulate(points);

            lastTime = sw.Elapsed.TotalMilliseconds;
            Write();
            currentCount++;

            GenericCode.Debug.Log("Written: " + (currentCount * 100f / maxCount).ToString() + " %");

            if (currentCount >= maxCount)
            {
                UnityEditor.EditorApplication.isPlaying = false;
            }
        }
    }

    void Write()
    {
        string path = Path.Combine(Application.dataPath, fileName + ".txt");

        if (!File.Exists(path))
        {
            string createText = "# n dt" + Environment.NewLine;
            File.WriteAllText(path, createText);
        }

        string appendText = currentCount + " " + lastTime + Environment.NewLine;
        File.AppendAllText(path, appendText);
    }

    void Overwrite()
    {
        string path = Path.Combine(Application.dataPath, fileName + ".txt");
        string createText = "# n dt" + Environment.NewLine;
        File.WriteAllText(path, createText);
    }
}
