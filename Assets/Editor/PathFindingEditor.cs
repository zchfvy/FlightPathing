using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(PathFinding))]
public class PathFindingEditor : Editor {

    private string _filename = "";

    public override void OnInspectorGUI() {
        DrawDefaultInspector();

        PathFinding pathFinding = (PathFinding)target;
        if (GUILayout.Button("Build Pathfinding")) {
            pathFinding.Build();
        }
        if (GUILayout.Button("Save Pathfinding")) {
            pathFinding.SaveToDisk(pathFinding.PathDataFile);
        }
        if (GUILayout.Button("Load Pathfinding")) {
            pathFinding.LoadFromDisk(pathFinding.PathDataFile);
        }
        
    }

}
