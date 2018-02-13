using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathNode : MonoBehaviour {

    public float SearchRadius = 10.0f;


	// Use this for initialization
	void Start () {
		
	}

    void OnDrawGizmos() {
        if (PathFinding.Instance.DrawPathNodes) {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(transform.position, PathFinding.Instance.PathWidth);
        }
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
