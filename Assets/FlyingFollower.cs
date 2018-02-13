using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class FlyingFollower : MonoBehaviour {

    public Transform Target;
    private Rigidbody _rb;
    private PathFinding _path;
    private Vector3 _target;
    private IEnumerable<Vector3> _currpath;

    public float MoveForce = 1.0f;
    public float RotationSpeed = 0.1f;

	// Use this for initialization
	void Start () {
        _rb = gameObject.GetComponent<Rigidbody>();
        _path = GameObject.FindObjectOfType<PathFinding>();

        StartCoroutine(RepathLoop());
	}

    void OnDrawGizmos() {
        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, _target);
        
        if (_currpath != null) {
            Gizmos.color = Color.blue;
            var prevPos = transform.position;
            foreach(var node in _currpath) {
                Gizmos.DrawLine(prevPos, node);
                prevPos = node;
            }
        }

    }
	
	// Update is called once per frame
	void Update () {
        // Forward Motion
        _rb.AddForce(MoveForce * transform.forward);

        // Get current target

        var start = transform.position;
        var end = Target.transform.position; 
        // var newPath = _path.GetPath(start, end).ToList();
        // UpdatePathing(newPath);

	}

    IEnumerator RepathLoop() {
        while (gameObject != null) {
            yield return new WaitForSeconds(1.0f);
            var start = transform.position;
            var end = Target.transform.position; 
            _path.GetPathAmmortized(UpdatePathing, start, end);
        }
    }

    void UpdatePathing(IEnumerable<Vector3> newPath) {
        _currpath = newPath;

        var pos = transform.position;
        foreach (var node in _currpath.Reverse()) {
            if (_path.IsPathable(pos, node)) {
                _target = node;
                break;
            }
        }

        // Rotation Update
        Vector3 toTarget = _target - transform.position;
        Quaternion desiredFacing = Quaternion.LookRotation(toTarget);
        transform.rotation = Quaternion.Slerp(transform.rotation, desiredFacing, RotationSpeed * Time.deltaTime);
    }
}
