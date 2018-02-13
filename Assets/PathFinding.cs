using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.IO;

using UnityEditor;

public class PathFinding : MonoBehaviour {

    private Dictionary<int, float>[] _weights;
    //private Vector3[] _nodes;
    private KDTree _nodes;

    public float PathWidth = 1.0f;
    public float SparseSpacing = 5.0f;
    public int NumFibTraces = 32;
    public float DenseNodeSurfaceOffset = 0.5f;
    public float DenseNodeCullDistance = 2.0f;
    public bool DoCornerNodes;
    public float CornerSearchRadius = 5.0f;
    public float CornerMinAggregate = 1.0f;
    public float CornerOffsetRadius = 3.0f;
    public float CornerCullDisantce = 2.0f;
    public float LinkSearchRadius = 7.0f;
    public int LinkMaxDegree = 5;

    public Bounds AutogenArea;

    public bool DrawPathNodes = false;
    public bool DrawPaths = false;

    private static PathFinding _instance;
    public static PathFinding Instance {
        get {
            if (_instance == null) {
                var pf = FindObjectOfType<PathFinding>();
                if (pf == null) {
                    throw new Exception("No pathfinding object in the scene! Please create one!");
                }
                _instance = pf;
            }

            return _instance;
        }
    }

    #region Graph Building

    /// <summary>
    /// Determine if there is any freely pathable space at all inside a sphere
    /// </summary>
    /// <param name="point">Center of the sphere</param>
    /// <param name="radius">Radius of the sphere</param>
    /// <returns>True if there is any pathable space</returns>
    public bool IsFreeSpaceInSphere(Vector3 point, float radius) {
        // TODO: this algorithm is shit
        float stepSize = PathWidth / 4.0f;
        float innerRad2 = Mathf.Pow(radius - PathWidth, 2);

        for (float x = -radius; x < radius; x += stepSize) {
            var x2 = Mathf.Pow(x,2);
            for (float y = -radius; y < radius; y += stepSize) {
                var y2 = Mathf.Pow(y, 2);
                for (float z = -radius; z < radius; z += stepSize) {
                    var z2 = Mathf.Pow(z,2);
                    if (x2+y2+z2 < innerRad2) {
                        if (!Physics.CheckSphere(new Vector3(x,y,z) + point, PathWidth)) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    public IEnumerable<Vector3> FibSphere(int samples, float rnd = 1.0f) {
        var offset = 2.0f / samples;
        var increment = Mathf.PI * (3.0f * Mathf.Sqrt(0.5f));

        for (int i = 0; i < samples; i++) {
            var y = ((i * offset) - 1) + (offset / 2.0f);
            var r = Mathf.Sqrt(1 - Mathf.Pow(y, 2));
            var phi = ((i + rnd) % samples) * increment;

            var x = Mathf.Cos(phi) * r;
            var z = Mathf.Sin(phi) * r;

            yield return new Vector3(x,y,z);
        }

    }


    public IEnumerable<Vector3> GenerateGrid() {
        float sparseSpacing = SparseSpacing;
        var sparseGrid = new List<Vector3>();

        var a = AutogenArea;
        for (float x = a.min.x; x < a.max.x; x += sparseSpacing) {
            for (float y = a.min.y; y < a.max.y; y += sparseSpacing) {
                for (float z = a.min.z; z < a.max.z; z += sparseSpacing) {
                    var v = new Vector3(x,y,z);
                    if (!Physics.CheckSphere(v, PathWidth)) {
                        sparseGrid.Add(v);
                    }
                }
            }
        }


        var denseGrid = BuildDenseGrid(sparseGrid, sparseSpacing * 2.0f);

        if (DoCornerNodes) {
            AddCornerNodes(denseGrid);
        }

        return (denseGrid.Concat(sparseGrid));
    }
    
    List<Vector3> BuildDenseGrid(IEnumerable<Vector3> sparseGrid, float projectRadius) {
        UInt32 fnv = 0x811c9dc5;
        UInt32 fnv_prime = 16777619;

        var denseGrid = new List<Vector3>();
        var surfaceGrace = DenseNodeSurfaceOffset;
        var cullDIstance = DenseNodeCullDistance;
        var count = sparseGrid.Count();
        for (int i = 0; i < count; i++) {
            var p = sparseGrid.ElementAt(i);
            unchecked {
                fnv ^= (UInt32)i;
                fnv = fnv * fnv_prime;
            }
            var rnd = (float)fnv / UInt32.MaxValue;
            foreach (var f in FibSphere(NumFibTraces, rnd)) {
                Ray r = new Ray(p, f);
                RaycastHit hit;
                Physics.Raycast(r, out hit, projectRadius);
                var point = hit.point + hit.normal * (PathWidth + surfaceGrace);

                if (denseGrid.Where(d => Vector3.Distance(d, point) < cullDIstance).Any()) {
                    continue;
                }
                if (Physics.CheckSphere(point, PathWidth)) {
                    continue;
                }
                denseGrid.Add(point);
            }
        }

        return denseGrid;
    }

    void AddCornerNodes(List<Vector3> denseGrid) {
        var startCount = denseGrid.Count;
        var searchRadius = CornerSearchRadius;
        var minaggregate = CornerMinAggregate;
        var offsetRadius = CornerOffsetRadius;
        var cullDisantce2 = CornerCullDisantce;
        for (int i = 0; i < startCount; i++) {
            var p = denseGrid[i];
            var cantidates = denseGrid
                .Where(c => Vector3.Distance(c, p) < searchRadius) // The other node would path back
                .Where(c => IsPathable(c, p))
                .ToList();

            var aggregateOffset = Vector3.zero;
            foreach (var c in cantidates) {
                aggregateOffset += p - c;
            }

            if (aggregateOffset.magnitude > minaggregate) {
                var newNode = p + offsetRadius * aggregateOffset.normalized;
                if (denseGrid.Where(d => Vector3.Distance(d, newNode) < cullDisantce2).Any()) {
                    continue;
                }
                var newcantidates = denseGrid
                    .Where(c => Vector3.Distance(c, newNode) < searchRadius) // The other node would path back
                    .Where(c => IsPathable(c, newNode))
                    .ToList();

                if (newcantidates.Except(Enumerable.Repeat(p,1)).Except(cantidates).Count() >= 2) {
                    denseGrid.Add(newNode);
                }

            }

        }
    }

    #endregion

    #region Graph Linking
    public void LinkGrid() {
        _weights = new Dictionary<int, float>[_nodes.Length];
        var searchRadius = LinkSearchRadius;
        var maxDegree = LinkMaxDegree;

        //for (int i = 0; i < _nodes.Length; i++ ) {
        foreach (int i in _nodes.BreadthFirstIndexes()) {
            var connected = FindConnectionNodes(i, searchRadius, maxDegree);
            // Build the weight dict with...
            _weights[i] = connected.ToDictionary<int, int, float>(
                n => n,                                         // The node as the key
                n => Vector3.Distance(_nodes[i], _nodes[n])     // The distance/weight as the value
                );
        }
    }

    private void LinkBackwardsGrid() {
        //for (int i = 0; i < _nodes.Length; i++) {
        foreach (int i in _nodes.BreadthFirstIndexes()) {
            foreach(var l in _weights[i].Keys) {
                if (!_weights[l].ContainsKey(i)) {
                    _weights[l][i] = _weights[i][l];
                }
            }

        }
    }

    private IEnumerable<int> FindConnectionNodes(int sourceNode, float searchRadius, int maxDegree) {
        var s = sourceNode;
        return FindNodesInRadius(_nodes[s], searchRadius)
                         .Except(Enumerable.Repeat(sourceNode, 1))
                         .Where(c => Vector3.Distance(_nodes[s], _nodes[c]) < searchRadius) // The other node would path back
                         .Where(c => IsPathable(_nodes[s], _nodes[c]))  // Have a clear path
                         .OrderBy(c => Vector3.Distance(_nodes[c], _nodes[s])) // We are the ebst cantidate
                         .Take(maxDegree);
    }

    #endregion

    #region UTILS
    /// <summary>
    /// Determine if a straight line between two points is a valid path.
    /// </summary>
    /// <param name="start">One of the points</param>
    /// <param name="end">Another of the points</param>
    /// <returns>True if there is a straight path between the points</returns>
    public bool IsPathable(Vector3 start, Vector3 end) {
        var dist = Vector3.Distance(start, end);
        return !Physics.SphereCast(new Ray(start, end-start), PathWidth, dist) &&
               !Physics.SphereCast(new Ray(end, start-end)  , PathWidth, dist);
    }

    IEnumerable<int> FindNodesInRadius(Vector3 point, float radius) {
        //for (int i = 0; i < _nodes.Length; i++) {
        foreach (int i in _nodes.BreadthFirstIndexes()) {
            if (Vector3.Distance(point, _nodes[i]) < radius) {
                yield return i;
            }
        }
    }

    int FindClosestNode(Vector3 point) {
        return _nodes.GetNearestToPoint(point);
    }

#endregion

#region Pathfinding
    struct OpenNode {
        public int id;
        public float fScore;

        public OpenNode(int _id, float _fScore) {
            id = _id;
            fScore = _fScore;
        }
    }

    class OpenNodeComparer : Comparer<OpenNode> {
        public override int Compare(OpenNode x, OpenNode y) {
            return x.fScore.CompareTo(y.fScore);
        }
    }

    IEnumerable<int> ReconstructPath(IDictionary<int, int> cameFrom, int endNode) {
        var current = endNode;
        // We are done! Unroll and return!
        while (cameFrom.ContainsKey(current)) {
            yield return current;
            current = cameFrom[current];
        }
        yield return current;
        yield break;
    }

    IEnumerable<int> GetPathBetweenNodes(int start, int end) {
        var closed = new HashSet<int>(); // Closed set: add and contains only
        IPathfindingPriorityQueue open; // Open Set: add, pop lowest, contains?? and update(rare)
        open = new HeapBasedPathQueue(); 

        var cameFrom = new Dictionary<int, int>(); // Came from dict, not performance critical, only add/replace during main algo, lookup during path unrolling at end
        var gScore = new Dictionary<int, float>(); //Gscore lookup table, only add and lookup: used to know when we have a better score ofr a node ready

        gScore[start] = 0f;
        open.Update(start, DistanceHeuristic(start, end));

        while (open.Count() > 0) {
            var current = open.Pop();

            if (current == end) {
                // We are done! Unroll and return!
                return ReconstructPath(cameFrom, current);
            }

            closed.Add(current); // Add this node to closed set, as we are new processing it

            // Iterate over neighbors
            foreach (var neighbor in _weights[current].Keys) {

                // Skip closed nodes, we have already traversed these
                if (closed.Contains(neighbor)) {
                    continue;
                }

                // Add to gscore if we arent already there
                if (!gScore.ContainsKey(neighbor)) {
                    gScore[neighbor] = Mathf.Infinity;
                }

                var tentative_gScore = gScore[current] + _weights[current][neighbor];
                if (tentative_gScore < gScore[neighbor]) {
                    // We found a new best route to this node!

                    // Update the cameFrom to indicate the best route
                    cameFrom[neighbor] = current;
                    // Update the gscore to indicate distance of best route
                    gScore[neighbor] = tentative_gScore;
                    // Update the open set with the best overall fscore
                    var fScore = tentative_gScore + DistanceHeuristic(neighbor, end);
                    open.Update(neighbor, fScore);
                }
            }
        }

        // No more nodes! Failed to find path!
        throw new Exception("Failed to pathfind!");
    }


    IEnumerator GetPathBetweenNodesAmmortized(int maxSteps, Action<IEnumerable<int>> callback, int start, int end) {
        var closed = new HashSet<int>(); // Closed set: add and contains only
        IPathfindingPriorityQueue open; // Open Set: add, pop lowest, contains?? and update(rare)
        open = new HeapBasedPathQueue(); 

        var cameFrom = new Dictionary<int, int>(); // Came from dict, not performance critical, only add/replace during main algo, lookup during path unrolling at end
        var gScore = new Dictionary<int, float>(); //Gscore lookup table, only add and lookup: used to know when we have a better score ofr a node ready

        gScore[start] = 0f;
        open.Update(start, DistanceHeuristic(start, end));

        int stepsLeft = maxSteps;
        while (open.Count() > 0) {
            var current = open.Pop();

            if (current == end) {
                // We are done! Unroll and return!
                callback(ReconstructPath(cameFrom, current));
                break;
            }

            closed.Add(current); // Add this node to closed set, as we are new processing it

            // Iterate over neighbors
            foreach (var neighbor in _weights[current].Keys) {

                // Skip closed nodes, we have already traversed these
                if (closed.Contains(neighbor)) {
                    continue;
                }

                // Add to gscore if we arent already there
                if (!gScore.ContainsKey(neighbor)) {
                    gScore[neighbor] = Mathf.Infinity;
                }

                var tentative_gScore = gScore[current] + _weights[current][neighbor];
                if (tentative_gScore < gScore[neighbor]) {
                    // We found a new best route to this node!

                    // Update the cameFrom to indicate the best route
                    cameFrom[neighbor] = current;
                    // Update the gscore to indicate distance of best route
                    gScore[neighbor] = tentative_gScore;
                    // Update the open set with the best overall fscore
                    var fScore = tentative_gScore + DistanceHeuristic(neighbor, end);
                    open.Update(neighbor, fScore);
                }

            }

            stepsLeft--;
            if (stepsLeft <= 0) {
                yield return null;
                stepsLeft = maxSteps;
            }
        }
    }

    public IEnumerable<Vector3> GetPath(Vector3 start, Vector3 end) {
        var startNode = FindClosestNode(start);
        var endNode = FindClosestNode(end);

        var path = GetPathBetweenNodes(startNode, endNode);
        return ConvertPath(path, end);
    }

    public void GetPathAmmortized(Action<IEnumerable<Vector3>> callback, Vector3 start, Vector3 end, int maxSteps = 50) {
        var startNode = FindClosestNode(start);
        var endNode = FindClosestNode(end);

        // This inner callback just remaps from int to vector3
        Action<IEnumerable<int>> innerCb = (p) => callback(ConvertPath(p, end));
        var coroutine = GetPathBetweenNodesAmmortized(maxSteps, innerCb, startNode, endNode);
        StartCoroutine(coroutine);
    }

    IEnumerable<Vector3> ConvertPath(IEnumerable<int> path, Vector3 end) {
        return path.Reverse().Select(n => _nodes[n]).Concat(Enumerable.Repeat(end, 1));
    }



    float DistanceHeuristic(int start, int end) {
        return Vector3.Distance(_nodes[start], _nodes[end]);
    }

#endregion

#region Graph Visualization

    void OnDrawGizmos() {
        if (DrawPathNodes && _nodes != null) {
            Gizmos.color = Color.blue;
            for (int i = 0; i < _nodes.Length; i++) {
                Gizmos.DrawSphere(_nodes[i], 0.1f);
            }
        }

        if (DrawPaths && _weights != null) {
            Gizmos.color = new Color(1f, 0f, 0f, 0.15f);
            foreach (var i in _nodes.BreadthFirstIndexes()) {
                foreach (var j in _weights[i].Keys) {
                    Gizmos.DrawLine(_nodes[i], _nodes[j]);
                }
            }
        }
    }

#endregion

    public void SaveToDisk(string filename) {
        var formatter = new BinaryFormatter();
        using (var f = File.Open(filename, FileMode.OpenOrCreate, FileAccess.Write)) {
            // var nodes = _nodes.Select(v => new float[3]{v.x, v.y, v.z}).ToArray();
            // formatter.Serialize(f, nodes);
            _nodes.Serialize(formatter, f);
            formatter.Serialize(f, _weights);
        }
    }

    public void LoadFromDisk(string filename) {
        var formatter = new BinaryFormatter();
        using (var f = File.Open(filename, FileMode.Open, FileAccess.Read)) {
            // nodes = (float[][])formatter.Deserialize(f);
            // _nodes = nodes.Select(v => new Vector3(v[0], v[1], v[2])).ToArray();
            _nodes.Deserialize(formatter, f);
            _weights = (Dictionary<int, float>[])formatter.Deserialize(f);
        }
    }

    public void Build () {
        var sw = new System.Diagnostics.Stopwatch();

        print("Building Pathfinding");
        sw.Start();
        var tempNodes = GenerateGrid();
        print("Grid Generated " + sw.ElapsedMilliseconds.ToString() + "ms");
        var pathNodes = FindObjectsOfType<PathNode>().Select(n => n.transform.position);
        _nodes = new KDTree(tempNodes.Concat(pathNodes));
        print("KD Tree Built " + sw.ElapsedMilliseconds.ToString() + "ms");
        //_nodes = tempNodes.ToArray();
        
        LinkGrid();
        print("Grid Linked " + sw.ElapsedMilliseconds.ToString() + "ms");
        LinkBackwardsGrid();
        print("Grid Back-Linked " + sw.ElapsedMilliseconds.ToString() + "ms");
        sw.Stop();
    }

    void Start() {
        Build();
        foreach (var node in GameObject.FindObjectsOfType<PathNode>()) {
            Destroy(node.gameObject);
        }
	}
	
	// Update is called once per frame
	void Update () {
		
	}
}
