using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.Serialization.Formatters.Binary;
using UnityEngine;

class KDTree {
    private Vector3[] _points;
    private float[] _planes;
    private IEnumerable<Vector3> tempNodes;

    enum Side {
        None,
        Left,
        Right
    }

    #region public acessors

    public Vector3 this[int index] {
        get {
            return _points[index];
        }
    }

    public int Length { get { return _points.Length; } }

    public int GetNearestToPoint(Vector3 point) {
        return GetNearestRecursive(point, 0, 0);
    }

    public IEnumerable<int> BreadthFirstIndexes() {
        for(int i = 0; i < _planes.Length; i++) {
            if (!float.IsNaN(_planes[i])) {
                yield return i;
            }
        }
    }

    private int GetNearestRecursive(Vector3 point, int rootIndex, int axis) {
        int best;
        Side selectedSide = Side.None;
        var axisSelector = AxisSelect(axis);

        // Setp 1: recurse downwards finding the best leaf node
        if (HasLeft(rootIndex) && axisSelector(point) < _planes[rootIndex]) {
            best = GetNearestRecursive(point, GetLeft(rootIndex), (axis+1) % 3);
            selectedSide = Side.Left;
        }
        else if(HasRight(rootIndex)) {
            best = GetNearestRecursive(point, GetRight(rootIndex), (axis+1) % 3);
            selectedSide = Side.Right;
        }
        else {
            // Case when we _are_ the leaf, nothing left to do
            return rootIndex;
        }

        // Step 2: If our node beats our leaf then replace it
        var distToBest2 = Vector3.SqrMagnitude(point - _points[best]);
        var distToMyPoint2 = Vector3.SqrMagnitude(point - _points[rootIndex]);
        if (distToMyPoint2 < distToBest2) {
            best = rootIndex;
            distToBest2 = distToMyPoint2;
        }

        // Step 3: Check if it's possible the other subtree (that we didnt explore in step 1)
        //         might be closer, and if so check it
        if (Mathf.Pow(axisSelector(point) - _planes[rootIndex], 2f) < distToMyPoint2) {
            if (selectedSide == Side.Left) {
                var alt = GetNearestRecursive(point, GetRight(rootIndex), (axis+1) % 3);
                if (Vector3.SqrMagnitude(_points[alt] - point) < distToBest2) {
                    best = alt;
                }
            }
            else if (selectedSide == Side.Right) {
                var alt = GetNearestRecursive(point, GetLeft(rootIndex), (axis+1) % 3);
                if (Vector3.SqrMagnitude(_points[alt] - point) < distToBest2) {
                    best = alt;
                }
            }
        }

        return best;
    }

    public IEnumerable<Vector3> GetWithinRadius(Vector3 point, float radius) {
        throw new NotImplementedException();
    }


    public KDTree(IEnumerable<Vector3> nodes) {
        var elems = nodes.Count();
        var treeSize = 1;
        while (treeSize < elems) {
            treeSize *= 2;
        }
        treeSize *= 2; // TODO : we are doing this becasue tree might not be balanced perfectly

        _points = new Vector3[treeSize];
        _planes = new float[treeSize];
        for (int i = 0; i < treeSize; i++) {
            _planes[i] = float.NaN;
        }
        Treeify(nodes, 0, 0);
    }

    #endregion

    Func<Vector3, float> AxisSelect(int axis) {
        switch(axis) {
            case 0: return (v => v.x);
            case 1: return (v => v.y);
            case 2: return (v => v.z);
            default: throw new Exception("Cannot index by that axis");
        }
    }

    void Treeify(IEnumerable<Vector3> currentSet, int rootIndex, int axis) {
        if (currentSet.Count() == 1) {
            _planes[rootIndex] = 0.0f;
            _points[rootIndex] = currentSet.First();
            return;
        }

        IOrderedEnumerable<Vector3> sorted;
        int amount;
        IEnumerable<Vector3> leftSet, rightSet;

        Func<Vector3, float> axisSelector = AxisSelect(axis);
        sorted = currentSet.OrderBy(axisSelector);
        amount = currentSet.Count() / 2;
        leftSet = sorted.Take(amount);
        rightSet = sorted.Skip(amount);

        _points[rootIndex] = rightSet.First();
        _planes[rootIndex] = axisSelector(_points[rootIndex]);
        Treeify(leftSet.ToList(), 2*rootIndex+1, (axis+1) %3);
        Treeify(rightSet.ToList(), 2*rootIndex+2, (axis+1) %3);
    }

    int GetLeft(int from) {
        return 2*from + 1;
    }

    int GetRight(int from) {
        return 2*from + 2;
    }

    int GetParent(int from) {
        return (from-1)/2;
    }

    bool HasLeft(int from) {
        if (2*from > _points.Length) {
            return false;
        }
        if (float.IsNaN(_planes[GetLeft(from)])) {
            return false;
        }
        return true;
    }

    bool HasRight(int from) {
        if (2*from > _points.Length) {
            return false;
        }
        if (float.IsNaN(_planes[GetRight(from)])) {
            return false;
        }
        return true;
    }

    public void Serialize(BinaryFormatter formatter, FileStream stream) {
        var points = _points.Select(v => new float[3]{v.x, v.y, v.z}).ToArray();
        formatter.Serialize(stream, points);
        formatter.Serialize(stream, _planes);
    }

    public void Deserialize(BinaryFormatter formatter, FileStream stream) {
        var points = (float[][])formatter.Deserialize(stream);
        _points = points.Select(v => new Vector3(v[0], v[1], v[2])).ToArray();
        _planes = (float[])formatter.Deserialize(stream);
    }
}
