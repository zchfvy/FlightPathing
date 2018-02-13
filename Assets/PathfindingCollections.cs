using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

interface IPathfindingPriorityQueue {

    /// <summary>
    /// Add or Update a node in the queue
    /// </summary>
    /// <param name="value"></param>
    /// <param name="score"></param>
    void Update(int value, float score);

    /// <summary>
    /// Return the best scoring node
    /// </summary>
    /// <returns></returns>
    int Pop();

    /// <summary>
    /// Get the number of elements in the Queue
    /// </summary>
    /// <returns></returns>
    int Count();
}

class NaieveSortedQueue : IPathfindingPriorityQueue {

    private List<QueueElem> _sortedKeys;

    public NaieveSortedQueue() {
        _sortedKeys = new List<QueueElem>();
    }

    private struct QueueElem {
        public float score;
        public int value;
    }

    public int Count() {
        return _sortedKeys.Count;
    }

    public int Pop() {
        var rv = _sortedKeys.OrderBy(e => e.score).First();
        _sortedKeys.Remove(rv);
        return rv.value;
    }

    public void Update(int value, float score) {
        var elem = new QueueElem();
        elem.value = value;
        elem.score = score;
        _sortedKeys.Add(elem);
    }
}

/// <summary>
/// Heap-like structure. It has some shortcuts to optimize pathfinding that make it unsuitable as a general purpose heap
/// </summary>
class HeapBasedPathQueue : IPathfindingPriorityQueue {
    private struct HeapElem {
        public float score;
        public int value;
    }
    private HeapElem[] _heap;
    private int _count = 0;
    private const int GrowFactor = 2;

    private HashSet<int> _seenVals = new HashSet<int>();


    public HeapBasedPathQueue(int initialCapacity = 100) {
        _heap = new HeapElem[initialCapacity];
    }

    public int GetMin() {
        if (_count == 0) throw new InvalidOperationException("Heap is empty");
        return _heap[0].value;
    }

    public int Pop() {
        if (_count == 0) throw new InvalidOperationException("Heap is empty");
        HeapElem ret = _heap[0];
        _count--;
        Swap(_count, 0); // swap last element into first slot
        HeapifyDown(0);
        return ret.value;
    }

    private void HeapifyUp(int i) {
        if (i != 0 && !(_heap[GetParent(i)].score < _heap[i].score)) {
            // We arent root or already ordered
            Swap(i, GetParent(i));
            HeapifyUp(GetParent(i));
        }
    }

    private void HeapifyDown(int i) {
        var right = GetRight(i);
        var left = right - 1;  // Cheecky optimization
        if (right < _count) {
            // We have both children
            if (_heap[left].score < _heap[right].score) {
                // Left is the better choice
                if(_heap[left].score < _heap[i].score) {
                    Swap(i, left);
                    HeapifyDown(left);
                }
            }
            else {
                // Right is better or tie
                if(_heap[right].score < _heap[i].score) {
                    Swap(i, right);
                    HeapifyDown(right);
                }
            }
        }
        else if (left < _count) {
            // Just one child
            if (_heap[left].score < _heap[i].score) {
                Swap(i, left);
                HeapifyDown(left);
            }
        }
    }

    private static int GetParent(int from) {
        return (from - 1)/2;
    }

    private static int GetLeft(int from) {
        return 2*from + 1;
    }

    private static int GetRight(int from) {
        return 2*from + 2;
    }

    private void Swap(int i, int j) {
        HeapElem tmp = _heap[i];
        _heap[i] = _heap[j];
        _heap[j] = tmp;
    }

    private void Grow()
    {
        int newCapacity = _heap.Length*GrowFactor;
        var newHeap = new HeapElem[newCapacity];
        Array.Copy(_heap, newHeap, _heap.Length);
        _heap = newHeap;
    }

    public void Add(int value, float score) {
        _seenVals.Add(value);
        var elem = new HeapElem();
        elem.value = value;
        elem.score = score;
        if (_count == _heap.Length)
            Grow();

        _heap[_count++] = elem;
        HeapifyUp(_count - 1);
    }

    public void Replace(int value, float score) {
        int index = 0;
        for (int i = 0; i < _count; i++) {
            if (_heap[i].value == value) {
                index = i;
            }
        }
        _heap[index].score = score;
        HeapifyUp(index); // we only update if we have a better score, so we will never heapify down from a replacement
    }

    public void Update(int value, float score) {
        if (!_seenVals.Contains(value)) {
            Add(value, score);
        }
        else {
            Replace(value, score);
        }

    }

    public int Count() {
        return _count;
    }
}
