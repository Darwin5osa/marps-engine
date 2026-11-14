namespace Marps.Engine.Routing
{
    /// <summary>Binary priority queue (float key, int value) with no allocations per operation.</summary>
    internal sealed class MinHeap
    {
        private float[] _keys;
        private int[] _values;

        public MinHeap(int capacity)
        {
            _keys = new float[capacity];
            _values = new int[capacity];
        }

        public int Count { get; private set; }

        public void Clear() => Count = 0;

        public void Push(float key, int value)
        {
            if (Count == _keys.Length) Grow();
            int i = Count++;
            while (i > 0)
            {
                int parent = (i - 1) >> 1;
                if (_keys[parent] <= key) break;
                _keys[i] = _keys[parent];
                _values[i] = _values[parent];
                i = parent;
            }
            _keys[i] = key;
            _values[i] = value;
        }

        public int Pop()
        {
            int top = _values[0];
            int last = --Count;
            if (last > 0)
            {
                float key = _keys[last];
                int value = _values[last];
                int i = 0;
                while (true)
                {
                    int child = 2 * i + 1;
                    if (child >= last) break;
                    if (child + 1 < last && _keys[child + 1] < _keys[child]) child++;
                    if (_keys[child] >= key) break;
                    _keys[i] = _keys[child];
                    _values[i] = _values[child];
                    i = child;
                }
                _keys[i] = key;
                _values[i] = value;
            }
            return top;
        }

        private void Grow()
        {
            int size = _keys.Length * 2;
            System.Array.Resize(ref _keys, size);
            System.Array.Resize(ref _values, size);
        }
    }
}
