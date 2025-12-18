#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <iostream>

namespace py = pybind11;

class DisjointSet {
public:

    DisjointSet(int n) : n(n), components(n) {
        parent = new int[n];
        rank = new int[n];
        size = new int[n];

        for (int i = 0; i < n; i++) {
            parent[i] = i;
            rank[i] = 0;
            size[i] = 1;
        }
    }

    DisjointSet(py::array_t<float> array, float unit_dist_threshold) {
        // input a 2D array, <N, M> where N is the number of points and M is the number of dimensions
        auto buf = array.mutable_unchecked<2>();
        uint32_t rows = buf.shape(0);
        uint32_t cols = buf.shape(1);
        
        n = rows;
        components = rows;

        parent = new int[n];
        rank = new int[n];
        size = new int[n];

        for (uint32_t i = 0; i < n; i++) {
            parent[i] = i;
            rank[i] = 0;
            size[i] = 1;
        }

        for (uint32_t i = 0; i < n; i++) {
            for (uint32_t j = i + 1; j < n; j++) {
                float dist = 0.0f;

                for (uint32_t k = 0; k < cols; k++) {
                    float diff = buf(i, k) - buf(j, k);
                    dist += diff * diff;

                    // No need to continue if distance exceeds threshold
                    if (dist > unit_dist_threshold * unit_dist_threshold) {
                        break;
                    }
                }

                // Only unite if the squared distance is within the threshold
                if (dist <= unit_dist_threshold * unit_dist_threshold) {
                    unite(i, j);
                }
            }
        }
    }

    DisjointSet(py::array_t<float> array, uint32_t rows, uint32_t cols, float unit_dist_threshold) {
        py::buffer_info buf = array.request();
        float* ptr = (float*)buf.ptr;

        if (buf.ndim != 2 || buf.shape[0] != rows || buf.shape[1] != cols) {
            throw std::runtime_error("Input array must be 2-dimensional and match the specified shape.");
        }

        n = rows;
        components = rows;

        parent = new int[n];
        rank = new int[n];
        size = new int[n];

        for (uint32_t i = 0; i < n; i++) {
            parent[i] = i;
            rank[i] = 0;
            size[i] = 1;
        }

        for (uint32_t i = 0; i < n; i++) {
            for (uint32_t j = i + 1; j < n; j++) {
                float dist = 0.0f;

                for (uint32_t k = 0; k < cols; k++) {
                    float diff = ptr[i * cols + k] - ptr[j * cols + k];
                    dist += diff * diff;

                    // No need to continue if distance exceeds threshold
                    if (dist > unit_dist_threshold * unit_dist_threshold) {
                        break;
                    }
                }

                // Only unite if the squared distance is within the threshold
                if (dist <= unit_dist_threshold * unit_dist_threshold) {
                    unite(i, j);
                }
            }
        }
    }

    ~DisjointSet() {
        delete[] parent;
        delete[] rank;
        delete[] size;
    }

    void add_edges(uint32_t a, py::array_t<uint32_t> b_array, uint32_t b_array_size) {
        py::buffer_info buf = b_array.request();
        if (buf.ndim != 1 || buf.shape[0] != b_array_size) {
            throw std::runtime_error("Input array must be 1-dimensional and match the specified size.");
        }
        uint32_t* ptr = (uint32_t*)buf.ptr;

        for (uint32_t i = 0; i < b_array_size; i++) {
            uint32_t b = ptr[i];
            unite(a, b);
        }
    }

    // void add_edges(uint32_t a, std::vector<uint32_t> b_array) {
    //     for (uint32_t i = 0; i < b_array.size(); i++) {
    //         uint32_t b = b_array[i];
    //         unite(a, b);
    //     }
    // }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]); // Path compression
        }
        return parent[x];
    }

    bool unite(int x, int y) {
        int rx = find(x);
        int ry = find(y);

        if (rx == ry) return false;

        if (rank[rx] < rank[ry]) {
            parent[rx] = ry;
            size[ry] += size[rx];
        } else if (rank[rx] > rank[ry]) {
            parent[ry] = rx;
            size[rx] += size[ry];
        } else {
            parent[ry] = rx;
            size[rx] += size[ry];
            rank[rx]++;
        }

        components--;
        return true;
    }

    int get_component_rank(int x) {
        return rank[find(x)];
    }

    int get_component_size(int x) {
        return size[find(x)];
    }

    int get_component_number() {
        return static_cast<int>(components);
    }

    py::array_t<int> get_ancestors() {
        py::array_t<int> result(n);
        auto res_buf = result.mutable_unchecked<1>();

        for (uint32_t i = 0; i < n; i++) {
            res_buf(i) = find(i);
        }

        return result;
    }

    std::vector<int> get_unique_ancestors() {
        std::set<int> unique_ancestors;
        for (uint32_t i = 0; i < n; i++) {
            unique_ancestors.insert(find(i));
        }
        return std::vector<int>(unique_ancestors.begin(), unique_ancestors.end());
    }

private:
    int* parent;
    int* rank;
    int* size;
    uint32_t n;
    uint32_t components;
};

PYBIND11_MODULE(djset, m) {
    py::class_<DisjointSet>(m, "DisjointSet")
        .def(py::init<int>())
        .def(py::init<py::array_t<float>, float>())
        .def(py::init<py::array_t<float>, uint32_t, uint32_t, float>())
        .def("add_edges", &DisjointSet::add_edges)
        .def("find", &DisjointSet::find)
        .def("unite", &DisjointSet::unite)
        .def("get_component_rank", &DisjointSet::get_component_rank)
        .def("get_component_size", &DisjointSet::get_component_size)
        .def("get_component_number", &DisjointSet::get_component_number)
        .def("get_ancestors", &DisjointSet::get_ancestors)
        .def("get_unique_ancestors", &DisjointSet::get_unique_ancestors);
}
