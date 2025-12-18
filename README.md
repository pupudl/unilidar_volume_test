## Version

- v0.3.2

## Build and Run Instructions

0. Environments

- python 3.10
- Ubuntu 22.04
- gcc/g++ 12.3
- cmake 3.22

1. Installation of dependencies in a Python virtual environment:

```bash
pip install pybind11[global] numpy==1.24 open3d scipy pyinstaller
```

2. Build the C++ extension module using `pybind11`:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j 2  # Build on 2 cores
```

3. Run the python script to test the extension:

- general

```bash
python c.py
```

- for sudo

```bash
sudo env "PATH=$PATH" python c.py # without rendering
sudo env "PATH=$PATH" "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"  python c.py # with rendering
```

4. Package the project and run the packaged project:

- package the project using `pyinstaller`:

```bash
python package.py
```

- run the packaged project:

```bash
cd dist/client
./client [--cli] [--args]

cd dist/server
./server
```

> using '--cli' argument to run the client in CLI mode, or use the GUI mode by default.

or directly double-click the executable files in the `dist` directory.

## Algorithm Overview

```Mermaid
flowchart TD
    A["Start <br/><sub>c.py#main()</sub>"]
    B["Parse Args <br/><sub>pylib.args.py#client_gui_args()</sub>"]
    C["Main Logic <br/> 1. Init and Start Lidar <br/> 2. Enter Main loop <br/><sub>c.py#run_logic()</sub>"]
	W["Compute Results <br/><sub>pylib.work.py#workflow()</sub>"]
    D["Upload Results <br/><sub>pylib.work.py#send_results_to_reporting_server()</sub>"]
    E["Visualize PCD <br/><sub>pylib.work.py#send_results_to_visualization_server()</sub>"]
    F["Waiting for Next Circle"]

    subgraph Workflow["workflow()"]
        H["Gather Points <br/><sub>pylib.work.py#gather_point_cloud()</sub>"]
        I["Extract Plane <br/><sub>pylib.utils.py#extract_plane_points()</sub>"]
        J["Update Floor Height <br/><sub>pylib.work.py#update_lowest_height()</sub>"]
        K["Extract Stacking Plane <br/><sub>pylib.utils.py#extract_non_floor_plane_points()</sub>"]
        L["Group Plane Points <br/><sub>pylib.utils.py#group_plane_points()</sub>"]
        M["Compute Metrics for Each Group <br/><sub>pylib.utils.py#compute_metrics()</sub>"]

        H --> I
        I --> J
        J --> K
        K --> L
        L --> M
    end
    
    A --> B
    B --> |args|C
    C --> |args, lidarManager| W
    W --> Workflow
    M --> |results|D
    D --> |results|E
    E --> F
    F --> |catch Exception|H
```
