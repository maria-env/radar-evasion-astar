# Radar-Evading Route Planning with A* Search

A spy plane must fly over a set of points of interest (POIs) while staying as hidden as possible from a field of randomly generated radars. This project models the airspace as a grid, builds a radar **detection map**, and plans the safest route between the POIs using **A\* heuristic search**.

> Artificial Intelligence course project (Universidad Carlos III de Madrid, 2024–25).

## How it works

1. **Scenario loading** – `scenarios.json` defines the map boundaries (lat/lon), grid size (`W` × `H`), number of radars and the ordered list of POIs.
2. **Radar generation** – `Map.generate_radars` places radars at random grid positions with random physical parameters (transmit power, antenna gain, wavelength, cross-section, sensitivity, losses). A fixed seed (`42`) makes every run reproducible.
3. **Detection map** – each radar has a maximum range from the radar equation, and its detection level inside that range follows a 2D Gaussian. Each cell keeps the highest level over all radars, normalised to `[EPSILON, 1]`, so every cell has a strictly positive cost.
4. **Graph** – the grid becomes a directed graph (4-connected). An edge into a cell exists only if that cell's detection level is `<= tolerance`, and its weight is that detection level.
5. **Search** – POIs are visited in the given order. One A\* search runs between each consecutive pair (`networkx.astar_path`), using one of two heuristics:
   - `h1`: Euclidean distance × `EPSILON`
   - `h2`: Manhattan distance × `EPSILON` (the one used by `main.py`)

   Scaling by `EPSILON` (the minimum possible edge cost) keeps both heuristics admissible.
6. **Output** – total path cost, number of expanded nodes, and three plots: radar locations, detection fields, and the final route.

## Project structure

| File | Description |
|------|-------------|
| `main.py` | Entry point: parses arguments, runs the pipeline, plots results |
| `Map.py` | Grid map, random radar generation, detection map computation |
| `Radar.py` | Radar model (max range, Gaussian detection level) |
| `Location.py` / `Boundaries.py` | Small data classes for coordinates and map limits |
| `SearchEngine.py` | Heuristics, graph construction, coordinate discretisation, A\* path finding, cost computation |
| `scenarios.json` | Ten scenarios (`scenario_0` … `scenario_9`) from 16×16 up to 1024×1024 grids |

## Requirements

- Python 3.9+
- `numpy`, `networkx`, `matplotlib`

```bash
pip install -r requirements.txt
```

## Usage

Run from the directory containing `scenarios.json` (the file is looked up in the current working directory):

```bash
python main.py <scenario_name> <tolerance>
```

- `scenario_name`: a key from `scenarios.json`, e.g. `scenario_2`
- `tolerance`: maximum detection level (in `(0, 1]`) the plane may enter; lower values force safer, longer routes, and if too low there may be no valid path

Example:

```bash
python main.py scenario_2 1.0
```

Sample output:

```
Total path cost: 4.5050448218715236
Number of expanded nodes: 1213
```

Three plot windows are shown in sequence (close each one to continue).

### Sample results (`tolerance = 1.0`, Manhattan heuristic)

| Scenario | Grid | Radars | POIs | Path cost | Expanded nodes |
|----------|------|--------|------|-----------|----------------|
| `scenario_0` | 16×16 | 1 | 2 | 0.24 | 240 |
| `scenario_1` | 16×16 | 3 | 2 | 1.03 | 211 |
| `scenario_2` | 32×32 | 7 | 4 | 4.51 | 1213 |
| `scenario_5` | 64×64 | 16 | 6 | 49.06 | 6500 |
| `scenario_7` | 128×128 | 20 | 6 | 38.65 | 26019 |
| `scenario_8` | 256×256 | 32 | 6 | 130.66 | 88729 |

`scenario_9` (1024×1024) is much heavier: the detection map is computed with pure-Python loops, so expect it to take a long time.

## Notes and limitations

- POIs are visited in the order given; the project does not optimise the visiting order.
- Distances use a flat approximation (1° ≈ 111 km), which is fine at this scale.
- Detection-map computation is O(cells × radars) in plain Python and is the main bottleneck on large grids.

## Authors

Team project by María Arias Rodríguez, Jorge Castañeda Vallenilla and Rodrigo Melero Moreno.
