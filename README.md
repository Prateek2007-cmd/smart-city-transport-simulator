# Smart City Transport Simulator (C)

A menu-driven, console-based smart city transport simulation implemented in C,
demonstrating graph-based routing and data structure concepts.

## Concepts Used
- Graph representation (adjacency list)
- Priority Queue (Min-Heap)
- Dijkstra’s Algorithm (time-based and cost-based)
- Path reconstruction
- Mode-based constraints and preferences

## Features
- Preloaded smart city map with multiple transport modes
- Add and manage cities and transport routes
- Shortest-time route computation with preferences
- Cheapest-route computation under budget constraints
- Nearest transport hub search (Metro / Bus / Air)
- ASCII-based city visualization

## File
- `smart_city_transport_simulator.c` – complete implementation

## How to Run
```bash
gcc smart_city_transport_simulator.c -o simulator
./simulator
