<<<<<<< HEAD
# OrbitalPursuit

**Autonomous Orbital Satellite Servicing Simulator**

A real-time simulation of autonomous spacecraft operations for servicing a constellation of space-based AI compute satellites. The C++ engine runs on a Raspberry Pi Zero 2 W, demonstrating resource-constrained autonomous decision-making, while a Three.js web frontend provides an interactive 3D visualization.

## The Problem

As space-based AI data centers become reality, maintaining constellations of compute satellites presents unprecedented challenges:

- **Thermal management** — Balancing solar heating, GPU waste heat, and radiative cooling in a vacuum
- **Hardware lifecycle** — GPU modules degrade and become obsolete (~4 year lifespan)
- **Fuel depletion** — Station-keeping propellant runs low, satellites drift out of formation
- **Autonomous servicing** — A servicer spacecraft must triage, navigate to, and repair satellites with minimal ground intervention

## Architecture

```
┌──────────────────────────┐      JSON / WebSocket     ┌─────────────────────────┐
│   Raspberry Pi Zero 2 W  │ ◄───────────────────────► │   Browser (Three.js)    │
│                          │                           │                         │
│  C++ Engine:             │   Telemetry stream ───►   │  3D visualization:      │
│  - Orbital mechanics     │                           │  - Earth + constellation │
│  - Thermal simulation    │   Waypoint commands ◄──   │  - Health indicators    │
│  - Trajectory planning   │                           │  - Servicer animation   │
│  - Mission state machine │                           │  - Interactive controls │
│  - Constellation triage  │                           │  - Mission HUD          │
└──────────────────────────┘                           └─────────────────────────┘
```

## Building

### Prerequisites

- CMake 3.16+
- C++17 compiler (GCC 8+ or Clang 7+)

### Build & Run

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
./orbital_pursuit
```

On Windows:
```powershell
mkdir build; cd build
cmake ..
cmake --build . --config Release
.\Release\orbital_pursuit.exe
```

### Frontend

Open `frontend/index.html` in a browser, or serve it:

```bash
cd frontend
python3 -m http.server 8080
```

Then navigate to `http://localhost:8080`.

## Project Structure

```
OrbitalPursuit/
├── engine/
│   ├── include/                  # Header files
│   │   ├── vec3.h                # 3D vector math
│   │   ├── quaternion.h          # Rotation math
│   │   ├── config.h              # Mission parameters & constants
│   │   ├── satellite.h           # Compute satellite model
│   │   ├── constellation.h       # Constellation manager + servicer
│   │   ├── thermal_model.h       # Stefan-Boltzmann thermal simulation
│   │   ├── orbital_mechanics.h   # Clohessy-Wiltshire relative motion
│   │   ├── mission_manager.h     # Mission state machine
│   │   └── telemetry.h           # JSON telemetry output
│   └── src/                      # Implementation files
│       ├── main.cpp              # Entry point & simulation loop
│       ├── satellite.cpp
│       ├── constellation.cpp
│       ├── thermal_model.cpp
│       ├── orbital_mechanics.cpp
│       ├── mission_manager.cpp
│       └── telemetry.cpp
├── frontend/
│   └── index.html                # Three.js 3D visualization (self-contained)
├── CMakeLists.txt
└── README.md
```

## Mission Phases

1. **MONITORING** — Constellation health monitoring, watching for anomalies
2. **TRIAGE** — Evaluating service requests, selecting highest priority target
3. **TRANSIT** — Orbital transfer to target satellite
4. **RENDEZVOUS** — Proximity operations and approach
5. **DOCKING** — Final approach and docking
6. **SERVICING** — Performing repairs (thermal panel swap, GPU replacement, refuel)
7. **COMPLETE** — Service complete, returning to parking orbit

## Key Algorithms

- **Clohessy-Wiltshire Equations** — Relative orbital motion between spacecraft
- **Quaternion Attitude Control** — Gimbal-lock-free rotation math
- **PID Controllers** — Attitude and trajectory tracking
- **Stefan-Boltzmann Thermal Model** — Radiative heat dissipation in vacuum
- **Priority-based Triage** — Multi-factor urgency scoring for service scheduling
- **Tsiolkovsky Rocket Equation** — Fuel consumption modeling

## License

MIT
=======
# orbital-sat
>>>>>>> 57b5d0de6dcd3640f0c354379dbf0484581b55cd
