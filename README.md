# alns_battery

Route optimizer for a **battery-electric delivery fleet** — the Electric
Vehicle Routing Problem with Time Windows and partial recharging (EVRP-TW).
Given a hub, a set of delivery orders, a set of charging stations, and a
single vehicle spec (battery + load capacity), it decides which vehicle
visits which customers, in what order, and where each vehicle needs to
stop to recharge along the way — minimizing total route time while
respecting delivery time windows, vehicle load capacity, and battery range.

This is the battery/EV counterpart to `../alns` (which handles
diesel/conventional fleets via a different, simpler distance-only energy
model). The two are independent codebases that happen to share a similar
job-based API shape.

## How the optimization actually works

The core algorithm is **Adaptive Large Neighborhood Search (ALNS)**, a
metaheuristic (not an exact solver — it doesn't guarantee the true optimum,
but converges to a good solution quickly even for problems too large to
solve exactly):

1. **Build a greedy initial solution** (`initial_sol.py`): one vehicle per
   customer, each one inserting a charging stop if it can't reach the
   customer and back on a full battery. This is deliberately bad/expensive
   (lots of vehicles) — it's just a valid starting point.
2. **Iteratively improve it** (`ALNS.py` + the main loop in `main.py`):
   each iteration either:
   - **Destroys** part of the solution — removes a batch of customers
     (several removal strategies: random, worst-cost, time-oriented,
     "Shaw"/related-customer removal) or removes a station visit, and
   - **Repairs** it — reinserts the removed customers using a cheapest/
     regret-insertion heuristic, or reinserts a station visit.
   - The new solution is accepted if it's better, or *sometimes* accepted
     even if it's worse (a simulated-annealing-style criterion, using a
     cooling temperature `T`) — this is what lets the search escape local
     optima instead of getting stuck.
   - Which specific removal/insertion operator gets used each time is
     chosen via **adaptive roulette-wheel selection**: operators that have
     recently produced improvements get weighted higher.
3. **Stop condition**: up to `MaxIter` outer iterations (each containing
   `MaxInnerIter` inner destroy/repair attempts — defaults 625 × 800,
   mirrored from the original research code), or early-stop once the best
   cost hasn't changed for 100 consecutive outer iterations, or the cooling
   temperature `T` drops below a floor.

**The objective being minimized is total route TIME, not distance** —
`cost_function`'s returned `f_cost` is a time-cost sum, confirmed by
tracing the code; `veh_cost` (distance) is tracked and reported too, but
isn't what the search is optimizing against. In practice this rarely makes
routes meaningfully different (shorter time usually correlates with
shorter distance at a fixed average speed), but it's worth knowing when
interpreting results.

### Battery/energy model

- Discharge per km = `alpha + beta × current_load_kg` (Wh). Alpha is the
  vehicle's base consumption rate; beta adds extra drain per kg carried,
  so a fully-loaded vehicle uses more energy per km than an empty one.
- A vehicle will only divert to a station if it can't otherwise complete
  its next leg (or the return to depot) on remaining battery — stations
  aren't visited for their own sake.

### Charging curve (stations)

Each station's six charging fields describe a **3-segment piecewise
charging curve**, not a flat rate — real EV chargers taper off as the
battery fills (fast up to ~85%, then progressively slower):

| Field | Meaning |
|---|---|
| `break_point_1` | Energy (Wh) to charge from empty to **85%** of battery capacity |
| `bp1_time` | Hours for that 0→85% stretch |
| `break_point_2` | Energy (Wh) for the **next** segment, 85%→95% (a delta, i.e. 10% of capacity — not cumulative) |
| `bp2_time` | Hours for that 85%→95% segment |
| `full` | Energy (Wh) for the **final** segment, 95%→100% (5% of capacity) |
| `full_time` | Hours for that last segment |

`reading_input.py` (and `api_server.py`, matching it) converts each pair
into an **hours-per-Wh rate** rather than storing raw time:

```python
if_rate[station] = [
    (break_point_1, bp1_time / break_point_1),  # segment 1 rate (fastest)
    (break_point_2, bp2_time / break_point_2),   # segment 2 rate (slower)
    (full,          full_time / full),            # segment 3 rate (slowest)
]
```

When a vehicle actually needs to charge, `charge_calc()` (`ALNS.py:399`)
picks whichever segment(s) its *current* and *target* battery levels span
and sums the corresponding rates × energy, e.g. a vehicle arriving already
above 95% only pays the slow top-segment rate for the remainder; one
arriving empty pays all three in sequence.

**Important caveat**: the 85%/95%/100% split points are **hardcoded** in
`charge_calc`/`initial_sol.py` (literal `0.85*battery_cap`, `0.95*battery_cap`
in the Python) — they are *not* derived from your breakpoint values. The
breakpoint numbers only feed into each segment's rate calculation
(`time ÷ energy`). For the resulting charge-time estimates to be
physically meaningful, a station's breakpoints should be chosen
proportional to *that run's* configured battery capacity:

```
break_point_1 = 0.85 × battery_capacity
break_point_2 = 0.10 × battery_capacity
full          = 0.05 × battery_capacity
```

If they don't match (e.g. a charging-curve template reused across runs
with a different battery capacity each time), nothing crashes — the rates
are just computed against the wrong denominator, so charge-time estimates
quietly become inaccurate. `BatterySteps/VehicleBatteryConfigStep.jsx`
shows a non-blocking warning when a station's breakpoints are off by more
than 5% from these expected proportions, once a battery capacity has been
entered.

### Capacity is a hard constraint

`feasibility_check` rejects any candidate route whose **total** demand
(summed across every customer on it) exceeds `load_capacity` — confirmed
live. If every individual order's quantity already exceeds the configured
load capacity, no two orders can ever be merged onto one vehicle, and
you'll correctly get one vehicle per customer — that's not the optimizer
failing, it's the only feasible answer given those inputs. For a
meaningful demo, order quantities should be comfortably smaller than the
configured load capacity (so multiple orders can share a vehicle).

### Distance model

`reading_input.py` has two distance-matrix builders:

- `distance_power_matrix()` — plain Euclidean distance (`dx² + dy²`). Only
  used by the standalone batch script (`read_input()`), whose academic
  benchmark `.txt` instances have x/y already on a flat plane, not
  geographic coordinates.
- `distance_power_matrix_osrm()` — used by `api_server.py` (what the
  frontend talks to). Calls a self-hosted OSRM `/table` service
  (`https://osrm.goodmove.in/table/v1/driving/...`, the same instance
  `alns/reading_input.py` uses for the diesel flow) for real road
  distances, not straight lines. Coordinates are batched into chunks of 50
  (OSRM's per-request cap) across all three node types here — customers,
  charging stations, and the depot — and every chunk-pair gets one `/table`
  call; distances come back in meters and are converted to km. There is no
  fallback: if OSRM is unreachable, `/find-route-battery/start` returns a
  500 saying so, matching the diesel API's behavior.

Both builders now also return a matching set of **travel-time** matrices
(`dist_mat_cc_time`/`dist_mat_ccs_time`/`dist_mat_cd_time`/`dist_mat_csd_time`,
8 matrices total instead of 4) that `ALNS.py`/`initial_sol.py` use for every
arrival-time and charge-timing calculation — `avg_speed` is no longer
divided into anywhere in the algorithm layer:

- In the OSRM path, each time matrix is OSRM's own `/table` `duration`
  field (seconds → hours) multiplied by `OSRM_TIME_MULTIPLIER = 2`, the
  same free-flow-speed correction `alns/reading_input.py` applies for the
  diesel flow. So two different routes covering the same distance can now
  have different travel times (one on fast arterial roads, one through
  dense local streets) — `avg_speed` no longer forces every km to cost the
  same amount of time.
- In the batch/Euclidean path, each time matrix is still exactly
  `distance / avg_speed` — identical to the original behavior, just
  precomputed once per matrix entry instead of divided inline everywhere a
  leg's time was needed. The standalone script's results are numerically
  unchanged by this.

`avg_speed` itself is still accepted everywhere (removing it from every
function signature that no longer divides by it directly was not worth the
risk for zero behavior change) but no longer drives any OSRM-path timing —
it's only load-bearing for the batch script now. `alpha`/`beta` (Wh/km) are
unaffected either way — calling the lower-level Euclidean function with raw
lat/lng, or calling the OSRM function with the batch script's abstract x/y
units, would both still silently produce nonsense.

## Running it

### As an API server (what the frontend uses)

```bash
cd alns_battery
source ../alns/.venv/bin/activate   # needs fastapi, uvicorn, pandas, numpy, pyyaml
uvicorn api_server:app --port 8001
# or, equivalently (the __main__ block supports this directly):
python3 api_server.py --mode api --port 8001
# for an Ubuntu/background deployment:
nohup python3 api_server.py --mode api > battery.log 2>&1 &
```

Health check: `curl http://localhost:8001/health` → `{"status":"ok"}`.

This is what `frontend/src/pages/BatteryHome.jsx` already talks to
(`FIND_ROUTE_BATTERY_BASE_URL = "http://localhost:8001/find-route-battery"`).

### As a standalone batch script (original research-code entry point)

```bash
cd alns_battery
python3 main.py   # processes every .txt file in this directory (Schneider/
                   # Keskin EVRP-TW benchmark format), writes results.csv
```
Only runs when invoked directly (`if __name__ == "__main__"` guards it) —
importing `main_ALNS` from elsewhere (e.g. `api_server.py`) does not
trigger this.

## API contract (`api_server.py`)

Job-based, mirroring `../alns/main.py`'s pattern — start a job, poll its
status, fetch the result once complete, with pause/resume (and, while
paused, mid-run order/station editing) in between.

Pause/resume is built differently from diesel's `OptimizationJob` though.
Diesel refactored its whole solve loop into methods on a job object so an
external HTTP handler could reach into live solver state while paused.
`main_ALNS` (in `main.py`) keeps its original single-function shape instead
— pause/resume is three `threading.Event`s (`pause_requested`,
`paused_confirmed`, `resume_requested`) passed in via an optional `job`
parameter and checked once per inner iteration, at the top of the loop. The
standalone batch script never passes a `job`, so this is a no-op there.
`job.resume_requested.wait()` genuinely blocks the background thread (zero
CPU while paused) — resuming just continues the same Python call frame
exactly where it left off, since `osolution`/`BestSol`/scores/`T` are all
still live local variables, not reconstructed from anything.

Editing while paused reuses that same locality instead of diesel's
job-owns-everything approach: `customers`/`recharge_stns`/`demand`/
`time_window`/`dist_mat_*`/`if_rate`/`initial_cost` are never *reassigned*
inside `main_ALNS` — only read — so `api_server.py` aliases `job.customers`
etc to the exact same dict objects `main_ALNS` is running with (set once in
`_run_job`), and every edit mutates those dicts **in place**
(`.pop()`/`[k]=v`/`.clear()+.update()`, never rebinding them). `main_ALNS`'s
local variables see the change automatically with no extra sync code. Only
`osolution`/`BestSol` are ever rebound wholesale (every accepted move /
improving outer iteration), so those two are the only state explicitly
synced across the pause checkpoint.

Order edits (`_apply_order_edits`) reuse `update_all_fields`
(`ALNS.py`) — the same primitive the normal customer-removal operators use
every iteration — to recompute a route after a customer is removed, and the
normal regret-2 `insertion_operation` to place new/edited customers,
exactly like diesel's `_apply_order_edits`.

Station edits have no diesel precedent (diesel has no charging stations).
A station add/edit/remove always commits — it's never rejected for
disrupting an in-progress route. Instead, any route that referenced the
changed station gets repaired automatically: `feasibility_check` (the same
composed primitive insertion feasibility already relies on — unlike the
lower-level `battery_update`/`stn_insertion` chain, it gives a clean
Reject/Insert signal rather than possibly-infeasible numbers with no error)
rebuilds that route's battery/timing/station-placement from scratch; if it
still can't be made feasible, that route's customers are pulled out
entirely and fed through the same insertion pool new orders use, landing on
whatever vehicle (existing or fresh) can take them. Only a customer that
truly can't be placed anywhere comes back in `failed_to_place` — the
station edit itself always succeeds.

### `POST /find-route-battery/start`

```jsonc
{
  "orders": [
    {
      "supplier_name": "ChennaiHub", "supplier_lat": 13.0827, "supplier_lng": 80.2707,
      "receiver_name": "TNagarStore", "receiver_lat": 13.0418, "receiver_lng": 80.2341,
      "quantity": 150,
      // Relative-hour-offset floats, NOT absolute datetimes -- same
      // convention frontend/src/pages/HomeSteps/timeUtils.js's
      // convertPickupTimeArray already produces for the diesel flow (an
      // arbitrary +24h baseline constant; the backend re-zeros it onto
      // this batch's own minimum, so the constant itself doesn't matter).
      "pickup_time_from": 24.0, "pickup_time_to": 25.0,
      "dropoff_time_from": 26.0, "dropoff_time_to": 36.0,
      "loading_service_time": 0.25,      // hours, at the hub
      "unloading_service_time": 0.5      // hours, at the customer -- this becomes the ALNS "ServiceTime"
    }
  ],
  "stations": [
    {
      "station_name": "ChennaiCentralCharge", "lat": 13.0732, "lng": 80.2609,
      "charger_type": "fast",              // currently just a label
      "break_point_1": 13600, "bp1_time": 0.31,   // Wh, hours-to-reach-it
      "break_point_2": 1600,  "bp2_time": 0.08,
      "full": 800, "full_time": 0.12
    }
  ],
  "vehicle_config": {
    "battery_capacity": 16000,   // Wh
    "load_capacity": 600,        // kg
    "alpha": 103.80847125,       // Wh/km, base consumption
    "beta": 0.122625,            // Wh/(km*kg), extra consumption per kg carried
    "avg_speed": 40               // km/h
  },
  "depot_config": { "latitude": 13.0827, "longitude": 80.2707 }   // optional; defaults to orders[0]'s supplier location
}
```

Every order's `supplier_*` is treated as the same hub (goods load there
once) — this mirrors the diesel flow's model exactly. Only one vehicle
spec for the whole run; no per-truck-type list like the diesel flow has.

### `POST /find-route-battery/{job_id}/pause`

Requests a pause and blocks (up to 30s) until the background thread actually
reaches the checkpoint, so a successful response means the job is genuinely
halted, not just flagged:

```jsonc
{
  "status": "success", "job_status": "paused",
  "iteration": 14, "best_cost": 2.32, "best_cost_history": [2.8, 2.32]
}
```
A 504 means the checkpoint wasn't reached within 30s (pathologically slow
single inner iteration). Calling it again on an already-paused/completed/
failed job just returns the current status, no-op.

### `POST /find-route-battery/{job_id}/resume`

Fire-and-forget — sets the resume flag and returns immediately without
waiting for `job_status` to flip back to `"running"`:

```jsonc
{ "status": "success", "job_status": "running" }
```
400 if the job isn't currently `"paused"`.

### `GET`/`POST /find-route-battery/{job_id}/orders`

Only accepted while the job is `"paused"` (400 otherwise). GET lists the
current live order set, reflecting any prior edits:

```jsonc
{ "status": "success", "orders": [
  { "customer_id": "C0", "receiver_name": "TNagarStore", "receiver_lat": 13.0418, "receiver_lng": 80.2341,
    "quantity": 150, "dropoff_time_from": 26.0, "dropoff_time_to": 36.0, "unloading_service_time": 0.5 }
] }
```
POST accepts `add`/`remove`/`edit` in one call — `add`/`edit` rows use the
same shape `/start`'s `orders[]` does (`edit` additionally names
`customer_id`), `remove` is just a list of ids:

```jsonc
{ "add": [{ "...": "same OrderIn shape as /start" }], "remove": ["C2"], "edit": [{ "customer_id": "C0", "...": "..." }] }
```
Response: `{ "status": "success", "added": [...], "removed": [...], "edited": [...], "failed_to_place": [...] }` —
`failed_to_place` lists any added/edited id that couldn't be placed
anywhere feasible (no error raised for that case, just reported).

### `GET`/`POST /find-route-battery/{job_id}/stations`

Same shape and gating as `/orders`, for charging stations — `edit`/`remove`
address a station by its `station_name` (stations don't get a minted id;
the name *is* the id). Unlike orders, station edits never fail outright —
see the "editing while paused" section above for how a disrupted route gets
automatically repaired or its customers re-placed elsewhere, surfaced (if
anything couldn't be re-placed) in the same `failed_to_place` field.

### `GET /find-route-battery/{job_id}/status`

```jsonc
{
  "job_status": "running" | "paused" | "completed" | "failed",
  "iteration": 42, "max_iterations": 625,
  "best_cost": 2.04,                 // current best f_cost (time, hours)
  "best_cost_history": [2.8, 2.6, ...],  // one entry per completed outer iteration
  "error": null
}
```

### `GET /find-route-battery/{job_id}/result`

```jsonc
{
  "status": "success",
  "data": {
    "total_distance_km": 81.76, "total_time_hours": 2.04, "num_vehicles": 4,
    "best_cost_history": [...],
    "routes": [
      {
        "vehicle_id": 1, "total_distance_km": 35.8,
        "stops": [
          { "role": "depot", "label": "ChennaiHub", "lat": 13.0827, "lng": 80.2707, "demand": 0, "arrival_time": 26.0 },
          { "role": "customer", "label": "TNagarStore", "lat": 13.0418, "lng": 80.2341, "demand": 150, "battery_level": 15931.99, "arrival_time": 26.51 },
          { "role": "recharge", "label": "ChennaiCentralCharge", "lat": 13.0732, "lng": 80.2609, "demand": 0, "charge_time_hours": 0.17, "arrival_time": 27.1 },
          { "role": "depot", "label": "ChennaiHub", "lat": 13.0827, "lng": 80.2707, "demand": 0, "arrival_time": 27.8 }
        ]
      }
    ]
  }
}
```

`arrival_time` is on the same relative-hour-offset convention the request
used (re-zeroed back onto it) — `frontend/src/pages/BatteryHome.jsx`
already runs this through `convertTravelTimetoRoutes` to turn it into an
absolute datetime string, same as the diesel flow. Per-leg distance/travel
time and route duration/avg-speed aren't in this response — the frontend
(`BatterySteps/ResultsStep.jsx`) derives them client-side via haversine on
each stop's real lat/lng plus the configured `avg_speed`.

## Files

| File | Role |
|---|---|
| `reading_input.py` | Parses the `.txt` benchmark format (batch/CLI mode only) into the internal dict structures; `distance_power_matrix()` (Euclidean, batch mode) and `distance_power_matrix_osrm()` (real road distance, used by `api_server.py`) both live here |
| `initial_sol.py` | Greedy initial solution: one vehicle per customer |
| `ALNS.py` | Core operators: customer/station removal & insertion, feasibility checking (time window / battery / load), cost function |
| `main.py` | `main_ALNS()` — the outer/inner iteration loop (with a pause/resume checkpoint at the top of the inner loop), adaptive operator weighting, acceptance criterion; also the original batch-script entry point (guarded behind `__main__`) |
| `api_server.py` | FastAPI wrapper — JSON request → internal structures → `main_ALNS()` (background thread) → JSON response. Not part of the original research code. |
| `input.txt` | Sample EVRP-TW instance (Schneider/Keskin benchmark format) for the standalone batch script |

## Known issues fixed along the way

These were real, pre-existing bugs in the original research code
(`main.py`/`ALNS.py`), only ever reachable once this got wrapped as an
importable service instead of a self-contained batch script:

- **`main.py` had no `__main__` guard** — importing `main_ALNS` used to
  silently kick off the entire batch script (every `.txt` file, full
  iteration budget) as a side effect. Now guarded.
- **`main_ALNS` referenced an undefined `stn_operation`** instead of its
  own `stn_insertion` parameter — only "worked" before because the batch
  script happened to define a same-named module global first. Fixed to
  use the actual parameter.
- **`ALNS.py`'s `feasibility_check` has a hardcoded literal `'D0'`**
  depot-ID lookup, reachable once battery constraints get tight enough to
  need station-insertion feasibility checks. Not touched (too dense/risky
  to patch blind) — worked around in `api_server.py` by always using
  `"D0"` as the internal depot id regardless of the real hub's name, with
  the real name kept separately for display.

## Known limitations / open items

- No vehicle-config editing while paused (battery capacity, load capacity,
  alpha/beta, avg_speed) — matches diesel's boundary; only orders and
  stations are editable mid-run, the same scope diesel's order-only editing
  has.
- Service time: `unloading_service_time` maps straight to ALNS's
  `ServiceTime`; there's no dedicated UI input for it yet on the battery
  wizard (defaults to 15 min in `BatteryHome.jsx`) — revisit if that needs
  to be user-configurable.
- Tight battery constraints (forcing frequent station insertion) make each
  ALNS iteration noticeably slower — confirmed via a deliberately
  unrealistic low-battery stress test. Not a bug, just a real performance
  characteristic of the station-insertion feasibility logic.
