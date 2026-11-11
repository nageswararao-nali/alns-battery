import asyncio
import math
import sys
import threading
import time
import traceback
import uuid
import _pickle as cPickle
from typing import List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from initial_sol import initial_solution
from main import main_ALNS
from reading_input import distance_power_matrix_osrm
from ALNS import update_all_fields, feasibility_check, insertion_operation, cost_function


def log(msg):
    print(msg, file=sys.stderr)


app = FastAPI(title="ALNS Battery Route Optimizer API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ===== Request models =====
# Field names/convention deliberately match what frontend/src/pages/
# BatteryHome.jsx already sends (it was written against this planned
# contract before this backend existed) -- orders' time fields are the same
# relative-hour-offset floats convertPickupTimeArray already produces for
# the diesel flow (an arbitrary +24h-baseline constant, re-zeroed below),
# not absolute datetimes.
class OrderIn(BaseModel):
    supplier_name: str
    supplier_lat: float
    supplier_lng: float
    receiver_name: str
    receiver_lat: float
    receiver_lng: float
    quantity: float
    pickup_time_from: float
    pickup_time_to: float
    dropoff_time_from: float
    dropoff_time_to: float
    loading_service_time: float = 0.0
    unloading_service_time: float = 0.25


class StationIn(BaseModel):
    station_name: str
    lat: float
    lng: float
    charger_type: str = "fast"
    break_point_1: float
    bp1_time: float
    break_point_2: float
    bp2_time: float
    full: float
    full_time: float


class VehicleConfigIn(BaseModel):
    battery_capacity: float
    load_capacity: float
    alpha: float
    beta: float
    avg_speed: float


class DepotConfigIn(BaseModel):
    latitude: float
    longitude: float


class FindRouteBatteryRequest(BaseModel):
    orders: List[OrderIn]
    stations: List[StationIn]
    vehicle_config: VehicleConfigIn
    depot_config: Optional[DepotConfigIn] = None


# ===== Mid-run edit models (paused jobs only) =====
# add/edit use the same shapes sent to /start; edit additionally names which
# existing row to replace -- by customer_id (minted server-side) for orders,
# by station_name (the station's own natural id) for stations.
class OrderEdit(OrderIn):
    customer_id: str


class EditOrdersRequest(BaseModel):
    add: List[OrderIn] = []
    remove: List[str] = []
    edit: List[OrderEdit] = []


class EditStationsRequest(BaseModel):
    add: List[StationIn] = []
    remove: List[str] = []
    edit: List[StationIn] = []


def build_model_inputs(req: FindRouteBatteryRequest):
    if not req.orders:
        raise ValueError("At least one order is required")
    if not req.stations:
        raise ValueError("At least one charging station is required")

    depot_lat = req.depot_config.latitude if req.depot_config else req.orders[0].supplier_lat
    depot_lng = req.depot_config.longitude if req.depot_config else req.orders[0].supplier_lng
    # ALNS.py's feasibility_check has a hardcoded literal 'D0' depot-id
    # lookup (confirmed live: KeyError: 'D0' the instant a run's battery
    # constraints got tight enough to actually need a station-insertion
    # feasibility check -- a real upload's hub is never actually named
    # "D0"). Keeping the internal depot id fixed at "D0" regardless of the
    # uploaded hub's real name sidesteps that without touching the dense
    # feasibility_check internals; depot_display_name is what the API
    # response actually shows.
    depot_name = "D0"
    depot_display_name = req.orders[0].supplier_name or "HUB"

    # customers/recharge_stns/depots carry raw (lat, lng) -- OSRM needs real
    # coordinates, and nothing in initial_sol.py/ALNS.py ever indexes these
    # dicts for coordinate values (confirmed by grep); they're only read via
    # .keys() for membership/iteration, so this is a safe swap from the
    # local-projection approach distance_power_matrix_osrm's Euclidean
    # sibling (distance_power_matrix, kept for the standalone batch script)
    # used to need.
    depots = {depot_name: (depot_lat, depot_lng)}
    depot_latlng = {depot_name: (depot_lat, depot_lng)}

    customers = {}
    demand = {}
    time_window = {depot_name: (0.0, 999999.0, 0.0)}
    customer_meta = {}

    # Re-zeroing onto this batch's own minimum dropoff-from makes the
    # incoming constant offset (whatever it is) irrelevant -- only the
    # relative spacing between orders' windows matters to the algorithm.
    min_ready = min(o.dropoff_time_from for o in req.orders)

    for idx, o in enumerate(req.orders):
        cid = f"C{idx}"
        customers[cid] = (o.receiver_lat, o.receiver_lng)
        demand[cid] = o.quantity
        ready = o.dropoff_time_from - min_ready
        due = o.dropoff_time_to - min_ready
        if due <= ready:
            due = ready + 1.0
        time_window[cid] = (ready, due, o.unloading_service_time)
        customer_meta[cid] = o

    recharge_stns = {}
    station_latlng = {}
    station_charger_type = {}
    if_rate = {}
    for s in req.stations:
        recharge_stns[s.station_name] = (s.lat, s.lng)
        station_latlng[s.station_name] = (s.lat, s.lng)
        station_charger_type[s.station_name] = s.charger_type
        # Matches reading_input.py's own if_rate construction exactly:
        # each breakpoint tuple is (energy_level, hours-per-Wh-up-to-there).
        if_rate[s.station_name] = [
            (s.break_point_1, s.bp1_time / s.break_point_1),
            (s.break_point_2, s.bp2_time / s.break_point_2),
            (s.full, s.full_time / s.full),
        ]

    try:
        (dist_mat_cc, dist_mat_ccs, dist_mat_cd, dist_mat_csd,
         dist_mat_cc_time, dist_mat_ccs_time, dist_mat_cd_time, dist_mat_csd_time) = distance_power_matrix_osrm(customers, recharge_stns, depots)
    except Exception as e:
        raise RuntimeError(f"Failed to compute OSRM distances: {e}. Make sure the OSRM server (osrm.goodmove.in) is reachable") from e

    return {
        "customers": customers, "recharge_stns": recharge_stns, "depots": depots,
        "demand": demand, "time_window": time_window, "if_rate": if_rate,
        "dist_mat_cc": dist_mat_cc, "dist_mat_ccs": dist_mat_ccs,
        "dist_mat_cd": dist_mat_cd, "dist_mat_csd": dist_mat_csd,
        "dist_mat_cc_time": dist_mat_cc_time, "dist_mat_ccs_time": dist_mat_ccs_time,
        "dist_mat_cd_time": dist_mat_cd_time, "dist_mat_csd_time": dist_mat_csd_time,
        "depot_name": depot_name, "depot_display_name": depot_display_name,
        "depot_latlng": depot_latlng, "station_latlng": station_latlng,
        "station_charger_type": station_charger_type,
        "customer_meta": customer_meta, "min_ready": min_ready,
        "battery_cap": req.vehicle_config.battery_capacity, "load_cap": req.vehicle_config.load_capacity,
        "fc_alpha": req.vehicle_config.alpha, "fc_beta": req.vehicle_config.beta,
        "avg_speed": req.vehicle_config.avg_speed,
    }


# ===== Result translation =====
# Walks BestSol's solution/re_stn_map/arrival_time/battery structures into
# the routes[].stops[] shape frontend/src/pages/BatterySteps/ResultsStep.jsx
# expects. The depot-id-or-customer-id-as-dict-key check for where a
# recharge stop falls mirrors ALNS.py's own cost_function exactly (same
# membership tests), rather than re-deriving that logic independently --
# confirmed against live BestSol output during development, not guessed
# from reading the removal/insertion operators alone.
def build_result(model_inputs, best_sol):
    depot_name = model_inputs["depot_name"]
    depot_display_name = model_inputs["depot_display_name"]
    depot_lat, depot_lng = model_inputs["depot_latlng"][depot_name]
    station_latlng = model_inputs["station_latlng"]
    customer_meta = model_inputs["customer_meta"]
    demand = model_inputs["demand"]
    min_ready = model_inputs["min_ready"]

    sol = best_sol["sol"]
    re_stn_map = best_sol["re_stn_map"]
    arrival_time = best_sol["arrival_time"]
    battery = best_sol["battery"]
    veh_cost = best_sol["veh_cost"]

    def depot_stop(arrival_hours, leg, battery_level=None):
        leg_km, leg_hours = leg
        return {
            "role": "depot", "label": depot_display_name,
            "lat": depot_lat, "lng": depot_lng, "demand": 0,
            "arrival_time": arrival_hours + min_ready if arrival_hours is not None else None,
            "battery_level": battery_level,
            "leg_distance_km": leg_km, "leg_travel_minutes": leg_hours * 60,
        }

    # entry[0] is the station DEPARTURE time (arrival-at-station + charge
    # time -- see ALNS.py tw_update's `fs_stn_map[...] = [atime+ctime, ...]`
    # construction), not arrival. Subtract entry[7] (charge_time) back out
    # so the "Arrival Time" column means the same thing for every row type.
    #
    # entry[2]/entry[6] are the battery level on arrival (before charging)
    # and the target level charging brings it up to (after charging) --
    # confirmed live: charge_time_hours * the station's own fast-segment
    # rate (break_point_1/bp1_time) reproduces entry[6]-entry[2] almost
    # exactly for an empty-arrival charge, e.g. 4130.46Wh / 0.0942h =
    # ~43,857 Wh/h against a configured ~43,871 Wh/h rate -- not just a
    # guess from the construction sites' variable names.
    def recharge_stop(entry, leg):
        station_id = entry[4]
        s_lat, s_lng = station_latlng[station_id]
        charge_time = entry[7]
        leg_km, leg_hours = leg
        return {
            "role": "recharge", "label": station_id,
            "lat": s_lat, "lng": s_lng, "demand": 0,
            "charge_time_hours": charge_time,
            "arrival_time": entry[0] - charge_time + min_ready,
            "battery_start": entry[2],
            "battery_end": entry[6],
            "leg_distance_km": leg_km, "leg_travel_minutes": leg_hours * 60,
        }

    dist_mat_cc = model_inputs["dist_mat_cc"]
    dist_mat_ccs = model_inputs["dist_mat_ccs"]
    dist_mat_cd = model_inputs["dist_mat_cd"]
    dist_mat_csd = model_inputs["dist_mat_csd"]
    dist_mat_cc_time = model_inputs["dist_mat_cc_time"]
    dist_mat_ccs_time = model_inputs["dist_mat_ccs_time"]
    dist_mat_cd_time = model_inputs["dist_mat_cd_time"]
    dist_mat_csd_time = model_inputs["dist_mat_csd_time"]
    fc_alpha = model_inputs["fc_alpha"]
    battery_cap = model_inputs["battery_cap"]

    # Road distance/time for the leg arriving AT (cur_role, cur_id) FROM
    # (prev_role, prev_id) -- dispatches to the same matrix ALNS.py itself
    # reads for that exact transition (confirmed against tw_update/
    # battery_update's own lookups), so these numbers always agree with
    # the arrival_time column rather than being a separate, potentially-
    # inconsistent estimate (this table used to show a client-side
    # haversine/avg_speed guess instead -- see frontend/.../ResultsStep.jsx
    # -- which diverged badly from the real OSRM-time-driven arrival times
    # once those stopped being avg_speed-based).
    # dist_mat_ccs is only ever populated customer->station (see
    # distance_power_matrix_osrm's comment on skipped reverse pairs), so
    # the station->customer leg below reuses it with a flipped index --
    # exactly what ALNS.py's own tw_update does for that same leg.
    def leg_for(prev_role, prev_id, cur_role, cur_id):
        if prev_role is None:
            return (0.0, 0.0)
        if cur_role == "customer":
            if prev_role == "depot":
                return (dist_mat_cd[cur_id][depot_name], dist_mat_cd_time[cur_id][depot_name])
            if prev_role == "recharge":
                return (dist_mat_ccs[cur_id][prev_id], dist_mat_ccs_time[cur_id][prev_id])
            return (dist_mat_cc[prev_id][cur_id], dist_mat_cc_time[prev_id][cur_id])
        if cur_role == "recharge":
            if prev_role == "depot":
                return (dist_mat_csd[cur_id][depot_name], dist_mat_csd_time[cur_id][depot_name])
            return (dist_mat_ccs[prev_id][cur_id], dist_mat_ccs_time[prev_id][cur_id])
        # cur_role == "depot" (final return leg)
        if prev_role == "recharge":
            return (dist_mat_csd[prev_id][depot_name], dist_mat_csd_time[prev_id][depot_name])
        return (dist_mat_cd[prev_id][depot_name], dist_mat_cd_time[prev_id][depot_name])

    routes = []
    total_distance_km = 0.0
    for vid, cust_list in enumerate(sol):
        if not cust_list:
            continue
        stn_map = re_stn_map[vid] if vid < len(re_stn_map) else {}
        arr = arrival_time[vid] if vid < len(arrival_time) else {}
        bat = battery[vid] if vid < len(battery) else []

        stops = [depot_stop(0.0, (0.0, 0.0), battery_level=battery_cap)]
        prev_role, prev_id = "depot", depot_name
        # Tracks the battery level leaving the most recent stop, so the
        # final depot-return row's own level can be derived the same way
        # its arrival *time* already is -- the algorithm itself never
        # persists a "battery on return to depot" value anywhere, only the
        # per-customer levels in `battery` and the per-station start/end
        # levels in `re_stn_map`.
        last_battery = battery_cap
        if depot_name in stn_map:
            entry = stn_map[depot_name]
            leg = leg_for(prev_role, prev_id, "recharge", entry[4])
            stops.append(recharge_stop(entry, leg))
            last_battery = entry[6]
            prev_role, prev_id = "recharge", entry[4]

        last_arrival = 0.0
        for pos, cid in enumerate(cust_list):
            o = customer_meta[cid]
            cid_arrival = arr.get(cid)
            cid_battery = bat[pos] if pos < len(bat) else None
            if cid_arrival is not None:
                last_arrival = cid_arrival
            if cid_battery is not None:
                last_battery = cid_battery
            leg = leg_for(prev_role, prev_id, "customer", cid)
            stops.append({
                "role": "customer", "label": o.receiver_name,
                "lat": o.receiver_lat, "lng": o.receiver_lng,
                "demand": demand[cid],
                "battery_level": cid_battery,
                "arrival_time": (cid_arrival + min_ready) if cid_arrival is not None else None,
                "leg_distance_km": leg[0], "leg_travel_minutes": leg[1] * 60,
            })
            prev_role, prev_id = "customer", cid
            if cid in stn_map:
                entry = stn_map[cid]
                leg = leg_for(prev_role, prev_id, "recharge", entry[4])
                stops.append(recharge_stop(entry, leg))
                last_arrival = entry[0]
                last_battery = entry[6]
                prev_role, prev_id = "recharge", entry[4]

        # No load term (fc_beta) on this leg -- by the time a route returns
        # to the depot every order on it has already been delivered.
        final_leg = leg_for(prev_role, prev_id, "depot", depot_name)
        final_battery = max(0.0, last_battery - final_leg[0] * fc_alpha)
        stops.append(depot_stop(last_arrival + final_leg[1], final_leg, battery_level=final_battery))

        dist = veh_cost[vid] if vid < len(veh_cost) else 0.0
        routes.append({"vehicle_id": vid + 1, "total_distance_km": dist, "stops": stops})
        total_distance_km += dist

    return {
        "routes": routes,
        "total_distance_km": total_distance_km,
        "total_time_hours": best_sol.get("f_cost", 0),
        "num_vehicles": len(routes),
    }


# ===== Background job runner =====
JOBS = {}
JOBS_LOCK = threading.Lock()


class BatteryJob:
    def __init__(self, job_id):
        self.job_id = job_id
        self.status = "running"  # running | paused | completed | failed
        self.error = None
        self.result = None
        self.it2 = 0
        self.max_iter = None
        self.best_cost = None
        self.best_cost_history = []
        self.created_at = time.time()

        # Checked inside main_ALNS's inner loop (main.py) -- see the
        # pause/resume endpoints below. resume_requested.wait() blocks the
        # background thread running main_ALNS directly, so pausing costs
        # zero CPU and resuming continues exactly where the solve left off.
        self.pause_requested = threading.Event()
        self.paused_confirmed = threading.Event()
        self.resume_requested = threading.Event()

        # Problem data + live solver state, aliased in _run_job onto the
        # exact same objects main_ALNS is running with. Edits made here
        # while paused (see _apply_order_edits/_apply_station_edits below)
        # mutate these IN PLACE -- never rebind them to a new dict/list --
        # so main_ALNS's own local variables see the change automatically
        # with no extra sync code, since they're the same objects. Only
        # osolution/best_sol are ever rebound wholesale; those get synced
        # explicitly across the pause/resume boundary in main.py's
        # checkpoint.
        self.customers = None
        self.recharge_stns = None
        self.depots = None
        self.demand = None
        self.time_window = None
        self.if_rate = None
        self.dist_mat_cc = None
        self.dist_mat_ccs = None
        self.dist_mat_cd = None
        self.dist_mat_csd = None
        self.dist_mat_cc_time = None
        self.dist_mat_ccs_time = None
        self.dist_mat_cd_time = None
        self.dist_mat_csd_time = None
        self.initial_cost = None
        self.depot_name = None
        self.station_latlng = None
        self.station_charger_type = None
        self.customer_meta = None
        self.min_ready = None
        self.battery_cap = None
        self.load_cap = None
        self.fc_alpha = None
        self.fc_beta = None
        self.avg_speed = None
        self.next_customer_idx = 0
        self.osolution = None
        self.best_sol = None
        # Serializes concurrent /orders or /stations edit calls against the
        # same job -- not against the solve thread, which is blocked on
        # resume_requested.wait() whenever status == "paused" (the only
        # time edits are accepted), so there's no race with it to guard.
        self.edit_lock = threading.Lock()


def _run_job(job, model_inputs):
    try:
        customers = model_inputs["customers"]
        recharge_stns = model_inputs["recharge_stns"]
        depots = model_inputs["depots"]
        demand = model_inputs["demand"]
        time_window = model_inputs["time_window"]
        battery_cap = model_inputs["battery_cap"]
        load_cap = model_inputs["load_cap"]
        dist_mat_cc = model_inputs["dist_mat_cc"]
        dist_mat_ccs = model_inputs["dist_mat_ccs"]
        dist_mat_cd = model_inputs["dist_mat_cd"]
        dist_mat_csd = model_inputs["dist_mat_csd"]
        dist_mat_cc_time = model_inputs["dist_mat_cc_time"]
        dist_mat_ccs_time = model_inputs["dist_mat_ccs_time"]
        dist_mat_cd_time = model_inputs["dist_mat_cd_time"]
        dist_mat_csd_time = model_inputs["dist_mat_csd_time"]
        fc_alpha = model_inputs["fc_alpha"]
        fc_beta = model_inputs["fc_beta"]
        if_rate = model_inputs["if_rate"]
        avg_speed = model_inputs["avg_speed"]
        depot = model_inputs["depot_name"]

        isolution, icost, ivehcost, ifcost, icus_map, istn_map, iarrival, ibattery, iload = initial_solution(
            customers, depot, demand, time_window, battery_cap, load_cap,
            dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time,
            dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time,
            fc_alpha, fc_beta, if_rate, avg_speed,
        )

        initial_cost = {}
        for ivcid, ivc in enumerate(ivehcost):
            initial_cost[list(customers.keys())[ivcid]] = ivc

        Solution = {
            'sol': isolution, 'cost': icost, 'cus_map': icus_map, 're_stn_map': istn_map,
            'arrival_time': iarrival, 'battery': ibattery, 'load': iload, 'f_cost': ifcost,
            'runtime': 0, 'veh_cost': ivehcost,
        }

        # Same tuning constants main.py's own batch script uses.
        stn_insertion = 11
        max_inner_iter = 800
        max_iter = int(500000 / max_inner_iter)
        job.max_iter = max_iter
        start = time.time()

        def on_progress(it2, _max_iter, best_f_cost, best_cost_list):
            job.it2 = it2
            job.best_cost = best_f_cost
            job.best_cost_history = list(best_cost_list)

        # Alias problem data onto job as the exact same objects main_ALNS is
        # about to run with -- see BatteryJob.__init__ and main.py's pause
        # checkpoint for why this makes /orders and /stations edits visible
        # to the solve loop with no further sync code.
        job.customers, job.recharge_stns, job.depots = customers, recharge_stns, depots
        job.demand, job.time_window, job.if_rate = demand, time_window, if_rate
        job.dist_mat_cc, job.dist_mat_ccs = dist_mat_cc, dist_mat_ccs
        job.dist_mat_cd, job.dist_mat_csd = dist_mat_cd, dist_mat_csd
        job.dist_mat_cc_time, job.dist_mat_ccs_time = dist_mat_cc_time, dist_mat_ccs_time
        job.dist_mat_cd_time, job.dist_mat_csd_time = dist_mat_cd_time, dist_mat_csd_time
        job.initial_cost = initial_cost
        job.depot_name = depot
        job.station_latlng = model_inputs["station_latlng"]
        job.station_charger_type = model_inputs["station_charger_type"]
        job.customer_meta = model_inputs["customer_meta"]
        job.min_ready = model_inputs["min_ready"]
        job.battery_cap, job.load_cap = battery_cap, load_cap
        job.fc_alpha, job.fc_beta, job.avg_speed = fc_alpha, fc_beta, avg_speed
        job.next_customer_idx = len(customers)
        job.osolution = Solution
        job.best_sol = Solution

        best_sol, *_rest = main_ALNS(
            Solution, customers, recharge_stns, depots, demand, time_window, battery_cap, load_cap,
            dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time,
            dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time,
            fc_alpha, fc_beta, if_rate, avg_speed,
            stn_insertion, max_iter, max_inner_iter, 30, 0.98, 0, 50, initial_cost, start, 1 / 9,
            progress_callback=on_progress, job=job,
        )

        result = build_result(model_inputs, best_sol)
        result["best_cost_history"] = job.best_cost_history
        job.result = result
        job.status = "completed"
    except Exception as e:  # noqa: BLE001 -- job failure must be reported, not crash the thread silently
        log(f"Job {job.job_id} failed: {e}")
        traceback.print_exc(file=sys.stderr)
        job.error = str(e)
        job.status = "failed"


def _get_job_or_404(job_id: str) -> BatteryJob:
    with JOBS_LOCK:
        job = JOBS.get(job_id)
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")
    return job


# ===== Order editing (paused jobs only) =====
# Mirrors alns/main.py's _apply_order_edits/_remove_customer_from_job
# pattern closely -- see the plan's "Key design decisions" for why
# update_all_fields (the same primitive customer_removal_operation's
# modes 0/1/2/5 use every normal iteration) is reused here rather than
# hand-deriving the station-anchor-relocation edge cases independently.

def _remove_customer_from_job(job, sol_dict, cid):
    if cid not in sol_dict['cus_map']:
        return
    vid = sol_dict['cus_map'][cid]
    # Deep-copied BEFORE stripping -- update_all_fields compares the
    # ORIGINAL (unstripped) route length against route_pool's length to
    # decide partial-removal vs emptying, and does its own removal from
    # this copy internally.
    o_s_sol = cPickle.loads(cPickle.dumps(sol_dict['sol'], -1))
    route_pool = {i: [] for i in range(len(sol_dict['sol']))}
    route_pool[vid].append(cid)
    # Pre-stripping arrival_time/sol/cus_map here matters for the
    # "removing the LAST customer in a route" case specifically --
    # update_all_fields's empty-route branch doesn't itself clear
    # tw_check[vid], only sol/stn_map/load, so without this the stale
    # arrival_time entry would survive.
    del sol_dict['arrival_time'][vid][cid]
    sol_dict['sol'][vid].remove(cid)
    del sol_dict['cus_map'][cid]
    (
        sol_dict['sol'], sol_dict['cus_map'], sol_dict['battery'],
        sol_dict['re_stn_map'], sol_dict['arrival_time'], sol_dict['load'],
    ) = update_all_fields(
        route_pool, sol_dict['cus_map'], o_s_sol, sol_dict['battery'], job.battery_cap,
        sol_dict['arrival_time'], job.time_window, sol_dict['re_stn_map'], job.dist_mat_ccs, job.dist_mat_ccs_time,
        job.dist_mat_cc, job.dist_mat_cc_time, job.dist_mat_cd, job.dist_mat_cd_time,
        job.dist_mat_csd, job.dist_mat_csd_time, job.avg_speed, job.fc_alpha, job.fc_beta,
        job.if_rate, job.depot_name, job.demand, sol_dict['load'], job.load_cap,
    )


def _set_customer_data(job, cid, order):
    job.customers[cid] = (order.receiver_lat, order.receiver_lng)
    job.customer_meta[cid] = order
    job.demand[cid] = order.quantity
    # Same re-zero-onto-this-job's-min_ready convention build_model_inputs
    # uses for the initial batch -- keeps a mid-run addition's time window
    # on the same relative-hour baseline as everything else already in play.
    ready = order.dropoff_time_from - job.min_ready
    due = order.dropoff_time_to - job.min_ready
    if due <= ready:
        due = ready + 1.0
    job.time_window[cid] = (ready, due, order.unloading_service_time)


def _add_customer_to_job(job, order):
    cid = f"C{job.next_customer_idx}"
    job.next_customer_idx += 1
    _set_customer_data(job, cid, order)
    return cid


def _prune_customer_from_matrices(job, cid):
    job.dist_mat_cc.pop(cid, None)
    for row in job.dist_mat_cc.values():
        row.pop(cid, None)
    job.dist_mat_ccs.pop(cid, None)
    job.dist_mat_cd.pop(cid, None)
    job.dist_mat_cc_time.pop(cid, None)
    for row in job.dist_mat_cc_time.values():
        row.pop(cid, None)
    job.dist_mat_ccs_time.pop(cid, None)
    job.dist_mat_cd_time.pop(cid, None)
    job.demand.pop(cid, None)
    job.time_window.pop(cid, None)
    job.customer_meta.pop(cid, None)
    job.initial_cost.pop(cid, None)


def _pad_empty_route_slots(sol_dict, n):
    # sol/battery/arrival_time/load are plain lists (see initial_sol.py's
    # add_vehicle, which .append()s to each) -- but re_stn_map is a dict
    # keyed by integer vehicle index (add_vehicle extends it via
    # `stn_iden_map.update({len(stn_iden_map): {}})`), not a list. Matching
    # that construction exactly here, not guessing a shape.
    for _ in range(n):
        sol_dict['sol'].append([])
        sol_dict['battery'].append([])
        sol_dict['re_stn_map'][len(sol_dict['re_stn_map'])] = {}
        sol_dict['arrival_time'].append({})
        sol_dict['load'].append([])


def _run_insertion(job, sol_dict, pool):
    """Places every id in pool (customer ids needing a new/updated spot)
    via the same regret-2 insertion mode (b=2) the normal ALNS loop uses,
    mutating sol_dict in place. Returns ids that still couldn't be placed
    anywhere feasible -- cheap_regret_in silently drops a customer from
    the pool with no placement rather than raising, so this is the only
    way to detect that."""
    (
        sol_dict['sol'], sol_dict['cus_map'], sol_dict['re_stn_map'],
        sol_dict['battery'], sol_dict['arrival_time'], sol_dict['load'],
    ) = insertion_operation(
        2, list(pool), sol_dict['sol'], job.depot_name, job.demand, sol_dict['cus_map'], sol_dict['re_stn_map'],
        job.fc_alpha, job.fc_beta, job.if_rate, job.time_window, job.avg_speed, job.battery_cap,
        job.dist_mat_cd, job.dist_mat_cd_time, job.dist_mat_cc, job.dist_mat_cc_time,
        job.dist_mat_ccs, job.dist_mat_ccs_time, job.dist_mat_csd, job.dist_mat_csd_time,
        sol_dict['battery'], sol_dict['arrival_time'], job.initial_cost, sol_dict['load'], job.load_cap,
    )
    return [cid for cid in pool if cid not in sol_dict['cus_map']]


def _finalize_edit(job, sol_dict):
    tcus = len(job.customers)
    sol_dict['cost'], sol_dict['veh_cost'], sol_dict['f_cost'] = cost_function(
        sol_dict['sol'], job.depot_name, job.dist_mat_cc, job.dist_mat_cc_time, job.dist_mat_ccs, job.dist_mat_ccs_time,
        job.dist_mat_cd, job.dist_mat_cd_time, job.dist_mat_csd, job.dist_mat_csd_time,
        sol_dict['re_stn_map'], tcus, job.avg_speed, job.time_window, job.demand, job.load_cap, True,
    )
    job.best_sol = sol_dict
    job.osolution = sol_dict


def _apply_order_edits(job, body: EditOrdersRequest):
    sol_dict = job.best_sol

    removed_ids = []
    for cid in body.remove:
        if cid in job.customers:
            _remove_customer_from_job(job, sol_dict, cid)
            del job.customers[cid]
            _prune_customer_from_matrices(job, cid)
            removed_ids.append(cid)

    edited_ids = []
    to_insert = []
    for edit in body.edit:
        cid = edit.customer_id
        if cid not in job.customers:
            continue  # unknown id -- ignore rather than fail the whole batch
        _remove_customer_from_job(job, sol_dict, cid)
        _set_customer_data(job, cid, edit)
        to_insert.append(cid)
        edited_ids.append(cid)

    added_ids = []
    for order in body.add:
        cid = _add_customer_to_job(job, order)
        to_insert.append(cid)
        added_ids.append(cid)

    failed_ids = []
    if to_insert:
        (new_cc, new_ccs, new_cd, new_csd,
         new_cc_time, new_ccs_time, new_cd_time, new_csd_time) = distance_power_matrix_osrm(job.customers, job.recharge_stns, job.depots)
        job.dist_mat_cc.clear(); job.dist_mat_cc.update(new_cc)
        job.dist_mat_ccs.clear(); job.dist_mat_ccs.update(new_ccs)
        job.dist_mat_cd.clear(); job.dist_mat_cd.update(new_cd)
        job.dist_mat_csd.clear(); job.dist_mat_csd.update(new_csd)
        job.dist_mat_cc_time.clear(); job.dist_mat_cc_time.update(new_cc_time)
        job.dist_mat_ccs_time.clear(); job.dist_mat_ccs_time.update(new_ccs_time)
        job.dist_mat_cd_time.clear(); job.dist_mat_cd_time.update(new_cd_time)
        job.dist_mat_csd_time.clear(); job.dist_mat_csd_time.update(new_csd_time)
        for cid in to_insert:
            job.initial_cost[cid] = job.dist_mat_cd[cid][job.depot_name] * 2

        # "Unlimited fleet" model, matching the rest of the solver -- one
        # fresh empty route slot per id needing placement, in case nothing
        # existing fits.
        _pad_empty_route_slots(sol_dict, len(to_insert))
        failed_ids = _run_insertion(job, sol_dict, to_insert)

    _finalize_edit(job, sol_dict)

    return {
        "added": added_ids,
        "removed": removed_ids,
        "edited": edited_ids,
        "failed_to_place": failed_ids,
    }


# ===== Station editing (paused jobs only) =====
# No diesel precedent -- stations are new territory. Per the plan's "re-
# optimise" decision: a station add/edit/remove always commits, and any
# route it disrupts is repaired automatically (local feasibility_check
# rebuild first; customers from routes that still can't be fixed go through
# the same insertion pool as the order-add path above).

def _station_rate_tuple(s):
    return [
        (s.break_point_1, s.bp1_time / s.break_point_1),
        (s.break_point_2, s.bp2_time / s.break_point_2),
        (s.full, s.full_time / s.full),
    ]


def _prune_station_from_matrices(job, name):
    job.recharge_stns.pop(name, None)
    job.if_rate.pop(name, None)
    job.station_latlng.pop(name, None)
    job.station_charger_type.pop(name, None)
    for row in job.dist_mat_ccs.values():
        row.pop(name, None)
    job.dist_mat_csd.pop(name, None)
    for row in job.dist_mat_ccs_time.values():
        row.pop(name, None)
    job.dist_mat_csd_time.pop(name, None)


def _affected_vehicles_for_station(sol_dict, station_name):
    # re_stn_map is a dict keyed by integer vehicle index (see
    # _pad_empty_route_slots), not a list -- iterate with .items(), not
    # enumerate() (which would hand back the dict's keys as values here).
    return [
        vid for vid, stn_map in sol_dict['re_stn_map'].items()
        if any(entry[4] == station_name for entry in stn_map.values())
    ]


def _repair_routes_after_station_change(job, sol_dict, affected_vids):
    """For each route that referenced a now-removed/moved station, rebuilds
    its battery/timing/station-placement from scratch via feasibility_check
    (the same composed primitive cheap_regret_in relies on for insertion
    feasibility -- unlike the lower-level battery_update/stn_insertion
    chain, it gives a clean Reject/Insert signal instead of silently
    producing possibly-infeasible numbers). Routes feasibility_check can't
    fix are emptied entirely, their customers returned for the caller to
    feed through the normal insertion pool."""
    to_reinsert = []
    for vid in affected_vids:
        route = sol_dict['sol'][vid]
        if not route:
            continue
        status, batt, tw_check, new_stn_map, dem_map = feasibility_check(
            route, job.battery_cap, {}, job.fc_alpha, job.fc_beta, job.if_rate, vid, job.avg_speed,
            job.time_window, job.depot_name, job.dist_mat_cc, job.dist_mat_cc_time,
            job.dist_mat_ccs, job.dist_mat_ccs_time, job.dist_mat_cd, job.dist_mat_cd_time,
            job.dist_mat_csd, job.dist_mat_csd_time, job.demand, job.load_cap,
        )
        if status == "Insert":
            sol_dict['battery'][vid] = batt
            sol_dict['arrival_time'][vid] = tw_check
            sol_dict['re_stn_map'][vid] = new_stn_map
            sol_dict['load'][vid] = dem_map
        else:
            to_reinsert.extend(route)
            sol_dict['sol'][vid] = []
            sol_dict['battery'][vid] = []
            sol_dict['re_stn_map'][vid] = {}
            sol_dict['arrival_time'][vid] = {}
            sol_dict['load'][vid] = []
            for cid in route:
                sol_dict['cus_map'].pop(cid, None)
    return to_reinsert


def _apply_station_edits(job, body: EditStationsRequest):
    sol_dict = job.best_sol
    affected_vids = set()

    removed_names = []
    for name in body.remove:
        if name in job.recharge_stns:
            affected_vids.update(_affected_vehicles_for_station(sol_dict, name))
            _prune_station_from_matrices(job, name)
            removed_names.append(name)

    edited_names = []
    for s in body.edit:
        if s.station_name not in job.recharge_stns:
            continue  # unknown name -- ignore rather than fail the whole batch
        affected_vids.update(_affected_vehicles_for_station(sol_dict, s.station_name))
        job.recharge_stns[s.station_name] = (s.lat, s.lng)
        job.if_rate[s.station_name] = _station_rate_tuple(s)
        job.station_latlng[s.station_name] = (s.lat, s.lng)
        job.station_charger_type[s.station_name] = s.charger_type
        edited_names.append(s.station_name)

    added_names = []
    for s in body.add:
        if s.station_name in job.recharge_stns:
            raise ValueError(f"Station '{s.station_name}' already exists")
        job.recharge_stns[s.station_name] = (s.lat, s.lng)
        job.if_rate[s.station_name] = _station_rate_tuple(s)
        job.station_latlng[s.station_name] = (s.lat, s.lng)
        job.station_charger_type[s.station_name] = s.charger_type
        added_names.append(s.station_name)

    if added_names or edited_names:
        # A station's lat/lng or rates changed (or a new one appeared) --
        # affects every customer's distance to it, not just current users,
        # so this is a full recompute rather than a targeted patch (same
        # "simplest correct, not incremental" tradeoff as order edits).
        (new_cc, new_ccs, new_cd, new_csd,
         new_cc_time, new_ccs_time, new_cd_time, new_csd_time) = distance_power_matrix_osrm(job.customers, job.recharge_stns, job.depots)
        job.dist_mat_cc.clear(); job.dist_mat_cc.update(new_cc)
        job.dist_mat_ccs.clear(); job.dist_mat_ccs.update(new_ccs)
        job.dist_mat_cd.clear(); job.dist_mat_cd.update(new_cd)
        job.dist_mat_csd.clear(); job.dist_mat_csd.update(new_csd)
        job.dist_mat_cc_time.clear(); job.dist_mat_cc_time.update(new_cc_time)
        job.dist_mat_ccs_time.clear(); job.dist_mat_ccs_time.update(new_ccs_time)
        job.dist_mat_cd_time.clear(); job.dist_mat_cd_time.update(new_cd_time)
        job.dist_mat_csd_time.clear(); job.dist_mat_csd_time.update(new_csd_time)

    failed_ids = []
    if affected_vids:
        to_reinsert = _repair_routes_after_station_change(job, sol_dict, sorted(affected_vids))
        if to_reinsert:
            _pad_empty_route_slots(sol_dict, len(to_reinsert))
            failed_ids = _run_insertion(job, sol_dict, to_reinsert)

    _finalize_edit(job, sol_dict)

    return {
        "added": added_names,
        "removed": removed_names,
        "edited": edited_names,
        "failed_to_place": failed_ids,
    }


@app.post("/find-route-battery/start")
async def start_job(request: FindRouteBatteryRequest):
    try:
        model_inputs = build_model_inputs(request)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        # Covers OSRM failures (network error, non-"Ok" response) raised as
        # RuntimeError by build_model_inputs -- surfaced as a helpful 500
        # instead of an opaque "Internal Server Error".
        raise HTTPException(status_code=500, detail=str(e))

    job_id = str(uuid.uuid4())
    job = BatteryJob(job_id)
    with JOBS_LOCK:
        JOBS[job_id] = job
    threading.Thread(target=_run_job, args=(job, model_inputs), daemon=True).start()
    return {"job_id": job_id}


@app.post("/find-route-battery/{job_id}/pause")
async def pause_job(job_id: str):
    """Requests a pause and waits (up to 30s) for the solve loop to
    actually reach a safe checkpoint before returning, so the caller can
    rely on the job being genuinely halted once this returns successfully."""
    job = _get_job_or_404(job_id)
    if job.status in ("completed", "failed", "paused"):
        return {"status": "success", "job_status": job.status, "iteration": job.it2}

    job.paused_confirmed.clear()
    job.pause_requested.set()
    confirmed = await asyncio.to_thread(job.paused_confirmed.wait, 30)
    if not confirmed and job.status not in ("completed", "failed"):
        raise HTTPException(status_code=504, detail="Timed out waiting for job to pause")

    return {
        "status": "success",
        "job_status": job.status,
        "iteration": job.it2,
        "best_cost": job.best_cost,
        "best_cost_history": job.best_cost_history,
    }


@app.post("/find-route-battery/{job_id}/resume")
async def resume_job(job_id: str):
    job = _get_job_or_404(job_id)
    if job.status != "paused":
        raise HTTPException(status_code=400, detail=f"Job is not paused (status={job.status})")
    job.resume_requested.set()
    return {"status": "success", "job_status": "running"}


@app.get("/find-route-battery/{job_id}/orders")
async def get_orders(job_id: str):
    """Lists the job's current live order set (reflects any prior
    add/remove/edit calls), so a client can reference specific orders by
    customer_id in a subsequent edit."""
    job = _get_job_or_404(job_id)
    orders = []
    for cid, (lat, lng) in job.customers.items():
        meta = job.customer_meta[cid]
        ready, due, service = job.time_window[cid]
        orders.append({
            "customer_id": cid,
            "receiver_name": meta.receiver_name,
            "receiver_lat": lat, "receiver_lng": lng,
            "quantity": job.demand[cid],
            # Re-adding min_ready undoes _set_customer_data's re-zero, so
            # these come back on the same relative-hour baseline the
            # frontend already sent (and that route arrival_times use).
            "dropoff_time_from": ready + job.min_ready,
            "dropoff_time_to": due + job.min_ready,
            "unloading_service_time": service,
        })
    return {"status": "success", "orders": orders}


@app.post("/find-route-battery/{job_id}/orders")
async def edit_orders(job_id: str, body: EditOrdersRequest):
    """Adds/removes/edits orders on a PAUSED job, inserting/removing them
    from the current solution immediately. Rejected if the job isn't
    paused, to avoid racing the background solve thread."""
    job = _get_job_or_404(job_id)
    if job.status != "paused":
        raise HTTPException(status_code=400, detail=f"Job must be paused before editing orders (status={job.status})")
    with job.edit_lock:
        try:
            summary = _apply_order_edits(job, body)
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            raise HTTPException(status_code=500, detail=f"Error applying order edits: {e}")
    return {"status": "success", **summary}


@app.get("/find-route-battery/{job_id}/stations")
async def get_stations(job_id: str):
    job = _get_job_or_404(job_id)
    stations = []
    for name, (lat, lng) in job.recharge_stns.items():
        (bp1, r1), (bp2, r2), (full, r3) = job.if_rate[name]
        stations.append({
            "station_name": name, "lat": lat, "lng": lng,
            "charger_type": job.station_charger_type.get(name, "fast"),
            "break_point_1": bp1, "bp1_time": r1 * bp1,
            "break_point_2": bp2, "bp2_time": r2 * bp2,
            "full": full, "full_time": r3 * full,
        })
    return {"status": "success", "stations": stations}


@app.post("/find-route-battery/{job_id}/stations")
async def edit_stations(job_id: str, body: EditStationsRequest):
    """Adds/removes/edits charging stations on a PAUSED job. Always
    commits -- any route a removal/edit disrupts is repaired automatically
    (see _apply_station_edits); only individual customers that truly
    couldn't be placed anywhere come back in failed_to_place."""
    job = _get_job_or_404(job_id)
    if job.status != "paused":
        raise HTTPException(status_code=400, detail=f"Job must be paused before editing stations (status={job.status})")
    with job.edit_lock:
        try:
            summary = _apply_station_edits(job, body)
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            raise HTTPException(status_code=500, detail=f"Error applying station edits: {e}")
    return {"status": "success", **summary}


@app.get("/find-route-battery/{job_id}/status")
async def job_status(job_id: str):
    job = _get_job_or_404(job_id)
    return {
        "status": "success",
        "job_status": job.status,
        "iteration": job.it2,
        "max_iterations": job.max_iter,
        "best_cost": job.best_cost,
        "best_cost_history": job.best_cost_history,
        "error": job.error,
    }


@app.get("/find-route-battery/{job_id}/result")
async def job_result(job_id: str):
    job = _get_job_or_404(job_id)
    if job.status == "failed":
        raise HTTPException(status_code=500, detail=job.error or "Job failed")
    if job.status != "completed":
        raise HTTPException(status_code=409, detail=f"Job not completed yet (status={job.status})")
    return {"status": "success", "data": job.result}


@app.get("/health")
async def health():
    return {"status": "ok"}


if __name__ == "__main__":
    # alns/main.py's own --mode api path hardcodes port=8000 regardless of
    # any --port flag (there isn't one) -- this one actually honors it, so
    # `nohup python3 api_server.py --mode api --port 8001 &` on a server
    # works as typed instead of needing `uvicorn api_server:app --port
    # 8001` directly.
    import argparse
    import uvicorn

    parser = argparse.ArgumentParser(description="ALNS Battery Route Optimizer API")
    parser.add_argument("--mode", choices=["api"], default="api", help="Run mode (only 'api' is supported)")
    parser.add_argument("--port", type=int, default=8001, help="Port to listen on")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to bind to")
    args = parser.parse_args()

    uvicorn.run(app, host=args.host, port=args.port)
