import pandas as pd
import re, math, sys
import json
import urllib.request

#--------------------------------------Phraseing_input_data--------------------------------------------------------------------------------------#
def read_input(fname):
	data= pd.read_table(fname,skipfooter=5, engine='python',sep=r'\s+')
	df_len = len(data)
	customers, recharge_stns, depots = {}, {}, {}
	time_window, demand = {}, {}
	if_rate = {}
	for iid, i in enumerate(data['StringID']):
		if 'C' in i:
			customers.update({i:(data['x'][iid],data['y'][iid])})
			time_window.update({i:(data['ReadyTime'][iid],data['DueDate'][iid], data['ServiceTime'][iid])}) # Actual time windows
#			time_window.update({i:(0,10, data['ServiceTime'][iid])}) # for montoya case
			demand.update({i:data['demand'][iid]})
		elif 'S' in i:
			recharge_stns.update({i:(data['x'][iid],data['y'][iid])})
			if_rate.update({i:[(data['BreakPoint1'][iid],data['BP1Time'][iid]/data['BreakPoint1'][iid]),(data['BreakPoint2'][iid],data['BP2Time'][iid]/data['BreakPoint2'][iid]),(data['Full'][iid],data['FTime'][iid]/data['Full'][iid])]})
		else:
			time_window.update({i:(0,10, data['ServiceTime'][iid])})
			depots.update({i:(data['x'][iid],data['y'][iid])})

	add_data = pd.read_table(fname,skiprows=df_len+1, engine='python', header=None)
	stri = [str(list(add_data.iloc[0,])[1])]
	for row in range(1,len(add_data)):
		stri.append(str(list(add_data.iloc[row,])[1]))
	fuel_cap, load_cap, fc_alpha, fc_beta, avg_speed = [float(re.findall(r'\/([^]]*)\/', i)[0]) for i in stri]

	(dist_mat_cc, dist_mat_ccs, dist_mat_cd, dist_mat_csd,
	 dist_mat_cc_time, dist_mat_ccs_time, dist_mat_cd_time, dist_mat_csd_time) = distance_power_matrix(customers, recharge_stns, depots, avg_speed)
	return (customers, recharge_stns, depots, demand, time_window, fuel_cap, load_cap, fc_alpha, fc_beta, if_rate, avg_speed,
		dist_mat_cc, dist_mat_ccs, dist_mat_cd, dist_mat_csd,
		dist_mat_cc_time, dist_mat_ccs_time, dist_mat_cd_time, dist_mat_csd_time)

#--------------------------------------Computing_distances--------------------------------------------------------------------------#
# avg_speed here derives a travel-TIME matrix alongside the Euclidean
# distance matrix (distance/avg_speed, precomputed once instead of divided
# inline wherever a leg's time is needed) -- this only exists so the batch
# script has the same "distance + time matrices" interface the OSRM-backed
# API path provides below, with the exact same distance/avg_speed behavior
# it always had, now computed once instead of 58 times.

def distance_power_matrix(cus, charge_stn, dpt, avg_speed):
	nb_cus, nd_crg_stn, nb_dpt = len(cus), len(charge_stn), len(dpt)
	dist_mat_cc = {i:{} for i in cus.keys()}
	dist_mat_ccs = {i:{} for i in cus.keys()}
	dist_mat_cd = {i:{} for i in cus.keys()}
	dist_mat_csd = {i:{} for i in charge_stn.keys()}
	dist_mat_cc_time = {i:{} for i in cus.keys()}
	dist_mat_ccs_time = {i:{} for i in cus.keys()}
	dist_mat_cd_time = {i:{} for i in cus.keys()}
	dist_mat_csd_time = {i:{} for i in charge_stn.keys()}
	temp_cus = list(cus.keys())
	for orgn in cus.keys():
		for dstn in temp_cus:
			di_cc = math.sqrt(math.pow(cus[orgn][0]-cus[dstn][0], 2) + math.pow(cus[orgn][1]-cus[dstn][1], 2))
			dist_mat_cc[orgn].update({dstn: di_cc})
			dist_mat_cc[dstn].update({orgn: di_cc})
			dist_mat_cc_time[orgn].update({dstn: di_cc/avg_speed})
			dist_mat_cc_time[dstn].update({orgn: di_cc/avg_speed})
			if orgn == dstn:
				dist_mat_cc[orgn].update({dstn:0})
				dist_mat_cc_time[orgn].update({dstn:0})
		temp_cus.remove(orgn)

		for dstn in charge_stn.keys():
			di_ccs=math.sqrt(math.pow(cus[orgn][0]-charge_stn[dstn][0], 2) + math.pow(cus[orgn][1]-charge_stn[dstn][1], 2))
			dist_mat_ccs[orgn].update({dstn: di_ccs})
			dist_mat_ccs_time[orgn].update({dstn: di_ccs/avg_speed})

		for dstn in dpt.keys():
			di_cd=math.sqrt(math.pow(cus[orgn][0]-dpt[dstn][0], 2) + math.pow(cus[orgn][1]-dpt[dstn][1], 2))
			dist_mat_cd[orgn].update({dstn: di_cd})
			dist_mat_cd_time[orgn].update({dstn: di_cd/avg_speed})

	for orgn in charge_stn.keys():
		for dstn in dpt.keys():
			di_csd=math.sqrt(math.pow(charge_stn[orgn][0]-dpt[dstn][0], 2) + math.pow(charge_stn[orgn][1]-dpt[dstn][1], 2))
			dist_mat_csd[orgn].update({dstn: di_csd})
			dist_mat_csd_time[orgn].update({dstn: di_csd/avg_speed})

	return (dist_mat_cc, dist_mat_ccs, dist_mat_cd, dist_mat_csd,
		dist_mat_cc_time, dist_mat_ccs_time, dist_mat_cd_time, dist_mat_csd_time)


#--------------------------------------Computing_distances_via_OSRM----------------------------------------------------------------#
# Mirrors alns/reading_input.py's OSRM /table integration exactly (same
# self-hosted instance, same chunking idea to stay under its ~100-coordinate
# cap per request, same OSRM_TIME_MULTIPLIER correction), extended from
# diesel's 2 node types (customer, depot) to this project's 3 (customer,
# station, depot). Unlike distance_power_matrix above (kept for the
# standalone batch script, whose benchmark input files use abstract x/y
# units, not real coordinates), this expects actual (lat, lng) pairs.

# OSRM assumes free-flow/highway-speed driving, which systematically
# under-estimates real-world travel time (traffic, stops, turns) -- same
# correction factor diesel's reading_input.py applies to its own OSRM
# durations.
OSRM_TIME_MULTIPLIER = 2

def osrm_table_request(coords_lonlat, source_idx, dest_idx):
	"""One OSRM /table call restricted to source_idx/dest_idx within
	coords_lonlat. Returns (distances, durations) sized len(source_idx) x
	len(dest_idx), in meters and seconds respectively."""
	coords_str = ';'.join(coords_lonlat)
	sources_str = ';'.join(str(i) for i in source_idx)
	dests_str = ';'.join(str(i) for i in dest_idx)
	url = 'https://osrm.goodmove.in/table/v1/driving/' + coords_str + '?sources=' + sources_str + '&destinations=' + dests_str + '&annotations=distance,duration'
	req = urllib.request.Request(url)
	response = urllib.request.urlopen(req, timeout=30)
	resStr = json.loads(response.read().decode('utf-8'))
	if resStr.get('code') != 'Ok':
		raise RuntimeError(f"OSRM table error: {resStr.get('code')} - {resStr.get('message')}")
	return resStr['distances'], resStr['durations']


def distance_power_matrix_osrm(cus, charge_stn, dpt):
	"""Same (dist_mat_cc, dist_mat_ccs, dist_mat_cd, dist_mat_csd) shape as
	distance_power_matrix, sourced from OSRM road distances instead of
	Euclidean, plus the matching dist_mat_*_time matrices sourced from
	OSRM's own road travel time (x OSRM_TIME_MULTIPLIER) instead of
	distance/avg_speed. cus/charge_stn/dpt values must be (lat, lng)
	tuples."""
	cus_ids, stn_ids, dpt_ids = list(cus.keys()), list(charge_stn.keys()), list(dpt.keys())
	all_ids = cus_ids + stn_ids + dpt_ids
	cus_set, stn_set, dpt_set = set(cus_ids), set(stn_ids), set(dpt_ids)

	lonlat = {i: f"{cus[i][1]},{cus[i][0]}" for i in cus_ids}
	lonlat.update({i: f"{charge_stn[i][1]},{charge_stn[i][0]}" for i in stn_ids})
	lonlat.update({i: f"{dpt[i][1]},{dpt[i][0]}" for i in dpt_ids})

	dist_mat_cc = {i: {} for i in cus_ids}
	dist_mat_ccs = {i: {} for i in cus_ids}
	dist_mat_cd = {i: {} for i in cus_ids}
	dist_mat_csd = {i: {} for i in stn_ids}
	dist_mat_cc_time = {i: {} for i in cus_ids}
	dist_mat_ccs_time = {i: {} for i in cus_ids}
	dist_mat_cd_time = {i: {} for i in cus_ids}
	dist_mat_csd_time = {i: {} for i in stn_ids}

	CHUNK = 50
	chunks = [all_ids[i:i + CHUNK] for i in range(0, len(all_ids), CHUNK)]

	for a in chunks:
		for b in chunks:
			needed_ids = a if a == b else a + b
			if len(needed_ids) < 2:
				continue
			local_coords = [lonlat[i] for i in needed_ids]
			local_index = {node_id: k for k, node_id in enumerate(needed_ids)}
			source_idx = [local_index[i] for i in a]
			dest_idx = [local_index[i] for i in b]
			distances, durations = osrm_table_request(local_coords, source_idx, dest_idx)
			for si, i in enumerate(a):
				for sj, j in enumerate(b):
					if i == j:
						continue
					dist = distances[si][sj]
					dur = durations[si][sj]
					if dist is None or dur is None:
						continue
					dist_km = dist / 1000
					dur_hours = (dur / 3600) * OSRM_TIME_MULTIPLIER
					if i in cus_set and j in cus_set:
						dist_mat_cc[i][j] = dist_km
						dist_mat_cc_time[i][j] = dur_hours
					elif i in cus_set and j in stn_set:
						dist_mat_ccs[i][j] = dist_km
						dist_mat_ccs_time[i][j] = dur_hours
					elif i in cus_set and j in dpt_set:
						dist_mat_cd[i][j] = dist_km
						dist_mat_cd_time[i][j] = dur_hours
					elif i in stn_set and j in dpt_set:
						dist_mat_csd[i][j] = dist_km
						dist_mat_csd_time[i][j] = dur_hours
					# station->customer, depot->customer, depot->station,
					# station->station: not read anywhere in ALNS.py, skipped.

	for i in cus_ids:
		dist_mat_cc[i][i] = 0
		dist_mat_cc_time[i][i] = 0

	return (dist_mat_cc, dist_mat_ccs, dist_mat_cd, dist_mat_csd,
		dist_mat_cc_time, dist_mat_ccs_time, dist_mat_cd_time, dist_mat_csd_time)
