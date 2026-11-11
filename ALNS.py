import math, random, copy, time, sys, yaml, itertools
import numpy
import _pickle as cPickle
import operator, heapq, copy
# import gc

_dispatcher = {}

def _copy_list(l, dispatch):
	ret = l.copy()
	for idx, item in enumerate(ret):
		cp = dispatch.get(type(item))
		if cp is not None:
			ret[idx] = cp(item, dispatch)
	return ret

def _copy_dict(d, dispatch):
	ret = d.copy()
	for key, value in ret.items():
		cp = dispatch.get(type(value))
		if cp is not None:
			ret[key] = cp(value, dispatch)

	return ret

_dispatcher[list] = _copy_list
_dispatcher[dict] = _copy_dict

def deepcopy(sth):
	cp = _dispatcher.get(type(sth))
	if cp is None:
		return sth
	else:
		return cp(sth, _dispatcher)
#-------------------------------------------------------------------cost_function----------------------------------------------------------------------
'''
Cost function calcuates the total travel time and charging time
if you use Cost objective is distance minimization
else if you use tcost objective is to minimize the total time
'''
def most_frequent(List): 
    dict = {} 
    count, itm = 0, '' 
    for item in reversed(List): 
        dict[item] = dict.get(item, 0) + 1

    if len(dict):
    	return (max(dict.values())) 
    return 0

#-------------------------------------------------------------------------------------------------------------------------------------------------------
def cost_function(sol, depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, stn_iden_map, total_customer, avg_speed, time_window, demand, load_cap, total):
	le = len(sol)
	load = [[] for _ in range(le)]
	cost = [0 for _ in range(le)]
	tcost = [0 for _ in range(le)]
	total_customer_visited = len(sum(sol, []))

	if total_customer_visited < total_customer:
		return (10**10, [], 10**10)
	if total:
		csck = []
		for i in stn_iden_map:
			if stn_iden_map[i]:
				csck += list(stn_iden_map[i].values()) 

		cschkl = {}
		for cv in csck:
			if cv[4] in cschkl.keys():
				orl = cschkl[cv[4]]
				orl.append([cv[0]-cv[7],cv[0]])
				orl.sort(key = lambda x: x[0]) 
			else:
				orl = [[cv[0]-cv[7],cv[0]]]
			cschkl.update({cv[4]:orl})
		a = []
		for csv,csk in cschkl.items():
			for kl in csk:
				a += [round(lm,2) for lm in numpy.arange(round(kl[0],2), round(kl[1],2), 0.01)]
					
			if most_frequent(a) >3:
				return (10**10, [], 10**10)

	for vehid, veh in enumerate(sol):
		cupos = 0
		while cupos < len(veh):
			next_customer = veh[cupos]
			if cupos == 0 and depot in stn_iden_map[vehid].keys():
				dist = dist_mat_csd[stn_iden_map[vehid][depot][4]][depot]+dist_mat_ccs[next_customer][stn_iden_map[vehid][depot][4]]
				cost[vehid] += dist
				tcost[vehid] += dist_mat_csd_time[stn_iden_map[vehid][depot][4]][depot]+dist_mat_ccs_time[next_customer][stn_iden_map[vehid][depot][4]]+stn_iden_map[vehid][depot][7]

			elif cupos ==0: ### check if a station  is visited before this customer if not enter
				dist = dist_mat_cd[next_customer][depot]
				cost[vehid] += dist
				tcost[vehid] += dist_mat_cd_time[next_customer][depot]
			elif veh[cupos-1] in stn_iden_map[vehid].keys():
				dist = dist_mat_ccs[veh[cupos-1]][stn_iden_map[vehid][veh[cupos-1]][4]]+dist_mat_ccs[next_customer][stn_iden_map[vehid][veh[cupos-1]][4]]
				cost[vehid] += dist
				tcost[vehid] += dist_mat_ccs_time[veh[cupos-1]][stn_iden_map[vehid][veh[cupos-1]][4]]+dist_mat_ccs_time[next_customer][stn_iden_map[vehid][veh[cupos-1]][4]]+stn_iden_map[vehid][veh[cupos-1]][7]

			else:
				dist = dist_mat_cc[veh[cupos-1]][next_customer]
				cost[vehid] += dist
				tcost[vehid] += dist_mat_cc_time[veh[cupos-1]][next_customer]
			cupos += 1

		if veh:
			next_customer = veh[-1]
			if next_customer in stn_iden_map[vehid].keys():
				dist = dist_mat_ccs[next_customer][stn_iden_map[vehid][next_customer][4]]+dist_mat_csd[stn_iden_map[vehid][next_customer][4]][depot]
				cost[vehid] += dist
				tcost[vehid] += dist_mat_ccs_time[next_customer][stn_iden_map[vehid][next_customer][4]]+dist_mat_csd_time[stn_iden_map[vehid][next_customer][4]][depot]+stn_iden_map[vehid][next_customer][7]
			else:
				dist = dist_mat_cd[next_customer][depot]
				cost[vehid] += dist
				tcost[vehid] += dist_mat_cd_time[next_customer][depot]

	sum_cost = sum(cost)
	veh_used = sum([1 for i in cost if i>0])
	time_cost = sum(tcost)
	return (sum_cost, cost, time_cost)
#--------------------------------------------------------------------END_cost_function-----------------------------------------------------------------
#-------------------------------------------------------------------roulette_wheel_selection-----------------------------------------------------------
def roulette_wheel(w_list,n):
	su = sum(w_list)
	for idx, i in enumerate(w_list):
		if n<=i/su:
			return idx
		else:
			w_list[idx+1] += i
#-----------------------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------customer_removal_operations------------------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------------------------------------
def removal_time_gain(s, time_window):
	gain =[]
	for idx, val in enumerate(s):
		for idx2 in range(len(val)):
			t = time_window[val[idx2]][1]-time_window[val[idx2]][0]
			gain.append([t,val[idx2]])

	return gain
#-----------------------------------------------------------------------------------------------------------------------------------------------------
def removal_gain(s, stn_map, dist, dist_cd, dist_cs, dist_csd, depot, initial_cost):
	gain =[]
	for ind,val in enumerate(s):
		if len(val)==0:
			continue
		elif len(val) == 1:
			gain.append([-initial_cost[val[0]], val[0]])
		else:
			for idx in range(len(val)-1):
				if idx == 0:
					if depot in stn_map[ind].keys():
						t = (dist_cs[val[idx+1]][stn_map[ind][depot][4]]-dist[val[idx]][val[idx+1]]-dist_cs[val[idx]][stn_map[ind][depot][4]])
					else:
						t = (-dist_cd[val[idx]][depot]+dist_cd[val[idx+1]][depot]-dist[val[idx+1]][val[idx]])

				else:
					if val[idx-1] in stn_map[ind].keys():
						t = (-dist_cs[val[idx]][stn_map[ind][val[idx-1]][4]]-dist[val[idx+1]][val[idx]]+dist_cs[val[idx+1]][stn_map[ind][val[idx-1]][4]])
					else:
						t = (-dist[val[idx]][val[idx-1]]-dist[val[idx]][val[idx+1]]+dist[val[idx+1]][val[idx-1]])

				gain.append([t,val[idx]])

			if val[-1] in stn_map[ind].keys():
				t = (-dist_cs[val[-1]][stn_map[ind][val[-1]][4]]+dist_cs[val[-2]][stn_map[ind][val[-1]][4]]-dist[val[-1]][val[-2]])
				gain.append([t,val[-1]])
			else:
				t = (-dist_cd[val[-1]][depot]-dist[val[-1]][val[-2]]+dist_cd[val[-2]][depot])
				gain.append([t,val[-1]])

	return gain
#---------------------------------------------------------customer_removal_operations_main_function---------------------------------------------------
def customer_removal_operation(a,s_sol, depot, time_window, in_map, battery_cap, battery, tw_check, stn_iden_map, avg_speed, fc_alpha, fc_beta, if_rate, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, initial_cost, recharge_stns, demand, load, load_cap):
	q_min = min(30, int(len(in_map.keys())*0.1))
	q_max = min(60, int(len(in_map.keys())*0.4))
	q = random.randint(q_min, q_max)
	c_pool = []
	route_pool = {i:[] for i in range(len(s_sol))}
	o_s_sol = deepcopy(s_sol)
	if a == 0: ## Random customer removal
		c_pool = random.sample(list(in_map.keys()), q)
		for c in c_pool:
			vid = in_map[c]
			del tw_check[vid][c]
			battery[vid].remove(battery[vid][s_sol[vid].index(c)])
			s_sol[vid].remove(c)
			route_pool[vid].append(c)
			del in_map[c]
		s_sol, in_map, battery, stn_iden_map, tw_check, load = update_all_fields(route_pool, in_map, o_s_sol, battery, battery_cap, tw_check, time_window, stn_iden_map, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, avg_speed, fc_alpha, fc_beta, if_rate, depot, demand, load, load_cap)

	elif a == 1: ## worst distance removal
		while len(c_pool) < q:
			gain = removal_gain(s_sol, stn_iden_map, dist_mat_cc, dist_mat_cd, dist_mat_ccs, dist_mat_csd, depot, initial_cost)
			gain_sorted = sorted(gain)
			c = gain_sorted[0][1]
			vid = in_map[c]
			c_pool.append(c)
			del tw_check[vid][c]
			battery[vid].remove(battery[vid][s_sol[vid].index(c)])
			route_pool[vid].append(c)
			del in_map[c]
			s_sol[vid].remove(c)

		s_sol, in_map, battery, stn_iden_map, tw_check, load = update_all_fields(route_pool, in_map, o_s_sol, battery, battery_cap, tw_check, time_window, stn_iden_map, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, avg_speed, fc_alpha, fc_beta, if_rate, depot, demand, load, load_cap)
	elif a == 2: ## related removal
		seed = random.sample(list(in_map.keys()),1)[0]
		c_pool = []
		q -= 1
		dist = dist_mat_cc[seed]
		dist_sorted = sorted(dist.items(), key=operator.itemgetter(1))[1:]
		if q:
			temp_c = [i[0] for idx, i in enumerate(dist_sorted) if idx <= q]
			c_pool.extend(temp_c)
		for c in c_pool:
			vid = in_map[c]
			del tw_check[vid][c]
			battery[vid].remove(battery[vid][s_sol[vid].index(c)])
			s_sol[vid].remove(c)
			route_pool[vid].append(c)
			del in_map[c]
		s_sol, in_map, battery, stn_iden_map, tw_check, load = update_all_fields(route_pool, in_map, o_s_sol, battery, battery_cap, tw_check, time_window, stn_iden_map, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, avg_speed, fc_alpha, fc_beta, if_rate, depot, demand,load, load_cap)
	elif a== 5: # station related customer removal
		seed = random.sample(list(recharge_stns.keys()),1)[0]
		flat_stn = [[key, val] for pkey in stn_iden_map.keys() if stn_iden_map[pkey] for key, val in stn_iden_map[pkey].items()]
		stn_iden_map = {vehid:{} for vehid in range(len(s_sol))}
		for (k, val) in flat_stn:
			if val[4] != seed:
				stn_iden_map[val[3]].update({k:val})
		c_pool = []
		dist = [[i,dist_mat_ccs[i][seed]] for i in dist_mat_ccs.keys()]
		dist_sorted = sorted(dist, key=operator.itemgetter(1))
		if q:
			temp_c = [i[0] for idx, i in enumerate(dist_sorted) if idx <= q]
			c_pool.extend(temp_c)
		for c in c_pool:
			vid = in_map[c]
			del tw_check[vid][c]
			battery[vid].remove(battery[vid][s_sol[vid].index(c)])
			s_sol[vid].remove(c)
			route_pool[vid].append(c)
			del in_map[c]
		s_sol, in_map, battery, stn_iden_map, tw_check,load = update_all_fields(route_pool, in_map, o_s_sol, battery, battery_cap, tw_check, time_window, stn_iden_map, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, avg_speed, fc_alpha, fc_beta, if_rate, depot, demand, load, load_cap)
	elif a == 3: ## random route removal
		w = random.randint(int(0.1*len(s_sol)),int(len(s_sol)*0.4))
		q_list = random.sample(range(len(s_sol)),w)
		c_pool = []
		for q in q_list:
			c_pool.extend(s_sol[q])
			s_sol[q], battery[q], tw_check[q], stn_iden_map[q], load[q] = [], [], {}, {}, []
		for c in c_pool:
			del in_map[c]
	elif a == 4: ## greddy route removal
		w = random.randint(int(0.1*len(s_sol)),int(len(s_sol)*0.4))
		gr_list = sorted([(len(veh), vid) for vid, veh in enumerate(s_sol) if veh], key = lambda x: x[0])[:w]
		q_list = [gl[1] for gl in gr_list]
		c_pool = []
		for q in q_list:
			c_pool.extend(s_sol[q])
			s_sol[q], battery[q], tw_check[q], stn_iden_map[q], load[q] = [], [], {}, {}, []
		for c in c_pool:
			del in_map[c]
	return(c_pool, s_sol, in_map, stn_iden_map, tw_check, battery, load)
#-----------------------------------------------------------END_customer_removal_operations---------------------------------------------------
def update_all_fields(route_pool, in_map, s_sol, battery, battery_cap, tw_check, time_window, stn_iden_map, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, avg_speed, fc_alpha, fc_beta, if_rate, depot, demand, load, load_cap):
	for vsid, vs in enumerate(s_sol):
		if len(vs) > 0 and len(route_pool[vsid])>0 and len(vs)>len(route_pool[vsid]):
			for v in route_pool[vsid]:
				if v in stn_iden_map[vsid].keys():
					if depot == stn_iden_map[vsid][v][5]:
						new_cus = s_sol[vsid][-2]
					else:
						ind = s_sol[vsid].index(v)-1
						if ind < 0:
							new_cus = depot
						else:
							new_cus = s_sol[vsid][ind]
					stn_iden_map[vsid].update({new_cus:stn_iden_map[vsid][v]})
					del stn_iden_map[vsid][v]

				elif depot in stn_iden_map[vsid].keys():
					if v == stn_iden_map[vsid][depot][5]:
						stn_iden_map[vsid][depot][5] = s_sol[vsid][1]
				for k,kv in stn_iden_map[vsid].items():
					if kv[5] == v:
						ind = s_sol[vsid].index(v)+1
						if ind >= len(s_sol[vsid]) :
							new_cus = depot
						else:
							new_cus = s_sol[vsid][ind]
						stn_iden_map[vsid][k][5] = new_cus
						break
				s_sol[vsid].remove(v)

			if len(s_sol[vsid]) == 1:
				stn_iden_map, battery, tw_check, load = one_cus_sol(s_sol[vsid][0], dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, avg_speed, if_rate, depot, time_window, vsid, battery_cap, battery, stn_iden_map, tw_check, load, demand)
			elif len(s_sol[vsid]) > 1:
				battery[vsid], bat_dpt, stn_iden_map[vsid], load[vsid] = battery_update(s_sol[vsid], stn_iden_map[vsid], dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate, vsid, avg_speed, demand)
				out, tw_check[vsid], stn_iden_map[vsid] = tw_update(s_sol[vsid], battery_cap, stn_iden_map[vsid], fc_alpha, fc_beta, if_rate, avg_speed, time_window, depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, battery[vsid], vsid, load[vsid])
				if out == 'fs_check':
					out2, ba, tw, st, ld = stn_insertion([s_sol[vsid]], [battery[vsid]], [tw_check[vsid]], {0:stn_iden_map[vsid]}, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate, time_window, 0, avg_speed, load, demand, load_cap)
					battery[vsid], tw_check[vsid], stn_iden_map[vsid], load[vsid] = ba[0], tw[0], st[0], ld[0]
			elif not s_sol[vsid]:
				stn_iden_map[vsid] = {}
				load[vsid] = []

		elif len(vs) == 0 or len(vs)==len(route_pool[vsid]):
			s_sol[vsid] = []
			stn_iden_map[vsid] = {}
			load[vsid] = []

	return s_sol, in_map, battery, stn_iden_map, tw_check, load

#-----------------------------------------------------------------------------------------------------------------------------------------------------
def battery_update(fs_sol, fs_stn_map, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate, veh, avg_speed, demand):
	count = 1
	battery_tmp = 0
	tw_check = {}
	bat_dpt = 0
	iid = len(fs_sol)
	batt = [0 for _ in fs_sol]
	ldcount = 1
	if fs_sol:
		tload = []
		for cu in fs_sol[::-1]:
			if ldcount:
				tload.append(demand[cu])
				ldcount = 0
			else:
				tload.append(tload[-1]+demand[cu])
		fload = tload[::-1]

		for i in fs_sol[::-1]:
			iid -= 1
			if count: # strating from depot
				if i in list(fs_stn_map.keys()):
					near_rech_stn = fs_stn_map[i][4]
					battery_tmp = dist_mat_ccs[i][near_rech_stn]*fc_alpha
					fs_stn_map[i][1] = dist_mat_ccs[i][fs_stn_map[i][4]]*fc_alpha
					fs_stn_map[i][2] = 0
					fs_stn_map[i][5] = depot
					fs_stn_map[i][6] = dist_mat_csd[fs_stn_map[i][4]][depot]*fc_alpha
					fs_stn_map[i][7] = charge_calc(dist_mat_csd[fs_stn_map[i][4]][depot]*fc_alpha, if_rate, fs_stn_map[i][4],0, battery_cap)
				else:
					battery_tmp = dist_mat_cd[i][depot]*fc_alpha
				batt[iid] = battery_tmp
				count = 0
			elif i in list(fs_stn_map.keys()): # if any recharge station is visited
				battery_tmp = batt[iid+1]+dist_mat_ccs[fs_sol[iid+1]][fs_stn_map[i][4]]*fc_alpha+dist_mat_ccs[fs_sol[iid+1]][fs_stn_map[i][4]]*fc_beta*fload[iid+1]
				if battery_tmp > battery_cap:
					del fs_stn_map[i]
					bat = dist_mat_cc[fs_sol[iid+1]][i]*fc_alpha+dist_mat_cc[fs_sol[iid+1]][i]*fc_beta*fload[iid+1]
					batt[iid] = batt[iid+1]+bat
				else:
					fs_stn_map[i][1] = dist_mat_ccs[i][fs_stn_map[i][4]]*fc_alpha+dist_mat_ccs[i][fs_stn_map[i][4]]*fc_beta*fload[iid+1]
					fs_stn_map[i][2] = 0
					fs_stn_map[i][5] = fs_sol[iid+1]
					fs_stn_map[i][6] = battery_tmp
					fs_stn_map[i][7] = charge_calc(battery_tmp, if_rate, fs_stn_map[i][4],0, battery_cap)
					batt[iid] = dist_mat_ccs[i][fs_stn_map[i][4]]*fc_alpha+dist_mat_ccs[i][fs_stn_map[i][4]]*fc_beta*fload[iid+1]

			else:
				bat = dist_mat_cc[fs_sol[iid+1]][i]*fc_alpha+dist_mat_cc[fs_sol[iid+1]][i]*fc_beta*fload[iid+1]
				batt[iid] = batt[iid+1]+bat
		i = fs_sol[0]
		bat_dpt = 0
		if depot in list(fs_stn_map.keys()):
			near_rech_stn = fs_stn_map[depot][4]
			if batt[0]+dist_mat_cd[i][depot]*fc_alpha+dist_mat_cd[i][depot]*fc_beta*fload[0] < battery_cap:
				del fs_stn_map[depot]
			else:
				bat_dpt = batt[0]+dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*fload[0]
				if bat_dpt > battery_cap:
					del fs_stn_map[depot]
					return batt, bat_dpt, fs_stn_map, fload
				fs_stn_map[depot][1] = battery_cap
				fs_stn_map[depot][2] = battery_cap-dist_mat_csd[fs_stn_map[depot][4]][depot]*fc_alpha-dist_mat_csd[fs_stn_map[depot][4]][depot]*fc_beta*fload[0]
				fs_stn_map[depot][5] = i
				fs_stn_map[depot][6] = bat_dpt
				fs_stn_map[depot][7] = charge_calc(bat_dpt, if_rate, fs_stn_map[depot][4],battery_cap-dist_mat_csd[fs_stn_map[depot][4]][depot]*fc_alpha-dist_mat_csd[fs_stn_map[depot][4]][depot]*fc_beta*fload[0], battery_cap)
		else:
			battery_tmp = batt[0]+dist_mat_cd[i][depot]*fc_alpha+dist_mat_cd[i][depot]*fc_beta*fload[0]
			bat_dpt = battery_tmp
			if battery_tmp > battery_cap:
				near_rech_stn = min(dist_mat_ccs[i], key=dist_mat_ccs[i].get)
				if batt[0]+dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*fload[0] > battery_cap:
					return batt, battery_tmp+dist_mat_cd[i][depot]*fc_alpha+dist_mat_cd[i][depot]*fc_beta*fload[0], fs_stn_map, fload
				else:
					battery = batt[0]+dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*fload[0]
					bat_avl = battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*fload[0]
					ctime = charge_calc(battery, if_rate, near_rech_stn, bat_avl, battery_cap)
					leaving = dist_mat_csd_time[near_rech_stn][depot]+ctime
					fs_stn_map.update({depot:[leaving,battery_cap,battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*fload[0],veh,near_rech_stn,i,battery, ctime]})

	return batt, bat_dpt, fs_stn_map, fload
#------------------------------------------------------------------------------------------------------------------------------------------
def charge_calc(battery_tmp, if_rate, near_rech_stn, bat_avl, battery_cap):
	if bat_avl < 0:
		return 10*10
	if bat_avl > 0.95*battery_cap:
		ctime = (battery_tmp-bat_avl)*if_rate[near_rech_stn][2][1]
	elif bat_avl > 0.85*battery_cap:
		if battery_tmp > 0.95*battery_cap:
			ctime = (0.95*battery_cap-bat_avl)*if_rate[near_rech_stn][1][1]+(battery_tmp-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
		else:
			ctime = (battery_tmp-bat_avl)*if_rate[near_rech_stn][1][1]
	else:
		if battery_tmp >0.95*battery_cap:
			ctime = (0.85*battery_cap-bat_avl)*if_rate[near_rech_stn][0][1]+0.1*battery_cap*if_rate[near_rech_stn][1][1]+(battery_tmp-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
		elif battery_tmp > 0.85*battery_cap:
			ctime = (0.85*battery_cap-bat_avl)*if_rate[near_rech_stn][0][1]+(battery_tmp-0.85*battery_cap)*if_rate[near_rech_stn][1][1]
		else:
			ctime = (battery_tmp-bat_avl)*if_rate[near_rech_stn][0][1]
	return ctime
#------------------------------------------------------------------------------------------------------------------------------------------
def tw_update(fs_sol, battery_cap, fs_stn_map, fc_alpha, fc_beta, if_rate, avg_speed, time_window, depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, batt, veh, fload):
	#arrival time update
	tw_check = {}
	for iid, i in enumerate(fs_sol):
		if iid ==0 and depot in list(fs_stn_map.keys()):
			near_rech_stn = fs_stn_map[depot][4]
			atime = dist_mat_csd_time[near_rech_stn][depot]
			# batt[iid] = fs_stn_map[depot][6]-dist_mat_ccs[i][near_rech_stn]*fc_alpha-dist_mat_ccs[i][near_rech_stn]*fc_beta*fload[iid]
			ctime = fs_stn_map[depot][7]
			atime += ctime+dist_mat_ccs_time[i][near_rech_stn]
			if atime < time_window[i][0]:
				atime = time_window[i][0]
			tw_check.update({i:atime+time_window[i][2]})
		elif iid == 0:
			# batt[iid] = battery_cap-dist_mat_cd[i][depot]*fc_alpha-dist_mat_cd[i][depot]*fc_beta*fload[iid]
			atime = dist_mat_cd_time[i][depot]
			if atime < time_window[i][0]:
				atime = time_window[i][0]
			tw_check.update({i:atime+time_window[i][2]})
		elif fs_sol[iid-1] in list(fs_stn_map.keys()):
			near_rech_stn = fs_stn_map[fs_sol[iid-1]][4]
			atime = tw_check[fs_sol[iid-1]]+dist_mat_ccs_time[fs_sol[iid-1]][near_rech_stn]
			bat_req = fs_stn_map[fs_sol[iid-1]][6]
			bat_rem = batt[iid-1]-dist_mat_ccs[fs_sol[iid-1]][near_rech_stn]*fc_alpha-dist_mat_ccs[fs_sol[iid-1]][near_rech_stn]*fc_beta*fload[iid]
			if bat_rem <-1E-7:
				return("fs_check", tw_check, fs_stn_map)
			ctime = charge_calc(bat_req, if_rate, near_rech_stn, bat_rem, battery_cap)
			fs_stn_map[fs_sol[iid-1]] = [atime+ctime,batt[iid-1], bat_rem, veh, near_rech_stn,i,bat_req,ctime]
			atime += ctime+dist_mat_ccs_time[i][near_rech_stn]
			if atime < time_window[i][0]:
				atime = time_window[i][0]
			# batt[iid] = fs_stn_map[fs_sol[iid-1]][6]-dist_mat_ccs[i][near_rech_stn]*fc_alpha-dist_mat_ccs[i][near_rech_stn]*fc_beta*fload[iid]
			tw_check.update({i:atime+time_window[i][2]})
		else:
			atime = tw_check[fs_sol[iid-1]]+dist_mat_cc_time[i][fs_sol[iid-1]]
			# batt[iid] = batt[iid-1]-dist_mat_cc[i][fs_sol[iid-1]]*fc_alpha-dist_mat_cc[i][fs_sol[iid-1]]*fc_beta*fload[iid]
			if atime < time_window[i][0]:
				atime = time_window[i][0]
			tw_check.update({i:atime+time_window[i][2]})
		if atime > time_window[i][1]:
			return("fs_check", tw_check, fs_stn_map)
	i = fs_sol[-1]
	if i in list(fs_stn_map.keys()):
		if batt[-1] >= dist_mat_cd[i][depot]*fc_alpha:
			del fs_stn_map[i]
			atime = tw_check[i]+dist_mat_cd_time[i][depot]
			if atime > time_window[depot][1]:
				return("fs_check",  tw_check, fs_stn_map)
			return('ok', tw_check, fs_stn_map)
		else:
			atime = tw_check[i]+dist_mat_ccs_time[i][fs_stn_map[i][4]]
			bat_rem = batt[-1]-dist_mat_ccs[i][fs_stn_map[i][4]]*fc_alpha
			if bat_rem < -1E-7:
				return("fs_check", tw_check, fs_stn_map)
			bat_req = dist_mat_csd[fs_stn_map[i][4]][depot]*fc_alpha
			ctime = charge_calc(bat_req, if_rate, fs_stn_map[i][4], bat_rem, battery_cap)
			fs_stn_map[i] = [atime+ctime,batt[-1],bat_rem, veh,fs_stn_map[i][4],depot,bat_req,ctime]
			atime += dist_mat_csd_time[fs_stn_map[i][4]][depot]+ctime
			if atime > time_window[depot][1]:
				return("fs_check", tw_check, fs_stn_map)
	else:
		atime = tw_check[i]+dist_mat_cd_time[i][depot]
		bat = batt[-1] - dist_mat_cd[i][depot]*fc_alpha
		if (atime > time_window[depot][1] or bat < -1E-7):
			return("fs_check", tw_check, fs_stn_map)

	return 'ok', tw_check, fs_stn_map
#-----------------------------------------------------------------------------------------------------------------------------------------------------
def one_cus_sol(en_customer, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, avg_speed, if_rate, depot, time_window, i, battery_cap, battery, stn_iden_map, tw_check, load, demand):
	stn_iden_map[i] = {}
	dist = dist_mat_cd[en_customer][depot]
	battery_discharged = dist*fc_alpha+dist*fc_beta*demand[en_customer]
	battery_discharged_end = dist*fc_alpha
	battery[i] = [battery_cap-battery_discharged]
	load[i] = [demand[en_customer]]
	arrival = dist_mat_cd_time[en_customer][depot]
	if arrival < time_window[en_customer][0]:
		arrival = time_window[en_customer][0]
	tw_check[i].update({en_customer: arrival+time_window[en_customer][2]})
	near_rech_stn = min(dist_mat_ccs[en_customer], key=dist_mat_ccs[en_customer].get)
	bat = battery_cap-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha-dist_mat_ccs[en_customer][near_rech_stn]*fc_beta*demand[en_customer]- battery_discharged
	if bat >= 0 and battery_discharged+battery_discharged_end > battery_cap:
		battery_req_dpt = dist_mat_csd[near_rech_stn][depot]*fc_alpha
		chtime = charge_calc(battery_req_dpt,if_rate,near_rech_stn, bat,battery_cap)
		leaving = arrival+dist_mat_ccs_time[en_customer][near_rech_stn]+chtime
		stn_iden_map[i].update({en_customer:[leaving,battery[i][0], bat,i,near_rech_stn, depot, battery_req_dpt, chtime]})

	elif bat < 0 and battery_discharged+battery_discharged_end > battery_cap :
		battery_req_stn = battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*demand[en_customer]
		chtime = charge_calc(battery_cap, if_rate, near_rech_stn, battery_req_stn, battery_cap)
		leaving = dist_mat_csd_time[near_rech_stn][depot]+chtime
		arrival = leaving+dist_mat_ccs_time[en_customer][near_rech_stn]
		stn_iden_map[i].update({depot:[leaving, battery_cap, battery_req_stn,i,near_rech_stn, en_customer, battery_cap, chtime]})
		battery[i][0] = battery_cap-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha-dist_mat_ccs[en_customer][near_rech_stn]*fc_beta*demand[en_customer]
		if arrival < time_window[en_customer][0]:
			arrival = time_window[en_customer][0]
		tw_check[i].update({en_customer:arrival+time_window[en_customer][2]})
		battery_req_stn = dist_mat_csd[near_rech_stn][depot]*fc_alpha
		bat = battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha
		chtime = charge_calc(battery_req_stn, if_rate, near_rech_stn, bat, battery_cap)
		leaving = time_window[en_customer][2]+arrival+dist_mat_ccs_time[en_customer][near_rech_stn]+chtime
		stn_iden_map[i].update({en_customer:[leaving,battery[i][0], bat,i,near_rech_stn, depot, battery_req_stn, chtime]})
		load[i] = [demand[en_customer],demand[en_customer],0]
	return stn_iden_map, battery, tw_check, load
#-----------------------------------------------------------END_customer_removal_operations----------------------------------------------------------
#---------------------------------------------------------Insertion_operation------------------------------------------------------------------------
def cheap_insertion(s, in_val, stn_map, dist, dist_cd, dist_cs, dist_csd, depot, initial_cost):
	gain = [0 for _ in range(len(s))]
	for ind,val in enumerate(s):
		temp = []
		if len(val)==0:
			temp.append([initial_cost[in_val],'NA', ind,'nc'])
		else:
			for idx in range(len(val)):
				if idx == 0:
					if depot in stn_map[ind].keys():
						t = (-dist_csd[stn_map[ind][depot][4]][depot]+dist_cd[in_val][depot]+dist_cs[in_val][stn_map[ind][depot][4]])
						temp.append([t,idx, ind, 'b'])
						t = (-dist_cs[val[idx]][stn_map[ind][depot][4]]+dist_cs[in_val][stn_map[ind][depot][4]]+dist[val[idx]][in_val])
						temp.append([t,idx, ind, 'a'])
						continue
					else:
						t = (-dist_cd[val[idx]][depot]+dist_cd[in_val][depot]+dist[val[idx]][in_val])
				else:
					if val[idx-1] in stn_map[ind].keys():
						t = (-dist_cs[val[idx]][stn_map[ind][val[idx-1]][4]]+dist[in_val][val[idx]]+dist_cs[in_val][stn_map[ind][val[idx-1]][4]])
						temp.append([t,idx, ind, 'a'])
						t = (-dist_cs[val[idx-1]][stn_map[ind][val[idx-1]][4]]+dist_cs[in_val][stn_map[ind][val[idx-1]][4]]+dist[val[idx-1]][in_val])
						temp.append([t,idx, ind, 'b'])
						continue
					else:
						t = (-dist[val[idx]][val[idx-1]]+dist[val[idx]][in_val]+dist[in_val][val[idx-1]])

				temp.append([t,idx, ind, 'nc'])
			if val[idx] in stn_map[ind].keys():
				t = (-dist_csd[stn_map[ind][val[idx]][4]][depot]+dist_cd[in_val][depot]+dist_cs[in_val][stn_map[ind][val[idx]][4]])
				temp.append([t,idx+1, ind, 'a'])
				t = (-dist_cs[val[idx]][stn_map[ind][val[idx]][4]]+dist_cs[in_val][stn_map[ind][val[idx]][4]]+dist[val[idx]][in_val])
				temp.append([t,idx+1, ind, 'b'])
			else:
				t = (-dist_cd[val[idx]][depot]+dist[val[idx]][in_val]+dist_cd[in_val][depot])
				temp.append([t,idx+1, ind, 'nc'])

		gain[ind] = temp

	return(gain)
#------------------------------------------------------------------------------------------------------------------------------------------------------
def regret_cost(s, c_p, stn_map, dist, dist_cd, dist_cs, dist_csd, depot, initial_cost):
	gain_all = {}
	for in_val in c_p:
		ga_temp = cheap_insertion(s, in_val, stn_map, dist, dist_cd, dist_cs, dist_csd, depot, initial_cost)
		gain_all.update({in_val:ga_temp})
	return(gain_all)
#------------------------------------------------------------------------------------------------------------------------------------------------------
def regret_time(s,c_p, dist_time, time_window):
	gain_all = {}
	for in_val in c_p:
		gain = [0 for _ in range(len(s))]
		for ind,val in enumerate(s):
			temp = [[] for _ in range(len(val))]
			if len(val)==0:
				temp.append([(time_window[in_val][1]-time_window[in_val][0])/100,'NA', ind,'nc'])
			else:
				for idx in range(len(val)):
					if idx == 0:
						t =  time_window[in_val][1]-time_window[in_val][0]
						leaving = time_window[in_val][0]+time_window[val[idx]][2]*2
						temp[idx] = [t,idx, ind,'nc']
					else:
						## replace leaving with tw_check
						arrival = leaving+dist_time[val[idx]][val[idx-1]]
						t = time_window[in_val][1]-arrival
						temp[idx] = [t,idx, ind,'nc']
						leaving = arrival+time_window[val[idx]][2]*2

			gain[ind] = temp
		gain_all.update({in_val:gain})
	return(gain_all)
#-----------------------------------------------------------------------------------------------------------------------------------------------------
def cheap_regret_in(c, gain_sorted, d, in_map, c_pool, p_sol, battery_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, depot, time_window, stn_iden_map, fc_alpha, fc_beta, if_rate,  avg_speed, battery, tw_check, load, load_cap):
	for j in gain_sorted:
		veh = j[2]
		if j[1] == "NA": # if customer is inserted in a newly created route
			p_sol[veh].append(c)
			in_map.update({c:veh})
			c_pool.remove(c)
			stn_iden_map, battery, tw_check, load = one_cus_sol(c, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, avg_speed, if_rate, depot, time_window, veh, battery_cap, battery, stn_iden_map, tw_check, load, d)
			return (p_sol, in_map, c_pool, stn_iden_map, battery, tw_check, load)

		else:
			pos = j[1]
			# gc.disable()
			fs_sol = deepcopy(p_sol[veh])
			fs_stn_map = deepcopy(stn_iden_map[veh])
			# gc.enable()
			if j[3] == 'a':
				if pos-1 < 0:
					fs_stn_map[depot][5] = c
				else:
					fs_stn_map[fs_sol[pos-1]][5] = c
			elif j[3] == 'b':
				if pos-1 < 0:
					fs_stn_map.update({c:fs_stn_map[depot]})
					del fs_stn_map[depot]
				else:
					fs_stn_map.update({c:fs_stn_map[fs_sol[pos-1]]})
					del fs_stn_map[fs_sol[pos-1]]
			fs_sol.insert(pos,c)
			output, batt, twc, stn_iden_map_tmp, fload = feasibility_check(fs_sol, battery_cap, fs_stn_map, fc_alpha, fc_beta, if_rate,  veh, avg_speed, time_window, depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, d, load_cap)
			if output == "Insert":
				p_sol[veh].insert(pos,c)
				stn_iden_map[veh] = stn_iden_map_tmp
				battery[veh] = batt
				tw_check[veh] = twc
				load[veh] = fload
				c_pool.remove(c)
				in_map.update({c:veh})
				return (p_sol, in_map, c_pool, stn_iden_map, battery, tw_check, load)

	else:
		c_pool.remove(c)
	return (p_sol, in_map, c_pool, stn_iden_map, battery, tw_check, load)
#------------------------------------------------------------------------------------------------------------------------------------------
def insertion_operation(b,c_pool,p_sol,depot, d,in_map, stn_iden_map, fc_alpha, fc_beta, if_rate, time_window, avg_speed, battery_cap, dist_mat_cd, dist_mat_cd_time, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery, tw_check, initial_cost, load, load_cap):
	if b == 0: # greedy insertion
		while len(c_pool) >0:
			c = random.sample(c_pool,1)[0]
			gain = cheap_insertion(p_sol, c, stn_iden_map, dist_mat_cc, dist_mat_cd, dist_mat_ccs, dist_mat_csd, depot, initial_cost)
			gain_sorted = sorted(list(itertools.chain.from_iterable(gain)), key = lambda x: x[0])
			p_sol, in_map, c_pool, stn_iden_map, battery, tw_check, load = cheap_regret_in(c, gain_sorted, d, in_map, c_pool, p_sol, battery_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, depot, time_window, stn_iden_map, fc_alpha, fc_beta, if_rate, avg_speed, battery, tw_check, load, load_cap)
	elif b ==1:# greedy insertion pertrubated
		while len(c_pool) >0:
			c = random.sample(c_pool,1)[0]
			gain = cheap_insertion(p_sol, c, stn_iden_map, dist_mat_cc, dist_mat_cd, dist_mat_ccs, dist_mat_csd, depot, initial_cost)
			ngain = [[[item[0]*random.uniform(0.8,1.2),item[1], item[2],item[3]] for item in sublist] for sublist in gain]
			gain_sorted = sorted(list(itertools.chain.from_iterable(ngain)), key = lambda x: x[0])
			p_sol, in_map, c_pool, stn_iden_map, battery, tw_check, load = cheap_regret_in(c, gain_sorted, d, in_map, c_pool, p_sol, battery_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, depot, time_window, stn_iden_map, fc_alpha, fc_beta, if_rate, avg_speed, battery, tw_check, load, load_cap)
	elif b == 2: # regret-2 insertion
		while len(c_pool) >0:
			gain = regret_cost(p_sol, c_pool, stn_iden_map, dist_mat_cc, dist_mat_cd, dist_mat_ccs, dist_mat_csd, depot, initial_cost)
			regret_value = []
			gain_sorted = {key:[] for key in gain.keys()}
			for idx, gl in gain.items():
				gain_sorted[idx] = sorted(list(itertools.chain.from_iterable(gl)), key = lambda x: x[0])
				regret_value.append([gain_sorted[idx][0][0]-gain_sorted[idx][1][0], idx])
			c_in = sorted(regret_value, reverse=True)[0][1]
			p_sol, in_map, c_pool, stn_iden_map, battery, tw_check, load = cheap_regret_in(c_in, gain_sorted[c_in], d, in_map, c_pool, p_sol, battery_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, depot, time_window, stn_iden_map, fc_alpha, fc_beta, if_rate, avg_speed, battery, tw_check, load, load_cap)

	elif b == 3: # regret time_window insertion
		while len(c_pool) >0:
			gain = regret_time(p_sol, c_pool, dist_mat_cc_time, time_window)
			regret_value = []
			gain_sorted = {key:[] for key in gain.keys()}
			for idx, gl in gain.items():
				gain_sorted[idx] = sorted(list(itertools.chain.from_iterable(gl)), key = lambda x: x[0], reverse= True)
				regret_value.append([gain_sorted[idx][0][0]-gain_sorted[idx][1][0],idx])
			c_in = sorted(regret_value, reverse = True)[0][1]
			p_sol, in_map, c_pool, stn_iden_map, battery, tw_check, load = cheap_regret_in(c_in, gain_sorted[c_in], d, in_map, c_pool, p_sol, battery_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, depot, time_window, stn_iden_map, fc_alpha, fc_beta, if_rate, avg_speed, battery, tw_check, load, load_cap)

	return(p_sol, in_map, stn_iden_map, battery, tw_check, load)
#---------------------------------------------------------END_Insertion_operation--------------------------------------------------------------------
# #--------------------------------------------------------------------------------------------------------------------------------------------------
# Assumptions:
# 1. Battery at the time of leaving depot is full
# 2. Battery is zero on arrival at any depot
# 3. Battery on arrival at any charging station is the charge that satisfies battery constraints and time window constraints of customers
#    after the current cs till next cs or depot
#----------------------------------------------------------------------------------------------------------------------------------------------------
def feasibility_check(fs_sol, battery_cap, fs_stn_map, fc_alpha, fc_beta, if_rate,  veh, avg_speed, time_window, depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, demand, load_cap):
	count, pcount = 1, 0
	battery = 0
	battery_tmp = 0
	tw_check = {}
	iid = len(fs_sol)
	rscount = 0
	batt = [0 for _ in fs_sol]
	tload = []
	ldcount = 1
	for cu in fs_sol[::-1]:
		if ldcount:
			tload.append(demand[cu])
			ldcount = 0
		else:
			tload.append(tload[-1]+demand[cu])

	dem_map = tload[::-1]

	if dem_map[0] > load_cap:
		return("Reject", batt, tw_check, fs_stn_map, dem_map)

	for i in fs_sol[::-1]:
		iid -= 1
		if count and i in list(fs_stn_map.keys()): # strating from depot
			near_rech_stn = fs_stn_map[i][4]
			battery_tmp = dist_mat_csd[near_rech_stn][depot]*fc_alpha
			ctime = charge_calc(battery_tmp, if_rate, near_rech_stn,0,battery_cap)
			fs_stn_map.update({i:[0,dist_mat_ccs[i][near_rech_stn]*fc_alpha,0, veh, near_rech_stn, depot, battery_tmp, ctime]})
			batt[iid] = dist_mat_ccs[i][near_rech_stn]*fc_alpha
			count = 0
		elif count:
			batt[iid] = dist_mat_cd[i][depot]*fc_alpha
			count = 0
		elif i in list(fs_stn_map.keys()): # if any recharge station is visited
			battery_tmp = batt[iid+1]+dist_mat_ccs[fs_sol[iid+1]][fs_stn_map[i][4]]*fc_alpha+dist_mat_ccs[fs_sol[iid+1]][fs_stn_map[i][4]]*fc_beta*dem_map[iid+1]
			near_rech_stn = fs_stn_map[i][4]
			if battery_tmp > battery_cap:
				return("Reject", batt, tw_check, fs_stn_map, dem_map)
			ctime = charge_calc(battery_tmp, if_rate, near_rech_stn,0, battery_cap)
			fs_stn_map.update({i:[0,dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_map[iid],0,veh, near_rech_stn,fs_sol[iid+1],battery_tmp, ctime]})
			batt[iid] = dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_map[iid+1]

		else:
			batt[iid] = batt[iid+1]+dist_mat_cc[fs_sol[iid+1]][i]*fc_alpha+dist_mat_cc[fs_sol[iid+1]][i]*fc_beta*dem_map[iid+1]

		if batt[iid] >= battery_cap:
			## insert station since it is a battery failure
			try:
				battery_tmp = batt[iid+1]
				near_rech_stn = min(dist_mat_ccs[fs_sol[iid+1]], key=dist_mat_ccs[fs_sol[iid+1]].get)
				battery_req_stn = dist_mat_ccs[fs_sol[iid+1]][near_rech_stn]*fc_alpha+dist_mat_ccs[fs_sol[iid+1]][near_rech_stn]*fc_beta*dem_map[iid]

			except IndexError:
				battery_tmp = 0
				dk = list(dist_mat_csd.values())
				seq = [x['D0'] for x in dk]
				idx = seq.index(min(seq))
				near_rech_stn = list(dist_mat_csd.keys())[idx]
				battery_req_stn = dist_mat_csd[near_rech_stn][depot]*fc_alpha

			if battery_tmp+battery_req_stn <= battery_cap:
				battery_tmp += battery_req_stn
				ctime = charge_calc(battery_tmp, if_rate, near_rech_stn,0, battery_cap)
				try:
					fs_stn_map.update({i:[0,dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_map[iid],0,veh,near_rech_stn,fs_sol[iid+1],battery_tmp, ctime]})
					battery_tmp = dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_map[iid]
				except IndexError:
					fs_stn_map.update({i:[0,dist_mat_ccs[i][near_rech_stn]*fc_alpha,0,veh,near_rech_stn,depot,battery_tmp, ctime]})
					battery_tmp = dist_mat_ccs[i][near_rech_stn]*fc_alpha
			else:
				return("Reject", batt, tw_check, fs_stn_map, dem_map)

	i = fs_sol[0]
	if depot in list(fs_stn_map.keys()):
		if batt[0]+dist_mat_cd[i][depot]*fc_alpha+dist_mat_cd[i][depot]*fc_beta*dem_map[0] <= battery_cap:
			del fs_stn_map[depot]
		else:
			near_rech_stn = fs_stn_map[depot][4]
			pcount = 1
	else:
		battery_tmp = batt[0]+dist_mat_cd[i][depot]*fc_alpha+dist_mat_cd[i][depot]*fc_beta*dem_map[0]
		if battery_tmp > battery_cap:
			near_rech_stn = min(dist_mat_ccs[i], key=dist_mat_ccs[i].get)
			if batt[0]+dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_map[0] > battery_cap:
				near_rech_stn1 = min(dist_mat_ccs[fs_sol[1]], key=dist_mat_ccs[fs_sol[1]].get)
				if batt[1]+dist_mat_ccs[fs_sol[1]][near_rech_stn1]*fc_alpha+dist_mat_ccs[fs_sol[1]][near_rech_stn1]*fc_beta*dem_map[1] > battery_cap:
					return("Reject", batt, tw_check, fs_stn_map, dem_map)
				batt[0] = dist_mat_ccs[i][near_rech_stn1]*fc_alpha+dist_mat_ccs[i][near_rech_stn1]*fc_beta*dem_map[0]
				bat_req = batt[1]+dist_mat_ccs[fs_sol[1]][near_rech_stn1]*fc_alpha+dist_mat_ccs[fs_sol[1]][near_rech_stn1]*fc_beta*dem_map[1]
				ctime = charge_calc(bat_req, if_rate, near_rech_stn1, 0, battery_cap)
				fs_stn_map.update({fs_sol[0]:[0,batt[0],0, veh, near_rech_stn1,fs_sol[1], bat_req, ctime]})

				if batt[0]+dist_mat_cd[i][depot]*fc_alpha+dist_mat_cd[i][depot]*fc_beta*dem_map[0] > battery_cap:
					if batt[0]+dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_map[0] > battery_cap:
						return("Reject", batt, tw_check, fs_stn_map, dem_map)
					pcount = 1

			else:
				pcount = 1

	if pcount:
		battery_req = dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_map[0]
		if battery_req+batt[0] <= battery_cap:
			battery = batt[0]+battery_req
			bat_avl = battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*dem_map[0]
			ctime = charge_calc(battery, if_rate, near_rech_stn, bat_avl, battery_cap)
			leaving = dist_mat_csd_time[near_rech_stn][depot]+ctime

			fs_stn_map.update({depot:[leaving,battery_cap,battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*dem_map[0],veh,near_rech_stn,i,battery, ctime]})
		else:
			return("Reject", batt, tw_check, fs_stn_map, dem_map)
	strn, tw_check, fs_stn_map =tw_update(fs_sol, battery_cap, fs_stn_map, fc_alpha, fc_beta, if_rate, avg_speed, time_window, depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, batt, veh, dem_map)
	if strn == "fs_check":
		return("Reject", batt, tw_check, fs_stn_map, dem_map)

	return('Insert', batt, tw_check, fs_stn_map, dem_map)

#-----------------------------------------------------------------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------------------------------------
def battery_tw_update(fs_sol, fs_stn_map, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate, veh, avg_speed, time_window, demand):
	battery_tmp = 0
	tw_check = {}
	batt = [0 for _ in fs_sol]
	tload = []
	ldcount = 1
	for cu in fs_sol[::-1]:
		if ldcount:
			tload.append(demand[cu])
			ldcount = 0
		else:
			tload.append(tload[-1]+demand[cu])
	dem_temp = tload[::-1]

	if fs_sol:
		for iid,i in enumerate(fs_sol):
			if depot in list(fs_stn_map.keys()) and iid == 0:
				near_rech_stn = fs_stn_map[depot][4]
				atime = dist_mat_csd_time[near_rech_stn][depot]
				bat_req = dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_temp[iid]
				prev = i
				for jid, j in enumerate(fs_sol[1:]):
					if j in list(fs_stn_map.keys()):
						bat_req += dist_mat_cc[prev][j]*fc_alpha+dist_mat_cc[prev][j]*fc_beta*dem_temp[jid+1]
						try:
							bat_req += dist_mat_ccs[j][fs_stn_map[j][4]]*fc_alpha+dist_mat_ccs[j][fs_stn_map[j][4]]*fc_beta*dem_temp[jid+2]
						except:
							bat_req += dist_mat_ccs[j][fs_stn_map[j][4]]*fc_alpha
						if bat_req > battery_cap:
							bat_req = battery_cap
						break
					elif j == fs_sol[-1]:
						bat_req += dist_mat_cc[prev][j]*fc_alpha+dist_mat_cc[prev][j]*fc_beta*dem_temp[jid+1]
						bat_req += dist_mat_cd[j][depot]*fc_alpha
						if bat_req > battery_cap:
							bat_req = battery_cap
						break
					bat_req += dist_mat_cc[prev][j]*fc_alpha+dist_mat_cc[prev][j]*fc_beta*dem_temp[jid+1]
					if bat_req > battery_cap:
						bat_req = battery_cap
						break
					prev = j

				fs_stn_map[depot][1] = battery_cap
				fs_stn_map[depot][2] = battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*dem_temp[iid]
				fs_stn_map[depot][5] = i
				fs_stn_map[depot][6] = bat_req
				fs_stn_map[depot][7] = charge_calc(bat_req, if_rate, near_rech_stn,battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*dem_temp[iid], battery_cap)
				fs_stn_map[depot][0] = atime+fs_stn_map[depot][7]
				atime += fs_stn_map[depot][7]+dist_mat_ccs_time[i][near_rech_stn]
				if atime < time_window[i][0]:
					atime = time_window[i][0]
				tw_check.update({i:atime+time_window[i][2]})
				batt[iid] = bat_req-dist_mat_ccs[i][near_rech_stn]*fc_alpha-dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_temp[iid]
			elif iid == 0:
				batt[iid] = battery_cap-dist_mat_cd[i][depot]*fc_alpha-dist_mat_cd[i][depot]*fc_beta*dem_temp[iid]
				atime = dist_mat_cd_time[i][depot]
				if atime < time_window[i][0]:
					atime = time_window[i][0]
				tw_check.update({i:atime+time_window[i][2]})

			elif fs_sol[iid-1] in list(fs_stn_map.keys()): # if any recharge station is visited
				near_rech_stn = fs_stn_map[fs_sol[iid-1]][4]
				atime = tw_check[fs_sol[iid-1]]+dist_mat_ccs_time[fs_sol[iid-1]][near_rech_stn]
				battery_avl = batt[iid-1]-dist_mat_ccs[fs_sol[iid-1]][near_rech_stn]*fc_alpha-dist_mat_ccs[fs_sol[iid-1]][near_rech_stn]*fc_beta*dem_temp[iid]

				if battery_avl < -1E-7:
					del fs_stn_map[fs_sol[iid-1]]
					batt[iid] = batt[iid-1]-dist_mat_cc[fs_sol[iid-1]][i]*fc_alpha-dist_mat_cc[fs_sol[iid-1]][i]*fc_beta*dem_temp[iid]
					atime = tw_check[fs_sol[iid-1]]+dist_mat_cc_time[fs_sol[iid-1]][i]
					if atime < time_window[i][0]:
						atime = time_window[i][0]
					tw_check.update({i:atime+time_window[i][2]})
				else:
					battery_req = dist_mat_ccs[i][near_rech_stn]*fc_alpha+dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_temp[iid]
					prev = i
					if iid+1 == len(fs_sol):
						battery_req += dist_mat_cd[i][depot]*fc_alpha
					else:
						for jid, j in enumerate(fs_sol[iid+1:]):
							if j in list(fs_stn_map.keys()):
								battery_req += dist_mat_cc[prev][j]*fc_alpha+dist_mat_cc[prev][j]*fc_beta*dem_temp[jid+iid+1]
								battery_req += dist_mat_ccs[j][fs_stn_map[j][4]]*fc_alpha+dist_mat_ccs[j][fs_stn_map[j][4]]*fc_beta*dem_temp[jid+iid+1]
								if battery_req > battery_cap:
									battery_req = battery_cap
								break
							elif j == fs_sol[-1]:
								battery_req += dist_mat_cc[prev][j]*fc_alpha+dist_mat_cc[prev][j]*fc_beta*dem_temp[jid+iid+1]
								battery_req += dist_mat_cd[j][depot]*fc_alpha
								if battery_req > battery_cap:
									battery_req = battery_cap
								break
							battery_req += dist_mat_cc[prev][j]*fc_alpha+dist_mat_cc[prev][j]*fc_beta*dem_temp[iid+jid+1]
							if battery_req > battery_cap:
								battery_req = battery_cap
								break
							prev = j

					fs_stn_map[fs_sol[iid-1]][1] = batt[iid-1]
					fs_stn_map[fs_sol[iid-1]][2] = batt[iid-1]-dist_mat_ccs[fs_sol[iid-1]][near_rech_stn]*fc_alpha-dist_mat_ccs[fs_sol[iid-1]][near_rech_stn]*fc_beta*dem_temp[iid]
					fs_stn_map[fs_sol[iid-1]][5] = i
					fs_stn_map[fs_sol[iid-1]][6] = battery_req
					fs_stn_map[fs_sol[iid-1]][7] = charge_calc(battery_req, if_rate, near_rech_stn,battery_avl, battery_cap)
					fs_stn_map[fs_sol[iid-1]][0] = atime+fs_stn_map[fs_sol[iid-1]][7]
					atime += fs_stn_map[fs_sol[iid-1]][7]+dist_mat_ccs_time[i][near_rech_stn]
					if atime < time_window[i][0]:
						atime = time_window[i][0]
					tw_check.update({i:atime+time_window[i][2]})
					batt[iid] = battery_req-dist_mat_ccs[i][near_rech_stn]*fc_alpha-dist_mat_ccs[i][near_rech_stn]*fc_beta*dem_temp[iid]

			else:
				atime = tw_check[fs_sol[iid-1]]+dist_mat_cc_time[i][fs_sol[iid-1]]
				batt[iid] = batt[iid-1]-dist_mat_cc[fs_sol[iid-1]][i]*fc_alpha-dist_mat_cc[fs_sol[iid-1]][i]*fc_beta*dem_temp[iid]
				if atime < time_window[i][0]:
					atime = time_window[i][0]
				tw_check.update({i:atime+time_window[i][2]})
		i = fs_sol[-1]
		if i in list(fs_stn_map.keys()):
			near_rech_stn = fs_stn_map[i][4]
			if batt[-1]>=dist_mat_cd[i][depot]*fc_alpha:
				del fs_stn_map[i]
			else:
				atime = tw_check[i]+dist_mat_ccs_time[i][fs_stn_map[i][4]]
				bat_avl = batt[-1]-dist_mat_ccs[i][near_rech_stn]*fc_alpha

				if bat_avl >=0:
					fs_stn_map[i][1] = batt[-1]
					fs_stn_map[i][2] = batt[-1]-dist_mat_ccs[i][near_rech_stn]*fc_alpha
					fs_stn_map[i][5] = depot
					fs_stn_map[i][6] = dist_mat_csd[near_rech_stn][depot]*fc_alpha
					fs_stn_map[i][7] = charge_calc(dist_mat_csd[near_rech_stn][depot]*fc_alpha, if_rate, near_rech_stn,bat_avl, battery_cap)
					fs_stn_map[i][0] = atime+fs_stn_map[i][7]
					atime += fs_stn_map[i][7]+dist_mat_csd_time[fs_stn_map[i][4]][depot]
					if atime > time_window[depot][1]:
						del fs_stn_map[i]
				else:
					del fs_stn_map[i]
		else:
			battery_tmp = batt[-1]-dist_mat_cd[i][depot]*fc_alpha
			if battery_tmp < -1E-7:
				near_rech_stn = min(dist_mat_ccs[i], key=dist_mat_ccs[i].get)
				if batt[-1]-dist_mat_ccs[i][near_rech_stn]*fc_alpha >= 0:
					atime = tw_check[i]+dist_mat_ccs_time[i][near_rech_stn]
					bat_avl = batt[-1]-dist_mat_ccs[i][near_rech_stn]*fc_alpha
					bat_req = dist_mat_csd[near_rech_stn][depot]*fc_alpha
					ctime = charge_calc(bat_req, if_rate, near_rech_stn, bat_avl, battery_cap)
					fs_stn_map.update({i:[atime+ctime,batt[-1],batt[-1]-dist_mat_ccs[i][near_rech_stn]*fc_alpha,veh,near_rech_stn,depot,bat_req, ctime]})
					atime += ctime+dist_mat_csd_time[near_rech_stn][depot]
					if atime > time_window[depot][1]:
						del fs_stn_map[i]
			else:
				atime = tw_check[i]+dist_mat_cd_time[i][depot]

	return batt, tw_check, fs_stn_map, dem_temp
#-------------------------------------------Main function station_operations---------------------------------------------------------------
def station_removal_operation(c,d, sol,in_map, dem_map,time_window, stn_map, depots, battery_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, if_rate, avg_speed, tw_check, battery, demand, load_cap):
	flat_stn = [[key, val] for pkey in stn_map.keys() if stn_map[pkey] for key, val in stn_map[pkey].items()]

	q = min(10,math.ceil(len(flat_stn)*0.4))
	depot = list(depots.keys())[0]
	## station removal operations
	if c == 0: ## random charging station-removal
		'''
		removes q random recharge stations visited
		'''
		for_stn_map = random.sample(flat_stn, len(flat_stn)-q)

	elif c == 1: ## worst charge usage removal
		'''
		removes q recharge stations in decreasing order of charge remaining when visited
		'''
		for_stn_map = sorted(flat_stn, key= lambda x:x[1][1], reverse = True)[q:]

	elif c == 2: ## worst charging station-removal
		'''
		removes q recharge stations in decreasing order of charge required to reach them
		'''
		for_stn_map = sorted(flat_stn, key= lambda x:x[1][2], reverse = True)[q:]
	else:
		'''
		removes q recharge stations in increasing order of charge done at a station
		'''
		for_stn_map = sorted(flat_stn, key= lambda x:x[1][6], reverse = True)[q:]
	stn_map_new = {vehid:{} for vehid in range(len(sol))}
	for (k, val) in for_stn_map:
		stn_map_new[val[3]].update({k:val})

	##############################################################
	## station insertion operations
	##############################################################
	if d == 0: ## best-2 insertion
		'''
		inserts charging stations at best possible position by comparing the insertion in 2 previous arcs leading to current customer
		'''
		out, battery, tw_check, stn_map, load = stn_insertion(sol, battery, tw_check, stn_map_new, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta,if_rate,  time_window, 2, avg_speed, dem_map, demand, load_cap)
		if out == "Reject":
			vid = in_map['C1']
			sol[vid].remove('C1')
	elif d == 1: ## best-3 insertion
		'''
		inserts charging stations at best possible position by comparing the insertion in 3 previous arcs leading to current customer
		'''
		out, battery, tw_check, stn_map, load = stn_insertion(sol, battery, tw_check, stn_map_new, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta,if_rate,  time_window, 3, avg_speed, dem_map, demand, load_cap)
		if out == "Reject":
			vid = in_map['C1']
			sol[vid].remove('C1')
	elif d ==2: ## best insertion
		'''
		inserts charging stations at best possible position
		'''
		out, battery, tw_check, stn_map, load = stn_insertion(sol, battery, tw_check, stn_map_new, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate,  time_window, 'NA',avg_speed, dem_map, demand, load_cap)
		if out == "Reject":
			vid = in_map['C1']
			sol[vid].remove('C1')
	else:
		'''
		inserts charging stations just before vehicle runs out of battery
		'''
		out, battery, tw_check, stn_map, load = stn_insertion(sol, battery, tw_check, stn_map_new, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate,  time_window, 0, avg_speed, dem_map, demand, load_cap)
		if out == "Reject":
			vid = in_map['C1']
			sol[vid].remove('C1')

	return battery, stn_map, tw_check, load
#-----------------------------------------------------------END_station_removal----------------------------------------------------------------------
#--------------------------------------------------------------station_operations--------------------------------------------------------------------
def stn_insertion(sol, battery_new, tw_check_new, new_stn_map, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate, time_window, ncompare, avg_speed, dem_map, demand, load_cap):
	if ncompare != 'NA':
		compare = ncompare
	out = "insert"
	for vehid, veh in enumerate(sol):
		if not veh:
			continue
		count = 0
		compare_list = []
		check_list = []
		battery_new[vehid],tw_check_new[vehid], new_stn_map[vehid], dem_map[vehid] = battery_tw_update(veh, new_stn_map[vehid], dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate, vehid, avg_speed, time_window, demand)

		neg_check = next((i for i, x in enumerate(battery_new[vehid]) if x<0), None)
		if neg_check == None:
			'''check battery at end depot'''
			if veh[-1] not in new_stn_map[vehid].keys():
				if battery_new[vehid][-1] >= dist_mat_cd[veh[-1]][depot]*fc_alpha:
					continue
				else:
					neg_check = len(battery_new[vehid])
			else:
				if battery_new[vehid][-1] >= dist_mat_ccs[veh[-1]][new_stn_map[vehid][veh[-1]][4]]*fc_alpha:
					continue
				else:
					del new_stn_map[vehid][veh[-1]]
					neg_check = len(battery_new[vehid])

		if ncompare == 'NA':
			loc_cs_dpt = next((x for x in range(0,neg_check,-1) if veh[x] in temp_stn.keys()), None)
			if loc_cs_dpt == None:
				compare = len(battery_new[:neg_check])
			else:
				compare = len(battery_new[loc_cs_dpt+1:neg_check])
		stn_node = neg_check-1
		while True:
			if stn_node <1:
				dk = list(dist_mat_csd.values())
				seq = [x[depot] for x in dk]
				idx = seq.index(min(seq))
				near_stn = list(dist_mat_csd.keys())[idx]
				break
			else:
				node = veh[stn_node]
				near_stn = min(dist_mat_ccs[node], key=dist_mat_ccs[node].get) #get the nearest station for the customer at position stn_node
				battery_to_cs = dist_mat_ccs[veh[stn_node]][near_stn]*fc_alpha+fc_beta*dist_mat_ccs[veh[stn_node]][near_stn]*dem_map[vehid][stn_node]

				if battery_new[vehid][stn_node]<battery_to_cs:
					stn_node -= 1
					continue
				else:
					break

		if compare:
			for i in range(compare):
				try:
					if stn_node < 1:
						check_list.append([depot])
						break
					else:
						node = veh[stn_node-1-i]
						check_list.append([node])
				except:
					check_list.append([depot])
					break
		else:
			check_list.append([veh[stn_node]])

		while check_list:
			node_list = check_list.pop(0)
			# gc.disable()
			temp_stn = deepcopy(new_stn_map[vehid])
			temp_batt = deepcopy(battery_new[vehid])
			temp_tw = deepcopy(tw_check_new[vehid])
			temp_dem = deepcopy(dem_map[vehid])
			# gc.enable()
			for nodeid, node in enumerate(node_list):
				if node != depot:
					near_rech_stn = min(dist_mat_ccs[node], key=dist_mat_ccs[node].get)
					temp_stn.update({node:[0,dist_mat_ccs[node][near_rech_stn]*fc_alpha+fc_beta*dist_mat_ccs[node][near_rech_stn]*dem_map[vehid][nodeid-1],0,vehid,near_rech_stn,'NA',0, 0]})
				else:
					near_rech_stn = min(dist_mat_ccs[veh[0]], key=dist_mat_ccs[veh[0]].get)
					temp_stn.update({node:[0,battery_cap,battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*dem_map[vehid][nodeid],vehid,near_rech_stn,'NA',0, 0]})


			temp_batt, temp_tw, temp_stn, temp_dem = battery_tw_update(veh, temp_stn, dist_mat_cc, dist_mat_cc_time, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery_cap, depot, fc_alpha, fc_beta, if_rate, vehid, avg_speed, time_window, demand)
			neg_check = next((i for i, x in enumerate(temp_batt) if x<0), None)
			if neg_check == None:
				'''check battery at end depot'''
				if veh[-1] not in temp_stn.keys():
					if temp_batt[-1] >= dist_mat_cd[veh[-1]][depot]*fc_alpha:
						stn_new_tmp = {0:temp_stn}
						c, vc, TaCT = cost_function([veh],depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, stn_new_tmp, len(veh), avg_speed, time_window, demand, load_cap, False)
						compare_list.append([TaCT,temp_batt, temp_tw, temp_stn, temp_dem])
						continue
					else:
						neg_check = len(temp_batt)
				else:
					if temp_batt[-1] >= dist_mat_ccs[veh[-1]][temp_stn[veh[-1]][4]]*fc_alpha:
						stn_new_tmp = {0:temp_stn}
						c, vc, TaCT = cost_function([veh],depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, stn_new_tmp, len(veh), avg_speed, time_window, demand, load_cap, False)
						compare_list.append([TaCT,temp_batt, temp_tw, temp_stn, temp_dem])
						continue
					else:
						del temp_stn[veh[-1]]
						neg_check = len(temp_batt)

			if ncompare == 'NA':
				loc_cs_dpt = next((x for x in range(0,neg_check,-1) if veh[x] in temp_stn.keys()), None)
				if loc_cs_dpt == None:
					compare = len(battery_new[:neg_check])
				else:
					compare = len(battery_new[loc_cs_dpt+1:neg_check])
			stn_node = neg_check-1
			while True:
				if stn_node <1:
					dk = list(dist_mat_csd.values())
					seq = [x[depot] for x in dk]
					idx = seq.index(min(seq))
					near_stn = list(dist_mat_csd.keys())[idx]
					break
				else:
					near_stn = min(dist_mat_ccs[veh[stn_node]], key=dist_mat_ccs[veh[stn_node]].get) #get the nearest station for the customer at position stn_node
					battery_to_cs = dist_mat_ccs[veh[stn_node]][near_stn]*fc_alpha+dist_mat_ccs[veh[stn_node]][near_stn]*fc_beta*dem_map[vehid][stn_node]
					if temp_batt[stn_node]<battery_to_cs:
						stn_node -= 1
						continue
					else:
						break

			if veh[stn_node] not in node_list:
				check_list.append(node_list+[veh[stn_node]])
			else:
				continue
			if compare:
				for i in range(compare):
					try:
						node = veh[stn_node-1-i]
						if stn_node-1-i > veh.index(node_list[-1]) and node not in temp_stn.keys():
							check_list.append(node_list+[node])
						else:
							break
					except:
						if depot not in node_list:
							check_list.append(node_list+[depot])
						break
		try:
			TaCT,battery_new[vehid],tw_check_new[vehid],new_stn_map[vehid], dem_map[vehid] = sorted(compare_list)[0]
		except IndexError:
			return 'Reject',battery_new,tw_check_new,new_stn_map, dem_map
	return out, battery_new,tw_check_new,new_stn_map, dem_map
#------------------------------------------------------------------------------------------------------------------------------------------