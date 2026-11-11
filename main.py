import time, platform, os, csv, sys
from ALNS import *
from initial_sol import *
from reading_input import read_input

#-----------------------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------ALNS_Main------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------------------------------------
# The body below used to reference a module-level global named
# stn_operation instead of this function's own stn_insertion parameter --
# silently "worked" only because the batch script at the bottom of this
# file happened to define a same-valued module global of that exact name
# before ever calling main_ALNS. Calling main_ALNS from anywhere else
# (e.g. importing it, now that the batch script is guarded behind
# __main__) raised NameError: name 'stn_operation' is not defined.
def main_ALNS(osolution, customers, recharge_stns, depots, demand, time_window, battery_cap, load_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, if_rate, avg_speed, stn_insertion, MaxIter, MaxInnerIter, T, alpha, no_impact, rrop, initial_cost, start, k, progress_callback=None, job=None):
	score_low_removal, score_station_rm, score_station_in , score_insertion = [1, 1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1]
	temp_low_removal_rm, temp_station_rm, temp_station_in, temp_insertion = [0, 0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]
	BestSol = osolution
	tcus = len(customers)
	BestCostList = []
	scores_lr, scores_in, scores_sr, scores_si = [], [], [], []
	depot = list(depots.keys())[0]
	olen = sum([1 for i in osolution['veh_cost'] if i > 0])
	for it2 in range(MaxIter):
		for it in range(MaxInnerIter):
			# Pause checkpoint: only engaged when a real job object is
			# passed in (the standalone batch script below never passes
			# one, so this is a no-op there, identical to before this was
			# added). Checked every inner iteration, so a pause request is
			# honored within one iteration, not just at outer-loop
			# boundaries. resume_requested.wait() genuinely blocks this
			# thread (zero CPU) -- osolution/BestSol/scores/T are all still
			# local variables in this same call frame, so resuming just
			# continues the loop exactly where it left off.
			#
			# customers/recharge_stns/demand/time_window/dist_mat_*/if_rate/
			# initial_cost are never reassigned anywhere in this function --
			# only read -- and api_server.py aliases job.customers etc to
			# these exact same dict objects before calling main_ALNS. So as
			# long as an /orders or /stations edit mutates those dicts IN
			# PLACE (never rebinds them), this function sees the edit
			# automatically with no resync needed for them. osolution and
			# BestSol are the only two names ever REBOUND to a new dict
			# (every accepted move / improving outer iteration), so those
			# are the only two that need an explicit sync here.
			if job is not None and job.pause_requested.is_set():
				job.osolution = osolution
				job.best_sol = BestSol
				job.pause_requested.clear()
				job.status = "paused"
				job.paused_confirmed.set()
				job.resume_requested.wait()
				job.resume_requested.clear()
				job.paused_confirmed.clear()
				job.status = "running"
				osolution = job.osolution
				BestSol = job.best_sol
				tcus = len(customers)
			sol = cPickle.loads(cPickle.dumps(osolution['sol'], -1))
			load = cPickle.loads(cPickle.dumps(osolution['load'], -1))
			battery = cPickle.loads(cPickle.dumps(osolution['battery'], -1))
			cus_map = cPickle.loads(cPickle.dumps(osolution['cus_map'], -1))
			stn_map = cPickle.loads(cPickle.dumps(osolution['re_stn_map'], -1))
			tw_check = cPickle.loads(cPickle.dumps(osolution['arrival_time'], -1))
			if it%stn_insertion== 0 and it != 0:
				no_impact = stn_insertion
				new_sol = {}
				new_sol['sol'], new_sol['load'], new_sol['cus_map'] = sol, load, cus_map
				n = random.random()
				c = roulette_wheel(score_station_rm[:], n)
				n = random.random()
				d = roulette_wheel(score_station_in[:], n)
				new_sol['battery'], new_sol['re_stn_map'], new_sol['arrival_time'], new_sol['load'] = station_removal_operation(c,d, sol,cus_map, load,time_window, stn_map, depots, battery_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, if_rate, avg_speed, tw_check, battery, demand, load_cap)

			else:
				new_sol = {}
				# create neighbour
				n = random.random()
				if it%rrop == 0 and it != 0:
					a = roulette_wheel(score_low_removal[3:], n)
				else:
					a = roulette_wheel(score_low_removal[:3], n)
				customer_pool, partial_sol, in_map, stn_map,tw_check, battery, p_load = customer_removal_operation(a, sol, depot, time_window, cus_map, battery_cap, battery, tw_check, stn_map, avg_speed, fc_alpha, fc_beta, if_rate, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, initial_cost, recharge_stns, demand, load, load_cap)
				n = random.random()
				b = roulette_wheel(score_insertion[:], n)
				depot = list(depots.keys())[0]

				new_sol['sol'], new_sol['cus_map'], new_sol['re_stn_map'],new_sol['battery'], new_sol['arrival_time'], new_sol['load'] = insertion_operation(b,customer_pool, cPickle.loads(cPickle.dumps(partial_sol,-1)), depot, demand, cus_map, cPickle.loads(cPickle.dumps(stn_map,-1)), fc_alpha, fc_beta, if_rate, time_window, avg_speed, battery_cap, dist_mat_cd, dist_mat_cd_time, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, cPickle.loads(cPickle.dumps(battery,-1)), tw_check, initial_cost, p_load, load_cap)

			extra_stn_rm_list = []
			for stnk, stnv in new_sol['re_stn_map'].items():
				for sk, sv in stnv.items():
					if sv[7] < 0:
						extra_stn_rm_list.append([stnk,sk])

			for srm in extra_stn_rm_list:
				del new_sol['re_stn_map'][srm[0]][srm[1]]

			for bsba in new_sol['battery']:
				if any([bs<0 for bs in bsba]):
					new_sol['cost'],new_sol['veh_cost'], new_sol['f_cost'] = (10**10, [], 10**10)
					break
			else:
				new_sol['cost'],new_sol['veh_cost'], new_sol['f_cost'] = cost_function(new_sol['sol'], depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, new_sol['re_stn_map'], tcus, avg_speed, time_window, demand, load_cap, True)

				if new_sol['cost'] < 10**7:
					for vid, vc in enumerate(new_sol['veh_cost']):
						if vc:
							if float(vc) > float(battery_cap/fc_alpha) and not new_sol['re_stn_map'][vid]:
								new_sol['cost'],new_sol['veh_cost'], new_sol['f_cost'] = (10**10, [], 10**10)
								break

			if new_sol['f_cost'] <= osolution['f_cost'] and new_sol['sol'] != osolution['sol'] and new_sol['f_cost'] not in BestCostList:
				# new_sol is better, so it is accepted and score is updated
				if new_sol['f_cost'] < BestSol['f_cost']:
					if no_impact == stn_insertion:
						temp_station_rm[c] +=45
						temp_station_in[d] += 45
						no_impact = 0
					else:
						temp_low_removal_rm[a] += 45
						temp_insertion[b] += 45

				else:
					if no_impact == stn_insertion:
						temp_station_rm[c]+=15
						temp_station_in[d] += 15
						no_impact = 0
					else:
						temp_low_removal_rm[a] += 15
						temp_insertion[b] += 15
				osolution = new_sol
			elif random.random() <= math.exp(-(new_sol['f_cost']-osolution['f_cost'])/(k*T)):
				if no_impact == stn_insertion:
					temp_station_rm[c]+=10
					temp_station_in[d] += 10
					no_impact = 0
				else:
					temp_low_removal_rm[a] += 10
					temp_insertion[b] += 10
				osolution = new_sol
		if osolution['f_cost']< BestSol['f_cost']:
			BestSol = osolution
			BestSol['runtime'] = time.time()-start


		sum_low_rm = sum(temp_low_removal_rm)
		sum_in = sum(temp_insertion)
		cr, ci, sr, si = [], [], [], []
		for idx, i in enumerate(temp_low_removal_rm):
			if sum_low_rm:
				score_low_removal[idx] += i/sum_low_rm
				cr.append(0.7*score_low_removal[idx]+0.3*(i/sum_low_rm))
		for idx, i in enumerate(temp_insertion):
			if sum_in and it2:
				score_insertion[idx] += i/sum_in
				ci.append(0.7*score_insertion[idx]+0.3*(i/sum_in))
		temp_low_removal_rm, temp_insertion = [0,0,0,0,0], [0,0,0,0]
		sum_high_rm = sum(temp_station_rm)
		sum_high_in = sum(temp_station_in)
		for idx, i in enumerate(temp_station_rm):
			if sum_high_rm and it2:
				score_station_rm[idx] += i/sum_high_rm
				sr.append(0.7*score_station_rm[idx]+0.3*(i/sum_high_rm))
		for idx, i in enumerate(temp_station_in):
			if sum_high_in and it2:
				score_station_in[idx] += i/sum_high_in
				si.append(0.7*score_station_in[idx]+0.3*(i/sum_high_in))
		scores_lr.append(cr)
		scores_sr.append(sr)
		scores_in.append(ci)
		scores_si.append(si)
		temp_station_rm, temp_station_in = [0,0,0,0], [0,0,0,0]
#		if it2%5==0:
		print('Iteration:', it2, '- Best cost dist = ', BestSol['f_cost'], '-> Vehicles -->', sum([1 for i in BestSol['arrival_time'] if i]))
#		print('----------------------------------------------------------------------------------------------------------------------------------------')
		# Optional hook for callers that want live progress (e.g. a web API
		# polling job status) without changing this function's behavior for
		# the existing batch script, which never passes one.
		if progress_callback is not None:
			progress_callback(it2, MaxIter, BestSol['f_cost'], BestCostList)
		T = alpha*T
		pcost = round(BestSol['f_cost'],3)
		BestCostList.append(pcost)
		if BestCostList.count(pcost) == 100 or T < 1E-6:#150 not T<
			break

	weights = {}
	weights['customer_removal'], weights['customer_insertion'], weights['station_removal'], weights['station_insertion'] = score_low_removal, score_insertion, score_station_rm, score_station_in
	return (BestSol, weights, scores_si, scores_in, scores_sr, scores_lr, BestCostList, it2)
#---------------------------------------------------------END_ALNS_Main-------------------------------------------------------------------------------
# Guarded behind __main__ so importing main_ALNS elsewhere (the FastAPI
# wrapper needs to) doesn't trigger this whole batch script as a side
# effect -- confirmed live: an unguarded `from main import main_ALNS`
# silently kicked off the full 625x800-iteration run over every .txt file
# in this directory, writing results.csv, before the importer's own code
# ever got to run.
if __name__ == "__main__":
	path = os.path.dirname(os.path.abspath(__file__))
	filelist = [f for f in os.listdir(path) if f.endswith(".txt")]
	#filelist = ['tc0c20s4cf2.txt','tc1c20s3cf1.txt','tc1c20s3ct1.txt','tc1c20s4cf1.txt','tc1c20s4ct1.txt','tc1c20s4ct3.txt','tc2c20s3cf0.txt','tc2c20s3ct0.txt','tc2c20s4cf0.txt','tc2c20s4ct0.txt']
	# filelist = ['tc0c40s5cf0.txt','tc1c40s5cf1.txt','tc1c40s5ct1.txt','tc2c40s5cf3.txt']
	with open("results.csv", "w") as outfile:
		writer=csv.writer(outfile,lineterminator='\n',)
		writer.writerow(['File name', 'Total cost', 'No.of Vehicles','Run-time','Solution', 'si', 'in', 'sr', 'lr'])
		stn_operation = 11
		MaxInnerIter = 800
		MaxIter = int(500000/MaxInnerIter)
		rrop = 50
		k = 1/9
		T = 30
		alpha = 0.98
		no_impact = 0
		for i in range(3):
			for file in filelist:
				fname = (path+'/'+file)
				print(file)
				#continue
				start = time.time()
				(customers, recharge_stns, depots, demand, time_window, battery_cap, load_cap, fc_alpha, fc_beta, if_rate, avg_speed,
				 dist_mat_cc, dist_mat_ccs, dist_mat_cd, dist_mat_csd,
				 dist_mat_cc_time, dist_mat_ccs_time, dist_mat_cd_time, dist_mat_csd_time) = read_input(fname)
				# create initial solution
				depot = list(depots.keys())[0]
				isolution, icost, ivehcost, ifcost, icus_map, istn_map, iarrival, ibattery, iload = initial_solution(customers, depot, demand, time_window, battery_cap, load_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, if_rate, avg_speed)
				initial_cost = {}
				cu_list = list(customers.keys())

				for ivcid, ivc in enumerate(ivehcost):
					initial_cost.update({cu_list[ivcid]:ivc})
				start = time.time()
				Solution = {'sol': isolution, 'cost':icost, 'cus_map': icus_map, 're_stn_map':istn_map, 'arrival_time': iarrival, 'battery': ibattery, 'load': iload, 'f_cost':ifcost,'runtime': time.time()-start, 'veh_cost': ivehcost}
				BestSol, weights, scores_si, scores_in, scores_sr, scores_lr, BestCostList, it2 = main_ALNS(Solution, customers, recharge_stns, depots, demand, time_window, battery_cap, load_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, if_rate, avg_speed, stn_operation, MaxIter, MaxInnerIter, T, alpha, no_impact, rrop, initial_cost, start, k)
	#			print((time.time()-start))
	#			print(BestSol['sol'], BestSol['re_stn_map'], BestSol['battery'])
				writer.writerow([file, BestSol['f_cost'], sum([1 for i in BestSol['arrival_time'] if i]),BestSol['runtime'], BestSol, BestCostList, it2])#, scores_si, scores_in, scores_sr, scores_lr])
		# #-----------------------------------------------------------------------------------------------------------------------------------------------