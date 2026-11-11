import time, platform, os, csv, operator, sys
import numpy
from ALNS import *
from reading_input import read_input
#-----------------------------------------------------------------------------------------------------------------------------------------------------
def add_vehicle(last_visit_station, cost, stn_iden_map, not_visited_cus, cus_iden_map, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery, solution, tw_check, i, depot, fc_alpha, fc_beta, time_window, avg_speed, demand, battery_cap, if_rate, load):
    stn_iden_map.update({len(stn_iden_map):{}})
    en_customer = not_visited_cus[0]
    not_visited_cus.remove(en_customer)
    cus_iden_map.update({en_customer:len(solution)})
    cost.append(dist_mat_cd[en_customer][depot])
    battery_discharged = dist_mat_cd[en_customer][depot]*fc_alpha+dist_mat_cd[en_customer][depot]*fc_beta*demand[en_customer]
    battery_discharged_empty_mile = dist_mat_cd[en_customer][depot]*fc_alpha
    load.append([demand[en_customer]])
    battery.append([battery_cap-battery_discharged])
    solution.append([en_customer])
    last_visit_station.append([[0]])
    arrival = dist_mat_cd_time[en_customer][depot]
    if arrival < time_window[en_customer][0]:
        arrival = time_window[en_customer][0]
    tw_check.append({en_customer:arrival+time_window[en_customer][2]})
    i += 1
    near_rech_stn=min(dist_mat_ccs[en_customer], key=dist_mat_ccs[en_customer].get)
    battery_req_stn = battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha-dist_mat_csd[near_rech_stn][depot]*fc_beta*demand[en_customer]
    if battery_req_stn >= 0.85*battery_cap:
        if battery_req_stn >= 0.95*battery_cap:
            leaving = dist_mat_csd_time[near_rech_stn][depot]+(battery_cap-battery_req_stn)*if_rate[near_rech_stn][2][1]
        else:
            leaving = dist_mat_csd_time[near_rech_stn][depot]+(0.95*battery_cap-battery_req_stn)*if_rate[near_rech_stn][1][1]+(0.05*battery_cap*if_rate[near_rech_stn][2][1])

    else:
        leaving = dist_mat_csd_time[near_rech_stn][depot]+(0.85*battery_cap-battery_req_stn)*if_rate[near_rech_stn][0][1]+(0.1*battery_cap*if_rate[near_rech_stn][1][1]+0.05*battery_cap*if_rate[near_rech_stn][2][1])
    chtime = leaving-dist_mat_csd_time[near_rech_stn][depot]
    arrival1 = leaving+dist_mat_ccs_time[en_customer][near_rech_stn]
    bat = battery_cap-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha-dist_mat_ccs[en_customer][near_rech_stn]*fc_beta*demand[en_customer]-battery_discharged

    if bat > 0 and battery_discharged+battery_discharged_empty_mile > battery_cap:
        battery_req_dpt = dist_mat_csd[near_rech_stn][depot]*fc_alpha
        if bat >0.95*battery_cap:
            chtime = (battery_req_dpt-bat)*if_rate[near_rech_stn][2][1]
        elif bat >0.85*battery_cap:
            if battery_req_dpt > 0.95*battery_cap:
                chtime = (0.95*battery_cap- bat)*if_rate[near_rech_stn][1][1]+(battery_req_dpt-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
            else:
                chtime = (battery_req_dpt-bat)*if_rate[near_rech_stn][1][1]
        else:
            if battery_req_dpt > 0.95*battery_cap:
                chtime = (0.85*battery_cap- bat)*if_rate[near_rech_stn][0][1]+0.1*battery_cap*if_rate[near_rech_stn][1][1]+(battery_req_dpt-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
            elif battery_req_dpt > 0.85*battery_cap:
                chtime = (0.85*battery_cap- bat)*if_rate[near_rech_stn][0][1]+(battery_req_dpt-0.85*battery_cap)*if_rate[near_rech_stn][1][1]
            else:
                chtime = (battery_req_dpt-bat)*if_rate[near_rech_stn][0][1]

        leaving = arrival+dist_mat_ccs_time[en_customer][near_rech_stn]+chtime
        # if near_rech_stn in cschkl.keys():
        #     if len(cschkl[near_rech_stn]) == 1:
        #         cschkl[near_rech_stn].append([leaving-chtime, leaving])
        #     else:
        #         cschkl[near_rech_stn].sort(key = lambda x: x[0])
        #         csv = deepcopy(cschkl[near_rech_stn])
        #         csv.append([leaving-chtime, leaving])
        #         csv.sort(key = lambda x: x[0])
        #         a = []
        #         for kl in csv:
        #             a += [round(lm,2) for lm in numpy.arange(round(kl[0],2), round(kl[1],2), 0.01)]
                        
        #         if most_frequent(a) >2:
        #             # print(cschkl,'#######')
        #             for iid, j in enumerate(cschkl[near_rech_stn][:-1]):
        #                 if leaving < j[0] and iid == 0:
        #                     cschkl[near_rech_stn].insert(0,[leaving-chtime, leaving])
        #                     break
        #                 elif leaving-chtime > j[1] and leaving < cschkl[near_rech_stn][iid+1][0]:
        #                     cschkl[near_rech_stn].insert(0,[leaving-chtime, leaving])
        #                     # print(cschkl,'******',11111)
        #                     break
        #             else:
        #                 leaving = cschkl[near_rech_stn][-1][1]+chtime
        #                 cschkl[near_rech_stn].append([leaving-chtime, leaving])
        #                 # print(cschkl,'******',2)
        #         else:
        #             cschkl[near_rech_stn].append([leaving-chtime, leaving])

        # else:
        #     cschkl.update({near_rech_stn:[[leaving-chtime, leaving]]})

        
        stn_iden_map[i].update({en_customer:[leaving,battery[i][0], battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha,i,near_rech_stn, depot, battery_req_dpt, chtime]})
        cost[i] += dist_mat_ccs[en_customer][near_rech_stn]+dist_mat_csd[near_rech_stn][depot]
        last_visit_station[0].append([1,near_rech_stn, [leaving,battery[i][0], battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha,i,near_rech_stn, depot, battery_req_dpt, chtime]])

    elif battery_discharged+battery_discharged_empty_mile > battery_cap and arrival1 < time_window[en_customer][1]:
        arrival = arrival1
        # if near_rech_stn in cschkl.keys():
        #     if len(cschkl[near_rech_stn]) == 1:
        #         cschkl[near_rech_stn].append([leaving-chtime, leaving])
        #     else:
        #         cschkl[near_rech_stn].sort(key = lambda x: x[0])
        #         csv = deepcopy(cschkl[near_rech_stn])
        #         csv.append([leaving-chtime, leaving])
        #         csv.sort(key = lambda x: x[0])
        #         a = []
        #         for kl in csv:
        #             a += [round(lm,2) for lm in numpy.arange(round(kl[0],2), round(kl[1],2), 0.01)]
                        
        #         if most_frequent(a) >2:
        #             # print(cschkl,'#######')
        #             for iid, j in enumerate(cschkl[near_rech_stn][:-1]):
        #                 if leaving < j[0] and iid == 0:
        #                     cschkl[near_rech_stn].insert(0,[leaving-chtime, leaving])
        #                     break
        #                 elif leaving-chtime > j[1] and leaving < cschkl[near_rech_stn][iid+1][0]:
        #                     cschkl[near_rech_stn].insert(0,[leaving-chtime, leaving])
        #                     # print(cschkl,'******',11111)
        #                     break
        #             else:
        #                 leaving = cschkl[near_rech_stn][-1][1]+chtime
        #                 cschkl[near_rech_stn].append([leaving-chtime, leaving])
        #                 # print(cschkl,'******',2)
        #         else:
        #             cschkl[near_rech_stn].append([leaving-chtime, leaving])

        # else:
        #     cschkl.update({near_rech_stn:[[leaving-chtime, leaving]]})
        stn_iden_map[i].update({depot:[leaving, battery_cap, battery_req_stn,i,near_rech_stn, en_customer, battery_cap, chtime]})
        battery[i][0] = battery_cap-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha-dist_mat_ccs[en_customer][near_rech_stn]*fc_beta*demand[en_customer]
        cost[i] += dist_mat_ccs[en_customer][near_rech_stn]+dist_mat_csd[near_rech_stn][depot]-dist_mat_cd[en_customer][depot]
        if arrival < time_window[en_customer][0]:
            arrival = time_window[en_customer][0]
        tw_check[i].update({en_customer:arrival+time_window[en_customer][2]})
        if battery[i][0] < dist_mat_cd[en_customer][depot]*fc_alpha:
            battery_req_stn = dist_mat_csd[near_rech_stn][depot]*fc_alpha
            bat = battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha
            if bat >0.95*battery_cap:
                chtime = (battery_req_stn-bat)*if_rate[near_rech_stn][2][1]
            elif bat >0.85*battery_cap:
                if battery_req_stn > 0.95*battery_cap:
                    chtime = (0.95*battery_cap- bat)*if_rate[near_rech_stn][1][1]+(battery_req_stn-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
                else:
                    chtime = (battery_req_stn-bat)*if_rate[near_rech_stn][1][1]
            else:
                if battery_req_stn > 0.95*battery_cap:
                    chtime = (0.85*battery_cap- bat)*if_rate[near_rech_stn][0][1]+0.1*battery_cap*if_rate[near_rech_stn][1][1]+(battery_req_stn-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
                elif battery_req_stn > 0.85*battery_cap:
                    chtime = (0.85*battery_cap- bat)*if_rate[near_rech_stn][0][1]+(battery_req_stn-0.85*battery_cap)*if_rate[near_rech_stn][1][1]
                else:
                    chtime = (battery_req_stn-bat)*if_rate[near_rech_stn][0][1]

            leaving = arrival+dist_mat_ccs_time[en_customer][near_rech_stn]+chtime+time_window[en_customer][2]
            # cschkl[near_rech_stn].append([leaving-chtime, leaving])
            stn_iden_map[i].update({en_customer:[leaving,battery[i][0], battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha,i,near_rech_stn, depot, battery_req_stn, chtime]})
            cost[i] += dist_mat_ccs[en_customer][near_rech_stn]+dist_mat_csd[near_rech_stn][depot]
            last_visit_station[0].append([1,near_rech_stn, [leaving,battery[i][0], battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha,i,near_rech_stn, depot, battery_req_stn, chtime]])
    return (last_visit_station, cost, stn_iden_map, not_visited_cus, cus_iden_map, load, battery, solution, tw_check, i)
#--------------------------------------------------------------Initial_solution----------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------------------------------------
def initial_solution(customers, depot, demand, time_window, battery_cap, load_cap, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, fc_alpha, fc_beta, if_rate, avg_speed):
    total_demand = sum(demand.values())
    cus_iden_map = {} ## helps in eliminating nested for loops
    stn_iden_map = {}
    re_list = []
    solution = [[]]
    tw_check = [{}]
    battery = [[]]
    load = [[]]
    last_visit_station = []
    cost = [0]
    stn_iden_map = {0:{}}
    # cschkl = {}
    # cus_sorted_readytime = sorted(time_window.items(), key=operator.itemgetter(1)) ### sort customers based on the ready time
    not_visited_cus = [cu for cu in demand.keys()]
    i = 0
    if len(solution[i]) == 0:
        en_customer = not_visited_cus[0]
        dist = dist_mat_cd[en_customer][depot]
        cost[i] += dist
        battery_discharged = dist*fc_alpha+dist*fc_beta*demand[en_customer]
        battery_discharged_empty_mile = dist*fc_alpha
        cus_iden_map.update({en_customer:i})
        load[i].append(demand[en_customer])
        battery[i].append(battery_cap-battery_discharged)
        solution[i].append(en_customer)
        arrival = dist_mat_cd_time[en_customer][depot]
        if arrival < time_window[en_customer][0]:
            arrival = time_window[en_customer][0]
        tw_check[i].update({en_customer: arrival+time_window[en_customer][2]})
        not_visited_cus.remove(en_customer)
        near_rech_stn=min(dist_mat_ccs[en_customer], key=dist_mat_ccs[en_customer].get)

        battery_req_stn = battery_cap-dist_mat_csd[near_rech_stn][depot]*fc_alpha+dist_mat_csd[near_rech_stn][depot]*fc_beta*demand[en_customer]
        last_visit_station.append([[0]])
        if battery_req_stn >= 0.95*battery_cap:
            leaving = dist_mat_csd_time[near_rech_stn][depot]+(battery_cap-battery_req_stn)*if_rate[near_rech_stn][2][1]
        elif battery_req_stn >= 0.85*battery_cap:
            leaving = dist_mat_csd_time[near_rech_stn][depot]+(0.95*battery_cap-battery_req_stn)*if_rate[near_rech_stn][1][1]+(0.05*battery_cap*if_rate[near_rech_stn][2][1])
        else:
            leaving = dist_mat_csd_time[near_rech_stn][depot]+(0.85*battery_cap-battery_req_stn)*if_rate[near_rech_stn][0][1]+(0.1*battery_cap*if_rate[near_rech_stn][1][1]+0.05*battery_cap*if_rate[near_rech_stn][2][1])
        chtime = leaving-dist_mat_csd_time[near_rech_stn][depot]
        arrival1 = leaving+dist_mat_ccs_time[en_customer][near_rech_stn]

        bat = battery_cap-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha- battery_discharged
        if bat > 0 and battery_discharged+battery_discharged_empty_mile > battery_cap:
            battery_req_dpt = dist_mat_csd[near_rech_stn][depot]*fc_alpha
            if bat >0.95*battery_cap:
                chtime = (battery_req_dpt-bat)*if_rate[near_rech_stn][2][1]
            elif bat >0.85*battery_cap:
                if battery_req_dpt > 0.95*battery_cap:
                    chtime = (0.95*battery_cap- bat)*if_rate[near_rech_stn][1][1]+(battery_req_dpt-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
                else:
                    chtime = (battery_req_dpt-bat)*if_rate[near_rech_stn][1][1]
            else:
                if battery_req_dpt > 0.95*battery_cap:
                    chtime = (0.85*battery_cap- bat)*if_rate[near_rech_stn][0][1]+0.1*battery_cap*if_rate[near_rech_stn][1][1]+(battery_req_dpt-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
                elif battery_req_dpt > 0.85*battery_cap:
                    chtime = (0.85*battery_cap- bat)*if_rate[near_rech_stn][0][1]+(battery_req_dpt-0.85*battery_cap)*if_rate[near_rech_stn][1][1]
                else:
                    chtime = (battery_req_dpt-bat)*if_rate[near_rech_stn][0][1]

            leaving = arrival+dist_mat_ccs_time[en_customer][near_rech_stn]+chtime
            # cschkl.update({near_rech_stn:[[leaving-chtime, leaving]]})
            stn_iden_map[i].update({en_customer:[leaving,battery[i][0], battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha,i,near_rech_stn, depot, battery_req_dpt, chtime]})
            cost[i] += dist_mat_ccs[en_customer][near_rech_stn]+dist_mat_csd[near_rech_stn][depot]
            last_visit_station[0].append([1,near_rech_stn, [leaving,battery[i][0], battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha,i,near_rech_stn, depot, battery_req_dpt, chtime]])


        elif battery_discharged+battery_discharged_empty_mile > battery_cap and arrival1 < time_window[en_customer][1]:
            arrival = arrival1
            # cschkl.update({near_rech_stn:[[leaving-chtime, leaving]]})
            stn_iden_map[i].update({depot:[leaving, battery_cap, battery_req_stn,i,near_rech_stn, en_customer, battery_cap, chtime]})
            battery[i][0] = battery_cap-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha-dist_mat_ccs[en_customer][near_rech_stn]*fc_beta*demand[en_customer]
            cost[i] += dist_mat_ccs[en_customer][near_rech_stn]+dist_mat_csd[near_rech_stn][depot]-dist_mat_cd[en_customer][depot]
            if arrival < time_window[en_customer][0]:
                arrival = time_window[en_customer][0]
            tw_check[i].update({en_customer:arrival+time_window[en_customer][2]})
            if battery[i][0] < dist_mat_cd[en_customer][depot]*fc_alpha:
                battery_req_stn = dist_mat_csd[near_rech_stn][depot]*fc_alpha
                bat = battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha
                if bat >0.95*battery_cap:
                    chtime = (battery_req_stn-bat)*if_rate[near_rech_stn][2][1]
                elif bat >0.85*battery_cap:
                    if battery_req_stn > 0.95*battery_cap:
                        chtime = (0.95*battery_cap- bat)*if_rate[near_rech_stn][1][1]+(battery_req_stn-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
                    else:
                        chtime = (battery_req_stn-bat)*if_rate[near_rech_stn][1][1]
                else:
                    if battery_req_stn > 0.95*battery_cap:
                        chtime = (0.85*battery_cap- bat)*if_rate[near_rech_stn][0][1]+0.1*battery_cap*if_rate[near_rech_stn][1][1]+(battery_req_stn-0.95*battery_cap)*if_rate[near_rech_stn][2][1]
                    elif battery_req_stn > 0.85*battery_cap:
                        chtime = (0.85*battery_cap- bat)*if_rate[near_rech_stn][0][1]+(battery_req_stn-0.85*battery_cap)*if_rate[near_rech_stn][1][1]
                    else:
                        chtime = (battery_req_stn-bat)*if_rate[near_rech_stn][0][1]

                leaving = arrival+dist_mat_ccs_time[en_customer][near_rech_stn]+chtime+time_window[en_customer][2]
                # cschkl[near_rech_stn].append([leaving-chtime, leaving])
                stn_iden_map[i].update({en_customer:[leaving,battery[i][0], battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha,i,near_rech_stn, depot, battery_req_stn, chtime]})
                cost[i] += dist_mat_ccs[en_customer][near_rech_stn]+dist_mat_csd[near_rech_stn][depot]
                last_visit_station[0].append([1,near_rech_stn, [leaving,battery[i][0], battery[i][0]-dist_mat_ccs[en_customer][near_rech_stn]*fc_alpha,i,near_rech_stn, depot, battery_req_stn, chtime]])
    while len(not_visited_cus) > 0:# loop untill all customers are visited
        last_visit_station, cost, stn_iden_map, not_visited_cus, cus_iden_map, load, battery, solution, tw_check, i = add_vehicle(last_visit_station, cost, stn_iden_map, not_visited_cus, cus_iden_map, dist_mat_cd, dist_mat_cd_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_csd, dist_mat_csd_time, battery, solution, tw_check, i, depot, fc_alpha, fc_beta, time_window, avg_speed, demand, battery_cap, if_rate,load)


    if last_visit_station[i][-1][0] == 1:
        if battery[i][-1] < dist_mat_cd[solution[i][-1]][depot]:
            # stn_iden_map[i][solution[i][-1]].append(depot)
            cost[i] += dist_mat_csd[last_visit_station[i][-1][1]][depot]
        else:
            cost[i] += dist_mat_cd[solution[i][-1]][depot]-dist_mat_ccs[solution[i][-1]][last_visit_station[i][-1][1]]
            del stn_iden_map[i][solution[i][-1]]
    elif last_visit_station[i][-1][0] == 0:
        cost[i] += dist_mat_cd[solution[i][-1]][depot]
    total_customer = len(customers)
    tc, c, fc = cost_function_i(solution, depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, stn_iden_map, total_customer, avg_speed, time_window, demand, load_cap)
    # print(solution)
    return solution, tc, c, fc, cus_iden_map, stn_iden_map, tw_check, battery, load

#------------------------------------------------------End_initial_solution--------------------------------------------------------------------------#
def cost_function_i(sol, depot, dist_mat_cc, dist_mat_cc_time, dist_mat_ccs, dist_mat_ccs_time, dist_mat_cd, dist_mat_cd_time, dist_mat_csd, dist_mat_csd_time, stn_iden_map, total_customer, avg_speed, time_window, demand, load_cap):
    le = len(sol)
    load = [[] for _ in range(le)]
    cost = [0 for _ in range(le)]
    tcost = [0 for _ in range(le)]
    total_customer_visited = len(sum(sol, []))

    if total_customer_visited < total_customer:
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