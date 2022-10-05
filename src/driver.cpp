/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, March 2021
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "CBS.h"


/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("output,o", po::value<string>(), "output file for schedule")
		("outputPaths", po::value<string>(), "output file for paths")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(60), "cutoff time (seconds)")
		("nodeLimit", po::value<int>()->default_value(MAX_NODES), "node limit")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("stats", po::value<bool>()->default_value(false), "write to files some statistics")
		("agentIdx", po::value<string>()->default_value(""), "customize the indices of the agents (e.g., \"0,1\")")

		// params for instance generators
		("rows", po::value<int>()->default_value(0), "number of rows")
		("cols", po::value<int>()->default_value(0), "number of columns")
		("obs", po::value<int>()->default_value(0), "number of obstacles")
		("warehouseWidth", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instances")

		// params for CBS
		("heuristics", po::value<string>()->default_value("Zero"), "heuristics for the high-level search (Zero, CG,DG, WDG)")
		("prioritizingConflicts", po::value<bool>()->default_value(false), "conflict priortization. If true, conflictSelection is used as a tie-breaking rule.")
		("bypass", po::value<bool>()->default_value(false), "Bypass1")
		("disjointSplitting", po::value<bool>()->default_value(false), "disjoint splitting")
		("rectangleReasoning", po::value<string>()->default_value("None"), "rectangle reasoning strategy (None, R, RM, GR, Disjoint)")
		("corridorReasoning", po::value<string>()->default_value("None"), " corridor reasoning strategy (None, C, PC, STC, GC, Disjoint")
		("mutexReasoning", po::value<bool>()->default_value(false), "Using mutex reasoning")
		("targetReasoning", po::value<bool>()->default_value(false), "Using target reasoning")
		("restart", po::value<int>()->default_value(1), "number of restart times (at least 1)")
		("sipp", po::value<bool>()->default_value(false), "using sipp as the single agent solver")

        // save result path
        ("result_path", po::value<string>(), "save result file for paths")
        ("change_map_counts", po::value<int>(), "counts that map changes")

		;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help"))
	{
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);
	/////////////////////////////////////////////////////////////////////////
	/// check the correctness and consistence of params
    //////////////////////////////////////////////////////////////////////
	heuristics_type h;
	if (vm["heuristics"].as<string>() == "Zero")
		h = heuristics_type::ZERO;
	else if (vm["heuristics"].as<string>() == "CG")
		h = heuristics_type::CG;
	else if (vm["heuristics"].as<string>() == "DG")
		h = heuristics_type::DG;
	else if (vm["heuristics"].as<string>() == "WDG")
		h = heuristics_type::WDG;
	else
	{
		cout << "WRONG heuristics strategy!" << endl;
		return -1;
	}

	rectangle_strategy r;
	if (vm["rectangleReasoning"].as<string>() == "None")
		r = rectangle_strategy::NR;  // no rectangle reasoning
	else if (vm["rectangleReasoning"].as<string>() == "R")
		r = rectangle_strategy::R;  // rectangle reasoning for entire paths
	else if (vm["rectangleReasoning"].as<string>() == "RM")
		r = rectangle_strategy::RM;  // rectangle reasoning for path segments
    else if (vm["rectangleReasoning"].as<string>() == "GR")
        r = rectangle_strategy::GR;  // generalized rectangle reasoning
	else if (vm["rectangleReasoning"].as<string>() == "Disjoint")
		r = rectangle_strategy::DR; // disjoint rectangle reasoning
	else
	{
		cout << "WRONG rectangle reasoning strategy!" << endl;
		return -1;
	}

	corridor_strategy c;
	if (vm["corridorReasoning"].as<string>() == "None")
		c = corridor_strategy::NC;  // no corridor reasoning
	else if (vm["corridorReasoning"].as<string>() == "C")
		c = corridor_strategy::C;  // corridor reasoning
    else if (vm["corridorReasoning"].as<string>() == "PC")
        c = corridor_strategy::PC;  // corridor + pseudo-corridor reasoning
    else if (vm["corridorReasoning"].as<string>() == "STC")
        c = corridor_strategy::STC;  // corridor with start-target reasoning
    else if (vm["corridorReasoning"].as<string>() == "GC")
        c = corridor_strategy::GC;  // generalized corridor reasoning = corridor with start-target + pseudo-corridor
	else if (vm["corridorReasoning"].as<string>() == "Disjoint")
		c = corridor_strategy::DC; // disjoint corridor reasoning
	else
	{
		cout << "WRONG corridor reasoning strategy!" << endl;
		return -1;
	}


	///////////////////////////////////////////////////////////////////////////
	/// load the instance
    //////////////////////////////////////////////////////////////////////
	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["agentNum"].as<int>(), vm["agentIdx"].as<string>(),
		vm["rows"].as<int>(), vm["cols"].as<int>(), vm["obs"].as<int>(), vm["warehouseWidth"].as<int>());

	srand(vm["seed"].as<int>());

	int runs = vm["restart"].as<int>();

//    instance.printMap();

	//////////////////////////////////////////////////////////////////////
	/// initialize the solver
    //////////////////////////////////////////////////////////////////////

    instance.printMap();

//	CBS cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());

    CBS cbs(instance, vm["screen"].as<int>());

	cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
	cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
	cbs.setBypass(vm["bypass"].as<bool>());
	cbs.setRectangleReasoning(r);
	cbs.setCorridorReasoning(c);
	cbs.setHeuristicType(h);
	cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
	cbs.setMutexReasoning(vm["mutexReasoning"].as<bool>());
	cbs.setSavingStats(vm["stats"].as<bool>());
	cbs.setNodeLimit(vm["nodeLimit"].as<int>());

    instance.printAgentInfos();
	//////////////////////////////////////////////////////////////////////
	/// run
    //////////////////////////////////////////////////////////////////////

	double runtime = 0;
	int min_f_val = 0;
    const int CHANGE_TIMES = vm["change_map_counts"].as<int>(); //10;

    vector<DynamicObstacle> obstacle_add_v;
    vector<DynamicObstacle> obstacle_delete_v;


//    string result_save_directory = vm["result_path"].as<string>() + "/" +
//                                   std::to_string(vm["agentNum"].as<int>())  + "/";

    string result_save_directory = vm["result_path"].as<string>();

    bool is_modifyscen_exist = cbs.isFileExists(result_save_directory + "modifyscen.json");
    cout << "is_modifyscen_exist: " << is_modifyscen_exist << endl;


    double total_time_cost = 0;

    cbs.initResultInfos(vm, is_modifyscen_exist);

//    is_modifyscen_exist = false;
    if(!is_modifyscen_exist) {
//        clock_t start = clock();

        cbs.initModifyscenInfos(instance);
        int current_step = 0;
        while (current_step < CHANGE_TIMES) {

            cout << "current_step: " << current_step << endl;

            cbs.printNodeLabelInfos();
            cbs.setLabel(current_step);


            if(current_step == 0) {
                cbs.updateModifyscenInfos(current_step, false, obstacle_add_v);
                cbs.dijkstra();
                cbs.clear();
                cbs.initSolveParams(vm["cutoffTime"].as<double>(), min_f_val);
                bool is_generate_success = cbs.generateRoot();
                cout << "is_generate_success: " << is_generate_success << endl;

            }

            else {
                cout << "@@@@@@@@@@@@@@@@@@@@@@@" << endl;
                cout << "cbs.get_open_list_size(): " << cbs.get_open_list_size() << endl;

                cbs.addRandomObstacles(instance, obstacle_add_v, 5);
                cbs.updateModifyscenInfos(current_step, true, obstacle_add_v);

//                cbs.deleteRandomObstacles(instance, obstacle_delete_v, 5);
//                cbs.updateModifyscenInfos(current_step, false, obstacle_delete_v);

//                cbs.dijkstra();

                // (rand() % 2)

//                if((current_step % 2) == 1) {
//                    cout << "addRandomObstacles" << endl;
//                    cbs.addRandomObstacles(instance, obstacle_add_v, 5);
//                    cbs.updateModifyscenInfos(current_step, true, obstacle_add_v);
////                    cbs.dijkstra();
//                }
//                else if((current_step % 2) == 0) {
//                    cout << "deleteRandomObstacles" << endl;
//                    cbs.deleteRandomObstacles(instance, obstacle_delete_v, 5);
//                    cbs.updateModifyscenInfos(current_step, false, obstacle_delete_v);
////                    cbs.dijkstra();
//                }


//                cbs.dijkstra();

                if(cbs.get_open_list_size() == 0) { // || cbs.get_open_list_size() >= 500
                    cbs.clear();
                    cbs.initSolveParams(vm["cutoffTime"].as<double>(), min_f_val);
                    bool is_generate_success = cbs.generateRoot();
                    if (!is_generate_success) {

                        obstacle_delete_v.clear();
                        obstacle_add_v.clear();
                        cout << "cbs.generateRoot() failed" << endl;
                        cbs.runtime = 0;
                        cbs.updateResultInfos(current_step);
                        current_step++;
                        continue;
                    }
                }
            }


            cbs.solve(instance, obstacle_delete_v, obstacle_add_v);



//            cbs.printResults(current_step);
            cbs.updateResultInfos(current_step);

            current_step++;
            runtime += cbs.runtime;
//            min_f_val = cbs.solution_cost;
//            cbs.set_min_f_val(min_f_val);

            cbs.update_min_f_val();


            obstacle_delete_v.clear();
            obstacle_add_v.clear();

            cbs.checkValidateSolution(instance);

            total_time_cost += cbs.runtime;

        }

//        double total_time_cost = (double) (clock() - start) / CLOCKS_PER_SEC;
        cout << "total_time_cost: " << total_time_cost << endl;

        cbs.saveModifyscen(result_save_directory + "modifyscen.json");
        cbs.clearModifyscenInfos();

        cbs.saveResultInfos(result_save_directory + "new_cbs_result.json");
//    instance.printMap();
    }


//    is_modifyscen_exist = false;

    if(is_modifyscen_exist) {
        cout << "use modifyscen.json to compute old cbs" << endl;
//        clock_t start = clock();

        string modifyscen_infos_file_name = result_save_directory + "modifyscen.json";
        json modifyscen_infos = cbs.parseModifyscen(modifyscen_infos_file_name);
        obstacle_delete_v.clear();
        obstacle_add_v.clear();
        int current_step = 0;
        for(int i = 0; i < CHANGE_TIMES; ++i) {

            string key = std::to_string(i);
            json value = modifyscen_infos[key];

            cout << "current_step: " << current_step << endl;
            current_step++;

            if(key == "0") {
                cbs.dijkstra();
                cbs.clear();
                cbs.initSolveParams(vm["cutoffTime"].as<double>(), min_f_val);
                bool is_generate_success = cbs.generateRoot();
                cout << "is_generate_success: " << is_generate_success << endl;
            }
            else {

                cout << "@@@@@@@@@@@@@@@@@@@@@@@" << endl;
                cout << "key: " << key << endl;
                cout << "cbs.get_open_list_size(): " << cbs.get_open_list_size() << endl;

                cbs.clear();

                if(value["is_add_obstacles"]) {
                    cout << "addRandomObstacles" << endl;
                    if(value.contains("pos")) {
                        for(auto& [k, v]: value["pos"].items()) {
                            int x = v["x"];
                            int y = v["y"];
                            int obstacle_pos = instance.getCols() * y + x;
                            instance.changeMap(obstacle_pos, true);
                            cout << "add obstacle_pos: " << obstacle_pos << endl;
                        }
                    }
                }
                else if(!value["is_add_obstacles"]) {
                    cout << "deleteRandomObstacles" << endl;
                    if(value.contains("pos")) {
                        for(auto& [k, v]: value["pos"].items()) {
                            int x = v["x"];
                            int y = v["y"];
                            int obstacle_pos = instance.getCols() * y + x;
                            instance.changeMap(obstacle_pos, false);
                        }
                    }
                }

                cbs.dijkstra();
                cbs.initSolveParams(vm["cutoffTime"].as<double>(), min_f_val);
                bool is_generate_success = cbs.generateRoot();
                cout << "is_generate_success: " << is_generate_success << endl;
                if (!is_generate_success) {
                    obstacle_delete_v.clear();
                    obstacle_add_v.clear();
                    cbs.runtime = 0;
                    min_f_val = 0;
                    cbs.set_min_f_val(min_f_val);
                    cout << "cbs.generateRoot() failed" << endl;
                    cbs.updateResultInfos(i);
                    continue;
                }

            }

//            cbs.solve(instance, obstacle_delete_v, obstacle_add_v);
            cbs.solve(vm["cutoffTime"].as<double>(), min_f_val);

            cbs.updateResultInfos(i);

            runtime += cbs.runtime;
            min_f_val = 0;
            cbs.set_min_f_val(min_f_val);

            obstacle_delete_v.clear();
            obstacle_add_v.clear();

            cbs.checkValidateSolution(instance);

            total_time_cost += cbs.runtime;

        }


//        double total_time_cost = (double) (clock() - start) / CLOCKS_PER_SEC;
        cout << "total_time_cost: " << total_time_cost << endl;

        cbs.saveResultInfos(result_save_directory + "old_cbs_result.json");
    }


	cbs.runtime = runtime;

    //////////////////////////////////////////////////////////////////////
    /// write results to files
    //////////////////////////////////////////////////////////////////////
	if (vm.count("output"))
		cbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>()+":"+ vm["agentIdx"].as<string>());
	 cbs.saveCT(vm["output"].as<string>() + ".tree"); // for debug
	if (vm["stats"].as<bool>())
	{
		cbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>() + ":" + vm["agentIdx"].as<string>());
	}
    if (cbs.solution_found && vm.count("outputPaths"))
        cbs.savePaths(vm["outputPaths"].as<string>());
	cbs.clearSearchEngines();
	return 0;

}


