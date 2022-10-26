#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "CBS.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void CBS::updatePaths(CBSNode* curr)
{
	for (int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr != nullptr)
	{
		for (auto & path : curr->paths)
		{
			if (!updated[path.first])
			{
				paths[path.first] = &(path.second);
				updated[path.first] = true;
			}
		}
		curr = curr->parent;
	}
}


void CBS::copyConflicts(const list<shared_ptr<Conflict >>& conflicts,
	list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agents)
{
	for (const auto& conflict : conflicts)
	{
		bool found = false;
		for (auto a : excluded_agents)
		{
			if (conflict->a1 == a || conflict->a2 == a)
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			copy.push_back(conflict);
		}
	}
}


void CBS::findConflicts(CBSNode& curr, int a1, int a2)
{
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	for (size_t timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2)
		{
			shared_ptr<Conflict> conflict(new Conflict());
			if (target_reasoning && paths[a1]->size() == timestep + 1)
			{
				conflict->targetConflict(a1, a2, loc1, timestep);
			}
			else if (target_reasoning && paths[a2]->size() == timestep + 1)
			{
				conflict->targetConflict(a2, a1, loc1, timestep);
			}
			else
			{
				conflict->vertexConflict(a1, a2, loc1, timestep);
			}
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict);
		}
		else if (timestep < min_path_length - 1
				 && loc1 == paths[a2]->at(timestep + 1).location
				 && loc2 == paths[a1]->at(timestep + 1).location)
		{
			shared_ptr<Conflict> conflict(new Conflict());
			conflict->edgeConflict(a1, a2, loc1, loc2, (int)timestep + 1);
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict); // edge conflict
		}
	}
	if (paths[a1]->size() != paths[a2]->size())
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;
		for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{
				shared_ptr<Conflict> conflict(new Conflict());
				if (target_reasoning)
					conflict->targetConflict(a1_, a2_, loc1, timestep);
				else
					conflict->vertexConflict(a1_, a2_, loc1, timestep);
				assert(!conflict->constraint1.empty());
				assert(!conflict->constraint2.empty());
				curr.unknownConf.push_front(conflict); // It's at least a semi conflict
			}
		}
	}
}


void CBS::findConflicts(CBSNode& curr)
{
	clock_t t = clock();
	if (curr.parent != nullptr)
	{
		// Copy from parent
		list<int> new_agents;
		for (const auto& p : curr.paths)
		{
			new_agents.push_back(p.first);
		}
		copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);

		// detect new conflicts
		for (auto it = new_agents.begin(); it != new_agents.end(); ++it)
		{
			int a1 = *it;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (a1 == a2)
					continue;
				bool skip = false;
				for (auto it2 = new_agents.begin(); it2 != it; ++it2)
				{
					if (*it2 == a2)
					{
						skip = true;
						break;
					}
				}
				findConflicts(curr, a1, a2);
			}
		}
	}
	else
	{
		for (int a1 = 0; a1 < num_of_agents; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(curr, a1, a2);
			}
		}
	}
	// curr.tie_breaking = (int)(curr.unknownConf.size() + curr.conflicts.size());
	runtime_detect_conflicts += (double) (clock() - t) / CLOCKS_PER_SEC;
}


shared_ptr<Conflict> CBS::chooseConflict(const CBSNode& node) const
{
	if (screen == 3)
		printConflicts(node);
	shared_ptr<Conflict> choose;
	if (node.conflicts.empty() && node.unknownConf.empty())
		return nullptr;
	else if (!node.conflicts.empty())
	{
		choose = node.conflicts.back();
		for (const auto& conflict : node.conflicts)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	else
	{
		choose = node.unknownConf.back();
		for (const auto& conflict : node.unknownConf)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	return choose;
}


void CBS::computePriorityForConflict(Conflict& conflict, const CBSNode& node)
{
    conflict.secondary_priority = 0; // random
    /*switch (conflict.type) // Earliest
    {
        case conflict_type::STANDARD:
        case conflict_type::RECTANGLE:
        case conflict_type::TARGET:
        case conflict_type::MUTEX:
            conflict.secondary_priority = get<3>(conflict.constraint1.front());
            break;
        case conflict_type::CORRIDOR:
            conflict.secondary_priority = min(get<2>(conflict.constraint1.front()),
                                              get<3>(conflict.constraint1.front()));
            break;
    }*/
}


void CBS::classifyConflicts(CBSNode& node)
{
	// Classify all conflicts in unknownConf
	while (!node.unknownConf.empty())
	{
		shared_ptr<Conflict> con = node.unknownConf.front();
		int a1 = con->a1, a2 = con->a2;
		int a, loc1, loc2, timestep;
		constraint_type type;
		tie(a, loc1, loc2, timestep, type) = con->constraint1.back();
		node.unknownConf.pop_front();


		bool cardinal1 = false, cardinal2 = false;
		if (timestep >= (int) paths[a1]->size())
			cardinal1 = true;
		else //if (!paths[a1]->at(0).is_single())
		{
			mdd_helper.findSingletons(node, a1, *paths[a1]);
		}
		if (timestep >= (int) paths[a2]->size())
			cardinal2 = true;
		else //if (!paths[a2]->at(0).is_single())
		{
			mdd_helper.findSingletons(node, a2, *paths[a2]);
		}

		if (type == constraint_type::EDGE) // Edge conflict
		{
			cardinal1 = paths[a1]->at(timestep).is_single() && paths[a1]->at(timestep - 1).is_single();
			cardinal2 = paths[a2]->at(timestep).is_single() && paths[a2]->at(timestep - 1).is_single();
		}
		else // vertex conflict or target conflict
		{
			if (!cardinal1)
				cardinal1 = paths[a1]->at(timestep).is_single();
			if (!cardinal2)
				cardinal2 = paths[a2]->at(timestep).is_single();
		}

		/*int width_1 = 1, width_2 = 1;

		if (paths[a1]->size() > timestep){
		  width_1 = paths[a1]->at(timestep).mdd_width;
		}

		if (paths[a2]->size() > timestep){
		  width_2 = paths[a2]->at(timestep).mdd_width;
		}
		con -> mdd_width = width_1 * width_2;*/

		if (cardinal1 && cardinal2)
		{
			con->priority = conflict_priority::CARDINAL;
		}
		else if (cardinal1 || cardinal2)
		{
			con->priority = conflict_priority::SEMI;
		}
		else
		{
			con->priority = conflict_priority::NON;
		}

		// Corridor reasoning
		auto corridor = corridor_helper.run(con, paths, node);
		if (corridor != nullptr)
		{
			corridor->priority = con->priority;
			computePriorityForConflict(*corridor, node);
			node.conflicts.push_back(corridor);
			continue;
		}

		// Target Reasoning
		if (con->type == conflict_type::TARGET)
		{
			computePriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			continue;
		}

		// Rectangle reasoning
		if (rectangle_helper.strategy != rectangle_strategy::NR &&
			(int) paths[con->a1]->size() > timestep &&
			(int) paths[con->a2]->size() > timestep && //conflict happens before both agents reach their goal locations
			type == constraint_type::VERTEX && // vertex conflict
			con->priority != conflict_priority::CARDINAL) // not caridnal vertex conflict
		{
			auto mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
			auto rectangle = rectangle_helper.run(paths, timestep, a1, a2, mdd1, mdd2);
			if (rectangle != nullptr)
			{
				computePriorityForConflict(*rectangle, node);
				node.conflicts.push_back(rectangle);
				continue;
			}
		}

		// Mutex reasoning
		if (mutex_reasoning)
		{
			// TODO mutex reasoning is per agent pair, don't do duplicated work...
			auto mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());

			auto mutex_conflict = mutex_helper.run(paths, a1, a2, node, mdd1, mdd2);

			if (mutex_conflict != nullptr && (*mutex_conflict != *con)) // ignore the cases when mutex finds the same vertex constraints
			{
				computePriorityForConflict(*mutex_conflict, node);
				node.conflicts.push_back(mutex_conflict);
				continue;
			}
		}

		computePriorityForConflict(*con, node);
		node.conflicts.push_back(con);
	}


	// remove conflicts that cannot be chosen, to save some memory
	removeLowPriorityConflicts(node.conflicts);
}


void CBS::removeLowPriorityConflicts(list<shared_ptr<Conflict>>& conflicts) const
{
	if (conflicts.empty())
		return;
	unordered_map<int, shared_ptr<Conflict> > keep;
	list<shared_ptr<Conflict>> to_delete;
	for (const auto& conflict : conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2), a2 = max(conflict->a1, conflict->a2);
		int key = a1 * num_of_agents + a2;
		auto p = keep.find(key);
		if (p == keep.end())
		{
			keep[key] = conflict;
		}
		else if (*(p->second) < *conflict)
		{
			to_delete.push_back(p->second);
			keep[key] = conflict;
		}
		else
		{
			to_delete.push_back(conflict);
		}
	}

	for (const auto& conflict : to_delete)
	{
		conflicts.remove(conflict);
	}
}


bool CBS::findPathForSingleAgent(CBSNode* node, int ag, int lowerbound)
{
	clock_t t = clock();
	// build reservation table
	// CAT cat(node->makespan + 1);  // initialized to false
	// updateReservationTable(cat, ag, *node);
	// find a path
	Path new_path = search_engines[ag]->findPath(*node, initial_constraints[ag], paths, ag, lowerbound);
	num_LL_expanded += search_engines[ag]->num_expanded;
	num_LL_generated += search_engines[ag]->num_generated;
	runtime_build_CT += search_engines[ag]->runtime_build_CT;
	runtime_build_CAT += search_engines[ag]->runtime_build_CAT;
	runtime_path_finding += (double) (clock() - t) / CLOCKS_PER_SEC;
	if (!new_path.empty())
	{
		assert(!isSamePath(*paths[ag], new_path));
		node->paths.emplace_back(ag, new_path);
		node->g_val = node->g_val - (int) paths[ag]->size() + (int) new_path.size();
		paths[ag] = &node->paths.back().second;
		node->makespan = max(node->makespan, new_path.size() - 1);
		return true;
	}
	else
	{
		return false;
	}
}


bool CBS::generateChild(CBSNode* node, CBSNode* parent)
{
	clock_t t1 = clock();
	node->parent = parent;
	node->g_val = parent->g_val;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;
	int agent, x, y, t;
	constraint_type type;
	assert(node->constraints.size() > 0);
	tie(agent, x, y, t, type) = node->constraints.front();

//    cout << "generateChild type: " << type << endl;
	if (type == constraint_type::LEQLENGTH)
	{
		assert(node->constraints.size() <= 2);
		if ((int)node->constraints.size() == 2) // generated by corridor-target conflict
		{
			int a = get<0>(node->constraints.back()); // it is a G-length constraint or a range constraint on this agent
			int lowerbound = (int)paths[a]->size() - 1;
			if (!findPathForSingleAgent(node, a, lowerbound))
			{
				runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
				return false;
			}
		}
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
			{
				continue;
			}
			for (int i = t; i < (int) paths[ag]->size(); i++)
			{
				if (paths[ag]->at(i).location == x)
				{
					int lowerbound = (int) paths[ag]->size() - 1;
					if (!findPathForSingleAgent(node, ag, lowerbound))
					{
						runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
						return false;
					}
					break;
				}
			}
		}
	}
	else if (type == constraint_type::POSITIVE_VERTEX)
	{
		assert(node->constraints.size() == 1);
		for (const auto& constraint : node->constraints)
		{
			tie(agent, x, y, t, type) = constraint;
			for (int ag = 0; ag < num_of_agents; ag++)
			{
				if (ag == agent)
				{
					continue;
				}
				if (getAgentLocation(ag, t) == x)
				{
					if (!findPathForSingleAgent(node, ag, (int) paths[ag]->size() - 1))
					{
						runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
						return false;
					}
				}
			}
		}

	}
	else if (type == constraint_type::POSITIVE_EDGE)
	{
		assert(node->constraints.size() == 1);
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
			{
				continue;
			}
			int curr = getAgentLocation(ag, t);
			int prev = getAgentLocation(ag, t - 1);
			if (prev == x || curr == y ||
				(prev == y && curr == x))
			{
				if (!findPathForSingleAgent(node, ag, (int) paths[ag]->size() - 1))
				{
					runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
					return false;
				}
			}
		}

	}
	else
	{
		int lowerbound = (int) paths[agent]->size() - 1;
		if (!findPathForSingleAgent(node, agent, lowerbound))
		{
			runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	assert(!node->paths.empty());
	findConflicts(*node);
	heuristic_helper.computeQuickHeuristics(*node);
	runtime_generate_child += (double) (clock() - t1) / CLOCKS_PER_SEC;
	return true;
}


inline void CBS::pushNode(CBSNode* node)
{
	// update handles
	node->open_handle = open_list.push(node);
	num_HL_generated++;
	node->time_generated = num_HL_generated;

//    cout << "pushNode:" << endl;
//    cout << "node->g_val: " << node->g_val << endl;
//    cout << "node->h_val: " << node->h_val << endl;
//    cout << "focal_list_threshold: " << focal_list_threshold << endl;

	if (node->g_val + node->h_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	allNodes_table.push_back(node);
}


void CBS::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			 paths[i]->size() - 1 << "): ";
		for (const auto& t : *paths[i])
			cout << t.location << "->";
		cout << endl;
	}
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void CBS::updateFocalList()
{
	CBSNode* open_head = open_list.top();

//    if (open_head->g_val + open_head->h_val > min_f_val) {
//
//
//        cout << "v: " << open_head->g_val + open_head->h_val << ", " << min_f_val << endl;
//        cout << "open_list.size(): " << open_list.size() << endl;
//        cout << "focal_list.size(): " << focal_list.size() << endl;
//    }

    if (open_head->g_val + open_head->h_val > min_f_val)
	{
		if (screen == 3)
		{
			cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
		}
		min_f_val = open_head->g_val + open_head->h_val;
		double new_focal_list_threshold = min_f_val * focal_w;
		for (CBSNode* n : open_list)
		{
//            cout << endl;
//            cout << "n->g_val + n->h_val: " << n->g_val + n->h_val << endl;
//            cout << "focal_list_threshold: " << focal_list_threshold << endl;
//            cout << "new_focal_list_threshold: " << new_focal_list_threshold << endl;
//            cout << "min_f_val: " << min_f_val << endl;
//            cout << "focal_w: " << focal_w << endl;

			if (n->g_val + n->h_val > focal_list_threshold &&
				n->g_val + n->h_val <= new_focal_list_threshold)
            {
                n->focal_handle = focal_list.push(n);
            }

		}
		focal_list_threshold = new_focal_list_threshold;
		if (screen == 3)
		{
			cout << focal_list.size() << endl;
		}
	}


//    if(focal_list.size() == 0) {
//        cout << "v: " << open_head->g_val + open_head->h_val << ", " << min_f_val << endl;
//        cout << "open_list.size(): " << open_list.size() << endl;
//        cout << "focal_list.size(): " << focal_list.size() << endl;
//    }

    if(focal_list.size() == 0) {
        focal_list.push(open_head);
    }

//    if(focal_list.size() == 0) {
//        for (CBSNode* n : open_list)
//        {
////            n->focal_handle = focal_list.push(n);
//            cout << "n->depth: " << n->depth << endl;
//            if(n->depth == 1) {
//                n->focal_handle = focal_list.push(n);
//                break;
//            }
//        }
//
//        cout << "focal_list.size(): " << focal_list.size() << endl;
//
//        CBSNode* test_node = focal_list.top();
//        cout << "test_node->conflicts.size(): " << test_node->conflicts.size() << endl;
//        cout << "test_node->unknownConf.size(): " << test_node->unknownConf.size() << endl;
////        focal_list.push(open_list.top());
//        findConflicts(*test_node);
//        cout << "test_node->conflicts.size(): " << test_node->conflicts.size() << endl;
//        cout << "test_node->unknownConf.size(): " << test_node->unknownConf.size() << endl;
//
//    }
}


void CBS::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Optimal,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	cout << solution_cost << "," << runtime << "," <<
		 num_HL_expanded << "," << num_LL_expanded << "," << // HL_num_generated << "," << LL_num_generated << "," <<
		 min_f_val << "," << dummy_start->g_val << "," << dummy_start->g_val + dummy_start->h_val << "," <<
		 endl;

//    if (solution_cost >= 0) {
//        printPaths();
//    }

}

void CBS::saveResults(const string& fileName, const string& instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
				 "solution cost,min f value,root g value,root f value," <<
				 "#adopt bypasses," <<
				 "standard conflicts,rectangle conflicts,corridor conflicts,target conflicts,mutex conflicts," <<
				 "#merge MDDs,#solve 2 agents,#memoization," <<
				 "runtime of building heuristic graph,runtime of solving MVC," <<
				 "runtime of detecting conflicts," <<
				 "runtime of rectangle conflicts,runtime of corridor conflicts,runtime of mutex conflicts," <<
				 "runtime of building MDDs,runtime of building constraint tables,runtime of building CATs," <<
				 "runtime of path finding,runtime of generating child nodes," <<
				 "preprocessing runtime,solver name,instance name" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," <<
		  num_HL_expanded << "," << num_HL_generated << "," <<
		  num_LL_expanded << "," << num_LL_generated << "," <<

		  solution_cost << "," << min_f_val << "," << dummy_start->g_val << "," << dummy_start->g_val + dummy_start->h_val << "," <<

		  num_adopt_bypass << "," <<

		  num_standard_conflicts << "," << num_rectangle_conflicts << "," << num_corridor_conflicts << "," <<
		  num_target_conflicts << "," << num_mutex_conflicts << "," <<

		  heuristic_helper.num_merge_MDDs << "," <<
		  heuristic_helper.num_solve_2agent_problems << "," <<
		  heuristic_helper.num_memoization << "," <<
		  heuristic_helper.runtime_build_dependency_graph << "," <<
		  heuristic_helper.runtime_solve_MVC << "," <<

		  runtime_detect_conflicts << "," <<
		  rectangle_helper.accumulated_runtime << "," << corridor_helper.accumulated_runtime << "," << mutex_helper.accumulated_runtime << "," <<
		  mdd_helper.accumulated_runtime << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		  runtime_path_finding << "," << runtime_generate_child << "," <<

		  runtime_preprocessing << "," << getSolverName() << "," << instanceName << endl;
	stats.close();
}

void CBS::saveStats(const string &fileName, const string &instanceName) const
{
	ofstream stats(fileName + ".txt", std::ios::app);
	stats << instanceName << endl;
	stats << "agent 1,agent 2,node id,#expanded nodes, h" << endl;
	for (auto ins : heuristic_helper.sub_instances)
	{
		stats << get<0>(ins) << "," << get<1>(ins) << "," << get<2>(ins)->time_generated << "," << get<3>(ins) << "," << get<4>(ins) << endl;
	}
	stats.close();
	cout << "Write " << heuristic_helper.sub_instances.size() << " samples to files" << endl;
}


void CBS::saveCT(const string &fileName) const // write the CT to a file
{
	std::ofstream output;
	output.open(fileName, std::ios::out);
	output << "digraph G {" << endl;
	output << "size = \"5,5\";" << endl;
	output << "center = true;" << endl;
	for (auto node : allNodes_table)
	{
		output << node->time_generated << " [label=\"#" << node->time_generated
					<< "\ng+h="<< node->g_val << "+" << node->h_val
					<< "\nd=" << node->tie_breaking << "\"]" << endl;
		if (node == dummy_start)
			continue;
		output << node->parent->time_generated << " -> " << node->time_generated << " [label=\"";
		for (const auto &constraint : node->constraints)
			output << constraint;
		output << "\nAgents ";
        for (const auto &path : node->paths)
            output << path.first << "(+" << path.second.size() - paths_found_initially[path.first].size() << ") ";
        output << "\"]" << endl;
	}
	auto node = goal_node;
	while (node != nullptr)
	{
		output << node->time_generated << " [color=red]" << endl;
		node = node->parent;
	}
	output << "}" << endl;
	output.close();
}

void CBS::savePaths(const string &fileName) const
{
    std::ofstream output;
    output.open(fileName, std::ios::out);
    for (int i = 0; i < num_of_agents; i++)
    {
        output << "Agent " << i << ": ";
        for (const auto & t : *paths[i])
            output << "(" << search_engines[0]->instance.getRowCoordinate(t.location)
                   << "," << search_engines[0]->instance.getColCoordinate(t.location) << ")->";
        output << endl;
    }
    output.close();
}

void CBS::printConflicts(const CBSNode &curr)
{
	for (const auto& conflict : curr.conflicts)
	{
		cout << *conflict << endl;
	}
	for (const auto& conflict : curr.unknownConf)
	{
		cout << *conflict << endl;
	}
}


string CBS::getSolverName() const
{
	string name;
	if (disjoint_splitting)
		name += "Disjoint ";
	switch (heuristic_helper.type)
	{
	case heuristics_type::ZERO:
		if (PC)
			name += "ICBS";
		else
			name += "CBS";
		break;
	case heuristics_type::CG:
		name += "CG";
		break;
	case heuristics_type::DG:
		name += "DG";
		break;
	case heuristics_type::WDG:
		name += "WDG";
		break;
	case STRATEGY_COUNT:
		break;
	}
	if (rectangle_helper.getName() != "NR")
		name += "+" + rectangle_helper.getName();
	if (corridor_helper.getName() != "NC")
    	name += "+" + corridor_helper.getName();
	if (target_reasoning)
		name += "+T";
	if (mutex_reasoning)
		name += "+M";
	if (bypass)
		name += "+BP";
	name += " with " + search_engines[0]->getName();
	return name;
}



void CBS::initSolveParams(double _time_limit, int _cost_lowerbound, int _cost_upperbound)
{
    this->min_f_val = _cost_lowerbound;
    this->cost_upperbound = _cost_upperbound;
    this->time_limit = _time_limit;
}


bool CBS::solve(double _time_limit, int _cost_lowerbound, int _cost_upperbound)
{


	this->min_f_val = _cost_lowerbound;
	this->cost_upperbound = _cost_upperbound;
	this->time_limit = _time_limit;

	if (screen > 0) // 1 or 2
	{
		string name = getSolverName();
		name.resize(35, ' ');
		cout << name << ": ";
	}
//	 set timer
	start = clock();

//	generateRoot();



    ///
    num_HL_expanded = 0;
    num_HL_generated = 0;
    num_of_high_level_nodes_in_open_on_start = open_list.size();
    ///

	while (!open_list.empty() && !solution_found)
	{
		updateFocalList();
		if (min_f_val >= cost_upperbound)
		{
			solution_cost = (int) min_f_val;
			solution_found = false;
			break;
		}
		runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
		if (runtime > time_limit || num_HL_expanded > node_limit
		    || heuristic_helper.sub_instances.size() >= MAX_NUM_STATS)
		{  // time/node out
			solution_cost = -1;
			solution_found = false;
			break;
		}


		CBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		updatePaths(curr);

		if (screen > 1)
			cout << endl << "Pop " << *curr << endl;

		if (curr->unknownConf.size() + curr->conflicts.size() == 0) //no conflicts
		{// found a solution (and finish the while look)
			solution_found = true;
			solution_cost = curr->g_val;
			goal_node = curr;
			break;
		}

		// debug
		/*if (curr->time_generated == 15 && num_of_agents > 2)
		{
			int a1 = 51, a2 = 54;
			auto mdd1 = mdd_helper.getMDD(*curr, a1, paths[a1]->size());
			cout << "The mdd of agent " << a1 << endl;
			mdd1->printNodes();
			auto mdd2 = mdd_helper.getMDD(*curr, a2, paths[a2]->size());
			cout << "The mdd of agent " << a2 << endl;
			mdd2->printNodes();
			for (int t = 0; t < min(paths[a1]->size(), paths[a2]->size()); t++)
			{
				if (paths[a1]->at(t).location == paths[a2]->at(t).location)
					rectangle_helper.printOverlapArea(paths, t, a1, a2, mdd1, mdd2);
			}
			cout << "The constraints " << endl;
			curr->printConstraints(a1);
			curr->printConstraints(a2);
		}*/
		/*if (curr->time_generated == 1 && num_of_agents > 2)
		{
			int a1 = 12, a2 = 23;
			auto mdd1 = mdd_helper.getMDD(*curr, a1, paths[a1]->size());
			cout << "The mdd of agent " << a1 << endl;
			mdd1->printNodes();
			auto mdd2 = mdd_helper.getMDD(*curr, a2, paths[a2]->size());
			cout << "The mdd of agent " << a2 << endl;
			mdd2->printNodes();
			for (int t = 0; t < min(paths[a1]->size(), paths[a2]->size()); t++)
			{
				if (paths[a1]->at(t).location == paths[a2]->at(t).location)
					rectangle_helper.printOverlapArea(paths, t, a1, a2, mdd1, mdd2);
			}
			cout << "The constraints " << endl;
			curr->printConstraints(a1);
			curr->printConstraints(a2);
		}*/

		if (!curr->h_computed) // heuristics has not been computed yet
		{
			if (PC) // prioritize conflicts
				classifyConflicts(*curr);
			runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
			bool succ = heuristic_helper.computeInformedHeuristics(*curr, time_limit - runtime);
			runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
			if (runtime > time_limit)
			{  // timeout
				solution_cost = -1;
				solution_found = false;
				break;
			}
			if (!succ) // no solution, so prune this node
			{
				curr->clear();
				continue;
			}

			// reinsert the node
			curr->open_handle = open_list.push(curr);
			if (curr->g_val + curr->h_val <= focal_list_threshold)
				curr->focal_handle = focal_list.push(curr);
			if (screen == 2)
			{
				cout << "	Reinsert " << *curr << endl;
			}
			continue;
		}



		//Expand the node
		num_HL_expanded++;
		curr->time_expanded = num_HL_expanded;
		bool foundBypass = true;
		while (foundBypass)
		{
			if (curr->unknownConf.size() + curr->conflicts.size() == 0) //no conflicts
			{// found a solution (and finish the while look)
				solution_found = true;
				solution_cost = curr->g_val;
				goal_node = curr;
				break;
			}
			foundBypass = false;
			CBSNode* child[2] = { new CBSNode(), new CBSNode() };

			curr->conflict = chooseConflict(*curr);

			if (disjoint_splitting && curr->conflict->type == conflict_type::STANDARD)
			{
				int first = (bool) (rand() % 2);
				if (first) // disjoint splitting on the first agent
				{
					child[0]->constraints = curr->conflict->constraint1;
					int a, x, y, t;
					constraint_type type;
					tie(a, x, y, t, type) = curr->conflict->constraint1.back();
					if (type == constraint_type::VERTEX)
					{
						child[1]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
					}
					else
					{
						assert(type == constraint_type::EDGE);
						child[1]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
					}
				}
				else // disjoint splitting on the second agent
				{
					child[1]->constraints = curr->conflict->constraint2;
					int a, x, y, t;
					constraint_type type;
					tie(a, x, y, t, type) = curr->conflict->constraint2.back();
					if (type == constraint_type::VERTEX)
					{
						child[0]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
					}
					else
					{
						assert(type == constraint_type::EDGE);
						child[0]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
					}
				}
			}
			else
			{
				child[0]->constraints = curr->conflict->constraint1;
				child[1]->constraints = curr->conflict->constraint2;
				if (curr->conflict->type == conflict_type::RECTANGLE && rectangle_helper.strategy == rectangle_strategy::DR)
				{
					int i = (bool)(rand() % 2);
					for (const auto constraint : child[1 - i]->constraints)
					{
						child[i]->constraints.emplace_back(get<0>(constraint), get<1>(constraint), get<2>(constraint), get<3>(constraint),
																							constraint_type::POSITIVE_BARRIER);
					}
				}
				else if (curr->conflict->type == conflict_type::CORRIDOR && corridor_helper.getStrategy() == corridor_strategy::DC)
				{
					int i = (bool)(rand() % 2);
					assert(child[1 - i]->constraints.size() == 1);
					auto constraint = child[1 - i]->constraints.front();
					child[i]->constraints.emplace_back(get<0>(constraint), get<1>(constraint), get<2>(constraint), get<3>(constraint),
						constraint_type::POSITIVE_RANGE);
				}
			}

			if (screen > 1)
				cout << "	Expand " << *curr << endl <<
					 "	on " << *(curr->conflict) << endl;

			bool solved[2] = { false, false };
			vector<vector<PathEntry>*> copy(paths);

			for (int i = 0; i < 2; i++)
			{
				if (i > 0)
					paths = copy;
				solved[i] = generateChild(child[i], curr);
				if (!solved[i])
				{
					delete child[i];
					continue;
				}
				if (child[i]->g_val + child[i]->h_val == min_f_val && curr->unknownConf.size() + curr->conflicts.size() == 0) //no conflicts
				{// found a solution (and finish the while look)
					break;
				}
				else if (bypass && child[i]->g_val == curr->g_val && child[i]->tie_breaking < curr->tie_breaking) // Bypass1
				{
					if (i == 1 && !solved[0])
						continue;
					foundBypass = true;
					num_adopt_bypass++;
					curr->conflicts = child[i]->conflicts;
					curr->unknownConf = child[i]->unknownConf;
					curr->tie_breaking = child[i]->tie_breaking;
					curr->conflict = nullptr;
					for (const auto& path : child[i]->paths) // update paths
					{
						auto p = curr->paths.begin();
						while (p != curr->paths.end())
						{
							if (path.first == p->first)
							{
								p->second = path.second;
								paths[p->first] = &p->second;
								break;
							}
							++p;
						}
						if (p == curr->paths.end())
						{
							curr->paths.emplace_back(path);
							paths[path.first] = &curr->paths.back().second;
						}
					}
					if (screen > 1)
					{
						cout << "	Update " << *curr << endl;
					}
					break;
				}
			}
			if (foundBypass)
			{
				for (auto & i : child)
				{
					delete i;
					i = nullptr;
				}
                if (PC) // prioritize conflicts
                    classifyConflicts(*curr); // classify the new-detected conflicts
			}
			else
			{
				for (int i = 0; i < 2; i++)
				{
					if (solved[i])
					{
                        child[i]->label = label;
						pushNode(child[i]);
						if (screen > 1)
						{
							cout << "		Generate " << *child[i] << endl;
						}
					}
				}
			}
		}
		if (curr->conflict != nullptr)
		{
			switch (curr->conflict->type)
			{
			case conflict_type::RECTANGLE:
				num_rectangle_conflicts++;
				break;
			case conflict_type::CORRIDOR:
				num_corridor_conflicts++;
				break;
			case conflict_type::TARGET:
				num_target_conflicts++;
				break;
			case conflict_type::STANDARD:
				num_standard_conflicts++;
				break;
			case conflict_type::MUTEX:
				num_mutex_conflicts++;
				break;
			}
		}
		curr->clear();
	}  // end of while loop


	runtime = (double) (clock() - start) / CLOCKS_PER_SEC;

    ///
    num_of_high_level_nodes_expansion = num_HL_expanded;
    num_of_high_level_nodes_expansion_generated = num_HL_generated;
    num_of_high_level_nodes_in_open_on_finish = open_list.size();
    ///

	if (solution_found && !validateSolution())
	{
		cout << "Solution invalid!!!" << endl;
		printPaths();
		exit(-1);
	}
	if (screen == 2)
        printPaths();
	if (screen > 0) // 1 or 2
		printResults();
	return solution_found;
}



CBS::CBS(vector<SingleAgentSolver*>& search_engines,
		 const vector<ConstraintTable>& initial_constraints,
         vector<Path>& paths_found_initially, int screen) :
		screen(screen), focal_w(1),
		initial_constraints(initial_constraints), paths_found_initially(paths_found_initially),
		search_engines(search_engines),
		mdd_helper(initial_constraints, search_engines),
		rectangle_helper(search_engines[0]->instance),
		mutex_helper(search_engines[0]->instance, initial_constraints),
		corridor_helper(search_engines, initial_constraints),
		heuristic_helper(search_engines.size(), paths, search_engines, initial_constraints, mdd_helper)
{
	num_of_agents = (int) search_engines.size();
	mutex_helper.search_engines = search_engines;
}


CBS::CBS(Instance& instance, bool sipp, int screen) :
        screen(screen), focal_w(1),
        num_of_agents(instance.getDefaultNumberOfAgents()),
        mdd_helper(initial_constraints, search_engines),
        rectangle_helper(instance),
        mutex_helper(instance, initial_constraints),
        corridor_helper(search_engines, initial_constraints),
        heuristic_helper(instance.getDefaultNumberOfAgents(), paths, search_engines, initial_constraints, mdd_helper)
{
    clock_t t = clock();
    initial_constraints.resize(num_of_agents,
                               ConstraintTable(instance.num_of_cols, instance.map_size));

    search_engines.resize(num_of_agents);

    std::cout << "num_of_agents: " << num_of_agents << std::endl;

    for (int i = 0; i < num_of_agents; i++)
    {
        if (sipp)
            search_engines[i] = new SIPP(instance, i);
        else
            search_engines[i] = new SpaceTimeAStar(instance, i);

        initial_constraints[i].goal_location = search_engines[i]->goal_location;
    }
    runtime_preprocessing = (double) (clock() - t) / CLOCKS_PER_SEC;

    mutex_helper.search_engines = search_engines;

    if (screen >= 2) // print start and goals
    {
        instance.printAgents();
    }
}


CBS::CBS(Instance& instance, int screen):
        screen(screen), focal_w(1),
        num_of_agents(instance.getDefaultNumberOfAgents()),
        mdd_helper(initial_constraints, search_engines),
        rectangle_helper(instance),
        mutex_helper(instance, initial_constraints),
        corridor_helper(search_engines, initial_constraints),
        heuristic_helper(instance.getDefaultNumberOfAgents(), paths, search_engines, initial_constraints, mdd_helper)
{
    clock_t t = clock();
    initial_constraints.resize(num_of_agents,
                               ConstraintTable(instance.num_of_cols, instance.map_size));

    search_engines.resize(num_of_agents);

    std::cout << "num_of_agents: " << num_of_agents << std::endl;

    for (int i = 0; i < num_of_agents; i++)
    {

        search_engines[i] = new SpaceTimeAStar(instance, i);

        initial_constraints[i].goal_location = search_engines[i]->goal_location;
    }
    runtime_preprocessing = (double) (clock() - t) / CLOCKS_PER_SEC;

    mutex_helper.search_engines = search_engines;

    if (screen >= 2) // print start and goals
    {
        instance.printAgents();
    }

}


void CBS::dijkstra()
{

    for (int i = 0; i < num_of_agents; i++)
    {

//        search_engines[i] = new SpaceTimeAStar(instance, i);
//        initial_constraints[i].goal_location = search_engines[i]->goal_location;

//        cout << "search_engines[i]->instance.printMap()" << endl;
//        search_engines[i]->instance.printMap();
        search_engines[i]->compute_heuristics();
    }


}


bool CBS::solve(Instance& instance, vector<DynamicObstacle>& obstacle_delete_v, vector<DynamicObstacle>& obstacle_add_v){

    num_HL_expanded = 0;
    num_HL_generated = 0;
    num_of_high_level_nodes_in_open_on_start = open_list.size();

    /// obstacle delete case
    solveObstacleDeleted(instance, obstacle_delete_v);

    /// obstacle added case
//    update_heuristic_values(instance);
    update_add_obstacles_set(instance, obstacle_add_v);

    markNodeForAddObstacles(obstacle_add_v);

    //////////////////////////////////////////////////////////
    std::cout << "open_list.size(): " << open_list.size() << " " << !open_list.empty() << std::endl;

//     set timer
    start = clock();

    solution_found = false;
    solution_cost = -2;

    while (!open_list.empty() && !solution_found)
    {

        updateFocalList();
        if (min_f_val >= cost_upperbound)
        {
            solution_cost = (int) min_f_val;
            solution_found = false;
            break;
        }
        runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
        if (runtime > time_limit || num_HL_expanded > node_limit
            || heuristic_helper.sub_instances.size() >= MAX_NUM_STATS)
        {  // time/node out
            solution_cost = -1;
            solution_found = false;
            break;
        }

        CBSNode* curr = focal_list.top();
        focal_list.pop();
//        open_list.erase(curr->open_handle);

//        CBSNode* curr = open_list.top();


        // takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
        updatePaths(curr);

        if (screen > 1)
            cout << endl << "Pop " << *curr << endl;


        ObstacleAddEnum obstacle_add_update_sol = recomputePathCost(instance, curr, obstacle_add_v);
        if (obstacle_add_update_sol == ObstacleAddEnum::CONINTUE) {
//            cout << "curr->constraints.size(): " << curr->constraints.size() << endl;
//            if (curr->unknownConf.size() + curr->conflicts.size() == 0) //no conflicts
//            {// found a solution (and finish the while look)
//                cout << "curr->g_val: " << curr->g_val  << endl;
//                cout << "curr->h_computed: " << curr->h_computed  << endl;
//                solution_found = true;
//                solution_cost = curr->g_val;
//                goal_node = curr;
//                break;
//            }
            continue;
        }
        else if (obstacle_add_update_sol == ObstacleAddEnum::ERASE_NODE) {
            /// it means that the added obstacles are valid, but we can't find solution.
            open_list.erase(curr->open_handle);
            continue;
        }

//        cout << "(double) (clock() - start) / CLOCKS_PER_SEC: " << (double) (clock() - start) / CLOCKS_PER_SEC << endl;


        if (curr->unknownConf.size() + curr->conflicts.size() == 0) //no conflicts
        {// found a solution (and finish the while look)
            solution_found = true;
            solution_cost = curr->g_val;
            goal_node = curr;
            break;
        }


        if (!curr->h_computed) // heuristics has not been computed yet
        {
            runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
            bool succ = heuristic_helper.computeInformedHeuristics(*curr, time_limit - runtime);
            runtime = (double) (clock() - start) / CLOCKS_PER_SEC;
            if (runtime > time_limit)
            {  // timeout
                solution_cost = -1;
                solution_found = false;
                break;
            }
            if (!succ) // no solution, so prune this node
            {
                curr->clear();
                continue;
            }

            // reinsert the node
            open_list.erase(curr->open_handle);
            curr->open_handle = open_list.push(curr);
            if (curr->g_val + curr->h_val <= focal_list_threshold) {
                curr->focal_handle = focal_list.push(curr);
            }
            if (screen == 2)
            {
                cout << "	Reinsert " << *curr << endl;
            }
            continue;
        }

        //Expand the node
        num_HL_expanded++;
        curr->time_expanded = num_HL_expanded;
        bool foundBypass = true;
        int foundBypass_count = 0;
        while (foundBypass)
        {
            foundBypass_count++;

            if (curr->unknownConf.size() + curr->conflicts.size() == 0) //no conflicts
            {// found a solution (and finish the while look)
                solution_found = true;
                solution_cost = curr->g_val;
                goal_node = curr;
                break;
            }

            open_list.erase(curr->open_handle);

            foundBypass = false;
            CBSNode* child[2] = { new CBSNode(), new CBSNode() };

            curr->conflict = chooseConflict(*curr);


//            child[0]->constraints = curr->conflict->constraint1;
//            child[1]->constraints = curr->conflict->constraint2;


            if (disjoint_splitting && curr->conflict->type == conflict_type::STANDARD)
            {
                int first = (bool) (rand() % 2);
                if (first) // disjoint splitting on the first agent
                {
                    child[0]->constraints = curr->conflict->constraint1;
                    int a, x, y, t;
                    constraint_type type;
                    tie(a, x, y, t, type) = curr->conflict->constraint1.back();
                    if (type == constraint_type::VERTEX)
                    {
                        child[1]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
                    }
                    else
                    {
                        assert(type == constraint_type::EDGE);
                        child[1]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
                    }
                }
                else // disjoint splitting on the second agent
                {
                    child[1]->constraints = curr->conflict->constraint2;
                    int a, x, y, t;
                    constraint_type type;
                    tie(a, x, y, t, type) = curr->conflict->constraint2.back();
                    if (type == constraint_type::VERTEX)
                    {
                        child[0]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
                    }
                    else
                    {
                        assert(type == constraint_type::EDGE);
                        child[0]->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
                    }
                }
            }
            else
            {
                child[0]->constraints = curr->conflict->constraint1;
                child[1]->constraints = curr->conflict->constraint2;
                if (curr->conflict->type == conflict_type::RECTANGLE && rectangle_helper.strategy == rectangle_strategy::DR)
                {
                    int i = (bool)(rand() % 2);
                    for (const auto constraint : child[1 - i]->constraints)
                    {
                        child[i]->constraints.emplace_back(get<0>(constraint), get<1>(constraint), get<2>(constraint), get<3>(constraint),
                                                           constraint_type::POSITIVE_BARRIER);
                    }
                }
                else if (curr->conflict->type == conflict_type::CORRIDOR && corridor_helper.getStrategy() == corridor_strategy::DC)
                {
                    int i = (bool)(rand() % 2);
                    assert(child[1 - i]->constraints.size() == 1);
                    auto constraint = child[1 - i]->constraints.front();
                    child[i]->constraints.emplace_back(get<0>(constraint), get<1>(constraint), get<2>(constraint), get<3>(constraint),
                                                       constraint_type::POSITIVE_RANGE);
                }
            }

            if (screen > 1)
                cout << "	Expand " << *curr << endl <<
                     "	on " << *(curr->conflict) << endl;

            bool solved[2] = { false, false };
            vector<vector<PathEntry>*> copy(paths);

            for (int i = 0; i < 2; i++)
            {
                if (i > 0)
                    paths = copy;
                solved[i] = generateChild(child[i], curr);
                if (!solved[i])
                {
                    delete child[i];
                    continue;
                }
                if (child[i]->g_val + child[i]->h_val == min_f_val && curr->unknownConf.size() + curr->conflicts.size() == 0) //no conflicts
                {// found a solution (and finish the while look)
                    break;
                }
                else if (bypass && child[i]->g_val == curr->g_val && child[i]->tie_breaking < curr->tie_breaking) // Bypass1
                {
                    if (i == 1 && !solved[0])
                        continue;
                    foundBypass = true;
                    num_adopt_bypass++;
                    curr->conflicts = child[i]->conflicts;
                    curr->unknownConf = child[i]->unknownConf;
                    curr->tie_breaking = child[i]->tie_breaking;
                    curr->conflict = nullptr;
                    for (const auto& path : child[i]->paths) // update paths
                    {
                        auto p = curr->paths.begin();
                        while (p != curr->paths.end())
                        {
                            if (path.first == p->first)
                            {
                                p->second = path.second;
                                paths[p->first] = &p->second;
                                break;
                            }
                            ++p;
                        }
                        if (p == curr->paths.end())
                        {
                            curr->paths.emplace_back(path);
                            paths[path.first] = &curr->paths.back().second;
                        }
                    }
                    if (screen > 1)
                    {
                        cout << "	Update " << *curr << endl;
                    }
                    break;
                }
            }
            if (foundBypass)
            {
                for (auto & i : child)
                {
                    delete i;
                    i = nullptr;
                }
                if (PC) // prioritize conflicts
                    classifyConflicts(*curr); // classify the new-detected conflicts
            }
            else
            {
                for (int i = 0; i < 2; i++)
                {
//                    cout << "solved[i] " << i << ": " << solved[i] << endl;
                    if (solved[i])
                    {
                        child[i]->label = label;
                        pushNode(child[i]);
                        if (screen > 1)
                        {
                            cout << "		Generate " << *child[i] << endl;
                        }
                    }

//                    cout << "child[i]->g_val + child[i]->h_val: " << child[i]->g_val + child[i]->h_val << endl;
//                    cout << "child[i]->conflicts.size(): " << child[i]->conflicts.size() << endl;
//                    cout << "child[i]->unknownConf.size(): " << child[i]->unknownConf.size() << endl;
//                    cout << "CCCCCCCCCCCCCCCC" << endl;

//                    for (auto item = child[i]->paths.begin();
//                    item != child[i]->paths.end(); item++) {
//                        cout << item->first << ", " << item->second << endl;
//                    }
                }
//                cout << "DDDDDDDDDDDDDDD" << endl;

            }
        }

//        cout << "############curr->conflict: " << (curr->conflict != nullptr) << endl;
        if (curr->conflict != nullptr)
        {
            switch (curr->conflict->type)
            {
                case conflict_type::RECTANGLE:
                    num_rectangle_conflicts++;
                    break;
                case conflict_type::CORRIDOR:
                    num_corridor_conflicts++;
                    break;
                case conflict_type::TARGET:
                    num_target_conflicts++;
                    break;
                case conflict_type::STANDARD:
                    num_standard_conflicts++;
                    break;
                case conflict_type::MUTEX:
                    num_mutex_conflicts++;
                    break;
            }
        }
//        curr->printConflictGraph(2);
        curr->clear();
    }  // end of while loop

//    cout << "open_list.size(): " << open_list.size() << endl;
//    cout << "solution_found: " << solution_found << endl;

    runtime = (double) (clock() - start) / CLOCKS_PER_SEC;

    num_of_high_level_nodes_expansion = num_HL_expanded;
    num_of_high_level_nodes_expansion_generated = num_HL_generated;
    num_of_high_level_nodes_in_open_on_finish = open_list.size();


//    for(int i = 0; i<12;++i){
//        goal_node->printConstraints(i);
//        cout << endl;
//    }


    if (solution_found && !validateSolution())
    {
        cout << "Solution invalid!!!" << endl;
        printPaths();
        exit(-1);
    }
    if (screen == 2)
        printPaths();
    if (screen > 0) // 1 or 2
        printResults();
    return solution_found;

}

bool CBS::checkViolateObstacle(Instance& instance, CBSNode& curr, vector<DynamicObstacle>& obstacle_add_v){

    if (obstacle_add_v.empty()){
        return false;
    }

    for (int a = 0; a < num_of_agents; a++)
    {
        for(auto item: obstacle_add_v){
            int obstacle_add_loc = instance.getCols() * item.y + item.x;
            size_t path_length = paths[a]->size();
            for (size_t timestep = 0; timestep < path_length; timestep++) {
                int a_loc = paths[a]->at(timestep).location;
                if (obstacle_add_loc == a_loc){
                    return true;
                }
            }
        }
    }
    return false;
}

void CBS::markNodeForAddObstacles(const vector<DynamicObstacle>& obstacle_add_v) {
    if (obstacle_add_v.empty()){
        return;
    }

    focal_list.clear();

//    open_list.clear();
//    CBSNode *n = goal_node;
//    n->open_handle = open_list.push(n);


}

void CBS::update_heuristic_values(Instance& instance) {

    heuristic_values.clear();
    heuristic_values.resize(num_of_agents, MAX_TIMESTEP);
    for(int agent = 0; agent < num_of_agents; ++agent) {
        int start_location = instance.get_start_locations()[agent];
        int new_heuristic = search_engines[agent]->my_heuristic[start_location];
        heuristic_values[agent] = new_heuristic;
    }
}


void CBS::update_add_obstacles_set(Instance& instance, vector<DynamicObstacle>& obstacle_add_v) {

    for(auto item: obstacle_add_v) {
        int obstacle_add_loc = instance.getCols() * item.y + item.x;
        add_obstacles_set.insert(obstacle_add_loc);
    }

}

ObstacleAddEnum CBS::recomputePathCost(Instance& instance, CBSNode* curr, const vector<DynamicObstacle>& obstacle_add_v){

    clock_t t = clock();

    if (add_obstacles_set.empty()) {
        return ObstacleAddEnum::NEXT_STEP;
    }

    if (curr->label >= label) {
        return ObstacleAddEnum::NEXT_STEP;
    }

    bool need_update_path = false;
    vector<bool> violated(num_of_agents, false);
    for (int agent = 0; agent < num_of_agents; ++agent) {
        size_t path_length = paths[agent]->size();
        for (size_t timestep = 0; timestep < path_length; ++timestep) {
            int a_loc = paths[agent]->at(timestep).location;
            if (add_obstacles_set.find(a_loc) != add_obstacles_set.end()) {
                violated[agent] = true;
                need_update_path = true;
                break;
            }
        }
    }


    if(!need_update_path) {
        return ObstacleAddEnum::NEXT_STEP;
    }


    vector<bool> updated(num_of_agents, false);
    CBSNode* node = curr;
    curr->makespan = 0;
    while (node != nullptr) {
        for(auto item_path: node->paths) {
            int agent = item_path.first;
            if (node == curr) {
                curr->makespan = max(curr->makespan, item_path.second.size() - 1);
                updated[agent] = true;
            }
            else if(!updated[agent]) {
                curr->paths.emplace_back(agent, item_path.second);
                curr->makespan = max(curr->makespan, item_path.second.size() - 1);
                updated[agent] = true;
            }
        }
        node = node->parent;
    }

    for(int i = 0; i < num_of_agents; i++) {
        if (!updated[i]) {
            curr->paths.emplace_back(i, paths_found_initially[i]);
            curr->makespan = max(curr->makespan, paths_found_initially[i].size() - 1);
        }
    }


    curr->g_val = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        curr->g_val += (int) paths[i]->size() - 1;
    }

    for(auto& item_path: curr->paths) {
        int agent = item_path.first;
//        if(need_update_path) {
//        if((violated[agent] == true) || (label - curr->label) >= 2) {
//        int lower_bound = (int) paths[agent]->size() - 1;
        if(violated[agent] == true) {
//            clock_t t = clock();

//            min_f_val = 0;
            Path new_path = search_engines[agent]->findPath(*curr, initial_constraints[agent], paths,
                                                            agent, (int) paths[agent]->size() - 1); // (int) paths[agent]->size() - 1

            if (new_path.empty()) {
                return ObstacleAddEnum::ERASE_NODE;
            }

            curr->g_val = curr->g_val - (int)item_path.second.size() + (int)new_path.size();

            item_path.second = new_path;
            paths[agent] = &item_path.second;
            curr->makespan = max(curr->makespan, new_path.size() - 1);

            ///
            num_LL_expanded += search_engines[agent]->num_expanded;
            num_LL_generated += search_engines[agent]->num_generated;
            runtime_build_CT += search_engines[agent]->runtime_build_CT;
            runtime_build_CAT += search_engines[agent]->runtime_build_CAT;
//            runtime_path_finding += (double) (clock() - t) / CLOCKS_PER_SEC;


        }
    }

    findConflicts(*curr);
    curr->label = label;

    open_list.erase(curr->open_handle);
    curr->open_handle = open_list.push(curr);

    start = start + (clock() - t);

//    if (curr->g_val + curr->h_val <= focal_list_threshold)
//        curr->focal_handle = focal_list.push(curr);

    return ObstacleAddEnum::CONINTUE;


}



bool CBS::solveObstacleDeleted(Instance& instance, vector<DynamicObstacle>& obstacle_delete_v){

    if(obstacle_delete_v.empty()) {
        return false;
    }


    vector<int> new_heuristic_values{};
    new_heuristic_values.resize(num_of_agents, MAX_TIMESTEP);
    for(int agent = 0; agent < num_of_agents; ++agent) {
        int start_location = instance.get_start_locations()[agent];
        int new_heuristic = search_engines[agent]->my_heuristic[start_location];
        new_heuristic_values[agent] = new_heuristic;
    }


    recomputePathCost(instance, new_heuristic_values);


    for(auto item: obstacle_delete_v) {
        int obstacle_delete_loc = instance.getCols() * item.y + item.x;

        if (add_obstacles_set.find(obstacle_delete_loc) != add_obstacles_set.end()) {
            add_obstacles_set.erase(obstacle_delete_loc);
        }
    }


    return true;

}

bool CBS::recomputePathCost(Instance& instance, const vector<int>& costs_new) {

    focal_list.clear();
    min_f_val = 0;

    for(CBSNode* node: open_list) {

        vector<bool> updated(num_of_agents, false);
        CBSNode* curr = node;

        node->makespan = 0;
        while (curr != nullptr) {
            for(auto item_path: curr->paths) {
                int agent = item_path.first;
                if (curr == node) {
                    node->makespan = max(node->makespan, item_path.second.size() - 1);
                    updated[agent] = true;
                }

                else if(!updated[agent]) {
                    node->paths.emplace_back(agent, item_path.second);
                    node->makespan = max(node->makespan, item_path.second.size() - 1);
                    updated[agent] = true;
                }
            }
            curr = curr->parent;
        }

        for(int i = 0; i < num_of_agents; i++) {
            if (!updated[i]) {
                node->paths.emplace_back(i, paths_found_initially[i]);
                node->makespan = max(node->makespan, paths_found_initially[i].size() - 1);
            }

        }

        updatePaths(node);

        node->g_val = 0;
        for (int i = 0; i < num_of_agents; i++)
        {
            node->g_val += (int) paths[i]->size() - 1;
        }


        for(auto& item_path: node->paths) {
            int agent = item_path.first;
            int cost_old = (int)item_path.second.size() - 1;
            if (cost_old > costs_new[agent]){

                Path new_path = search_engines[agent]->findPath(*node, initial_constraints[agent],
                                                                paths, agent, costs_new[agent]);
                // (int)paths[agent]->size() - 1
//                node->g_val = node->g_val - cost_old + ((int)new_path.size() - 1);
                node->g_val = node->g_val - (int)item_path.second.size() + (int)new_path.size();

                item_path.second = new_path;
                paths[agent] = &item_path.second;
                node->makespan = max(node->makespan, new_path.size() - 1);

            }
        }

        findConflicts(*node);



    }

    pairing_heap< CBSNode*, compare<CBSNode::compare_node> > new_open_list;
    for(CBSNode* node: open_list) {
        node->open_handle = new_open_list.push(node);
    }

    open_list.clear();
    for(CBSNode* node: new_open_list) {
        node->open_handle = open_list.push(node);
    }


//    for(CBSNode* node: open_list) {
//        cout << "new node->g_val + node->h_val: " << node->g_val + node->h_val << endl;
//    }

    /*
    CBSNode* open_head = open_list.top();
    focal_list.clear();
    min_f_val = (double)open_head->g_val;
    focal_list_threshold = min_f_val * focal_w;
    focal_list.push(open_head);
     */



//    cout << "RecomputePathCost end" << endl;

    return true;
}


bool CBS::checkValidateSolution(Instance& instance) {

    if (solution_cost < 0) {
        return true;
    }

    vector<int> obstacle_locations;
    for(int loc = 0; loc < instance.map_size; ++loc){
        if(instance.isObstacle(loc)) {
            obstacle_locations.emplace_back(loc);
        }
    }

    for (int i = 0; i < num_of_agents; i++) {
        for (const auto &t: *paths[i]){

            if(std::find(obstacle_locations.begin(), obstacle_locations.end(), t.location) != obstacle_locations.end()) {
                cout << "There are conflicts in the results:             " << i << ", " << t.location << endl;
//                return false;
            }
            else {
//                cout << "There are conflicts in the results: " << i << ", " << t.location << endl;
            }
        }
    }

    return true;

}


void CBS::printTestInfos() {
    cout << "print test infos" << endl;

    cout << "open_list.size(): " << open_list.size() << endl;

    if(open_list.empty()) {
        return;
    }

    CBSNode* node = open_list.top();
    cout << "node: " << node << endl;
    cout << "node->parent: " << node->parent << endl;
    cout << "node->paths.size(): " << node->paths.size() << endl;
    cout << "node->depth: " << node->depth << endl;
    cout << "node->g_val: " << node->g_val << endl;
    cout << "node->h_val: " << node->h_val << endl;
    cout << "min_f_val: " << min_f_val << endl;

//    for(CBSNode* node: open_list) {
//        CBSNode* curr = node;
//        cout << "after node updated " << endl;
//        while(curr!= nullptr){
//
//            for(auto item_path: curr->paths) {
//
//                cout << "after path updated: " << item_path.first << ": " << item_path.second << endl;
//                cout << "after path updated cost: " << curr->g_val + curr->h_val << endl;
//            }
//            curr = curr->parent;
//        }
//    }
    cout << "************printPaths************" << endl;
    printPaths();
}




bool CBS::generateRoot()
{
	dummy_start = new CBSNode();
	dummy_start->g_val = 0;

    dummy_start->label = label;

	paths.resize(num_of_agents, nullptr);

	mdd_helper.init(num_of_agents);
	heuristic_helper.init();

	// initialize paths_found_initially
	if (paths_found_initially.empty())
	{
		paths_found_initially.resize(num_of_agents);

		// generate a random permutation of agent indices
		vector<int> agents(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
		{
			agents[i] = i;
		}

		if (randomRoot)
		{
			std::random_device rd;
			std::mt19937 g(rd());
			std::shuffle(std::begin(agents), std::end(agents), g);
		}

		for (auto i : agents)
		{
			//CAT cat(dummy_start->makespan + 1);  // initialized to false
			//updateReservationTable(cat, i, *dummy_start);
			paths_found_initially[i] = search_engines[i]->findPath(*dummy_start, initial_constraints[i], paths, i, 0);
			if (paths_found_initially[i].empty())
			{
				cout << "No path exists for agent " << i << endl;
				return false;
			}
			paths[i] = &paths_found_initially[i];
			dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
			dummy_start->g_val += (int) paths_found_initially[i].size() - 1;
			num_LL_expanded += search_engines[i]->num_expanded;
			num_LL_generated += search_engines[i]->num_generated;
		}
	}
	else
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			paths[i] = &paths_found_initially[i];
			dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
			dummy_start->g_val += (int) paths_found_initially[i].size() - 1;
		}
	}

	// generate dummy start and update data structures
	dummy_start->h_val = 0;
	dummy_start->depth = 0;
	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);

	num_HL_generated++;
	dummy_start->time_generated = num_HL_generated;
	allNodes_table.push_back(dummy_start);
	findConflicts(*dummy_start);
	// We didn't compute the node-selection tie-breaking value for the root node
	// since it does not need it.
	min_f_val = max(min_f_val, (double) dummy_start->g_val);
	focal_list_threshold = min_f_val * focal_w;

	if (screen >= 2) // print start and goals
	{
		printPaths();
	}


//    printTestInfos();

	return true;
}


inline void CBS::releaseNodes()
{
	open_list.clear();
	focal_list.clear();
	for (auto node : allNodes_table)
		delete node;
	allNodes_table.clear();
}



/*inline void CBS::releaseOpenListNodes()
{
	while (!open_list.empty())
	{
		CBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}*/

CBS::~CBS()
{
	releaseNodes();
	mdd_helper.clear();
}

void CBS::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();
}


bool CBS::validateSolution() const
{
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
					return false;
				}
				else if (timestep < min_path_length - 1
						 && loc1 == paths[a2]->at(timestep + 1).location
						 && loc2 == paths[a1]->at(timestep + 1).location)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
						 loc1 << "-->" << loc2 << ") at timestep " << timestep << endl;
					return false;
				}
			}
			if (paths[a1]->size() != paths[a2]->size())
			{
				int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
				int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
				int loc1 = paths[a1_]->back().location;
				for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
				{
					int loc2 = paths[a2_]->at(timestep).location;
					if (loc1 == loc2)
					{
						cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
						return false; // It's at least a semi conflict
					}
				}
			}
		}
	}
	return true;
}

inline int CBS::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t) 0);
	return paths[agent_id]->at(t).location;
}


// used for rapid random  restart
void CBS::clear()
{
	mdd_helper.clear();
	heuristic_helper.clear();
	releaseNodes();
	paths.clear();
	paths_found_initially.clear();
	dummy_start = nullptr;
	goal_node = nullptr;
	solution_found = false;
	solution_cost = -2;
}


vector<int> CBS::getNonRepeatingRandomNumber(int low, int high, int size) {

    vector<int> res{};
    if (low == 1 && high == 1 && size > 0){
        res.emplace_back(1);
        return res;
    }
    else if (low == 1 && high == 0) {
        return res;
    }
    else if (size > (high-low + 1)) {
        for(int i = low;i <= high; ++i)
        {
            res.emplace_back(i);
        }
        return res;
    }
    else {
        vector<int> number{};
        for(int i = low;i <= high; ++i)
        {
            number.emplace_back(i);
        }
        std::random_shuffle(number.begin(),number.end());

        for(int i = 0; i < size; ++i){
            res.emplace_back(number[i]);
        }
        return res;
    }
}


bool CBS::addRandomObstacles(Instance& instance, vector<DynamicObstacle>& obstacle_add_v, int size) {

    obstacle_add_v.clear();

    for(auto item: paths) {
        if(item == nullptr) {
            return false;
        }
    }

    //////////////////////////////
    vector<int> possible_add_obstacle_locations{};
    for(int loc = 0; loc < instance.map_size; ++loc){
        if((!instance.isObstacle(loc)) && (!instance.is_start_location(loc)) && (!instance.is_goal_location(loc))) {
            possible_add_obstacle_locations.emplace_back(loc);
        }
    }

    std::random_shuffle(possible_add_obstacle_locations.begin(),possible_add_obstacle_locations.end());

    int path_obstacle = 0;
    int random_add_size = possible_add_obstacle_locations.size() < (size - path_obstacle) ?
                            possible_add_obstacle_locations.size() : (size - path_obstacle);

    for(int i = 0; i < random_add_size; ++i) {
        int item_location = possible_add_obstacle_locations[i];
        int x = instance.getColCoordinate(item_location);
        int y = instance.getRowCoordinate(item_location);
        obstacle_add_v.emplace_back(x, y);
    }
    //////////////////////////////////////


    set<int> path_position_set;
    for (int agent = 0; agent < num_of_agents; agent++) {
        int start_location = instance.get_start_locations()[agent];
        int goal_location = instance.get_goal_locations()[agent];

        for (const auto& t : *paths[agent]) {
            if ((t.location != start_location) && (t.location != goal_location)) {
                path_position_set.insert(t.location);
            }
        }
    }

    vector<int> obstacle_locations;
    obstacle_locations.assign(path_position_set.begin(), path_position_set.end());
    std::random_shuffle(obstacle_locations.begin(),obstacle_locations.end());

//    int random_add_size = obstacle_locations.size() < size ? obstacle_locations.size() : size;
    random_add_size = obstacle_locations.size() < (size -  random_add_size) ? obstacle_locations.size() : (size -  random_add_size);
    for(int i = 0; i < random_add_size; ++i) {
        int item_location = obstacle_locations[i];
        int x = instance.getColCoordinate(item_location);
        int y = instance.getRowCoordinate(item_location);
        obstacle_add_v.emplace_back(x, y);
    }


    /// change map
    for(auto item_obstacle: obstacle_add_v){
        int obstacle_pos = instance.getCols() * item_obstacle.y + item_obstacle.x;
        instance.changeMap(obstacle_pos, true);
    }


    return true;
}


bool CBS::deleteRandomObstacles(Instance& instance, vector<DynamicObstacle>& obstacle_delete_v, int size) {

    obstacle_delete_v.clear();

    vector<int> obstacle_locations{};
    for(int loc = 0; loc < instance.map_size; ++loc){
        if(instance.isObstacle(loc)) {
            obstacle_locations.emplace_back(loc);
        }
    }
    std::random_shuffle(obstacle_locations.begin(),obstacle_locations.end());

    int random_delete_size = obstacle_locations.size() < size ? obstacle_locations.size() : size;
    for(int i = 0; i < random_delete_size; ++i) {
        int item_location = obstacle_locations[i];
        int x = instance.getColCoordinate(item_location);
        int y = instance.getRowCoordinate(item_location);
        obstacle_delete_v.emplace_back(x, y);
    }

    /// change map
    for(auto item_obstacle: obstacle_delete_v) {
        int obstacle_pos = instance.getCols() * item_obstacle.y + item_obstacle.x;
        instance.changeMap(obstacle_pos, false);
    }

    return true;
}

bool CBS::changeRandomObstacles(Instance& instance, vector<DynamicObstacle>& obstacle_add_v, vector<DynamicObstacle>& obstacle_delete_v) {


    for(int i = 0; i < 10; ++i) {
        if((rand() % 2) == 1) {
            addRandomObstacles(instance, obstacle_add_v, 2);
        }
        else if((rand() % 2) == 0) {
            deleteRandomObstacles(instance, obstacle_delete_v, 10);
        }
    }

    return false;
}


void CBS::initModifyscenInfos(Instance& instance) {

//    modify_scen_infos["map_size"]["row"] = instance.getRows();
//    modify_scen_infos["map_size"]["col"] = instance.getCols();
}

void CBS::updateModifyscenInfos(int step, bool is_add_obstacles, vector<DynamicObstacle>& obstacle_v) {

    string step_str = std::to_string(step);

    modify_scen_infos[step_str] = {};
    modify_scen_infos[step_str]["is_add_obstacles"] = is_add_obstacles;

    int id = 0;
    for(auto item: obstacle_v) {
        string id_str = std::to_string(id);
        modify_scen_infos[step_str]["pos"][id_str]["x"] = item.x;
        modify_scen_infos[step_str]["pos"][id_str]["y"] = item.y;
        id++;
    }


}

void CBS::clearModifyscenInfos() {
    modify_scen_infos.clear();
}

void CBS::saveModifyscen(const string &fileName)  {
    std::ofstream output;
    output.open(fileName, std::ios::out);

    output << modify_scen_infos;

    output.close();

}

json CBS::parseModifyscen(const string &fileName) {
    std::ifstream f(fileName);
    json j = json::parse(f);

    f.close();
    return j;

}

bool CBS::isFileExists(const string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

void CBS::printResults(int step) {

    string step_str = std::to_string(step);

    if (solution_cost >= 0) {
        savePaths(step_str);
    }
}

void CBS::update_min_f_val() {
    if (solution_cost >= 0) {
        min_f_val = solution_cost;
        focal_list_threshold = min_f_val * focal_w;
    }
}

void CBS::printNodeLabelInfos() {
    unordered_map<int, int> label_map;
    for (int i = 0; i <= label; ++i) {
        label_map[i] = 0;
    }

    for(CBSNode* node: open_list) {
        label_map[node->label]++;
    }

    for (const auto& pair: label_map) {
        cout << "label infos:" << pair.first << ": " << pair.second << endl;
    }

    if (open_list.size() > 0) {
        CBSNode* open_head = open_list.top();
        cout << "open_head->label:" << open_head->label << endl;
        cout << "open_head->g_val + open_head->h_val:" << open_head->g_val + open_head->h_val << endl;
    }

}


void CBS::initResultInfos(const boost::program_options::variables_map& vm, const bool& is_modifyscen_exist) {

    result_infos_json["test_infos"] = {};

    result_infos_json["test_infos"]["map"] = vm["map"].as<string>();
    result_infos_json["test_infos"]["agents"] = vm["agents"].as<string>();
    result_infos_json["test_infos"]["agentNum"] = vm["agentNum"].as<int>();
    result_infos_json["test_infos"]["agentIdx"] = vm["agentIdx"].as<string>();
    result_infos_json["test_infos"]["rows"] = vm["rows"].as<int>();
    result_infos_json["test_infos"]["cols"] = vm["cols"].as<int>();
    result_infos_json["test_infos"]["obs"] = vm["obs"].as<int>();
    result_infos_json["test_infos"]["warehouseWidth"] = vm["warehouseWidth"].as<int>();

}


void CBS::updateResultInfos(int step) {

    string step_str = std::to_string(step);

    result_infos_json[step_str] = {};
    result_infos_json[step_str]["solution_cost"] = solution_cost;

    if (solution_cost >= 0) // solved
        result_infos_json[step_str]["result"] = "Optimal";
    else if (solution_cost == -1) // time_out
        result_infos_json[step_str]["result"] = "Timeout";
    else if (solution_cost == -2) // no solution
        result_infos_json[step_str]["result"] = "No solutions";
    else if (solution_cost == -3) // nodes out
        result_infos_json[step_str]["result"] = "Nodesout";


    result_infos_json[step_str]["runtime"] = runtime;
    result_infos_json[step_str]["num_HL_expanded"] = num_HL_expanded;
    result_infos_json[step_str]["num_LL_expanded"] = num_LL_expanded;
    result_infos_json[step_str]["min_f_val"] = min_f_val;

    result_infos_json[step_str]["num_of_high_level_nodes_expansion"] = num_of_high_level_nodes_expansion;
    result_infos_json[step_str]["num_of_high_level_nodes_expansion_generated"] = num_of_high_level_nodes_expansion_generated;
    result_infos_json[step_str]["num_of_high_level_nodes_in_open_on_start"] = num_of_high_level_nodes_in_open_on_start;
    result_infos_json[step_str]["num_of_high_level_nodes_in_open_on_finish"] = num_of_high_level_nodes_in_open_on_finish;

}

void CBS::saveResultInfos(const string &fileName) {

    std::ofstream output;
    output.open(fileName, std::ios::out);

    output << result_infos_json;

    output.close();

}


void CBS::initStateParams() {
    solution_found = false;
    solution_cost = -2;
}





