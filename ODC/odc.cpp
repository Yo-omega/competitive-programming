#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <iomanip>
#include <tuple>

using namespace std;

// ==========================================
// GEOMETRY & DATA STRUCTURES
// ==========================================

struct Point
{
	double x, y;
};

struct Building
{
	int id;
	int type; // 0 = Landing Pad, >0 = Module
	int num_astronauts;
	Point p;
	map<int, int> astronaut_counts; // Target Type -> Count of Astronauts
};

struct Route
{
	int u, v; // Always u < v
	int capacity;
	bool is_teleporter;
};

struct Pod
{
	int id;
	vector<int> path;
};

// --- Geometry Helpers ---

double dist_sq(Point p1, Point p2)
{
	return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

double dist(Point p1, Point p2)
{
	return sqrt(dist_sq(p1, p2));
}

// 2D Cross Product for intersection tests
double cross_product(Point a, Point b, Point c)
{
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Check if segment AB intersects CD
bool segments_intersect(Point a, Point b, Point c, Point d)
{
	// If they share an endpoint, no intersection for game logic
	if (dist_sq(a, c) < 1e-5 || dist_sq(a, d) < 1e-5 ||
		dist_sq(b, c) < 1e-5 || dist_sq(b, d) < 1e-5)
		return false;

	auto cp1 = cross_product(a, b, c);
	auto cp2 = cross_product(a, b, d);
	auto cp3 = cross_product(c, d, a);
	auto cp4 = cross_product(c, d, b);

	// Strict crossing check
	if (((cp1 > 1e-7 && cp2 < -1e-7) || (cp1 < -1e-7 && cp2 > 1e-7)) &&
		((cp3 > 1e-7 && cp4 < -1e-7) || (cp3 < -1e-7 && cp4 > 1e-7)))
		return true;

	return false;
}

// ==========================================
// SOLVER
// ==========================================

class Solver
{
public:
	map<int, Building> buildings;
	vector<Route> routes;
	vector<Pod> pods;
	int resources;
	int next_pod_id = 1;

	// --- Turn Context ---
	map<pair<int, int>, Route *> route_map; // Look up route by (u, v)
	map<int, vector<int>> adj;				// Adjacency List
	vector<string> action_queue;

	// Union-Find / Component Tracking for Network Connectivity
	map<int, int> component_id;
	map<int, set<int>> component_types; // Component ID -> Set of module types present
	int num_components;

	// ----------------------------------------
	// SETUP & RESET
	// ----------------------------------------

	void reset_turn()
	{
		routes.clear();
		pods.clear();
		route_map.clear();
		adj.clear();
		action_queue.clear();

		component_id.clear();
		component_types.clear();
	}

	// Normalize edge key: u < v
	pair<int, int> edge_key(int u, int v)
	{
		if (u > v)
			swap(u, v);
		return {u, v};
	}

	int get_cost(int u, int v)
	{
		// Tube cost logic: floor(distance * 10)
		double d = dist(buildings[u].p, buildings[v].p);
		return max(1, (int)floor(d * 10.0));
	}

	bool has_route(int u, int v)
	{
		return route_map.count(edge_key(u, v));
	}

	// Geometry Check
	bool can_build_tube(int u, int v)
	{
		if (has_route(u, v))
			return false;
		Point A = buildings[u].p;
		Point B = buildings[v].p;

		for (const auto &r : routes)
		{
			if (r.is_teleporter)
				continue; // Teleporters are instant, no physical tube
			Point C = buildings[r.u].p;
			Point D = buildings[r.v].p;
			if (segments_intersect(A, B, C, D))
				return false;
		}
		return true;
	}

	// ----------------------------------------
	// CONNECTED COMPONENTS ANALYSIS
	// ----------------------------------------

	void build_components()
	{
		// Simple BFS to find components
		set<int> visited;
		num_components = 0;

		for (auto const &[id, b] : buildings)
		{
			if (visited.count(id))
				continue;

			num_components++;
			queue<int> q;
			q.push(id);
			visited.insert(id);
			component_id[id] = num_components;

			if (b.type != 0)
				component_types[num_components].insert(b.type);

			while (!q.empty())
			{
				int curr = q.front();
				q.pop();

				// Check types in current building
				if (buildings[curr].type != 0)
					component_types[num_components].insert(buildings[curr].type);

				for (int neighbor : adj[curr])
				{
					if (!visited.count(neighbor))
					{
						visited.insert(neighbor);
						component_id[neighbor] = num_components;
						q.push(neighbor);
					}
				}
			}
		}
	}

	// Returns true if building ID 'b_start' is strictly connected (via tubes) to a module of 'target_type'
	bool is_connected_to_type(int b_start, int target_type)
	{
		int c_id = component_id[b_start];
		return component_types[c_id].count(target_type) > 0;
	}

	// ----------------------------------------
	// MAIN LOGIC
	// ----------------------------------------

	void solve()
	{
		// 1. Rebuild Adjacency
		for (const auto &r : routes)
		{
			adj[r.u].push_back(r.v);
			adj[r.v].push_back(r.u);
		}

		// 2. Build Component Map
		build_components();

		// 3. Traffic Counts
		// Simple heuristic: count waiting passengers for each edge they *could* take next.
		// Actually, just knowing total Waiting vs. Capacity on existing edges is enough for UPGRADES.

		// --------------------------------------------
		// STRATEGY PHASE 1: CONNECT THE DISCONNECTED
		// --------------------------------------------

		// Priority Queue for Construction: <Cost, AstronautsWaiting, StartNode, EndNode>
		// Use double for priority to weigh Urgency vs Cost.
		vector<tuple<double, int, int>> build_proposals;

		for (auto &pair : buildings)
		{
			Building &b = pair.second;
			// Only care about Landing Pads with people
			if (b.type != 0 || b.num_astronauts == 0)
				continue;

			for (auto const &[target_type, count] : b.astronaut_counts)
			{
				if (count == 0)
					continue;

				// If already connected to target type, skip construction (Network handles it)
				if (is_connected_to_type(b.id, target_type))
					continue;

				// FIND BEST CONNECTION
				// Strategy: Connect to ANY building that is PART OF A COMPONENT containing target_type.
				// Or if none exist, connect to the target_type building itself (starting a new component).

				double best_dist = 1e9;
				int best_target = -1;

				// Candidate search
				for (auto &candidate : buildings)
				{
					if (candidate.first == b.id)
						continue;

					// Is this candidate useful?
					// Either it IS the type we want, OR it's connected to the type we want.
					bool useful = (candidate.second.type == target_type);
					if (!useful)
					{
						int c_id = component_id[candidate.first];
						if (c_id != 0 && component_types[c_id].count(target_type))
							useful = true;
					}

					if (useful)
					{
						// Check Euclidean dist
						double d = dist(b.p, candidate.second.p);
						if (d < best_dist)
						{
							// verify geometry now to save time
							if (can_build_tube(b.id, candidate.first))
							{
								best_dist = d;
								best_target = candidate.first;
							}
						}
					}
				}

				if (best_target != -1)
				{
					// Heuristic: Cost / (Count + 1). Lower is better.
					// We prioritize Cheap connections with High Passengers.
					double heuristic = best_dist / (double)(count + 1);
					build_proposals.emplace_back(heuristic, b.id, best_target);
				}
			}
		}

		// Sort: Best heuristic first (Smallest value)
		sort(build_proposals.begin(), build_proposals.end());

		// EXECUTE CONSTRUCTION
		for (const auto &prop : build_proposals)
		{
			int u = get<1>(prop);
			int v = get<2>(prop);

			if (has_route(u, v))
				continue;

			int cost = get_cost(u, v);

			// Check budget: Must be able to afford Tube AND a Pod (1000)
			if (resources >= cost + 1000)
			{
				if (can_build_tube(u, v))
				{
					resources -= cost;
					// Command
					action_queue.push_back("TUBE " + to_string(u) + " " + to_string(v));

					// Logic update (instant graph update for subsequent loop iterations in same turn?)
					// Ideally yes, but for now we rely on greedy distinct choices.
					// Let's create the object locally so future `can_build_tube` checks see it
					Route r = {min(u, v), max(u, v), 1, false};
					routes.push_back(r);
					route_map[edge_key(u, v)] = &routes.back(); // temporary

					// Build Pod immediately? Usually yes, to enable the route.
					// For the POD command, we can try to make a loop if applicable.

					// LOOP DETECTION for Pods
					// Do U and V share a common neighbor W?
					// Because 'routes' was updated, adjacency needs a local check or just check `route_map`
					int w = -1;
					for (auto const &nb : buildings)
					{
						int nid = nb.first;
						if (nid == u || nid == v)
							continue;
						if (has_route(u, nid) && has_route(v, nid))
						{
							w = nid;
							break;
						}
					}

					int pid = next_pod_id++; // Use virtual ID
					resources -= 1000;

					if (w != -1)
					{
						// Triangle route
						action_queue.push_back("POD " + to_string(pid) + " " + to_string(u) + " " + to_string(v) + " " + to_string(w) + " " + to_string(u));
					}
					else
					{
						// Linear route
						action_queue.push_back("POD " + to_string(pid) + " " + to_string(u) + " " + to_string(v) + " " + to_string(u));
					}
				}
			}
		}

		// --------------------------------------------
		// STRATEGY PHASE 2: POD MANAGEMENT
		// --------------------------------------------

		// If we have extra money, check for bottlenecks (Lines with NO pods).
		// Since we add pods with new tubes, this is for legacy lines or complex transfers.
		// Or if capacity is 1 but we need more pods.
		// SIMPLIFIED: For now, the creation phase handles most.
		// We scan for heavy edges without pods.

		if (resources > 1500)
		{
			for (const auto &r : routes)
			{
				if (r.is_teleporter)
					continue;
				// Basic check: is there a pod covering this edge?
				bool covered = false;
				for (const auto &p : pods)
				{
					for (size_t k = 0; k < p.path.size() - 1; ++k)
					{
						int p_u = p.path[k], p_v = p.path[k + 1];
						if (edge_key(p_u, p_v) == edge_key(r.u, r.v))
						{
							covered = true;
							break;
						}
					}
					if (covered)
						break;
				}

				if (!covered)
				{
					resources -= 1000;
					int pid = next_pod_id++; // Virtual
					// Attempt Triangle
					int w = -1;
					for (auto const &item : route_map)
					{
						int o1 = item.first.first;
						int o2 = item.first.second;
						// finding neighbor common to r.u and r.v
						// lazy iteration
					}
					// Faster way using Adjacency list (rebuilt or local)
					// ... defaulting to simple liner to be safe/fast
					action_queue.push_back("POD " + to_string(pid) + " " + to_string(r.u) + " " + to_string(r.v) + " " + to_string(r.u));
				}
				if (resources < 1500)
					break;
			}
		}

		// --------------------------------------------
		// STRATEGY PHASE 3: STRATEGIC UPGRADES
		// --------------------------------------------
		// Upgrades increase tube capacity.
		// Heuristic: If queue length > capacity * 5.
		// We don't have perfect queue info here without running simulation.
		// As a proxy: Total astronauts at B1 targeting Type T, and next hop is B2...
		// For now, simpler: Upgrade everything if we are filthy rich.
		// Or conservatively: upgrade nothing. The initial tests pass with default cap.

		if (resources > 5000)
		{
			// Luxury: Teleporters?
			// Only use Teleporters if map is huge and resource > 20000 or disconnected clusters impossible to reach.
			// Given previous failures, let's skip Teleporters unless explicitly obvious.
		}

		// --------------------------------------------
		// OUTPUT
		// --------------------------------------------

		if (action_queue.empty())
		{
			cout << "WAIT" << endl;
		}
		else
		{
			for (size_t i = 0; i < action_queue.size(); ++i)
			{
				cout << action_queue[i];
				if (i < action_queue.size() - 1)
					cout << ";";
			}
			cout << endl;
		}
	}
};

int main()
{
	Solver solver;
	while (1)
	{
		if (!(cin >> solver.resources))
			break;
		cin.ignore();

		int num_travel_routes;
		cin >> num_travel_routes;
		cin.ignore();

		solver.reset_turn();

		for (int i = 0; i < num_travel_routes; i++)
		{
			int u, v, capacity;
			cin >> u >> v >> capacity;
			cin.ignore();

			Route r;
			r.u = min(u, v);
			r.v = max(u, v);
			r.capacity = capacity;
			r.is_teleporter = (capacity == 0);

			solver.routes.push_back(r);
			solver.route_map[solver.edge_key(u, v)] = &solver.routes.back();
		}

		int num_pods;
		cin >> num_pods;
		cin.ignore();
		int max_id = 0;
		for (int i = 0; i < num_pods; i++)
		{
			string line;
			getline(cin, line);
			stringstream ss(line);
			Pod p;
			int num_stops, stop;
			ss >> p.id >> num_stops;
			max_id = max(max_id, p.id);
			while (ss >> stop)
				p.path.push_back(stop);
			solver.pods.push_back(p);
		}
		solver.next_pod_id = max_id + 1; // Ensure new IDs don't conflict

		int num_new_buildings;
		cin >> num_new_buildings;
		cin.ignore();
		for (int i = 0; i < num_new_buildings; i++)
		{
			string line;
			getline(cin, line);
			stringstream ss(line);

			Building b;
			int type_or_zero;
			ss >> type_or_zero;
			b.type = type_or_zero;

			if (type_or_zero == 0)
			{
				ss >> b.id >> b.p.x >> b.p.y >> b.num_astronauts;
				for (int k = 0; k < b.num_astronauts; ++k)
				{
					int type;
					ss >> type;
					b.astronaut_counts[type]++;
				}
			}
			else
			{
				ss >> b.id >> b.p.x >> b.p.y;
				b.num_astronauts = 0;
			}
			solver.buildings[b.id] = b;
		}

		solver.solve();
	}
	return 0;
}