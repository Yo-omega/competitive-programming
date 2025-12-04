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
	map<int, int> astronaut_counts; // Target Type -> Count

	// Helper to get total waiting (sum of all types)
	int total_waiting() const
	{
		int sum = 0;
		for (auto const &[t, c] : astronaut_counts)
			sum += c;
		return sum;
	}
};

struct Route
{
	int u, v; // u < v
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

double cross_product(Point a, Point b, Point c)
{
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Distance from point P to segment AB
double point_to_segment_dist(Point p, Point a, Point b)
{
	double l2 = dist_sq(a, b);
	if (l2 == 0)
		return dist(p, a);
	double t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
	t = max(0.0, min(1.0, t));
	Point projection = {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
	return dist(p, projection);
}

// Check if Segment AB overlaps Segment CD
bool segments_intersect(Point a, Point b, Point c, Point d)
{
	if (dist_sq(a, c) < 1e-5 || dist_sq(a, d) < 1e-5 ||
		dist_sq(b, c) < 1e-5 || dist_sq(b, d) < 1e-5)
		return false;

	auto cp1 = cross_product(a, b, c);
	auto cp2 = cross_product(a, b, d);
	auto cp3 = cross_product(c, d, a);
	auto cp4 = cross_product(c, d, b);

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
	// Game State
	map<int, Building> buildings;
	vector<Route> routes;
	vector<Pod> pods;
	int resources;
	int next_pod_id = 1;
	int turn_count = 0; // Turn Awareness

	// Per-Turn Logic
	map<pair<int, int>, Route *> route_map;
	map<int, vector<int>> adj;
	vector<string> action_queue;

	// Analytics
	map<int, int> component_id;
	map<int, set<int>> component_types;

	// Congestion Tracking: Store total waiting at Source ID for previous turn
	map<int, int> prev_total_waiting_at_source;

	// Reset for fresh turn
	void reset_turn()
	{
		routes.clear();
		pods.clear();
		route_map.clear();
		adj.clear();
		action_queue.clear();
		component_id.clear();
		component_types.clear();

		turn_count++;
	}

	pair<int, int> edge_key(int u, int v)
	{
		if (u > v)
			swap(u, v);
		return {u, v};
	}

	int get_tube_cost(int u, int v)
	{
		double d = dist(buildings[u].p, buildings[v].p);
		return max(1, (int)floor(d * 10.0));
	}

	bool has_route(int u, int v)
	{
		return route_map.count(edge_key(u, v));
	}

	// STRICT GEOMETRY CHECK: Collision with Routes AND Buildings
	bool is_valid_tube_geom(int u, int v)
	{
		if (has_route(u, v))
			return false;

		Point A = buildings[u].p;
		Point B = buildings[v].p;

		// 1. Check Intersection with existing tubes
		for (const auto &r : routes)
		{
			if (r.is_teleporter)
				continue;
			Point C = buildings[r.u].p;
			Point D = buildings[r.v].p;
			if (segments_intersect(A, B, C, D))
				return false;
		}

		// 2. STRICT CHECK: Does it pass through another Building?
		// Using threshold 1.5 units (safe for standard buildings)
		double safety_radius = 1.5;
		for (auto const &[id, b] : buildings)
		{
			if (id == u || id == v)
				continue;
			if (point_to_segment_dist(b.p, A, B) < safety_radius)
			{
				return false; // Collides with a building
			}
		}

		return true;
	}

	void build_components()
	{
		set<int> visited;
		int num_components = 0;
		for (auto const &[id, b] : buildings)
		{
			if (visited.count(id))
				continue;
			num_components++;
			queue<int> q;
			q.push(id);
			visited.insert(id);
			component_id[id] = num_components;

			// Register self
			if (b.type != 0)
				component_types[num_components].insert(b.type);

			while (!q.empty())
			{
				int curr = q.front();
				q.pop();
				// Register type
				if (buildings[curr].type != 0)
					component_types[num_components].insert(buildings[curr].type);

				for (int n : adj[curr])
				{
					if (!visited.count(n))
					{
						visited.insert(n);
						component_id[n] = num_components;
						q.push(n);
					}
				}
			}
		}
	}

	// Finds best path type: 1 = u-v-w-u (triangle), 2 = u-v-w-z-u (quad)
	string create_smart_pod(int u, int v)
	{
		// Linear default
		string linear = to_string(u) + " " + to_string(v) + " " + to_string(u);

		// Try Triangle
		for (int w : adj[v])
		{
			if (w == u)
				continue;
			bool connected_to_start = false;
			for (int k : adj[w])
				if (k == u)
					connected_to_start = true;

			if (connected_to_start)
			{
				// Triangle Found: U->V->W->U
				return to_string(u) + " " + to_string(v) + " " + to_string(w) + " " + to_string(u);
			}
		}
		return linear;
	}

	// ========================
	// LOGIC EXECUTION
	// ========================

	void solve()
	{
		// Init Graph
		for (const auto &r : routes)
		{
			adj[r.u].push_back(r.v);
			adj[r.v].push_back(r.u);
		}
		build_components();

		// Proposals Vector: <Score (lower=better), From, To, IsTeleport>
		vector<tuple<double, int, int, bool>> proposals;
		bool saving_mode = false;

		// 1. IDENTIFY NEEDS
		for (auto &[b_id, b] : buildings)
		{
			// Turn Aware: Don't start new expensive paths late game if empty
			if (turn_count > 18)
				continue;

			if (b.type != 0 || b.num_astronauts == 0)
				continue;

			for (auto const &[type, count] : b.astronaut_counts)
			{
				// Check connectivity via Components
				if (component_id[b.id] != 0 && component_types[component_id[b.id]].count(type))
					continue;

				double best_score = 1e18;
				int best_target = -1;
				bool needs_teleporter = false;

				for (auto &[cand_id, cand] : buildings)
				{
					if (cand_id == b_id)
						continue;

					// Candidate useful if it matches type OR connects to component with type
					bool useful = (cand.type == type);
					if (!useful && component_id[cand_id] != 0 && component_types[component_id[cand_id]].count(type))
						useful = true;

					if (useful)
					{
						double d = dist(b.p, cand.p);

						// "BALANCE SCORING": Prioritize sources with FEWER astronauts.
						// Standard efficient: Score = Cost / Count (High count -> Low score -> Best).
						// Balance efficient: Score = Cost * Count. (Low count -> Low score -> Best).
						double score = d * (double)count;

						// Check Geometric viability
						if (is_valid_tube_geom(b.id, cand_id))
						{
							if (score < best_score)
							{
								best_score = score;
								best_target = cand_id;
								needs_teleporter = false;
							}
						}
						// Teleporter Check: If standard is blocked or dist is huge
						else if (count > 20)
						{													 // Teleport valuable loads only
							double tele_heuristic = (5000.0 / 10.0) * count; // Base cost roughly 500 equivalent dist
							if (d > 25.0)
								tele_heuristic /= 2.0; // Distance discount

							if (tele_heuristic < best_score)
							{
								best_score = tele_heuristic;
								best_target = cand_id;
								needs_teleporter = true;
							}
						}
					}
				}

				if (best_target != -1)
				{
					proposals.emplace_back(best_score, b.id, best_target, needs_teleporter);
				}
			}
		}

		// Sort Best Proposals
		sort(proposals.begin(), proposals.end());

		// 2. BUILD PHASE
		for (auto &prop : proposals)
		{
			int u = get<1>(prop);
			int v = get<2>(prop);
			bool is_tele = get<3>(prop);

			if (has_route(u, v))
				continue;

			if (is_tele)
			{
				// SAVING LOGIC
				if (resources < 5000)
				{
					if (resources > 3500)
					{
						// We are close! Stop building tubes to afford this next turn.
						saving_mode = true;
						break;
					}
					continue; // Cant afford yet, check next proposal
				}

				action_queue.push_back("TELEPORT " + to_string(u) + " " + to_string(v));
				resources -= 5000;

				// Logic update
				adj[u].push_back(v);
				adj[v].push_back(u);
				route_map[edge_key(u, v)] = nullptr;
			}
			else
			{
				if (saving_mode)
					break; // Don't spend small change

				int cost = get_tube_cost(u, v);
				if (resources >= cost + 1000)
				{
					// Double check geometry safety just in case greedy order changed things (unlikely with this solver logic, but safe)
					if (is_valid_tube_geom(u, v))
					{
						action_queue.push_back("TUBE " + to_string(u) + " " + to_string(v));
						resources -= cost;

						// Update locally to detect triangles immediately
						adj[u].push_back(v);
						adj[v].push_back(u);
						route_map[edge_key(u, v)] = nullptr;

						// Create Pod
						int pid = next_pod_id++;
						resources -= 1000;
						action_queue.push_back("POD " + to_string(pid) + " " + create_smart_pod(u, v));
					}
				}
			}
		}

		// 3. DYNAMIC UPGRADES
		if (!saving_mode)
		{
			for (auto const &[edge, r] : route_map)
			{
				if (!r || r->is_teleporter)
					continue;

				int u = edge.first;
				int v = edge.second;

				// Check source congestion of both ends
				bool needs_upgrade = false;

				// Look at buildings U and V
				// If the TOTAL number of waiting people strictly increased, upgrade.
				// We use sum(wait) of neighbors because explicit path calculation is heavy.
				int curr_u = buildings[u].total_waiting();
				int curr_v = buildings[v].total_waiting();

				// Retrieve prev stats
				int prev_u = prev_total_waiting_at_source.count(u) ? prev_total_waiting_at_source[u] : 0;
				int prev_v = prev_total_waiting_at_source.count(v) ? prev_total_waiting_at_source[v] : 0;

				// Logic: If queue growing, and significant absolute size
				if (curr_u > prev_u && curr_u > 5 * r->capacity)
					needs_upgrade = true;
				if (curr_v > prev_v && curr_v > 5 * r->capacity)
					needs_upgrade = true;

				if (needs_upgrade)
				{
					int cost = get_tube_cost(u, v);
					if (resources >= cost)
					{
						action_queue.push_back("UPGRADE " + to_string(u) + " " + to_string(v));
						resources -= cost;
						r->capacity++;
					}
				}
			}
		}

		// Update Prev Stats for next turn
		prev_total_waiting_at_source.clear();
		for (auto const &[id, b] : buildings)
		{
			if (b.type == 0)
				prev_total_waiting_at_source[id] = b.total_waiting();
		}

		// Output
		if (action_queue.empty())
			cout << "WAIT" << endl;
		else
		{
			for (size_t i = 0; i < action_queue.size(); ++i)
			{
				cout << action_queue[i] << (i < action_queue.size() - 1 ? ";" : "");
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

		int num_routes;
		cin >> num_routes;
		cin.ignore();
		solver.reset_turn();

		for (int i = 0; i < num_routes; ++i)
		{
			int u, v, c;
			cin >> u >> v >> c;
			cin.ignore();
			solver.routes.push_back({min(u, v), max(u, v), c, c == 0});
			solver.route_map[solver.edge_key(u, v)] = &solver.routes.back();
		}

		int num_pods;
		cin >> num_pods;
		cin.ignore();
		int max_pid = 0;
		for (int i = 0; i < num_pods; ++i)
		{
			string l;
			getline(cin, l);
			stringstream ss(l);
			Pod p;
			int cnt, stop;
			ss >> p.id >> cnt;
			max_pid = max(max_pid, p.id);
			while (ss >> stop)
				p.path.push_back(stop);
			solver.pods.push_back(p);
		}
		solver.next_pod_id = max_pid + 1;

		int num_builds;
		cin >> num_builds;
		cin.ignore();
		for (int i = 0; i < num_builds; ++i)
		{
			string l;
			getline(cin, l);
			stringstream ss(l);
			Building b;
			int t;
			ss >> t;
			b.type = t;
			if (t == 0)
			{
				ss >> b.id >> b.p.x >> b.p.y >> b.num_astronauts;
				int type_req;
				while (ss >> type_req)
					b.astronaut_counts[type_req]++;
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
}