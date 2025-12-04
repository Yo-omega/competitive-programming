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

using namespace std;

// ==========================================
// GEOMETRY & STRUCTS
// ==========================================

struct Point
{
	double x, y;
};

struct Building
{
	int id;
	int type; // 0 = Landing Pad, 1-20 = Module
	int num_astronauts;
	Point p;
	map<int, int> astronaut_counts; // Type -> Count
};

struct Route
{
	int b1, b2;
	int capacity; // 0 = Teleporter
	bool is_teleporter;
};

struct Pod
{
	int id;
	vector<int> path;
};

// Geometry Helpers
double dist(Point p1, Point p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// Cross product to check orientation
double cross_product(Point a, Point b, Point c)
{
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Check if point c is on segment ab
bool on_segment(Point a, Point b, Point c)
{
	return min(a.x, b.x) <= c.x + 1e-7 && c.x <= max(a.x, b.x) + 1e-7 &&
		   min(a.y, b.y) <= c.y + 1e-7 && c.y <= max(a.y, b.y) + 1e-7;
}

// Main intersection logic
bool segments_intersect(Point a, Point b, Point c, Point d)
{
	// If they share an endpoint, they don't "cross" in a bad way for this game
	if (dist(a, c) < 1e-7 || dist(a, d) < 1e-7 || dist(b, c) < 1e-7 || dist(b, d) < 1e-7)
		return false;

	double cp1 = cross_product(a, b, c);
	double cp2 = cross_product(a, b, d);
	double cp3 = cross_product(c, d, a);
	double cp4 = cross_product(c, d, b);

	if (((cp1 > 1e-7 && cp2 < -1e-7) || (cp1 < -1e-7 && cp2 > 1e-7)) &&
		((cp3 > 1e-7 && cp4 < -1e-7) || (cp3 < -1e-7 && cp4 > 1e-7)))
		return true;

	return false;
}

// ==========================================
// SOLVER CLASS
// ==========================================

class Solver
{
public:
	// Game State
	map<int, Building> buildings; // Persists across turns
	vector<Route> routes;		  // Refreshed every turn
	vector<Pod> pods;			  // Refreshed every turn
	int resources;
	int next_pod_id = 1;

	// Helpers for logic
	map<pair<int, int>, int> route_capacity_map; // {b1,b2} -> capacity
	map<pair<int, int>, bool> has_pod_map;		 // {b1,b2} -> has pod
	map<pair<int, int>, int> route_traffic;		 // {b1,b2} -> estimated traffic
	vector<string> action_queue;

	void reset_turn()
	{
		routes.clear();
		pods.clear();
		route_capacity_map.clear();
		has_pod_map.clear();
		route_traffic.clear();
		action_queue.clear();
	}

	// --- Cost Calculations ---
	int get_tube_cost(int b1, int b2)
	{
		double d = dist(buildings[b1].p, buildings[b2].p);
		return max(1, (int)floor(d * 10.0));
	}

	int get_upgrade_cost(int b1, int b2)
	{
		return get_tube_cost(b1, b2);
	}

	// --- Graph Helpers ---

	bool route_exists(int b1, int b2)
	{
		if (b1 > b2)
			swap(b1, b2);
		return route_capacity_map.count({b1, b2});
	}

	int get_route_capacity(int b1, int b2)
	{
		if (b1 > b2)
			swap(b1, b2);
		if (!route_capacity_map.count({b1, b2}))
			return 0;
		return route_capacity_map[{b1, b2}];
	}

	bool has_pod(int b1, int b2)
	{
		if (b1 > b2)
			swap(b1, b2);
		return has_pod_map[{b1, b2}];
	}

	bool is_valid_geometry(int b1, int b2)
	{
		Point p1 = buildings[b1].p;
		Point p2 = buildings[b2].p;

		for (const auto &r : routes)
		{
			if (r.is_teleporter)
				continue;
			Point p3 = buildings[r.b1].p;
			Point p4 = buildings[r.b2].p;
			if (segments_intersect(p1, p2, p3, p4))
				return false;
		}
		return true;
	}

	// --- Advanced Pathfinding ---

	struct PathInfo
	{
		int distance;
		vector<int> path;
		bool reachable;
	};

	PathInfo get_path_to_type(int start_id, int target_type)
	{
		PathInfo result;
		result.distance = 9999;
		result.reachable = false;

		if (buildings[start_id].type == target_type)
		{
			result.distance = 0;
			result.reachable = true;
			result.path = {start_id};
			return result;
		}

		// Build Adjacency List
		map<int, vector<pair<int, int>>> adj;
		for (const auto &r : routes)
		{
			int w = (r.is_teleporter) ? 0 : 1;
			adj[r.b1].push_back({r.b2, w});
			adj[r.b2].push_back({r.b1, w});
		}

		priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
		map<int, int> dists;
		map<int, int> parent;

		pq.push({0, start_id});
		dists[start_id] = 0;
		parent[start_id] = -1;

		while (!pq.empty())
		{
			int d = pq.top().first;
			int u = pq.top().second;
			pq.pop();

			if (dists.count(u) && d > dists[u])
				continue;

			if (buildings[u].type == target_type)
			{
				result.distance = d;
				result.reachable = true;

				// Reconstruct path
				int curr = u;
				while (curr != -1)
				{
					result.path.push_back(curr);
					curr = parent[curr];
				}
				reverse(result.path.begin(), result.path.end());
				return result;
			}

			for (auto &edge : adj[u])
			{
				int v = edge.first;
				int weight = edge.second;
				if (!dists.count(v) || dists[v] > d + weight)
				{
					dists[v] = d + weight;
					parent[v] = u;
					pq.push({dists[v], v});
				}
			}
		}
		return result;
	}

	// Estimate traffic on a route based on astronaut destinations
	void calculate_traffic()
	{
		for (auto &pair : buildings)
		{
			Building &b = pair.second;
			if (b.type != 0 || b.num_astronauts == 0)
				continue;

			for (auto const &[type, count] : b.astronaut_counts)
			{
				if (count == 0)
					continue;

				PathInfo info = get_path_to_type(b.id, type);
				if (info.reachable && info.path.size() >= 2)
				{
					for (size_t i = 0; i < info.path.size() - 1; i++)
					{
						int u = info.path[i];
						int v = info.path[i + 1];
						if (u > v)
							swap(u, v);
						route_traffic[{u, v}] += count;
					}
				}
			}
		}
	}

	// Find nearest module of given type
	int find_nearest_module(int from_id, int target_type)
	{
		int best = -1;
		double best_dist = 1e9;

		for (auto &pair : buildings)
		{
			if (pair.second.type == target_type)
			{
				double d = dist(buildings[from_id].p, pair.second.p);
				if (d < best_dist)
				{
					best_dist = d;
					best = pair.first;
				}
			}
		}
		return best;
	}

	// Find clusters of nearby buildings
	vector<int> find_cluster(int start_id, double max_dist)
	{
		vector<int> cluster;
		cluster.push_back(start_id);

		for (auto &pair : buildings)
		{
			if (pair.first == start_id)
				continue;
			double d = dist(buildings[start_id].p, pair.second.p);
			if (d <= max_dist)
			{
				cluster.push_back(pair.first);
			}
		}
		return cluster;
	}

	// --- Main Optimization Logic ---

	void solve()
	{
		calculate_traffic();

		// Priority 1: UPGRADE overloaded tubes
		for (auto &r : routes)
		{
			if (r.is_teleporter || r.capacity >= 10)
				continue;

			int u = r.b1, v = r.b2;
			if (u > v)
				swap(u, v);

			int traffic = route_traffic[{u, v}];
			int throughput = r.capacity * 20; // 20 days per turn

			// If traffic exceeds 80% of throughput, upgrade
			if (traffic > throughput * 0.8 && has_pod(u, v))
			{
				int cost = get_upgrade_cost(r.b1, r.b2);
				if (resources >= cost)
				{
					action_queue.push_back("UPGRADE " + to_string(r.b1) + " " + to_string(r.b2));
					resources -= cost;
					route_capacity_map[{u, v}]++;
				}
			}
		}

		// Priority 2: Connect stranded astronauts with SMART routing
		vector<tuple<int, int, int, int>> connection_candidates; // {priority, from, to, type}

		for (auto &pair : buildings)
		{
			Building &b = pair.second;
			if (b.type != 0 || b.num_astronauts == 0)
				continue;

			for (auto const &[type, count] : b.astronaut_counts)
			{
				if (count == 0)
					continue;

				PathInfo info = get_path_to_type(b.id, type);

				// Unreachable - CRITICAL priority
				if (!info.reachable)
				{
					int target = find_nearest_module(b.id, type);
					if (target != -1)
					{
						int priority = count * 1000; // High priority
						connection_candidates.push_back({priority, b.id, target, type});
					}
				}
				// Reachable but long path - consider teleporter
				else if (info.distance > 6 && count > 30)
				{
					int target = find_nearest_module(b.id, type);
					if (target != -1 && resources > 7000)
					{
						// Check if teleporter would help
						int u = b.id, v = target;
						if (u > v)
							swap(u, v);
						if (!route_exists(u, v))
						{
							action_queue.push_back("TELEPORT " + to_string(b.id) + " " + to_string(target));
							resources -= 5000;

							Route r = {b.id, target, 0, true};
							routes.push_back(r);
							route_capacity_map[{u, v}] = 0;
						}
					}
				}
			}
		}

		// Sort by priority and build connections
		sort(connection_candidates.begin(), connection_candidates.end(), greater<tuple<int, int, int, int>>());

		for (auto &cand : connection_candidates)
		{
			int from = get<1>(cand);
			int to = get<2>(cand);

			if (route_exists(from, to))
				continue;
			if (!is_valid_geometry(from, to))
				continue;

			int tube_cost = get_tube_cost(from, to);
			int pod_cost = 1000;

			if (resources >= tube_cost + pod_cost)
			{
				// Build tube
				action_queue.push_back("TUBE " + to_string(from) + " " + to_string(to));
				resources -= tube_cost;

				// Build pod with ping-pong schedule
				int pod_id = next_pod_id++;
				action_queue.push_back("POD " + to_string(pod_id) + " " + to_string(from) + " " + to_string(to) + " " + to_string(from));
				resources -= pod_cost;

				// Update state
				Route r = {from, to, 1, false};
				routes.push_back(r);
				int u = from, v = to;
				if (u > v)
					swap(u, v);
				route_capacity_map[{u, v}] = 1;
				has_pod_map[{u, v}] = true;
			}
		}

		// Priority 3: Add pods to tubes without pods (CRITICAL - prevents astronaut deadlock)
		for (const auto &r : routes)
		{
			if (r.is_teleporter)
				continue;

			int u = r.b1, v = r.b2;
			if (u > v)
				swap(u, v);

			if (!has_pod(u, v) && resources >= 1000)
			{
				int pod_id = pods.size() + 500 + rand() % 1000;
				action_queue.push_back("POD " + to_string(pod_id) + " " + to_string(r.b1) + " " + to_string(r.b2) + " " + to_string(r.b1));
				resources -= 1000;
				has_pod_map[{u, v}] = true;
			}
		}

		// Priority 4: Optimize pod schedules for clusters
		if (resources > 2000)
		{
			for (auto &pair : buildings)
			{
				Building &b = pair.second;
				if (b.type == 0 && b.num_astronauts > 20)
				{
					vector<int> cluster = find_cluster(b.id, 15.0);

					if (cluster.size() >= 3)
					{
						// Try to create a circular route
						vector<int> connected;
						connected.push_back(b.id);

						for (int node : cluster)
						{
							if (node == b.id)
								continue;
							bool all_connected = true;
							for (int prev : connected)
							{
								if (!route_exists(prev, node))
								{
									all_connected = false;
									break;
								}
							}
							if (all_connected || route_exists(connected.back(), node))
							{
								connected.push_back(node);
							}
							if (connected.size() >= 4)
								break;
						}

						// Create circular pod if we have a good cluster
						if (connected.size() >= 3 && resources >= 1000)
						{
							int pod_id = pods.size() + 700 + rand() % 1000;
							string pod_cmd = "POD " + to_string(pod_id);
							for (int node : connected)
							{
								pod_cmd += " " + to_string(node);
							}
							pod_cmd += " " + to_string(connected[0]); // Complete circle

							action_queue.push_back(pod_cmd);
							resources -= 1000;
							break; // Only one circular route per turn
						}
					}
				}
			}
		}

		// Priority 5: Strategic teleporters for high-volume long routes
		if (resources > 8000)
		{
			for (auto &traffic_pair : route_traffic)
			{
				int u = traffic_pair.first.first;
				int v = traffic_pair.first.second;
				int traffic = traffic_pair.second;

				if (traffic > 100)
				{
					// Check actual path length
					PathInfo info = get_path_to_type(u, v);
					if (info.reachable && info.distance > 5 && !route_exists(u, v))
					{
						if (is_valid_geometry(u, v))
						{
							action_queue.push_back("TELEPORT " + to_string(u) + " " + to_string(v));
							resources -= 5000;

							Route r = {u, v, 0, true};
							routes.push_back(r);
							route_capacity_map[{u, v}] = 0;
							break;
						}
					}
				}
			}
		}

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

// ==========================================
// MAIN LOOP
// ==========================================

int main()
{
	Solver solver;

	while (1)
	{
		cin >> solver.resources;
		cin.ignore();

		int num_travel_routes;
		cin >> num_travel_routes;
		cin.ignore();

		solver.reset_turn();

		for (int i = 0; i < num_travel_routes; i++)
		{
			int b1, b2, capacity;
			cin >> b1 >> b2 >> capacity;
			cin.ignore();

			Route r;
			r.b1 = b1;
			r.b2 = b2;
			r.capacity = capacity;
			r.is_teleporter = (capacity == 0);

			solver.routes.push_back(r);

			int u = b1, v = b2;
			if (u > v)
				swap(u, v);
			solver.route_capacity_map[{u, v}] = capacity;
		}

		int num_pods;
		cin >> num_pods;
		cin.ignore();
		for (int i = 0; i < num_pods; i++)
		{
			string line;
			getline(cin, line);
			stringstream ss(line);
			Pod p;
			int num_stops, stop;
			ss >> p.id >> num_stops;
			while (ss >> stop)
				p.path.push_back(stop);
			solver.pods.push_back(p);

			// Mark routes as having pods
			for (size_t j = 0; j < p.path.size() - 1; j++)
			{
				int u = p.path[j];
				int v = p.path[j + 1];
				if (u > v)
					swap(u, v);
				solver.has_pod_map[{u, v}] = true;
			}
		}

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

			if (type_or_zero == 0)
			{
				b.type = 0;
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
				b.type = type_or_zero;
				ss >> b.id >> b.p.x >> b.p.y;
				b.num_astronauts = 0;
			}
			solver.buildings[b.id] = b;
		}

		solver.solve();
	}
}