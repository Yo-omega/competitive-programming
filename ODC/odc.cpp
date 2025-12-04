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

	// Collinear checks omitted for brevity as game guarantees no collinear overlaps usually,
	// but pure crossing is the main illegal move.
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

	// Helpers for logic
	map<pair<int, int>, bool> existing_route_map; // Check if connection exists
	vector<string> action_queue;

	void reset_turn()
	{
		routes.clear();
		pods.clear();
		existing_route_map.clear();
		action_queue.clear();
	}

	// --- Cost Calculations ---
	int get_tube_cost(int b1, int b2)
	{
		double d = dist(buildings[b1].p, buildings[b2].p);
		return floor(d * 10.0); // 1 resource per 0.1km
	}

	// --- Graph Helpers ---

	// Checks if a route (Tube/Teleporter) already exists between b1 and b2
	bool route_exists(int b1, int b2)
	{
		if (b1 > b2)
			swap(b1, b2);
		return existing_route_map.count({b1, b2});
	}

	// Checks if a NEW tube between b1 and b2 would cross ANY existing tube
	bool is_valid_geometry(int b1, int b2)
	{
		Point p1 = buildings[b1].p;
		Point p2 = buildings[b2].p;

		for (const auto &r : routes)
		{
			if (r.is_teleporter)
				continue; // Teleporters don't block tubes
			Point p3 = buildings[r.b1].p;
			Point p4 = buildings[r.b2].p;
			if (segments_intersect(p1, p2, p3, p4))
				return false;
		}
		return true;
	}

	// --- Pathfinding (The Astronaut Brain) ---
	// Returns distance (in tubes) to nearest module of 'target_type'
	// Returns 9999 if unreachable.
	int get_distance_to_type(int start_id, int target_type)
	{
		if (buildings[start_id].type == target_type)
			return 0;

		// Build Adjacency List just for this calculation
		map<int, vector<pair<int, int>>> adj; // Node -> {Neighbor, Weight}
		for (const auto &r : routes)
		{
			int w = (r.is_teleporter) ? 0 : 1;
			adj[r.b1].push_back({r.b2, w});
			adj[r.b2].push_back({r.b1, w});
		}

		priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
		map<int, int> dists;

		pq.push({0, start_id});
		dists[start_id] = 0;

		while (!pq.empty())
		{
			int d = pq.top().first;
			int u = pq.top().second;
			pq.pop();

			if (d > dists[u] && dists.count(u))
				continue;

			// Found a target?
			if (buildings[u].type == target_type)
				return d;

			for (auto &edge : adj[u])
			{
				int v = edge.first;
				int weight = edge.second;
				if (!dists.count(v) || dists[v] > d + weight)
				{
					dists[v] = d + weight;
					pq.push({dists[v], v});
				}
			}
		}
		return 9999;
	}

	// --- Main Logic ---

	void solve()
	{
		// Phase 1: Connect Stranded Astronauts
		// We iterate through every landing pad (Type 0).
		for (auto &pair : buildings)
		{
			Building &b = pair.second;
			if (b.type != 0 || b.num_astronauts == 0)
				continue;

			// Check every astronaut group at this pad
			for (auto const &[type, count] : b.astronaut_counts)
			{
				if (count == 0)
					continue;

				// Check if they can currently reach their destination
				int current_dist = get_distance_to_type(b.id, type);

				// If unreachable (infinity), we MUST build a tube
				if (current_dist > 1000)
				{
					// Strategy: Find the nearest module of that type
					int best_target = -1;
					double best_dist = 1e9;

					for (auto &target_pair : buildings)
					{
						if (target_pair.second.type == type)
						{
							double d = dist(b.p, target_pair.second.p);
							if (d < best_dist)
							{
								// Only candidate if Geometry allows it
								if (is_valid_geometry(b.id, target_pair.second.id) && !route_exists(b.id, target_pair.second.id))
								{
									best_dist = d;
									best_target = target_pair.second.id;
								}
							}
						}
					}

					// If we found a valid target, check budget and build
					if (best_target != -1)
					{
						int cost = get_tube_cost(b.id, best_target);
						// We need cost for Tube + cost for Pod (1000)
						if (resources >= cost + 1000)
						{
							// Build Tube
							action_queue.push_back("TUBE " + to_string(b.id) + " " + to_string(best_target));
							resources -= cost;

							// Immediately Build Pod (Ping-Pong: A -> B -> A)
							// Note: ID for new pod doesn't matter for command, usually auto-assigned or we pick 1.
							// The command is POD [podId] [path...]
							// We'll use a dummy ID like 1, game usually ignores ID in construction or handles it.
							// Actually, checking rules: "POD podId..." You pick the ID. Let's use a random high number or counter.
							int newPodId = pods.size() + 100;
							action_queue.push_back("POD " + to_string(newPodId) + " " + to_string(b.id) + " " + to_string(best_target) + " " + to_string(b.id));
							resources -= 1000;

							// Update internal state so we don't build duplicate this turn
							Route r = {b.id, best_target, 1, false};
							routes.push_back(r);
							int u = b.id, v = best_target;
							if (u > v)
								swap(u, v);
							existing_route_map[{u, v}] = true;
						}
					}
				}
			}
		}

		// Phase 2: Add Pods to empty tubes
		// If we have extra money, ensure every tube has at least one pod
		for (const auto &r : routes)
		{
			if (r.is_teleporter)
				continue;

			// Simple check: is there a pod on this exact segment?
			// (This is a simplified check. A robust one would check if any pod's path covers this tube)
			// For this basic solution, we rely on the creation logic in Phase 1.
			// But if we have money and a tube is empty, let's add a pod.
			if (resources > 1000)
			{
				// Check if any pod covers this route
				bool covered = false;
				for (const auto &p : pods)
				{
					for (size_t i = 0; i < p.path.size() - 1; ++i)
					{
						int u = p.path[i];
						int v = p.path[i + 1];
						if ((u == r.b1 && v == r.b2) || (u == r.b2 && v == r.b1))
						{
							covered = true;
							break;
						}
					}
				}

				if (!covered)
				{
					int newPodId = pods.size() + 500 + rand() % 1000;
					action_queue.push_back("POD " + to_string(newPodId) + " " + to_string(r.b1) + " " + to_string(r.b2) + " " + to_string(r.b1));
					resources -= 1000;
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
				cout << action_queue[i] << (i == action_queue.size() - 1 ? "" : ";");
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

			if (b1 > b2)
				swap(b1, b2);
			solver.existing_route_map[{b1, b2}] = true;
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
				// It is a Landing Pad
				b.type = 0;
				ss >> b.id >> b.p.x >> b.p.y >> b.num_astronauts;
				// Read astronaut types
				for (int k = 0; k < b.num_astronauts; ++k)
				{
					int type;
					ss >> type;
					b.astronaut_counts[type]++;
				}
			}
			else
			{
				// It is a Module (type_or_zero is the type)
				b.type = type_or_zero;
				ss >> b.id >> b.p.x >> b.p.y;
				b.num_astronauts = 0;
			}
			// Store persistent building data
			solver.buildings[b.id] = b;
		}
		solver.solve();
	}
}