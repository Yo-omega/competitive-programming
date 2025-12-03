#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

// looks more hardcoded, lol.

int calc_edges(string *table, const int n)
{
	int corners = 0;

	for (int y = 0; y < n; y++)
	{
		for (int x = 0; x < n; x++)
		{
			if (table[y][x] != '#')
				continue;

			bool up = (table[y - 1][x] == '#');
			bool down = (table[y + 1][x] == '#');
			bool left = (table[y][x - 1] == '#');
			bool right = (table[y][x + 1] == '#');

			bool upLeft = (table[y - 1][x - 1] == '#');
			bool upRight = (table[y - 1][x + 1] == '#');
			bool downLeft = (x > 0 && table[y + 1][x - 1] == '#');
			bool downRight = (table[y + 1][x + 1] == '#');

			if (!up && !left)
				corners++;
			if (!up && !right)
				corners++;
			if (!down && !left)
				corners++;
			if (!down && !right)
				corners++;

			if (up && left && !upLeft)
				corners++;
			if (up && right && !upRight)
				corners++;
			if (down && left && !downLeft)
				corners++;
			if (down && right && !downRight)
				corners++;
		}
	}

	return corners;
}

int main()
{
	int n;
	cin >> n;
	cin.ignore();
	cerr << n << endl;
	string table[n];
	for (int i = 0; i < n; i++)
	{
		string line;
		getline(cin, line);
		table[i] = line;
		cerr << line << endl;
	}

	cout << calc_edges(table, n) << endl;
}
