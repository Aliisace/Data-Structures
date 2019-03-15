#include <iostream>
#include <random>
#include <string>
#include <algorithm>
#include <limits>
#include <vector>
#include <queue>

using namespace std;

// struct for Coord
struct Coord {
	int row, col, type, dist;
};

// random number generator
mt19937 rng;
// size of graph (SIZE x SIZE matrix)
const int SIZE = 35;
// uniform random distribution for generating start and finish coords
uniform_int_distribution<mt19937::result_type> runif_SIZE(0, SIZE - 1);
// graph
Coord mat[SIZE][SIZE];
// pointer to start and finish nodes
Coord *start;
Coord *finish;
// queue of nodes to expand
queue<Coord*> q;

// randomly populate the graph with start, finish and obstacles
// (may or may not be solvable)
void gen_matrix(int obs_odds) {

	// arrays for start and finish coordinates
	int start_coords[2];
	int finish_coords[2];

	// randomly generate start coordinates
	for (int i = 0; i < 2; i++) {
		start_coords[i] = runif_SIZE(rng);
	}
	// generate finish coordinates and ensure they are not the same
	do {
		for (int i = 0; i < 2; i++) {
			finish_coords[i] = runif_SIZE(rng);
		}
	} while (start_coords[0] == finish_coords[0] && start_coords[1] == finish_coords[1]);

	uniform_int_distribution<mt19937::result_type> runif_obstacle(0, obs_odds);

	// populate graph
	for (int i = 0; i < SIZE; i++) {
		for (int j = 0; j < SIZE; j++) {
			mat[i][j] = { i, j, 0, 0 };
			// radomly make Coord an obstacle
			if (runif_obstacle(rng) == 0) {
				mat[i][j].type = 3; // obstacle
			}
			// set Coord as start if coordinates match
			if (i == start_coords[0] && j == start_coords[1]) {
				mat[i][j].type = 2; // start
				start = &mat[i][j];
			}
			// set Coord as finish if coordinates match
			if (i == finish_coords[0] && j == finish_coords[1]) {
				mat[i][j].type = 1; // finish
				finish = &mat[i][j];
			}
		}
	}
}

// print graph to console
void print_matrix() {

	// loop through row indices
	for (int i = 0; i < SIZE; i++) {
		// loop through column indices
		for (int j = 0; j < SIZE; j++) {
			// element to print
			string element = "";
			// type of Coord
			int type = mat[i][j].type;
			// if normal Coord
			if (type == 0) {
				int dist = mat[i][j].dist;
				if (dist >= 0 && dist <= 9) {
					element += " ";
				}
				if (dist == 0) {
					element += "-";
				}
				else {
					// element to print is dist (distance from start)*/
					element += to_string(dist);
				}
			}
			// print F for finish Coord
			else if (type == 1) {
				element += " F";
			}
			// print S for start Coord
			else if (type == 2) {
				element += " S";
			}
			// print # for obstacles
			else if (type == 3) {
				element += " #";
			}
			// print . for nodes on path
			else if (type == 4) {
				element += " .";
			}
			cout << element << "  ";
		}
		cout << "\n" << endl;
	}
	cout << endl;
}

// returns vector of (pointers to) all non-obstacle nodes adjacent to given
vector<Coord*> get_adj_nodes(const Coord * const n) {

	vector<Coord*> adj_nodes = {};

	// row and column indices of current Coord
	int row = n->row;
	int col = n->col;
	// row index - 1 (one row north)
	int north = n->row - 1;
	// row index + 1 (one row south)
	int south = n->row + 1;
	// column index - 1 (one column west)
	int west = n->col - 1;
	// column index + 1 (one column east)
	int east = n->col + 1;

	// for NSWE, add (pointer to) adjacent Coord to vector if
	// it is valid (within bounds of graph) and is either 
	// a standard Coord or the finish Coord
	if (north >= 0 && (mat[north][col].type <= 2)) {
		adj_nodes.push_back(&mat[north][col]);
	}
	if (south <= SIZE - 1 && (mat[south][col].type <= 2)) {
		adj_nodes.push_back(&mat[south][col]);
	}
	if (west >= 0 && (mat[row][west].type <= 2)) {
		adj_nodes.push_back(&mat[row][west]);
	}
	if (east <= SIZE - 1 && (mat[row][east].type <= 2)) {
		adj_nodes.push_back(&mat[row][east]);
	}

	// return vector of (pointers to) adjacent nodes
	return adj_nodes;
}


bool LeeAl() {
	// add (pointer to) starting Coord to queue
	q.push(start);
	// not done
	bool done = false;
	// while not done
	do {
		// get (pointer to) Coord at front of queue
		Coord *next_node = q.front();
		q.pop();
		// get (pointers to) adjacent nodes
		for (Coord *n : get_adj_nodes(next_node)) {
			// if adjacent Coord is finish, stop
			if (n->type == 1) {
				done = true;
			}
			// otherwise, if Coord is finish or normal Coord
			// and has not been expanded (i.e. dist is still 0)
			if (n->type <= 1 && n->dist == 0) {
				// set adjacent Coord's distance to distance of current Coord + 1
				n->dist = next_node->dist + 1;
				// add (pointer to) adjacent Coord to queue
				q.push(n);
			}
		}
	} while (!done && q.size() > 0);
	if (!done) {
		return false;
	}
	return true;
}

vector<Coord*> backtrace() {
	vector<Coord*> path{};
	int min_dist = numeric_limits<int>::max();
	Coord *pos = finish;
	bool done = false;
	while (!done) {
		for (Coord *n : get_adj_nodes(pos)) {
			if (n->type == 2) {
				done = true;
			}
			else if (n->dist < min_dist && n->dist > 0) {
				min_dist = n->dist;
				pos = n;
			}
		}
		if (!done) {
			pos->type = 4;
			path.insert(path.begin(), pos);
		}
	}
	return path;
}

int main() {
	for (int i = 0; i < 100; i++) {
		rng.seed(random_device()());

		gen_matrix(5);
		bool lee = LeeAl();
		if (lee) {
			vector<Coord*> path = backtrace();
			print_matrix();
			for (auto *v : path) {
				cout << "[" << v->col << ", " << v->row << "], ";
			}
			cout << endl << finish->dist << "\n\n\n";
			path.clear();
		}
		while (q.size() > 0) {
			q.pop();
		}
	}
	return 0;
}
