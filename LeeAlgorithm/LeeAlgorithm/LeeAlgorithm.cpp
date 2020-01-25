#include <iostream>
#include <random>
#include <queue>
#include <vector>
#include <algorithm>
#include <limits>

using namespace std;

struct Coord {
	int x, y, type, dist;
};

const int M = 100;
const int N = 100;

// random number generator
mt19937 rng;
// uniform random distibution for choosing start and finish coordinates
uniform_int_distribution<mt19937::result_type> runif_SIZE(0, M - 1);

// SIZE x SIZE matrix of Coords to represent graph
Coord mat[M][N];

Coord *start;
Coord *finish;

queue<Coord*> q;

// randomly populate the graph with start, finish and obstacles
// (may or may not be solvable)
void gen_matrix(int obs_odds) {

	uniform_int_distribution<mt19937::result_type> runif_obstacle(0, obs_odds);

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

	// populate graph
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
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


// returns vector of (pointers to) all non-obstacle Coords adjacent to given
vector<Coord*> get_adj_Coords(const Coord * const n) {

	vector<Coord*> adj_Coords = {};

	// row and column indices of current Coord
	int row = n->y;
	int col = n->x;
	// row index - 1 (one row north)
	int north = n->y - 1;
	// row index + 1 (one row south)
	int south = n->y + 1;
	// column index - 1 (one column west)
	int west = n->x - 1;
	// column index + 1 (one column east)
	int east = n->x + 1;

	// for NSWE, add (pointer to) adjacent Coord to vector if
	// it is valid (within bounds of graph) and is either 
	// a standard Coord or the finish Coord
	if (north >= 0 && (mat[north][col].type <= 2)) {
		adj_Coords.push_back(&mat[north][col]);
	}
	if (south <= N - 1 && (mat[south][col].type <= 2)) {
		adj_Coords.push_back(&mat[south][col]);
	}
	if (west >= 0 && (mat[row][west].type <= 2)) {
		adj_Coords.push_back(&mat[row][west]);
	}
	if (east <= M - 1 && (mat[row][east].type <= 2)) {
		adj_Coords.push_back(&mat[row][east]);
	}

	// return vector of (pointers to) adjacent Coords
	return adj_Coords;
}


bool LeeAl() {
	// add (pointer to) starting Coord to queue
	q.push(start);
	// not done
	bool done = false;
	int iterations = 0;
	// while not done
	while (!done) {
		// increment iterations counter
		iterations++;
		// get (pointer to) Coord at front of queue
		Coord *next_Coord = q.front();
		q.pop();
		// get (pointers to) adjacent Coords
		for (Coord *n : get_adj_Coords(next_Coord)) {
			// if adjacent Coord is finish, stop
			if (n->type == 1) {
				done = true;
			}
			// otherwise, if Coord is finish or normal Coord
			// and has not been expanded (i.e. dist is still 0)
			if (n->type <= 1 && n->dist == 0) {
				// set adjacent Coord's distance to distance of current Coord + 1
				n->dist = next_Coord->dist + 1;
				// add (pointer to) adjacent Coord to queue
				q.push(n);
			}
		}
	}
	/*if (!done) {
		return false;
	}*/
	//return true;
}

void backtrace() {

	int min_dist = numeric_limits<int>::max();
	Coord *pos = finish;
	bool done = false;
	while (!done) {
		for (Coord *n : get_adj_Coords(pos)) {
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
		}
	}
}


int main()
{
	// seed rng
	rng.seed(random_device()());
	// generate matrix
	gen_matrix(2);
	/*bool solved =*/ LeeAl();
	//if (solved) {
		backtrace();
	//}
		
	return 0;
}