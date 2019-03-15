#include <iostream>
#include <vector>
#include <random>
#include <limits>
#include <string>

using namespace std;

// struct for Coords in graph (unweighted)
struct Coord {
	int row, col, type, g, h, f;
	Coord *parent;
};


// size of graph
const int SIZE = 35;

// random number generator
mt19937 rng;
// uniform random distibution for choosing start and finish coordinates
uniform_int_distribution<mt19937::result_type> runif_SIZE(0, SIZE - 1);

// SIZE x SIZE matrix of Coords to represent graph
Coord mat[SIZE][SIZE];

// pointers to start and end Coords
Coord *start;
Coord *finish;

// open list: (pointers to) Coords to be expanded
vector<Coord*> open_list;
// closed list: (pointers to) Coords which have been expanded
vector<Coord*> closed_list;


// returns the index of the (pointer to) Coord in the open list
// with the lowest f value
int get_from_open_list() {
	// minimum f value so far
	int min_f = numeric_limits<int>::max();
	// index at which this occurs
	int min_f_i = 0;
	// traverse open list
	for (int i = 0; i < (int)open_list.size(); i++) {
		// update min_f and min_f_i if current element has
		// lower f value than min so far
		int current_f = open_list[i]->f;
		if (current_f <= min_f) {
			min_f = current_f;
			min_f_i = i;
		}
	}
	// return index
	return min_f_i;
}



// heuristic function for estimating cost of travelling to finish
// using manhattan distance
int heuristic(const Coord * const n) {
	// return manhattan distance from n to finish
	return abs(n->row - finish->row) + abs(n->col - finish->col);
}

// adds a (pointer to) Coord to the open list after
// setting its h and f values
void add_to_open_list(Coord * const n) {
	// set h based on heuristic
	n->h = heuristic(n);
	// set f as g + h
	n->f = n->g + n->h;
	// add n to open list
	open_list.push_back(n);
}


// check if (pointer to) Coord appears in given vector
bool is_in_list(const vector<Coord*> * const list, const Coord * const n_to_find) {
	// traverse given vector
	for (Coord *n : *list) {
		// return true if Coord is found
		if (n == n_to_find) {
			return true;
		}
	}
	// return false if (pointer to) Coord not found 
	return false;
}


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
	for (int i = 0; i < SIZE; i++) {
		for (int j = 0; j < SIZE; j++) {
			mat[i][j] = { i, j, 0, 0, 0, 0, nullptr };
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
	if (north >= 0 && (mat[north][col].type < 2)) {
		adj_Coords.push_back(&mat[north][col]);
	}
	if (south <= SIZE - 1 && (mat[south][col].type < 2)) {
		adj_Coords.push_back(&mat[south][col]);
	}
	if (west >= 0 && (mat[row][west].type < 2)) {
		adj_Coords.push_back(&mat[row][west]);
	}
	if (east <= SIZE - 1 && (mat[row][east].type < 2)) {
		adj_Coords.push_back(&mat[row][east]);
	}

	// return vector of (pointers to) adjacent Coords
	return adj_Coords;
}


bool astar() {
	bool solved = false;
	// add (pointer to) starting Coord to open list
	open_list.push_back(start);
	// loop while open list is not empty and maze is not solved
	while (open_list.size() > 0 && !solved) {
		// get index of (pointer to) Coord with lowest f value
		int lowest_f_i = get_from_open_list();
		Coord *current_Coord = open_list[lowest_f_i];
		// remove (pointer to) this Coord from open list
		open_list.erase(open_list.begin() + lowest_f_i);
		// if this Coord is finish, maze is solved
		if (current_Coord->type == 1) {
			solved = true;
		}
		// otherwise
		else {
			// next g (distance from start) is current g +1
			int next_g = current_Coord->g + 1;
			// for (pointer to) each adjacent Coord which is not an obstacle or the start
			for (Coord *n : get_adj_Coords(current_Coord)) {
				// check whether (pointer to) this Coord occurs in open or closed list
				bool in_closed_list = is_in_list(&closed_list, n);
				bool in_open_list = is_in_list(&open_list, n);
				// if (pointer to) Coord is present in open or closed lists and current g is lower
				// i.e. current path to this Coord is shorter than one previously found
				// update g of Coord and change its parent to current Coord
				if (next_g < n->g && (in_closed_list || in_open_list)) {
					n->g = next_g;
					n->parent = current_Coord;
				}
				// if Coord is not in either list, set its g to current g
				// and its parent to (pointer to) the current Coord, and
				// add it to the open list
				else if (!in_closed_list && !in_open_list) {
					n->parent = current_Coord;
					n->g = next_g;
					add_to_open_list(n);
				}
			}
			// after processing child Coords, add (pointer to) current Coord to closed list
			closed_list.push_back(current_Coord);
		}
	}
	// return false if maze is unsolved after open list
	// is depleted
	if (!solved) {
		return false;
	}
	// return true if maze is solved
	return true;
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
				int g = mat[i][j].g;
				if (g >= 0 && g <= 9) {
					element += " ";
				}
				if (g == 0) {
					element += "-";
				}
				else {
					// element to print is dist (distance from start)*/
					element += to_string(g);
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


// trace back from finish and set type of Coords in path
vector<Coord*> backtrace() {
	vector<Coord*> path{};
	// whether backtrace is finished
	bool done = false;
	// (pointer to) current Coord in path, initialized to 
	// (pointer to) parent of finish Coord
	Coord *pos = finish->parent;
	path.insert(path.begin(), finish);
	// loop until done (i.e. parent Coord is start)
	while (!done) {
		// set type to 4 / path
		if (pos->type != 2) {
			pos->type = 4;
			path.insert(path.begin(), pos);
			// done if parent type is 2 / start
			if (pos->parent->type == 2) {
				path.insert(path.begin(), pos->parent);
				done = true;
			}
			// otherwise move to parent Coord
			else {
				pos = pos->parent;
			}
		}
		else {
			path.insert(path.begin(), pos);
			done = true;
		}
	}
	return path;
}


int main() {
	for (int i = 0; i < 100; i++) {
		// seed rng
		rng.seed(random_device()());
		// generate matrix
		gen_matrix(5);
		// attempt to solve
		bool solved = astar();
		// backtrace if solved
		if (solved) {
			vector<Coord*> path = backtrace();
			print_matrix();
			for (auto *v : path) {
				cout << "[" << v->col << ", " << v->row << "], ";
			}
			cout << endl << finish->g << "\n\n\n";
			path.clear();
		}
		open_list.clear();
		closed_list.clear();
	}

	return 0;
}