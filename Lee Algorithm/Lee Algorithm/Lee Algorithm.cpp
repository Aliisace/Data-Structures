#include <iostream>
#include <cstdlib>
#include <random>
#include <math.h>
#include <iomanip>
#include <string>
#include <array>
#include <iterator>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <list>

using namespace std;

struct Coord {
	int x, y;
} rStart, rEnd;

#define M 20 
#define N 20

void printMatrix(int(*mat)[N][M]);
void printMatrix2(int (*mat)[N][M], Coord rEnd);
void addData(int (*mat)[N][M]);
Coord getPosition(Coord position, int(*mat)[N][M]);
void phase1(Coord rStart, Coord rEnd, int(*mat)[N][M]);
list<Coord> phase2(Coord rStart, Coord rEnd, int(*mat)[N][M]);
void autoMat(int(*mat)[N][M], list<Coord> path);

//Function to print the matrix
void printMatrix(int(*mat)[N][M]) {
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) {
			
			if ((*mat)[i][j] > -1 && (*mat)[i][j] < 10) {
				cout << " ";
			}
			cout << (*mat)[i][j] << "  ";
		}
		cout << "\n\n";
	}
	cout << "\n";
}


//Function to print the matrix
void printMatrix2(int (*mat)[N][M], Coord rEnd) {
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) {
			if (i == rEnd.y && j == rEnd.x) {
				cout << "-3  ";
			}
			else
			{
				if ((*mat)[i][j] > -1 && (*mat)[i][j] < 10) {
					cout << " ";
				}
				cout << (*mat)[i][j] << "  ";
			}		
		}
		cout << "\n\n";
	}
	cout << "\n";
}


// Function to add data to the matrix
// Generates pseudorandom obstacles
void addData(int (*mat)[N][M]) {
	srand(time(NULL));

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) {
			(*mat)[i][j] = 10;
		}
	}

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < M; j++) {
			if ((*mat)[i][j] == 10) {
				(*mat)[i][j] = -1;
			}
		}
	}

	for (int i = 0; i < M; i++) {
		int rand_row = rand() % M;
		int rand_col = rand() % N;
		(*mat)[rand_row][rand_col] = -2;
	}
}


//function to randomly generate start and end position
Coord getPosition(Coord position, int(*mat)[N][M]) {
	
	// Random seed
	random_device rd;

	// Initialize Mersenne Twister pseudo-random number generator
	mt19937 gen(rd());

	// Generate pseudo-random numbers uniformly distributed in range (1, M)
	uniform_int_distribution<> disM(1, (M - 1));
	uniform_int_distribution<> disN(1, (N - 1));

	position.x = disM(gen);
	position.y = disN(gen);
	if ((*mat)[position.y][position.x] == -2) {
		return getPosition(position, mat);
	}
	else {
		return position;
	} //Random position
}


void phase1(Coord rStart, Coord rEnd, int(*mat)[N][M]) {
	int kount = 0;
	bool changesMade = false; //Makes sure that no possible changes are missed
	(*mat)[rStart.y][rStart.x] = 0;
	do {
		changesMade = false; //Reset
		for (int i = 0; i < N; i++) { //Outerloop for the y value
			for (int j = 0; j < M; j++) { //Innerloop for the x value
				if ((*mat)[i][j] == kount) { //If the current position is the value being searched for
					if (i < N - 1 && (*mat)[i + 1][j] == -1) { //If the value next to it but still in the mat is empty, change it
						(*mat)[i + 1][j] = kount + 1;
						changesMade = true;
					}
					if (i > 0 && (*mat)[i - 1][j] == -1) {
						(*mat)[i - 1][j] = kount + 1;
						changesMade = true;
					}
					if (j < M - 1 && (*mat)[i][j + 1] == -1) {
						(*mat)[i][j + 1] = kount + 1;
						changesMade = true;
					}
					if (j > 0 && (*mat)[i][j - 1] == -1) {
						(*mat)[i][j - 1] = kount + 1;
						changesMade = true;
					}
				}
			}
		}
		kount = kount + 1;
		//Loop while the end value is -1 or the max value in the mat is the size of the mat
	} while ((kount <= N + M) && ((*mat)[rEnd.y][rEnd.x] <= -1));
	cout << "\nEnd Phase 1\n";
}


list<Coord> phase2(Coord rStart, Coord rEnd, int(*mat)[N][M]) {
	list<Coord> path = {};
	int kount = N + M;
	Coord position;
	position.y = rEnd.y;
	position.x = rEnd.x;
	path.push_front(position);
	while (kount > 0) {
		kount = (*mat)[position.y][position.x];
		if (position.y > 0 && (*mat)[position.y - 1][position.x] == kount - 1) {
			position.y = position.y - 1;
			path.push_front(position);
		}
		else if (position.y < N - 1 && (*mat)[position.y + 1][position.x] == kount - 1) {
			position.y = position.y + 1;
			path.push_front(position);
		}
		else if (position.x > 0 && (*mat)[position.y][position.x - 1] == kount - 1) {
			position.x = position.x - 1;
			path.push_front(position);
		}
		else if (position.x < M - 1 && (*mat)[position.y][position.x + 1] == kount - 1) {
			position.x = position.x + 1;
			path.push_front(position);
		}
		else if (kount == 0) {
			break;
		}
		else {
			cout << "ERROR" << endl;
			cout << "[" << position.x << ", " << position.y << "]" << endl;
			cout << kount << endl;
			break;
		}
	}
	cout << "\nEnd Phase 2\n";
	return path;
}

void autoMat(int(*mat)[N][M], list<Coord> path) {
	void printMatrix(int(*mat)[N][M]);
	void addData(int(*mat)[N][M]);
	Coord getPosition(Coord position, int(*mat)[N][M]); //Function to get valid positions
	void phase1(Coord rStart, Coord rEnd, int(*mat)[N][M]);
	list<Coord> phase2(Coord rStart, Coord rEnd, int(*mat)[N][M]);

	addData(mat);
	printMatrix(mat);
	rStart = getPosition(rStart, mat);
	rEnd = getPosition(rEnd, mat);
	
	if (rStart.x == rEnd.x && rStart.y == rEnd.y) {
		rEnd = getPosition(rEnd, mat);
	}
	
	phase1(rStart, rEnd, mat);
	path = phase2(rStart, rEnd, mat);
	printMatrix2(mat, rEnd);
	cout << "The Path from [" << rStart.x + 1 << ", " << rStart.y + 1 << "] to [" << rEnd.x + 1 << ", " << rEnd.y + 1 << "]:\n";
	while (path.empty() == false) {
		cout << "[" << path.front().x + 1 << ", " << path.front().y + 1 << "], ";
		path.pop_front();
	}
}

int main()
{
	void autoMat(int(*mat)[N][M], list<Coord> path);
	int mat[N][M] = { {} };
	list<Coord> path = {};

	autoMat(&mat, path);

	return 0;
}