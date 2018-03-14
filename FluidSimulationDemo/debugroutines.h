#ifndef NB_DEBUGROUTINES_H
#define NB_DEBUGROUTINES_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

// DEBUG QQQ
// Export/import routines for debugging using text files
void exportArray(const char* filename, float* ar, int sizeX, int sizeY) {
	std::ofstream myfile;
	myfile.open(filename);

	// Stretch goal: Save floats in hexfloat format
	// for lossless passing between applications

	for (int y = 0; y < sizeY; y++) {
		for (int x = 0; x < sizeX; x++) {
			myfile << (double)ar[x + sizeX*y];
			if (x != sizeX - 1) {
				myfile << '\t';
			}
		}
		if (y != sizeY - 1) {
			myfile << '\n';
		}
	}

	myfile.close();
}

void exportArray(const char* filename, int* ar, int sizeX, int sizeY) {
	std::ofstream myfile;
	myfile.open(filename);

	// Stretch goal: Save floats in hexfloat format
	// for lossless passing between applications

	for (int y = 0; y < sizeY; y++) {
		for (int x = 0; x < sizeX; x++) {
			myfile << ar[x + sizeX*y];
			if (x != sizeX - 1) {
				myfile << '\t';
			}
		}
		if (y != sizeY - 1) {
			myfile << '\n';
		}
	}

	myfile.close();
}

float* ReadArrayFromFile(const char* filename, int& outSizeX, int& outSizeY) {
	// Implementation based on the answers to https://stackoverflow.com/questions/1075712/reading-delimited-files-in-c
	// strtof reference: http://www.cplusplus.com/reference/cstdlib/strtof/

	using namespace std;
	vector<vector<float>> rows;
	ifstream myfile(filename);

	string row;
	string field;
	// This code works because of std::iftstream's explicit
	// (i.e. no implicit conversions) bool conversion, which
	// returns true if and only if fail() is false:
	while (getline(myfile, row, '\n')) {
		rows.push_back(vector<float>());
		istringstream ss(row);
		while (getline(ss, field, '\t')) {
			float v = strtof(field.c_str(), nullptr); // convert to float
			rows.back().push_back(v);
		}
	}

	// Convert to a 2D array
	// Compute width and height
	outSizeY = (int)rows.size();
	outSizeX = 0;
	for (int i = 0; i < outSizeY; i++) {
		if (rows[i].size() > outSizeX) {
			outSizeX = (int)rows[i].size();
		}
	}

	// Create new 2D array and fill its contents (fill with 0s)
	float* result = new float[outSizeX*outSizeY]();
	for (int y = 0; y < outSizeY; y++) {
		int rowlen = (int)rows[y].size();
		for (int x = 0; x < rowlen; x++) {
			result[x + outSizeX*y] = rows[y][x];
		}
	}

	return result;
}

float ComputeL2Norm(float* ar, int sizeX, int sizeY) {
	// Computes the L2 Norm of the array, which should be useful
	// for comparing if two arrays are different at any step.
	// Not useful for checking if two values have been rearranged.
	double sum = 0.0;
	int len = sizeX*sizeY;
	for (int i = 0; i < len; i++) {
		sum += ar[i] * ar[i];
	}
	return static_cast<float>(sqrt(sum));
}

void exportParticles(const char* filename, const std::vector<Particle>& particles) {
	std::ofstream myfile;
	myfile.open(filename);

	int len = (int)particles.size();
	for (int i = 0; i < len; i++) {
		myfile << particles[i].X << '\t' << particles[i].Y << '\t' << particles[i].uX << '\t' << particles[i].uY;
		if (i != len - 1) {
			myfile << '\n';
		}
	}

	myfile.close();
}



#endif // NB_DEBUGROUTINES_H
