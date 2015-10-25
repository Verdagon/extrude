#include "utilities.h"
#include <fstream>
#include <random>
#include <glm.hpp>
#include <iostream>

std::string readFileAsString(const char *filename)
{
	std::ifstream in(filename, std::ios::in | std::ios::binary);
	if (in)
	{
		std::string contents;
		in.seekg(0, std::ios::end);
		contents.resize(in.tellg());
		in.seekg(0, std::ios::beg);
		in.read(&contents[0], contents.size());
		in.close();
		return(contents);
	}
	std::cerr << "Error in readFileAsString of " << filename << std::endl;
	throw(errno);
}

static std::mt19937 rng;
static std::default_random_engine generator;
static std::uniform_real_distribution<double> distribution (0.0,1.0);

glm::vec4 randomColor() {
	return glm::vec4(distribution(generator), distribution(generator), distribution(generator), 1.0);
}
