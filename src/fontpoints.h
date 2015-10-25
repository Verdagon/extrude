#pragma once

#include "svg.h"
#include <glm.hpp>

class GlyphNotFoundException : public std::runtime_error {
public:
	GlyphNotFoundException(std::string fontFilename, int charCode) :
			std::runtime_error(concat("Char not found in font ", fontFilename, ":", charCode)) { }
};

class GlyphHasSubglyphsException : public std::runtime_error {
public:
	GlyphHasSubglyphsException(std::string fontFilename, int charCode) :
			std::runtime_error(concat("Glyph has subglyphs, ", fontFilename, ":", charCode)) { }
};

struct PointFromFont {
	glm::vec2 location;
	bool insideCurve;
};

std::vector<std::vector<PointFromFont>> pointsFromFont(
		shared_ptr<SVGLogger> svgLogger, std::string fontFilename, int charCode, int detail, float minDist);
