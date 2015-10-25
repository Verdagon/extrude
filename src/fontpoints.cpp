
#include "svg.h"
#include "utilities.h"
#include "fontpoints.h"

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <fstream>
#include <glm.hpp>
#include <ft2build.h>
#include FT_FREETYPE_H

using std::list;
using std::endl;
using std::string;
using std::vector;
using glm::vec2;

enum class PointType {
	ON,
	ON_BEGIN,
	ON_END,
	OFF_CONIC,
	OFF_CUBIC
};

struct Point {
	PointType pointType;
	vec2 point;
};

struct Curve {
	vector<vec2> points;

	vec2 getAt(float t) const {
		switch (points.size()) {
		case 2: // Linear
			return points[0] * (1 - t) + points[1] * t;
		case 3: // Conic/Quadratic
			return points[0] * 1.0f * (1.0f - t) * (1.0f - t) +
					points[1] * 2.0f * (1.0f - t) * t +
					points[2] * 1.0f * t * t;
		case 4:
			return points[0] * 1.0f * t * (1.0f - t) * (1.0f - t) * (1.0f - t) +
					points[1] * 3.0f * t * (1.0f - t) * (1.0f - t) +
					points[2] * 3.0f * t * t * (1.0f - t) +
					points[3] * 1.0f * t * t * t;
		default:
			throw std::runtime_error(concat("Curve has wrong number of points: ", points.size()));
		}
	}
	bool isLine() const {
		return points.size() == 2;
	}
};

vector<vector<Point>> getOutlineCompressedPointsFromFont(std::string fontFilename, int charCode) {
	int error = 0;

	FT_Library library = { };
	error = FT_Init_FreeType(&library);
	if (error) {
		throw std::runtime_error(concat("Freetype init error: ", error));
	}

	FT_Face face = { };
	error = FT_New_Face(
			library,
			fontFilename.c_str(),
			0,
			&face);
	if (error == FT_Err_Unknown_File_Format ) {
		throw std::runtime_error("The font file could be opened and read, but it appears that its font format is unsupported.");
	} else if (error) {
		throw std::runtime_error(concat("Font file could not be opened or read or it's broken.", error));
	}
	error = FT_Set_Char_Size(face, 10, 10, 72, 72);
	if (error) {
		throw std::runtime_error(concat("Error setting character size: ", error));
	}
	int glyphIndex = FT_Get_Char_Index(face, charCode);
	if (!glyphIndex) {
		throw GlyphNotFoundException(fontFilename, charCode);
	}
	error = FT_Load_Glyph(face, glyphIndex, FT_LOAD_NO_BITMAP | FT_LOAD_NO_RECURSE);
	if (face->glyph->num_subglyphs > 0)
		throw GlyphHasSubglyphsException(fontFilename, charCode);
	// Load it again, we only wanted FT_LOAD_NO_RECURSE to check for subglyphs
	// and it's got some side effects... it implies FT_LOAD_NO_SCALE and
	// FT_LOAD_IGNORE_TRANSFORM
	error = FT_Load_Glyph(face, glyphIndex, FT_LOAD_NO_BITMAP);
	FT_GlyphSlot glyph = face->glyph;
	FT_Outline outline = glyph->outline;

	vector<vector<Point>> contours;

	int contourPointsBeginIndex = 0;
	int contourPointsEndIndex = 0;
	for (int contourIndex = 0; contourIndex < outline.n_contours; contourIndex++) {
		contourPointsEndIndex = 1 + outline.contours[contourIndex];
		vector<Point> currentContourPoints;
		for (int i = contourPointsBeginIndex; i < contourPointsEndIndex; i++) {
			float x = outline.points[i].x;
			float y = outline.points[i].y;
			int flags = outline.tags[i];
			PointType pointType = PointType::ON;
			if (flags & 0x1) {
				pointType = PointType::ON;
			} else {
				if (flags & 0x2) {
					pointType = PointType::OFF_CUBIC;
				} else {
					pointType = PointType::OFF_CONIC;
				}
			}
			currentContourPoints.push_back({
				pointType,
				vec2(x, y)
			});
		}

		contours.push_back(currentContourPoints);
		contourPointsBeginIndex = contourPointsEndIndex;
	}

	vassert(contourPointsEndIndex == outline.n_points);

	FT_Done_FreeType(library);

	return contours;
}

vector<Point> reconstructAdjacentConicsAndEnds(const vector<Point> & compressedPoints) {
	vector<Point> newPoints;

	{
		for (int currentI = 0; currentI < compressedPoints.size(); currentI++) {
			Point previousPoint = compressedPoints[currentI];
			newPoints.push_back(previousPoint);

			// If there's a next point, and we're both off-conics, then put a
			// on point between us.
			int nextPointI = currentI + 1;
			if (nextPointI < compressedPoints.size()) {
				Point nextPoint = compressedPoints[nextPointI];

				// At this point, both this and the next point are conic, which means
				// we should insert an on point between.
				if (previousPoint.pointType == PointType::OFF_CONIC && nextPoint.pointType == PointType::OFF_CONIC) {
					newPoints.push_back({
						PointType::ON,
						(previousPoint.point + nextPoint.point) * 0.5f
					});
				}
			}
		}
	}

	// Sometimes freetype is retarded and leaves out the first or last ON,
	// which means we're supposed to pretend it's a loop.
	Point first = newPoints[0];
	Point last = newPoints[newPoints.size() - 1];
	if (first.pointType == PointType::ON) {
		if (last.pointType == PointType::ON) {
			if (first.point == last.point) {
				// The first point is equal to the last point, and they're both On,
				// which is what we hope to end with anyway, so do nothing.

				// See if this actually ever happens, feel free to take it out
				curiosity();
			} else {
		        // They're both ON, but they're at different positions.
		        // Copy the first one to the last to close the last's curve.
		        // It'll be a straight line, since the last point is an ON,
		        // and we're now connecting it to the first point, which is also ON.
		        newPoints.push_back(first);
			}
		} else {
			if (last.pointType == PointType::OFF_CUBIC) {
				// It would be weird to have our array end in [..., On, OFF_CUBIC]
				// This assert makes sure that if it ends in OFF_CUBIC, it ends in [..., On, OFF_CUBIC, OFF_CUBIC]
				vassert(newPoints[newPoints.size() - 3].pointType == PointType::ON);
				vassert(newPoints[newPoints.size() - 2].pointType == PointType::OFF_CUBIC);
			}
			// Last point is off, and first point is on. just add the first point to the end.
			// If last point was conic, then we're closing its conic curve.
			// If last point was cubic, then we're closing its cubic curve.
			newPoints.push_back(first);
		}
	} else {
		if (last.pointType == PointType::ON) {
			// First point is not an ON, but the last point is.
			if (first.point == last.point) {
				// TODO: implement
				vassert(false);
			} else {
				newPoints.insert(newPoints.begin(), last);
			}
		} else {
			// TODO: implement
			vassert(false);
		}
	}

	return newPoints;
}

vector<Point> insertEndPoints(const vector<Point> & compressedPoints) {
	vector<Point> points;

	// Until now, the On points have acted in these three capacities:
	// 1. begin curve
	// 2. end curve and begin new curve
	// 3. end curve

	// This function's goal is to separate the On point with:
	// 1. an OnBegin point
	// 2. an OnEnd point followed by an OnBegin point
	// 3. an OnEnd point

	// The way we do that is to replace all the On points with
	// OnEnd and OnBegin, and then afterwards, we'll cut off the
	// first OnEnd in the list and the last OnBegin in the list.
	for (Point point : compressedPoints) {
		if (point.pointType == PointType::ON) {
			points.push_back({
				PointType::ON_END,
				point.point
			});
			points.push_back({
				PointType::ON_BEGIN,
				point.point
			});
		} else {
			points.push_back(point);
		}
	}

  vassert(points.front().pointType == PointType::ON_END);
  points.erase(points.begin());
  vassert(points.back().pointType == PointType::ON_BEGIN);
  points.pop_back();

  return points;
}

vector<Point> decompress(const vector<Point> & compressedPoints) {
	vector<Point> points = compressedPoints;
	points = reconstructAdjacentConicsAndEnds(points);
	points = insertEndPoints(points);
	return points;
}

vector<Curve> pointsToCurves(const vector<Point> & points) {
	vector<Curve> curves;
	vector<vec2> currentCurvePoints;
	for (Point point : points) {
		currentCurvePoints.push_back(point.point);
		if (point.pointType == PointType::ON_END) {
			curves.push_back({
				currentCurvePoints
			});
			currentCurvePoints = vector<vec2>();
		}
	}
	return curves;
}

vector<PointFromFont> dedupePoints(const vector<PointFromFont> & originalPoints) {
	vector<PointFromFont> dedupedPoints;
	for (int pointI = 0; pointI < originalPoints.size(); pointI++) {
		PointFromFont point = originalPoints[pointI];
		if (!dedupedPoints.empty() && veeq(dedupedPoints.back().location, point.location)) {
			continue;
		}
		dedupedPoints.push_back(point);
	}
	return dedupedPoints;
}

vector<PointFromFont> decolinearPoints(const vector<PointFromFont> & originalPoints) {
	vector<PointFromFont> decolinearedContour;
	for (int pointI = 0; pointI < originalPoints.size(); pointI++) {
		PointFromFont point = originalPoints[pointI];
		int previousPointI = (pointI + originalPoints.size() - 1) % originalPoints.size();
		PointFromFont previousPoint = originalPoints[previousPointI];
		int nextPointI = (pointI + 1) % originalPoints.size();
		PointFromFont nextPoint = originalPoints[nextPointI];
		vec2 prevToThis = glm::normalize(point.location - previousPoint.location);
		vec2 nextToThis = glm::normalize(point.location - nextPoint.location);
		vec2 prevToThisPerpendicular = vec2(-prevToThis.y, prevToThis.x);
		float posDot = std::abs(glm::dot(nextToThis, prevToThisPerpendicular));
		if (posDot < 0.0001) {
			// Then prevToThis and nextToThis are basically one line, so skip
			continue;
		}
		decolinearedContour.push_back(point);
	}
	return decolinearedContour;
}

vector<vector<PointFromFont>> cleanup(const vector<vector<PointFromFont>> & contours) {
	vector<vector<PointFromFont>> newContours;
	for (const vector<PointFromFont> & originalPoints : contours) {
		if (originalPoints.empty()) {
			std::cerr << "Skipping empty contour from font" << std::endl;
			continue;
		}

		vector<PointFromFont> currentPoints = originalPoints;
		// This is in a loop because of Blocst3d.ttf's @ where we had something like
		// (0,0) (1,0) (1,1) (1,0) (2,0) where we remove the colinear, THEN the duplicate
		// Its conceivable we could get (0,0) (1,0) (1,1) (1,1) (1,0) (2,0)
		// where we remove duplicate, colinear, duplicate
		while (true) {
			vector<PointFromFont> dedupedPoints = dedupePoints(currentPoints);
			int numDuplicatesRemoved = currentPoints.size() - dedupedPoints.size();
			if (numDuplicatesRemoved)
				std::cerr << "Removed " << numDuplicatesRemoved << " duplicates" << std::endl;
			vector<PointFromFont> decolinearedPoints = decolinearPoints(dedupedPoints);
			int numColinearsRemoved = dedupedPoints.size() - decolinearedPoints.size();
			if (numColinearsRemoved)
				std::cerr << "Removed " << numColinearsRemoved << " colinears" << std::endl;
			currentPoints = decolinearedPoints;
			if (numDuplicatesRemoved + numColinearsRemoved == 0)
				break;
		}
		newContours.push_back(currentPoints);
	}
	return newContours;
}

vector<vector<PointFromFont>> pointsFromFont(shared_ptr<SVGLogger> svgLogger, std::string fontFilename, int charCode, int detail, float minDist) {
	vector<vector<Point>> compressedContoursPoints = getOutlineCompressedPointsFromFont(fontFilename, charCode);

	vector<vector<Curve>> contours;
	for (const vector<Point> & compressedContourPoints : compressedContoursPoints) {
		vector<Point> contourPoints = decompress(compressedContourPoints);
		vector<Curve> contourCurves = pointsToCurves(contourPoints);
		contours.push_back(contourCurves);
	}

	{
		auto svg = svgLogger->makeSVG();
		for (vector<Curve> & contour : contours) {
			for (Curve & curve : contour) {
				svg->addPath(curve.points);
			}
		}
		svgLogger->log(svg, "contours");
	}

	vector<vector<PointFromFont>> sampledContours;
	foreachI(contours, [&](int i, const vector<Curve> & contour) {
		vector<PointFromFont> sampledContour;
		foreachI(contour, [&](int j, const Curve & curve) {
			int pointsPerCurve = curve.isLine() ? 1 : detail;
			vec2 lastPoint = { };
			for (int f = 0; f < pointsPerCurve; f++) {
				vec2 point = curve.getAt((float)f / pointsPerCurve);
				if (f == 0 || (point - lastPoint).length() > minDist) {
					sampledContour.push_back(PointFromFont { point, f > 0 } );
					lastPoint = point;
				}
			}
		});
		sampledContours.push_back(sampledContour);
	});

	{
		auto svg = svgLogger->makeSVG();
		for (const vector<PointFromFont> & line : sampledContours) {
			for (int i = 0; i < line.size(); i++) {
				glm::vec2 point = line[i].location;
				glm::vec2 nextPoint = line[(i + 1) % line.size()].location;
				svg->addPoint(point, .1, "red");
				svg->addLine(point, nextPoint);
			}
		}
		svgLogger->log(svg, "sampled");
	}

	return cleanup(sampledContours);
}
