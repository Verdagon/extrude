#pragma once

#include "utilities.h"

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <sstream>
#include <fstream>
#include <glm.hpp>

class SVG {
public:
	virtual ~SVG() { }
	virtual void addPath(const std::vector<glm::vec2> & points) = 0;
	virtual void addPoint(glm::vec2 point, float radius, const std::string & color) = 0;
	virtual void addLine(glm::vec2 start, glm::vec2 end) = 0;
	virtual void addLine(glm::vec2 start, glm::vec2 end, std::string color) = 0;
	virtual void addText(glm::vec2 point, const std::string & text, float angle = 30.0) = 0;
	virtual std::string getOutput() const = 0;
};


class SVGLogger {
public:
	~SVGLogger() { }
	virtual owning_ptr<SVG> makeSVG() const = 0;
	virtual void log(shared_ptr<SVG> svg, const std::string & name) = 0;
};


class FakeSVGLogger : public SVGLogger {
public:
	class FakeSVG : public SVG {
	public:
		void addPath(const std::vector<glm::vec2> & points) { }
		void addPoint(glm::vec2 point, float radius, const std::string & color) { }
		void addLine(glm::vec2 start, glm::vec2 end) { }
		void addLine(glm::vec2 start, glm::vec2 end, std::string color) { }
		void addText(glm::vec2 point, const std::string & text, float angle = 30.0) { }
		std::string getOutput() const { return ""; }
	};

	virtual owning_ptr<SVG> makeSVG() const override {
		return owning_ptr<SVG>(new FakeSVG());
	}

	void log(shared_ptr<SVG> svg, const std::string & name) override { }
};


class SVGLoggerImpl : public SVGLogger {
	std::string seriesName;
	int snapshotNum = 1;

	class SVGImpl : public SVG {
		float minX = std::numeric_limits<float>::max();
		float minY = std::numeric_limits<float>::max();
		float maxX = std::numeric_limits<float>::min();
		float maxY = std::numeric_limits<float>::min();
		std::list<std::function<void(std::stringstream *, float)>> operations;

	public:
		SVGImpl() { }

		void updateMinAndMax(glm::vec2 point) {
			minX = std::min(minX, point.x);
			minY = std::min(minY, point.y);
			maxX = std::max(maxX, point.x);
			maxY = std::max(maxY, point.y);
		}

		void addPath(const std::vector<glm::vec2> & points) override {
			for (glm::vec2 point : points) {
				updateMinAndMax(point);
			}

			operations.push_back([points](std::stringstream * out, float yFlip) {
				*out << "<path d=\"";
				*out << "M " << points[0].x << " " << (yFlip - points[0].y) << " ";
				if (points.size() == 2) {
					*out << "L " << points[1].x << " " << (yFlip - points[1].y) << " ";
				} else {
					if (points.size() == 3) {
						*out << "Q ";
					} else {
						*out << "C ";
					}
					for (int i = 1; i < points.size(); i++) {
						*out << points[i].x << " " << (yFlip - points[i].y) << " ";
					}
				}
				*out << "\" fill=\"transparent\" stroke=\"black\"/>" << std::endl;
			});
		}

		void addPoint(glm::vec2 point, float radius, const std::string & color) override {
			updateMinAndMax(point);

			operations.push_back([point, radius, color](std::stringstream * out, float yFlip) {
				*out << "<circle cx=\"" << point.x
						<< "\" cy=\"" << (yFlip - point.y)
						<< "\" r=\"" << radius << "\" stroke=\"{}\" "
						<< "stroke-width=\"" << radius << "\" fill=\"" << color << "\" />" << std::endl;
			});
		}

		void addLine(glm::vec2 start, glm::vec2 end) override {
			addLine(start, end, "black");
		}

		void addLine(glm::vec2 start, glm::vec2 end, std::string color) override {
			updateMinAndMax(start);
			updateMinAndMax(end);

			operations.push_back([start, end, color](std::stringstream * out, float yFlip) {
				*out << "<line x1=\"" << start.x
						<< "\" y1=\"" << (yFlip - start.y)
						<< "\" x2=\"" << end.x
						<< "\" y2=\"" << (yFlip - end.y)
						<< "\" stroke=\"" << color << "\" stroke-width=\".1\" fill=\"" << color << "\"/>" << std::endl;
			});
		}

		void addText(glm::vec2 point, const std::string & text, float angle = 30.0) override {
			updateMinAndMax(point);

			operations.push_back([point, text, angle](std::stringstream * out, float yFlip) {
				*out << "<text x=\"" << point.x << "\" y=\"" << (yFlip - point.y) << "\" fill=\"red\" font-size=\"1.0\" transform=\"rotate(" << angle << " " << point.x << "," << (yFlip - point.y) << ")\">" << text << "</text>" << std::endl;
			});
		}

		std::string getOutput() const override {
			std::stringstream out;

		    // Print some header stuff
		    out << "<?xml version=\"1.0\" standalone=\"no\"?>" << std::endl;
		    out << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"" << std::endl;
		    out << "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">" << std::endl;

		    out << "<svg viewBox=\""
		    		<< (minX - 10.0) << " " << (minY - 10.0) << " "
		    		<< (maxX - minX + 20.0) << " " << (maxY - minY + 20.0) <<  "\" "
		    		<< "xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">" << std::endl;

			for (const std::function<void(std::stringstream *, float)> & operation : operations) {
				operation(&out, minY + maxY);
			}

			out << "</svg>";

			return out.str();
		}
	};

	virtual owning_ptr<SVG> makeSVG() const override {
		return owning_ptr<SVG>(new SVGImpl());
	}

public:
	SVGLoggerImpl(const std::string & seriesName) :
		seriesName(seriesName) { }

	void log(shared_ptr<SVG> svg, const std::string & name) override {
		std::ofstream out(concat(std::string("svglog-"), seriesName, "-", snapshotNum, std::string("-"), name, std::string(".svg")));
		out << svg->getOutput();
		snapshotNum++;
	}
};
