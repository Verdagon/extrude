#pragma once

#include "monotonify.h"
#include "svg.h"

namespace triangulate {

void triangulate(
		shared_ptr<SVGLogger> svgLogger,
		monotonify::GraphRef graph);

}