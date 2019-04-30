/**
 * Copyright (c) 2019 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef semantic_world_hpp_included_
#define semantic_world_hpp_included_

#include <vector>
#include <array>

#include <boost/variant.hpp>

#include "utility/enum-io.hpp"
#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"

namespace semantic {

namespace roof {

struct Rectangular {
    math::Size2f size;
    math::Point2d skew;
    double azimuth = 0.0;
    std::array<double, 4> curb = {};
    double ridgeHeight = 0.0;
    double curbHeight = 0.0;
    std::array<double, 4> eaveHeight = {};
};

struct Circular {
    double radius = 0.0;
    double curb = 0.0;
    double ridgeHeight = 0.0;
    double curbHeight = 0.0;
    double eaveHeight = 0.0;
};

typedef boost::variant<Rectangular, Circular> Type;

} // namespace roof

struct Roof {
    typedef std::vector<Roof> list;
    enum class Type { rectangular, circular };

    Type type = Type::rectangular;
    math::Point3 center;
    roof::Type instance;
};

struct Building {
    typedef std::vector<Building> list;

    math::Point3 origin;
    Roof::list roofs;
};

struct World {
    geo::SrsDefinition srs;
    bool adjustVertical = false;

    math::Point3 origin;

    Building::list buildings;
};

UTILITY_GENERATE_ENUM_IO(Roof::Type,
                      ((rectangular))
                      ((circular))
                      )

} // namespace semantic

#endif // semantic_world_hpp_included_
