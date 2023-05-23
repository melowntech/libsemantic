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

#ifndef semantic_mesh_detail_hpp_included_
#define semantic_mesh_detail_hpp_included_

#include <cmath>

#include "math/geometry_core.hpp"

namespace semantic { namespace detail {

typedef geometry::Face::index_type Index;

/** Append new mesh.
 */
inline void append(geometry::Mesh &mesh, const geometry::Mesh &add)
{
    Index vo(mesh.vertices.size());
    for (const auto &v : add.vertices) {
        mesh.vertices.push_back(v);
    }

    Index to(mesh.tCoords.size());
    mesh.tCoords.insert(mesh.tCoords.end(), add.tCoords.begin()
                       , add.tCoords.end());

    for (const auto &f : add.faces) {
        mesh.faces.emplace_back(f.a + vo, f.b + vo, f.c + vo
                                , f.ta + to, f.tb + to, f.tc + to
                                , f.imageId);
    }
}

inline math::Point3 rotate(double x, double y, double z, double angle)
{
    auto sa(std::sin(angle));
    auto ca(std::cos(angle));

    return { ca * x - sa * y, sa * x + ca * y, z };
}

inline math::Point3 rotate(const math::Point3 &p, double angle)
{
    return rotate(p(0), p(1), p(2), angle);
}

/** Check if segments between points (0, a), (t, b) and (1, c) are colinear
 */
inline bool colinear(double a, double b, double c, double t)
{
    // Interpolate expected height on line from a to c
    const auto expected((1 - t) * a + t * c);

    // Compare with provided height b
    return (std::abs(expected - b) < 1e-5);
}

inline Index computeArcPoints(const MeshConfig &config, double radius)
{
    return std::max(Index(config.minSegmentCount)
                    , Index(std::ceil
                            (M_PI / std::asin(config.maxCircleSegment
                                              / (2 * radius)))));
}

} } // namespace semantic::detail

#endif // semantic_mesh_detail_hpp_included_
