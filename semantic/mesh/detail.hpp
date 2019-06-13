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

#include <cmath>

#include "math/geometry_core.hpp"

namespace semantic { namespace detail {

typedef geometry::Face::index_type Index;

/** Append new mesh.
 */
inline void append(geometry::Mesh &mesh, const geometry::Mesh &add
            , const math::Point3 &origin)
{
    Index offset(mesh.vertices.size());
    for (const auto &v : add.vertices) {
        mesh.vertices.push_back(v + origin);
    }

    for (const auto &f : add.faces) {
        mesh.faces.emplace_back(f.a + offset, f.b + offset, f.c + offset);
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
    const auto expected(t * a + (1 - t) * c);

    // Compare with provided height b
    return (std::abs(expected - b) < 1e-5);
}

} } // namespace semantic::detail
