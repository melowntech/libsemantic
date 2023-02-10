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
#include <map>

#include "dbglog/dbglog.hpp"

#include "math/geometry.hpp"
#include "math/math.hpp"
#include "math/transform.hpp"

#include "../mesh.hpp"
#include "detail.hpp"
#include <math.h>

namespace semantic
{
namespace lod2
{
using detail::Index;
using uint32 = std::uint32_t;

namespace
{
namespace ublas = boost::numeric::ublas;

inline math::Matrix4 rotZ(double angle)
{
    math::Matrix4 r { math::identity4() };
    double c = std::cos(angle);
    r(0, 0) = c;
    r(1, 1) = c;
    double s = std::sin(angle);
    r(1, 0) = s;
    r(0, 1) = -s;
    return r;
}

inline math::Matrix4 rotY(double angle)
{
    math::Matrix4 r { math::identity4() };
    double c = std::cos(angle);
    r(0, 0) = c;
    r(2, 2) = c;
    double s = std::sin(angle);
    r(0, 2) = s;
    r(2, 0) = -s;
    return r;
}

inline math::Matrix4 rotAlignZwithNormal(const math::Point3& normal)
{
    // decompose normal to euler angles: z1-y-z2 (z1 = -z2)
    auto normtf { normal };

    // rotate in z (~ set azimuth)
    auto z1 { -std::atan2(normtf(1), normtf(0)) };
    auto rmatZ1 { rotZ(z1) };
    normtf = math::transform(rmatZ1, normtf);

    // rotate in y (~ set elevation)
    auto rmatY { rotY(-M_PI_2 + std::atan2(normtf(2), normtf(0))) };
    math::Matrix4 tf { ublas::prod(rmatY, rmatZ1) };
    return ublas::prod(rotZ(-z1), tf); // rotate back in z
}

inline math::Matrix4 global2FeatureCrs(const math::Point3& normal,
                                       const double angle,
                                       const math::Point3& origin)
{
    auto tf { rotAlignZwithNormal(normal) };
    tf = ublas::prod(rotZ(-angle), tf);
    return ublas::prod(tf, math::translate(math::Point3(-1.0 * origin)));
}

geometry::Mesh rectangleMesh(const math::Size2f& sz, const Material& material)
{
    const auto w(sz.width);
    const auto h(sz.height);
    geometry::Mesh mesh;
    mesh.vertices.emplace_back(w / 2.0, h / 2.0, 0.0);
    mesh.vertices.emplace_back(-w / 2.0, h / 2.0, 0.0);
    mesh.vertices.emplace_back(-w / 2.0, -h / 2.0, 0.0);
    mesh.vertices.emplace_back(w / 2.0, -h / 2.0, 0.0);
    mesh.faces.emplace_back(0, 1, 2);
    mesh.faces.emplace_back(0, 2, 3);

    // colorize
    for (auto& f : mesh.faces)
    {
        f.imageId = +material;
    }
    return mesh;
}

void tfToGlobal(geometry::Mesh& mesh,
                const math::Point3& normal,
                const double angle,
                const math::Point3& origin)
{
    math::transform(
        math::matrixInvert(global2FeatureCrs(normal, angle, origin)),
        mesh.vertices);
}

} // namespace

geometry::Mesh mesh(const PedestrianCrossing& pedestrianCrossing,
                    const MeshConfig& config,
                    const math::Point3& origin)
{
    math::Point3 pedestrianCrossingOrigin(pedestrianCrossing.origin);
    if (!config.worldCrs) { pedestrianCrossingOrigin += origin; }

    auto mesh { rectangleMesh(pedestrianCrossing.size,
                              Material::pedestrianCrossing) };
    tfToGlobal(mesh,
               pedestrianCrossing.normal,
               pedestrianCrossing.angle,
               pedestrianCrossingOrigin);

    return mesh;
}


geometry::Mesh mesh(const RoadArrow& arrow,
                    const MeshConfig& config,
                    const math::Point3& origin)
{
    math::Point3 arrowOrigin(arrow.origin);
    if (!config.worldCrs) { arrowOrigin += origin; }

    auto mesh { rectangleMesh(arrow.size, Material::roadArrow) };
    tfToGlobal(mesh, arrow.normal, arrow.angle, arrowOrigin);

    return mesh;
}

} // namespace lod2

} // namespace semantic
