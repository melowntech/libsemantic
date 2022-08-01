/**
 * Copyright (c) 2022 Melown Technologies SE
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
#include "math/transform.hpp"

#include "../mesh.hpp"
#include "detail.hpp"

namespace semantic {

namespace lod2 {

using detail::Index;

namespace {

std::tuple<math::Point3, math::Point3, math::Point3>
orthogonalVectors(const math::Point3 &n)
{
    std::tuple<math::Point3, math::Point3, math::Point3> res;
    auto &u(std::get<0>(res));
    auto &v(std::get<1>(res));
    auto &z(std::get<2>(res));
    z = math::normalize(n);

    if (n(0) == 0.0) {
        u = math::normalize(math::Point3(0.0, z(2), -n(1)));
    } else {
        u = math::normalize(math::Point3(-n(1), n(0), 0.0));
    }

    v = math::crossProduct(z, u);

    return res;
}

math::Matrix4 makeTrafo(const math::Point3 &n)
{
    namespace ublas = boost::numeric::ublas;

    math::Matrix4 trafo(ublas::identity_matrix<double>(4));

    auto e1_(ublas::column(trafo, 0));
    auto e2_(ublas::column(trafo, 1));
    auto e3_(ublas::column(trafo, 2));

    auto e1(ublas::subrange(e1_, 0, 3));
    auto e2(ublas::subrange(e2_, 0, 3));
    auto e3(ublas::subrange(e3_, 0, 3));

    std::tie(e1, e2, e3) = orthogonalVectors(n);

    return trafo;
}

void meshPole(geometry::Mesh &out, const Pole &pole
              , const MeshConfig &config, const math::Point3 &origin
              , Material material)
{
    geometry::Mesh mesh;

    const auto arcPoints(detail::computeArcPoints(config, pole.radius));

    auto trafo(makeTrafo(pole.direction));

    const auto &vertexPoint([&](const math::Point3 &p)
    {
        mesh.vertices.push_back
            (math::transform(trafo, p) + origin);
    });

    const auto &vertex([&](double x, double y, double z)
    {
        vertexPoint({x, y, z});
    });

    const auto &face([&](Index a, Index b, Index c)
    {
        mesh.faces.emplace_back(a, b, c, 0, 0, 0, +material);
    });

    const auto arcVertices(arcPoints * 2);

    // top cap is added always, bottom cap is added only when
    // config.closedSurface is true
    Index topCenter(0);
    Index bottomCenter(0);
    Index perimeterOffset(1);
    vertex(0, 0, pole.length);
    if (config.closedSurface) {
        bottomCenter = 1;
        perimeterOffset += 1;
        vertex(0, 0, pole.distanceToGround);
    }

    for (Index i(0); i < arcPoints; ++i) {
        const double angle((2 * M_PI * i) / arcPoints);

        const auto p(detail::rotate(0, pole.radius, pole.length, angle));
        vertexPoint(p);
        vertex(p(0), p(1), pole.distanceToGround);

        const auto &v([&](Index index) -> Index
        {
            return perimeterOffset + (index + 2 * i) % arcVertices;
        });

        // top side, always capped
        face(topCenter, v(0), v(2));
        face(v(0), v(3), v(2));

        // bottom side, capped only when asked for
        face(v(0), v(1), v(3));
        if (config.closedSurface) { face(v(1), bottomCenter, v(3)); }
    }

    detail::append(out, mesh);
}

} // namespace

geometry::Mesh mesh(const Pole &pole, const MeshConfig &config
                    , const math::Point3 &origin)
{
    geometry::Mesh m;

    // add tree trunk
    math::Point3 poleOrigin(pole.origin);
    if (!config.worldCrs) { poleOrigin += origin; }
    meshPole(m, pole, config, poleOrigin, Material::pole);

    return m;
}

} // namespace lod2

} // namespace semantic
