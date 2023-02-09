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
#include "math/transform.hpp"

#include "../mesh.hpp"
#include "detail.hpp"

namespace semantic
{
namespace lod2
{
using detail::Index;
using uint32 = std::uint32_t;

namespace
{
namespace ublas = boost::numeric::ublas;

void meshArrow(geometry::Mesh& out,
               const RoadArrow& arrow,
               const MeshConfig& /* config */,
               const math::Point3& origin,
               Material material)
{
    geometry::Mesh mesh;

    const auto w(arrow.size.width);
    const auto h(arrow.size.height);
    mesh.vertices.emplace_back(
        detail::rotate(math::Point3(w / 2, h / 2, 0), arrow.angle));
    mesh.vertices.emplace_back(
        detail::rotate(math::Point3(-w / 2, h / 2, 0), arrow.angle));
    mesh.vertices.emplace_back(
        detail::rotate(math::Point3(w / 2, -h / 2, 0), arrow.angle));
    mesh.vertices.emplace_back(
        detail::rotate(math::Point3(-w / 2, -h / 2, 0), arrow.angle));

    mesh.faces.emplace_back(0, 1, 2);
    mesh.faces.emplace_back(2, 1, 3);

    // colorize
    for (auto& f : mesh.faces)
    {
        f.imageId = +material;
    }

    math::Point3 zUV(0, 0, 1);
    math::Point3 n(arrow.normal);
    const double angle(std::asin(ublas::norm_2(math::crossProduct(zUV, n))));

    for (auto& v : mesh.vertices)
    {
        math::Point3 k(math::crossProduct(zUV, n) / angle);
        const double c(std::cos(angle));
        const double s(std::sin(angle));

        v = origin + (v * c) + (s * math::crossProduct(k, v))
            + (k * (ublas::inner_prod(k, v)) * (1 - c));
    }

    detail::append(out, mesh);
}

} // namespace

geometry::Mesh mesh(const RoadArrow& arrow,
                    const MeshConfig& config,
                    const math::Point3& origin)
{
    geometry::Mesh m;

    math::Point3 arrowOrigin(arrow.origin);
    if (!config.worldCrs) { arrowOrigin += origin; }
    meshArrow(m, arrow, config, arrowOrigin, Material::facade);

    return m;
}

} // namespace lod2

} // namespace semantic
