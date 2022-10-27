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

#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>

#include "dbglog/dbglog.hpp"

#include "shtools/shtools.hpp"

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
void meshManhole(geometry::Mesh& out,
                 const Manhole& manhole,
                 const math::Point3& origin,
                 Material material)
{
    geometry::Mesh mesh;

    for (const auto& point : manhole.boundingBox)
    {
        mesh.vertices.push_back(point + origin);
        mesh.vertices.emplace_back(math::Point3(point(0), point(1), -0.005)
                                   + origin);
    }

    mesh.faces.emplace_back(0, 2, 1);
    mesh.faces.emplace_back(3, 1, 2);
    mesh.faces.emplace_back(6, 0, 7);
    mesh.faces.emplace_back(1, 7, 0);
    mesh.faces.emplace_back(4, 6, 5);
    mesh.faces.emplace_back(7, 5, 6);
    mesh.faces.emplace_back(2, 4, 3);

    // colorize
    for (auto& f : mesh.faces)
    {
        f.imageId = +material;
    }

    detail::append(out, mesh);
}

} // namespace

geometry::Mesh mesh(const Manhole& manhole,
                    const MeshConfig& config,
                    const math::Point3& origin)
{
    geometry::Mesh m;

    math::Point3 manholeOrigin(manhole.origin);
    if (!config.worldCrs) { manholeOrigin += origin; }
    meshManhole(m, manhole, manholeOrigin, Material::manhole);

    return m;
}

} // namespace lod2

} // namespace semantic
