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
#include "math/math.hpp"

namespace ublas = boost::numeric::ublas;

namespace semantic
{
namespace lod2
{
using detail::Index;
using uint32 = std::uint32_t;

namespace
{
geometry::Mesh newMeshIcosahedronRaw()
{
    geometry::Mesh mesh;

    // https://www.danielsieger.com/blog/2021/01/03/generating-platonic-solids.html

    const double phi = (1.0 + std::sqrt(5.0)) * 0.5; // golden ratio
    const double a = 1.0;
    const double b = 1.0 / phi;

    mesh.vertices.emplace_back(math::normalize(math::Point3(0, +b, -a)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(+b, +a, 0)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(-b, +a, 0)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(0, +b, +a)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(0, -b, +a)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(-a, 0, +b)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(0, -b, -a)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(+a, 0, -b)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(+a, 0, +b)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(-a, 0, -b)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(+b, -a, 0)));
    mesh.vertices.emplace_back(math::normalize(math::Point3(-b, -a, 0)));

    mesh.faces.emplace_back(2, 1, 0);
    mesh.faces.emplace_back(1, 2, 3);
    mesh.faces.emplace_back(5, 4, 3);
    mesh.faces.emplace_back(4, 8, 3);
    mesh.faces.emplace_back(7, 6, 0);
    mesh.faces.emplace_back(6, 9, 0);
    mesh.faces.emplace_back(11, 10, 4);
    mesh.faces.emplace_back(10, 11, 6);
    mesh.faces.emplace_back(9, 5, 2);
    mesh.faces.emplace_back(5, 9, 11);
    mesh.faces.emplace_back(8, 7, 1);
    mesh.faces.emplace_back(7, 8, 10);
    mesh.faces.emplace_back(2, 5, 3);
    mesh.faces.emplace_back(8, 1, 3);
    mesh.faces.emplace_back(9, 2, 0);
    mesh.faces.emplace_back(1, 7, 0);
    mesh.faces.emplace_back(11, 9, 6);
    mesh.faces.emplace_back(7, 10, 6);
    mesh.faces.emplace_back(5, 11, 4);
    mesh.faces.emplace_back(10, 8, 4);

    return mesh;
}

geometry::Mesh newMeshSphereRegular(uint32 subdivisions)
{
    geometry::Mesh mesh = newMeshIcosahedronRaw();

    for (uint32 iter = 0; iter < subdivisions; iter++)
    {
        // subdivide the mesh
        geometry::Mesh tmp;
        tmp.vertices = mesh.vertices;
        struct Hasher
        {
            std::size_t operator()(const std::pair<uint32, uint32>& p) const
            {
                const auto h = std::hash<uint32>();
                return (h(p.first) ^ p.second) + h(p.second);
            }
        };
        std::unordered_map<std::pair<uint32, uint32>, uint32, Hasher> mapping;
        mapping.reserve(mesh.vertices.size() * 2);
        const auto& split = [&](uint32 a, uint32 b) -> uint32 {
            uint32& r = mapping[std::pair<uint32, uint32>(std::min(a, b),
                                                          std::max(a, b))];
            if (r == 0)
            {
                r = tmp.vertices.size();
                tmp.vertices.emplace_back(
                    math::normalize(mesh.vertices[a] + mesh.vertices[b]));
            }
            return r;
        };
        const uint32 originalFacesCount = mesh.faces.size();
        for (uint32 i = 0; i < originalFacesCount; i++)
        {
            const uint32 a = mesh.faces[i].vertex(0);
            const uint32 b = mesh.faces[i].vertex(1);
            const uint32 c = mesh.faces[i].vertex(2);
            const uint32 d = split(a, b);
            const uint32 e = split(b, c);
            const uint32 f = split(c, a);
            tmp.faces.emplace_back(a, d, f);
            tmp.faces.emplace_back(d, e, f);
            tmp.faces.emplace_back(b, e, d);
            tmp.faces.emplace_back(c, f, e);
        }
        std::swap(mesh, tmp);
    }

    return mesh;
}

void meshLamp(geometry::Mesh& out,
              const Lamp& lamp,
              const math::Point3& origin,
              Material material)
{
    // reusable base mesh
    static const auto meshBase(newMeshSphereRegular(2));

    geometry::Mesh m(meshBase);

    math::Point3 offset(origin);

    math::Matrix4 transformationMatrix(4,4);

    // fetch all 4 columns of transformation matrix
    auto e1_(ublas::row( transformationMatrix, 0 ));
    auto e2_(ublas::row( transformationMatrix, 1 ));
    auto e3_(ublas::row( transformationMatrix, 2 ));
    auto e4_(ublas::column( transformationMatrix, 3 ));

    auto e1(ublas::subrange(e1_, 0, 3));
    auto e2(ublas::subrange(e2_, 0, 3));
    auto e3(ublas::subrange(e3_, 0, 3));
    auto e4(ublas::subrange(e4_, 0, 3));

    // reset last row to (0, 0, 0, 1)
    ublas::row(transformationMatrix, 3) = ublas::unit_vector<double>(4, 3);

    // compute the other elements 
    // use crossProduct to preserve orientation
    e1 = math::normalize(lamp.dimensions[0]);
    e2 = math::normalize(math::crossProduct(lamp.dimensions[2], e1));
    e3 = math::crossProduct( e1, e2);

    // scale according to original length
    e1 = math::length(lamp.dimensions[0]) * e1;
    e2 = math::length(lamp.dimensions[1]) * e2;
    e3 = math::length(lamp.dimensions[2]) * e3;

    // add shift
    e4 = offset;

    // transform
    for (auto& v : m.vertices)
    {
        v = math::transform(transformationMatrix, v);
    }

    // colorize
    for (auto& f : m.faces)
    {
        f.imageId = +material;
    }

    detail::append(out, m);
}

} // namespace

geometry::Mesh
    mesh(const Lamp& lamp, const MeshConfig& config, const math::Point3& origin)
{
    geometry::Mesh m;

    math::Point3 lampOrigin(lamp.origin);
    if (!config.worldCrs) { lampOrigin += origin; }
    meshLamp(m, lamp, lampOrigin, Material::lamp);

    return m;
}

} // namespace lod2

} // namespace semantic
