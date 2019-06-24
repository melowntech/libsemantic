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

#include "dbglog/dbglog.hpp"

#include "../mesh.hpp"

namespace semantic {

namespace detail {

typedef geometry::Face::index_type Index;

/** Append new mesh.
 */
void append(geometry::Mesh &mesh, const geometry::Mesh &add
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

geometry::Mesh lod2roof(const roof::Rectangular &roof)
{
    (void) roof;

    geometry::Mesh mesh;
    return mesh;
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

geometry::Mesh lod2roof(const roof::Circular &roof)
{
    geometry::Mesh mesh;

    /** TODO: derive from radius size
     */
    const Index arcPoints(16);

    // roof center
    mesh.vertices.emplace_back(0.0, 0.0, roof.ridgeHeight);

    const auto &colinear([&]() -> bool
    {
        /** Interpolate expected height on line from ridge to eave
         */
        const auto expected((1 - roof.curb) * roof.eaveHeight
                            + roof.curb * roof.ridgeHeight);
        return (std::abs(expected - roof.curbHeight) < 1e-5);
    });

    if (colinear()) {
        const auto arcVertices(arcPoints * 2);

        for (Index i(0); i < arcPoints; ++i) {
            const double angle((2 * M_PI * i) / arcPoints);

            const auto end(rotate(0, roof.radius, roof.eaveHeight, angle));
            // eave point
            mesh.vertices.push_back(end);
            // ground point
            mesh.vertices.emplace_back(end(0), end(1), 0.0);

            const auto &v([&](Index index) -> Index
            {
                return 1 + ((index + 2 * i) % arcVertices);
            });

            mesh.faces.emplace_back(0, v(0), v(2));
            mesh.faces.emplace_back(v(0), v(3), v(2));
            mesh.faces.emplace_back(v(0), v(1), v(3));
        }
    } else {
        const auto arcVertices(arcPoints * 3);

        for (Index i(0); i < arcPoints; ++i) {
            const double angle((2 * M_PI * i) / arcPoints);

            // curb point
            mesh.vertices.push_back
                (rotate(0, roof.radius * roof.curb, roof.curbHeight, angle));

            const auto end(rotate(0, roof.radius, roof.eaveHeight, angle));
            // eave point
            mesh.vertices.push_back(end);
            // ground point
            mesh.vertices.emplace_back(end(0), end(1), 0.0);

            const auto &v([&](Index index) -> Index
            {
                return 1 + ((index + 3 * i) % arcVertices);
            });

            mesh.faces.emplace_back(0, v(0), v(3));
            mesh.faces.emplace_back(v(0), v(1), v(4));
            mesh.faces.emplace_back(v(0), v(4), v(3));
            mesh.faces.emplace_back(v(1), v(2), v(5));
            mesh.faces.emplace_back(v(1), v(5), v(4));
        }
    }

    return mesh;
}

geometry::Mesh lod2roof(const Roof &roof)
{
    struct Visitor : public boost::static_visitor<geometry::Mesh> {
        geometry::Mesh operator()(const roof::Rectangular &r) const {
            return lod2roof(r);
        }
        geometry::Mesh operator()(const roof::Circular &r) const {
            return lod2roof(r);
        }
    } v;
    return boost::apply_visitor(v, roof.instance);
}

geometry::Mesh lod2building(const Building &building)
{
    geometry::Mesh mesh;
    for (const auto &roof : building.roofs) {
        append(mesh, lod2roof(roof), building.origin + roof.center);
    }
    return mesh;
}

geometry::Mesh lod2(const World &world)
{
    geometry::Mesh mesh;
    for (const auto &building : world.buildings) {
        append(mesh, lod2building(building), world.origin);
    }
    return mesh;
}

} // namespace detail

/** Generate mesh in given LOD.
 */
geometry::Mesh mesh(const World &world, int lod)
{
    switch (lod) {
    case 2:
        return detail::lod2(world);

    case 0:
    case 1:
    case 3:
    case 4:
        LOGTHROW(err3, std::runtime_error)
            << "Generation of mesh in LOD " << lod << " not supported.";
        break;

    default:
        LOGTHROW(err3, std::runtime_error)
            << "Unknown LOD " << lod << ".";
        break;
    }
    throw;
}

} // namespace semantic
