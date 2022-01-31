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

#include "shtools/shtools.hpp"

#include "../mesh.hpp"
#include "detail.hpp"

namespace semantic {

namespace lod2 {

using detail::Index;

namespace {

void buildSphericalHarmonics(geometry::Mesh &mesh, const MeshConfig &config
                             , const std::vector<double> &harmonics
                             , Material material)
{
    const auto grid(shtools::makeGridDH
                    (harmonics, shtools::Sampling::equallySpaced));

    const auto rows(grid.height());
    const auto cols(grid.width());

    mesh.vertices.emplace_back(0, 0, grid(0, 0));

    for (int row(1); row < rows; ++row) {
        const double theta((M_PI * row) / rows);
        for (int col(0); col < cols; ++col) {
            const double phi((2 * M_PI * col) / cols);
            const auto r(grid(col, row));
            mesh.vertices.emplace_back
                (r * std::sin(theta) * std::cos(phi)
                 , r * std::sin(theta) * std::sin(phi)
                 , r * std::cos(theta));
        }
    }

    const auto &addFace([&](int a, int b, int c)
    {
        mesh.faces.emplace_back(a, b, c, 0, 0, 0, +material);
    });

    for (int col(0); col < cols; ++col) {
        addFace(0, col + 1, (col + 1) % cols + 1);
    }

    for (int row(2); row < rows; ++row) {
        const auto startRow((row - 1) * cols + 1);
        for (int col(0); col < cols; ++col) {
            const auto i(col + startRow);
            const auto next = (col + 1) % cols + startRow;

            addFace(i, next - cols, i - cols);
            addFace(i, next, next - cols);
        }
    }

    (void) config;
}

Material treeCrownMaterial(const Tree &tree)
{
    return ((tree.type == Tree::Type::deciduous)
            ? Material::tree_crown_deciduous
            : Material::tree_crown_coniferous);
}

Material treeTrunkMaterial(const Tree &tree)
{
    // if there is ever a need for different trunk material for different tree
    // types
    return ((tree.type == Tree::Type::deciduous)
            ? Material::tree_trunk
            : Material::tree_trunk);
}

void trunk(geometry::Mesh &out, const Tree &tree, const MeshConfig &config
           , const math::Point3 &origin, Material material)
{
    geometry::Mesh mesh;

    const auto height(tree.center[2] + tree.b);
    const auto r(std::sqrt(tree.a) / 6.0
                 + std::sqrt(height) / 8.0
                 - 0.25);
    const auto top(tree.center[2] - tree.b / 2.0);

    const auto arcPoints(detail::computeArcPoints(config, r));

    const auto &vertex([&](double x, double y, double z)
    {
        mesh.vertices.emplace_back
            (x + origin(0), y + origin(1), z + origin(2));
    });

    const auto &vertexPoint([&](const math::Point3 &p)
    {
        mesh.vertices.emplace_back
            (p(0) + origin(0), p(1) + origin(1), p(2) + origin(2));
    });

    const auto &face([&](Index a, Index b, Index c)
    {
        mesh.faces.emplace_back(a, b, c, 0, 0, 0, +material);
    });

    const auto arcVertices(arcPoints * 2);

    for (Index i(0); i < arcPoints; ++i) {
        const double angle((2 * M_PI * i) / arcPoints);

        const auto p(detail::rotate(0, r, top, angle));
        vertexPoint(p);
        vertex(p(0), p(1), 0.0);

        const auto &v([&](Index index) -> Index
        {
            return (index + 2 * i) % arcVertices;
        });

        face(v(0), v(1), v(3));
        face(v(0), v(3), v(2));
    }

    detail::append(out, mesh);
}

} // namespace

geometry::Mesh mesh(const Tree &tree, const MeshConfig &config
                    , const math::Point3 &origin)
{
    geometry::Mesh m;
    buildSphericalHarmonics(m, config, tree.harmonics
                            , treeCrownMaterial(tree));

    math::Point3 offset(tree.origin + tree.center);
    // shift to world origin if not local crs
    if (!config.worldCrs) { offset += origin; }
    for (auto &v : m.vertices) {
        v(0) = v(0) * tree.a + offset(0);
        v(1) = v(1) * tree.a + offset(1);
        v(2) = v(2) * tree.b + offset(2);
    }

    // add tree trunk
    math::Point3 trunkOrigin(tree.origin);
    if (!config.worldCrs) { trunkOrigin += origin; }
    trunk(m, tree, config, trunkOrigin, treeTrunkMaterial(tree));

    return m;
}

} // namespace lod2

} // namespace semantic
