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

} // namespace

geometry::Mesh mesh(const Tree &tree, const MeshConfig &config
                    , const math::Point3 &origin)
{
    geometry::Mesh m;
    buildSphericalHarmonics(m, config, tree.harmonics
                            , treeCrownMaterial(tree));

    const math::Point3 offset(origin + tree.origin + tree.center);

    for (auto &v : m.vertices) {
        v(0) = v(0) * tree.a + offset(0);
        v(1) = v(1) * tree.a + offset(1);
        v(2) = v(2) * tree.b + offset(2);
    }

    return m;
}

} // namespace lod2

} // namespace semantic
