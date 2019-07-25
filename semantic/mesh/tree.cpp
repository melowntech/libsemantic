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

class SphereBuilder {
public:
    SphereBuilder(geometry::Mesh &mesh, const MeshConfig &config
                  , Material material)
        : mesh_(mesh), config_(config)
        , material_(material)
    {
        run(2);
    }

private:
    typedef std::pair<Index, Index> Key;
    typedef std::map<Key, Index> VertexCache;

    Index addVertex(const math::Point3 &v) {
        const auto l(boost::numeric::ublas::norm_2(v));
        const auto i(mesh_.vertices.size());
        mesh_.vertices.emplace_back(v(0) / l, v(1) / l, v(2) / l);
        return i;
    }

    void addFace(Index a, Index b, Index c) {
        mesh_.faces.emplace_back(a, b, c, 0, 0, 0, +material_);
    }

    Index getMidpoint(Index p1, Index p2) {
        Key key(p1, p2);
        {
            if (key.first > key.second) { std::swap(key.first, key.second); }
            auto fvcache(vcache_.find(key));
            if (fvcache != vcache_.end()) { return fvcache->second; }
        }

        const auto &v1(mesh_.vertices[p1]);
        const auto &v2(mesh_.vertices[p2]);

        const auto index(addVertex((v1 + v2) / 2.0));

        vcache_.emplace(key, index);

        return index;
    }

    void run(int recursion) {
        const double t((1.0 + std::sqrt(5.0)) / 2.0);

        addVertex({-1, t, 0});
        addVertex({ 1, t, 0});
        addVertex({-1, -t, 0});
        addVertex({ 1, -t, 0});

        addVertex({ 0, -1, t});
        addVertex({ 0, 1, t});
        addVertex({ 0, -1, -t});
        addVertex({ 0, 1, -t});

        addVertex({ t, 0, -1});
        addVertex({ t, 0, 1});
        addVertex({-t, 0, -1});
        addVertex({-t, 0, 1});

        // 5 faces around point 0
        addFace(0, 11, 5);
        addFace(0, 5, 1);
        addFace(0, 1, 7);
        addFace(0, 7, 10);
        addFace(0, 10, 11);

        // 5 adjacent faces
        addFace(1, 5, 9);
        addFace(5, 11, 4);
        addFace(11, 10, 2);
        addFace(10, 7, 6);
        addFace(7, 1, 8);

        // 5 faces around point 3
        addFace(3, 9, 4);
        addFace(3, 4, 2);
        addFace(3, 2, 6);
        addFace(3, 6, 8);
        addFace(3, 8, 9);

        // 5 adjacent faces
        addFace(4, 9, 5);
        addFace(2, 4, 11);
        addFace(6, 2, 10);
        addFace(8, 6, 7);
        addFace(9, 8, 1);

        while (recursion--) {
            geometry::Face::list faces;
            std::swap(mesh_.faces, faces);
            for (const auto &face : faces) {
                // replace triangle by 4 triangles
                const auto a(getMidpoint(face.a, face.b));
                const auto b(getMidpoint(face.b, face.c));
                const auto c(getMidpoint(face.c, face.a));

                addFace(face.a, a, c);
                addFace(face.b, b, a);
                addFace(face.c, c, b);
                addFace(a, b, c);
            }
        }
    }

    geometry::Mesh &mesh_;
    const MeshConfig &config_;
    Material material_;
    VertexCache vcache_;
};

void buildSphere(geometry::Mesh &mesh, const MeshConfig &config
                 , Material material)
{
    SphereBuilder(mesh, config, material);
}

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

} // namespace

geometry::Mesh mesh(const Tree &tree, const MeshConfig &config
                    , const math::Point3 &origin)
{
    geometry::Mesh m;
#if 0
    buildSphere(m, config, Material::treeCrown);
#else
    buildSphericalHarmonics(m, config, tree.harmonics, Material::treeCrown);
#endif

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
