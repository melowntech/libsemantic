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

#include "../../mesh.hpp"
#include "../roof.hpp"
#include "../detail.hpp"

namespace semantic {

namespace lod2 {

using detail::Index;

class Composer {
public:
    Composer(geometry::Mesh &mesh)
        : mesh_(mesh), last_(nullptr)
    {}

    /** Returns index to p.
     *
     *  Updates last point to point to the retuned point.
     */
    Index point(const math::Point3 &p) {
        auto fcache(cache_.find(p));
        if (fcache == cache_.end()) {
            fcache = cache_.emplace(p, mesh_.vertices.size()).first;
            mesh_.vertices.push_back(p);
        }
        last_ = &fcache->first;
        return fcache->second;
    }

    /** Returns index to (last point + p).
     *
     *  Updates last point to point to the retuned point.
     *
     *  NB: last point must be defined!
     */
    Index updated(const math::Point3 &p) {
        return point(*last_ + p);
    }

    /** Same as point(Point3(x, y, z))
     */
    Index point(double x = .0, double y = .0, double z = .0) {
        return point({x, y, z});
    }

    /** Same as updated(Point3(x, y, z))
     */
    Index updated(double x = .0, double y = .0, double z = .0) {
        return updated({x, y, z});
    }

    /** Adds new face if triangle is not degenerate
     */
    void face(Index a, Index b, Index c, Material material) {
        if ((a != b) && (b != c) && (c != a)) {
            mesh_.faces.emplace_back(a, b, c, 0, 0, 0, +material);
        }
    }

    /** Adds face with indirect vertex indices.
     */
    template <typename Mapping>
    void face(const Mapping &mapping, Index a, Index b, Index c
              , Material material)
    {
        face(mapping[a], mapping[b], mapping[c], material);
    }

    /** Apply final scale and rotation
     */
    void transform(const roof::Rectangular &roof
                   , const math::Point3 &origin)
    {
        const auto sa(std::sin(roof.azimuth));
        const auto ca(std::cos(roof.azimuth));
        const math::Size2f hsize
            (roof.size.width / 2.0, roof.size.height / 2.0);
        for (auto &v : mesh_.vertices) {
            const auto x(v(0) * hsize.width);
            const auto y(v(1) * hsize.height);
            v(0) = ca * x + sa * y + origin(0);
            v(1) = -sa * x + ca * y + origin(1);
            v(2) += origin(2);
        }
    }

private:
    geometry::Mesh &mesh_;

    typedef std::map<math::Point3, Index> PointCache;
    PointCache cache_;
    const math::Point3 *last_;
};

/** Helper for constructing indirect faces whose vertices are not born yet.
 */
class DeferredFaces {
public:
    struct Face {
        Index a;
        Index b;
        Index c;
        Material material;

        Face(Index a, Index b, Index c, Material material)
            : a(a), b(b), c(c), material(material)
        {}
        typedef std::vector<Face> list;
    };

    /** Add face.
     */
    void face(Index a, Index b, Index c, Material material) {
        faces_.emplace_back(a, b, c, material);
    }

    /** Map indirect vertices to real vertices and add faces to mech composer.
     */
    template <typename Mapping>
    void expand(Composer &c, const Mapping &mapping)
    {
        for (const auto &face : faces_) {
            c.face(mapping, face.a, face.b, face.c, face.material);
        }
    }

private:
    Face::list faces_;
};

geometry::Mesh mesh(const roof::Rectangular &r, const MeshConfig &config
                    , const math::Point3 &origin)
{
    using Key = roof::Rectangular::Key;
    geometry::Mesh m;

    Composer c(m);

    auto curbLeft(r.curb[+Key::left]);
    auto curbRight(r.curb[+Key::right]);

    auto curbTop(r.curb[+Key::top]);
    auto curbBottom(r.curb[+Key::bottom]);

    const auto curbMiddle((curbLeft - curbRight) / 2.0);

    const auto &eaveHeightTop(r.eaveHeight[+Key::top]);
    const auto &eaveHeightBottom(r.eaveHeight[+Key::bottom]);
    const auto &eaveHeightLeft(r.eaveHeight[+Key::left]);
    const auto &eaveHeightRight(r.eaveHeight[+Key::right]);

    /** Skew tangens scaled to 1:1 space
     */
    const auto skewTopTan(std::tan(r.skew[+Key::top])
                          * r.size.width / r.size.height
                          );
    const auto skewBottomTan(std::tan(r.skew[+Key::bottom])
                             * r.size.width / r.size.height
                             );

    std::vector<Index> v(18);

    // *** Facade **
    // top-left
    v[0] = c.point(-1.0, 1.0 - skewTopTan * (1.0 - curbMiddle), 0.0);
    v[4] = c.updated(0.0, 0.0, std::min(eaveHeightLeft, eaveHeightTop));

    // top-right
    v[1] = c.point(1.0, 1.0 + skewTopTan * (1.0 + curbMiddle), 0.0);
    v[5] = c.updated(0.0, 0.0, std::min(eaveHeightRight, eaveHeightTop));

    // bottom-left
    v[2] = c.point(-1.0, -1.0 - skewBottomTan * (1.0 - curbMiddle), 0.0);
    v[6] = c.updated(0.0, 0.0, std::min(eaveHeightLeft, eaveHeightBottom));

    // bottom-right
    v[3] = c.point(1.0 , -1.0 + skewBottomTan * (1.0 + curbMiddle), 0.0);
    v[7] = c.updated(0.0, 0.0, std::min(eaveHeightRight, eaveHeightBottom));

    c.face(v, 0, 2, 4, Material::facade);
    c.face(v, 4, 2, 6, Material::facade);
    c.face(v, 1, 0, 5, Material::facade);
    c.face(v, 5, 0, 4, Material::facade);
    c.face(v, 3, 1, 7, Material::facade);
    c.face(v, 7, 1, 5, Material::facade);
    c.face(v, 2, 3, 6, Material::facade);
    c.face(v, 6, 3, 7, Material::facade);

    DeferredFaces deferred;

    // roof
    {
        const auto eaveHeight(std::max(eaveHeightLeft, eaveHeightTop));

        auto useCurbTop(curbTop);
        auto useCurbLeft(curbLeft);

        if (r.curbHeight <= eaveHeightLeft) {
            useCurbLeft = 1.0;
        } else if (r.curbHeight <= eaveHeightTop) {
            useCurbTop = 1.0;
        } else {
            useCurbLeft = std::min((eaveHeightTop - eaveHeightLeft) /
                                   (r.curbHeight - eaveHeightLeft)
                                   * (curbLeft - 1.0) + 1.0
                                   , 1.0);

            useCurbTop = std::min((eaveHeightLeft - eaveHeightTop) /
                               (r.curbHeight - eaveHeightTop)
                               * (curbTop - 1.0) + 1.0
                               , 1.0);
        }

        v[8] = c.point(-useCurbLeft
                       , useCurbTop - skewTopTan * (useCurbLeft - curbMiddle)
                       , eaveHeight);

        if (useCurbTop == 1.0) {
            deferred.face(5, 8, 9, Material::roof);
            deferred.face(5, 4, 8, Material::roof);
        } else {
            deferred.face(5, 4, 9, Material::roof);
            deferred.face(9, 4, 8, Material::roof);
        }
    }

    {
        const auto eaveHeight(std::max(eaveHeightRight, eaveHeightTop));

        auto useCurbTop(curbTop);
        auto useCurbRight(curbRight);

        if (r.curbHeight <= eaveHeightRight) {
            useCurbRight = 1.0;
        } else if (r.curbHeight <= eaveHeightTop) {
            useCurbTop = 1.0;
        } else {
            useCurbRight = std::min((eaveHeightTop - eaveHeightRight) /
                                    (r.curbHeight - eaveHeightRight)
                                    * (curbRight - 1.0) + 1.0
                                    , 1.0);

            useCurbTop = std::min((eaveHeightRight - eaveHeightTop) /
                                  (r.curbHeight - eaveHeightTop)
                                  * (curbTop - 1.0) + 1.0
                                  , 1.0);
        }

        v[9] = c.point(useCurbRight
                       , useCurbTop + skewTopTan * (useCurbRight + curbMiddle)
                       , eaveHeight);

        if (useCurbRight == 1.0) {
            deferred.face(7, 9, 11, Material::roof);
            deferred.face(7, 5, 9, Material::roof);
        } else {
            deferred.face(7, 5, 11, Material::roof);
            deferred.face(11, 5, 9, Material::roof);
        }
    }

    {
        const auto eaveHeight(std::max(eaveHeightLeft, eaveHeightBottom));

        auto useCurbBottom(curbBottom);
        auto useCurbLeft(curbLeft);

        if (r.curbHeight <= eaveHeightLeft) {
            useCurbLeft = 1.0;
        } else if (r.curbHeight <= eaveHeightBottom) {
            useCurbBottom = 1.0;
        } else {
            useCurbLeft = std::min((eaveHeightBottom - eaveHeightLeft) /
                                   (r.curbHeight - eaveHeightLeft)
                                   * (curbLeft - 1.0) + 1.0
                                   , 1.0);

            useCurbBottom = std::min((eaveHeightLeft - eaveHeightBottom) /
                               (r.curbHeight - eaveHeightBottom)
                               * (curbBottom - 1.0) + 1.0
                               , 1.0);
        }

        v[10] = c.point
            (-useCurbLeft
             , -useCurbBottom - skewBottomTan * (useCurbLeft - curbMiddle)
             , eaveHeight);

        if (useCurbBottom == 1.0) {
            deferred.face(4, 10, 8, Material::roof);
            deferred.face(4, 6, 10, Material::roof);
        } else {
            deferred.face(4, 6, 8, Material::roof);
            deferred.face(8, 6, 10, Material::roof);
        }
    }

    {
        const auto eaveHeight(std::max(eaveHeightRight, eaveHeightBottom));

        auto useCurbBottom(curbBottom);
        auto useCurbRight(curbRight);

        if (r.curbHeight <= eaveHeightRight) {
            useCurbRight = 1.0;
        } else if (r.curbHeight <= eaveHeightBottom) {
            useCurbBottom = 1.0;
        } else {
            useCurbRight = std::min((eaveHeightBottom - eaveHeightRight) /
                                    (r.curbHeight - eaveHeightRight)
                                    * (curbRight - 1.0) + 1.0
                                    , 1.0);

            useCurbBottom = std::min((eaveHeightRight - eaveHeightBottom) /
                                  (r.curbHeight - eaveHeightBottom)
                                  * (curbBottom - 1.0) + 1.0
                                  , 1.0);
        }

         v[11] = c.point
            (useCurbRight
             , -useCurbBottom + skewBottomTan * (useCurbRight + curbMiddle)
             , eaveHeight);

        if (useCurbRight == 1.0) {
            deferred.face(6, 11, 10, Material::roof);
            deferred.face(6, 7, 11, Material::roof);
        } else {
            deferred.face(6, 7, 10, Material::roof);
            deferred.face(10, 7, 11, Material::roof);
        }
    }

    deferred.expand(c, v);

    v[12] = c.point(-curbLeft
                    , curbTop - skewTopTan * (curbLeft - curbMiddle)
                    , r.curbHeight);

    v[13] = c.point(curbRight
                    , curbTop + skewTopTan * (curbRight + curbMiddle)
                    , r.curbHeight);

    v[14] = c.point(-curbMiddle, curbTop, r.ridgeHeight);

    v[15] = c.point(-curbLeft
                    , -curbBottom - skewBottomTan * (curbLeft - curbMiddle)
                    , r.curbHeight);

    v[16] = c.point(curbRight
                    , -curbBottom + skewBottomTan * (curbRight + curbMiddle)
                    , r.curbHeight);

    v[17] = c.point(-curbMiddle, -curbBottom, r.ridgeHeight);

    c.face(v, 8, 10, 12, Material::roof);
    c.face(v, 12, 10, 15, Material::roof);
    c.face(v, 9, 8, 13, Material::roof);
    c.face(v, 13, 8, 12, Material::roof);
    c.face(v, 11, 9, 16, Material::roof);
    c.face(v, 16, 9, 13, Material::roof);
    c.face(v, 10, 11, 15, Material::roof);
    c.face(v, 15, 11, 16, Material::roof);
    c.face(v, 12, 15, 14, Material::roof);
    c.face(v, 14, 15, 17, Material::roof);
    c.face(v, 14, 17, 13, Material::roof);
    c.face(v, 13, 17, 16, Material::roof);
    c.face(v, 12, 14, 13, Material::roof);
    c.face(v, 15, 16, 17, Material::roof);

    c.transform(r, origin);

    return m;

    (void) config;
}

} // namespace lod2

} // namespace semantic
