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

#include "dbglog/dbglog.hpp"

#include "../../ogr.hpp"
#include "../roof.hpp"

namespace semantic {

bool ogr(::OGRGeometryCollection &collection, math::Extent &verticalExtent
         , const roof::Rectangular &r, const math::Point3 &origin)
{
    using Key = roof::Rectangular::Key;

    auto curbLeft(r.curb[+Key::left]);
    auto curbRight(r.curb[+Key::right]);

    const auto curbMiddle((curbLeft - curbRight) / 2.0);

    /** Skew tangens scaled to 1:1 space
     */
    const auto skewTopTan(std::tan(r.skew[+Key::top])
                          * r.size.width / r.size.height
                          );
    const auto skewBottomTan(std::tan(r.skew[+Key::bottom])
                             * r.size.width / r.size.height
                             );
    const auto sa(std::sin(r.azimuth));
    const auto ca(std::cos(r.azimuth));
    const math::Size2f hsize(r.size.width / 2.0, r.size.height / 2.0);

    ::OGRLineString ls;

    const auto &vertex([&](double x, double y) -> math::Point3
    {
        const auto x_(x * hsize.width);
        const auto y_(y * hsize.height);
        return { ca * x_ + sa * y_ + origin(0)
                , -sa * x_ + ca * y_ + origin(1)
                , origin(2) };
    });

    const auto addVertex([&](const math::Point3 &v)
    {
        ls.addPoint(v(0), v(1), v(2));
        return v;
    });

    const auto add([&](double x, double y)
    {
        return addVertex(vertex(x, y));
    });

    // top-left
    const auto start(add(-1.0, 1.0 - skewTopTan * (1.0 - curbMiddle)));
    // top-right
    add(1.0, 1.0 + skewTopTan * (1.0 + curbMiddle));

    // bottom-right
    add(1.0 , -1.0 + skewBottomTan * (1.0 + curbMiddle));

    // bottom-left
    add(-1.0, -1.0 - skewBottomTan * (1.0 - curbMiddle));

    // close ring
    addVertex(start);

    if (OGRERR_NONE != collection.addGeometry(&ls)) { return false; }

    // vertical extent
    update(verticalExtent, origin(2));
    update(verticalExtent, origin(2) + r.ridgeHeight);
    for (auto height : r.eaveHeight) {
        update(verticalExtent, origin(2) + height);
    }

    return true;
}

} // namespace semantic
