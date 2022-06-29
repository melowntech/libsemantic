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

#include "../ogr.hpp"
#include "tree.hpp"

namespace semantic {

namespace {

std::unique_ptr< ::OGRGeometry>
makeCircle(const math::Point3 &center, double radius)
{
    auto cs(std::make_unique< ::OGRCircularString>());

    /** Closed circle between two arcs.
     */
    cs->addPoint(center(0) - radius, center(1), center(2));
    cs->addPoint(center(0) + radius, center(1), center(2));
    cs->addPoint(center(0) - radius, center(1), center(2));

    return cs;
}

OgrGeometry ogr(const tree::Aerial &t, const math::Point3 &origin)
{
    const math::Point3 center(origin + t.center);

    auto cs(makeCircle(center, t.a));

    math::Extent verticalExtent;
    update(verticalExtent, origin(2));
    update(verticalExtent, center(2) + t.b);
    update(verticalExtent, center(2) - t.b);

    return { std::move(cs), verticalExtent };
}

OgrGeometry ogr(const tree::GroundLevel &t, const math::Point3 &origin)
{
    // TODO: add trunk as well

    const math::Point3 center(origin + t.crown.center);
    auto cs(makeCircle(center, t.crown.radius));

    math::Extent verticalExtent;
    update(verticalExtent, origin(2));
    update(verticalExtent, center(2) + t.height);

    return { std::move(cs), verticalExtent };
}

} // namespace

OgrGeometry ogr(const Tree &tree, const math::Point3 &origin)
{
    struct Visitor : public boost::static_visitor<OgrGeometry> {
        const math::Point3 &origin;
        Visitor(const math::Point3 &origin) : origin(origin) {}

        OgrGeometry operator()(const tree::Aerial &t) const {
            return ogr(t, origin);
        }
        OgrGeometry operator()(const tree::GroundLevel &t) const {
            return ogr(t, origin);
        }
    } v(origin + tree.origin);

    return boost::apply_visitor(v, tree.instance);
}

} // namespace semantic
