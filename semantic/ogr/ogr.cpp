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

#include "../ogr.hpp"

#include "building.hpp"

namespace semantic {

bool ogr(::OGRGeometryCollection &collection, math::Extent &verticalExtent
         , const roof::Roof &roof, const math::Point3 &origin)
{
    struct Visitor : public boost::static_visitor<bool> {
        ::OGRGeometryCollection &collection;
        math::Extent &verticalExtent;
        const math::Point3 &origin;
        Visitor(::OGRGeometryCollection &collection
                , math::Extent &verticalExtent
                , const math::Point3 &origin)
            : collection(collection), verticalExtent(verticalExtent)
            , origin(origin) {}

        bool operator()(const roof::Rectangular &r) const {
            return ogr(collection, verticalExtent, r, origin);
        }
        bool operator()(const roof::Circular &r) const {
            return ogr(collection, verticalExtent, r, origin);
        }
    } v(collection, verticalExtent, origin);
    return boost::apply_visitor(v, roof.instance);
}

OgrGeometry ogr(const Building &building, const math::Point3 &origin_)
{
    const auto origin(origin_ + building.origin);

    // any other roof count: make a collection
    auto collection(std::make_unique< ::OGRGeometryCollection>());
    math::Extent verticalExtent(math::InvalidExtents{});
    for (const auto &roof : building.roofs) {
        if (!ogr(*collection, verticalExtent, roof, origin + roof.center)) {
            LOGTHROW(err1, std::runtime_error)
                << "Unable to add geometry to OGRGeometryCollection.";
        }
    }
    return { std::move(collection), verticalExtent };

}

} // namespace semantic
