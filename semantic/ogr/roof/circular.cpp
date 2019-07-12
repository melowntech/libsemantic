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

#include "../../ogr.hpp"
#include "../roof.hpp"

namespace semantic {

bool ogr(::OGRGeometryCollection &collection, math::Extent &verticalExtent
         , const roof::Circular &r, const math::Point3 &origin)
{
    ::OGRCircularString cs;

    /** Closed circle between two arcs.
     */
    cs.addPoint(origin(0) - r.radius, origin(1), origin(2));
    cs.addPoint(origin(0) + r.radius, origin(1), origin(2));
    cs.addPoint(origin(0) - r.radius, origin(1), origin(2));

    if (OGRERR_NONE != collection.addGeometry(&cs)) { return false; }

    // vertical extent
    update(verticalExtent, origin(2));
    update(verticalExtent, origin(2) + r.ridgeHeight);
    update(verticalExtent, origin(2) + r.curbHeight);
    update(verticalExtent, origin(2) + r.eaveHeight);

    return true;
}

} // namespace semantic
