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
#include "manhole.hpp"

namespace semantic
{
OgrGeometry ogr(const Manhole& manhole, const math::Point3& origin)
{
    auto cs(std::make_unique<::OGRLineString>());

    const math::Point3 center(origin + manhole.origin);

    const auto w(manhole.size.width);
    const auto h(manhole.size.height);

    auto ur(center + math::Point3(w / 2, h / 2, 0));
    auto lr(center + math::Point3(-w / 2, h / 2, 0));
    auto ll(center + math::Point3(-w / 2, -h / 2, 0));
    auto ul(center + math::Point3(w / 2, -h / 2, 0));
    cs->addPoint(ur(0), ur(1), ur(2));
    cs->addPoint(lr(0), lr(1), lr(2));
    cs->addPoint(ll(0), ll(1), ll(2));
    cs->addPoint(ul(0), ul(1), ul(2));

    math::Extent verticalExtent;
    update(verticalExtent, center(2) + 0.05);

    return { std::move(cs), verticalExtent };
}

} // namespace semantic
