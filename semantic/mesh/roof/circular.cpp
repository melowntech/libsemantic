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

#include "../roof.hpp"
#include "../detail.hpp"

namespace semantic {

namespace lod2 {

using detail::Index;

geometry::Mesh mesh(const roof::Circular &roof)
{
    geometry::Mesh mesh;

    /** TODO: derive from radius size
     */
    const Index arcPoints(16);

    // roof center
    mesh.vertices.emplace_back(0.0, 0.0, roof.ridgeHeight);

    if (detail::colinear(roof.ridgeHeight, roof.curbHeight, roof.eaveHeight
                         , roof.curb))
    {
        const auto arcVertices(arcPoints * 2);

        for (Index i(0); i < arcPoints; ++i) {
            const double angle((2 * M_PI * i) / arcPoints);

            const auto end
                (detail::rotate(0, roof.radius, roof.eaveHeight, angle));
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
                (detail::rotate
                 (0, roof.radius * roof.curb, roof.curbHeight, angle));

            const auto end
                (detail::rotate(0, roof.radius, roof.eaveHeight, angle));
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

} // namespace lod2

} // namespace semantic
