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

#ifndef semantic_mesh_mesh_incl_hpp_included_
#define semantic_mesh_mesh_incl_hpp_included_

#ifndef semantic_mesh_hpp_guard
#  error "This file must be included from mesh.hpp only."
#endif

#line 1 "shit-before"
#include "roof.hpp"
#line 1 "shit-after"
#include "detail.hpp"

namespace semantic {

namespace lod2 {

using detail::append;

template <typename MeshCallback>
void mesh(const Building &building, const MeshConfig &config
          , const math::Point3 &origin_
          , const MeshCallback &meshCallback)
{
    const auto origin(origin_ + building.origin);

    for (const auto &roof : building.roofs) {
        meshCallback(Class::building, building.id
                     , mesh(roof, config, math::Point3(origin + roof.center)));
    }
}

template <typename MeshCallback>
void mesh(const World &world, const MeshConfig &config
          , const MeshCallback &meshCallback)
{
    for (const auto &building : world.buildings) {
        mesh(building, config, world.origin, meshCallback);
    }
}

} // namespace lod2

template <typename MeshCallback>
void mesh(const World &world, const MeshConfig &config
          , const MeshCallback &meshCallback, int lod)
{
    switch (lod) {
    case 2: return lod2::mesh(world, config, meshCallback);

    case 0:
    case 1:
    case 3:
    case 4:
        LOGTHROW(err3, std::runtime_error)
            << "Generation of mesh in LOD " << lod << " not supported.";
        break;

    default:
        LOGTHROW(err3, std::runtime_error)
            << "Unknown LOD " << lod << ".";
        break;
    }
}

} // namespace semantic

#endif // semantic_mesh_mesh_incl_hpp_included_
