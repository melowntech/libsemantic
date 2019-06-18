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

#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "../mesh.hpp"
#include "detail.hpp"
#include "roof.hpp"

namespace semantic {

namespace lod2 {

using detail::append;

geometry::Mesh mesh(const Roof &roof
                    , const MeshConfig &config)
{
    struct Visitor : public boost::static_visitor<geometry::Mesh> {
        const MeshConfig &config;
        Visitor(const MeshConfig &config) : config(config) {}
        geometry::Mesh operator()(const roof::Rectangular &r) const {
            return mesh(r, config);
        }
        geometry::Mesh operator()(const roof::Circular &r) const {
            return mesh(r, config);
        }
    } v(config);
    return boost::apply_visitor(v, roof.instance);
}

geometry::Mesh mesh(const Building &building, const MeshConfig &config)
{
    geometry::Mesh m;
    for (const auto &roof : building.roofs) {
        append(m, mesh(roof, config), building.origin + roof.center);
    }
    return m;
}

geometry::Mesh mesh(const World &world, const MeshConfig &config)
{
    geometry::Mesh m;
    for (const auto &building : world.buildings) {
        append(m, mesh(building, config), world.origin);
    }
    return m;
}

} // namespace lod2

/** Generate mesh in given LOD.
 */
geometry::Mesh mesh(const World &world, const MeshConfig &config, int lod)
{
    switch (lod) {
    case 2: return lod2::mesh(world, config);

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
    throw;
}

std::vector<std::string> materials()
{
    std::vector<std::string> materials;
    for (auto material : enumerationValues(semantic::Material())) {
        materials.push_back(boost::lexical_cast<std::string>(material));
    }
    return materials;
}

} // namespace semantic
