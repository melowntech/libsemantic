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

#include "../mesh.hpp"
#include "building.hpp"
#include "multipolymesh.hpp"

namespace semantic {

namespace lod2 {

geometry::Mesh mesh(const roof::Roof &roof
                    , const MeshConfig &config
                    , const math::Point3 &origin)
{
    struct Visitor : public boost::static_visitor<geometry::Mesh> {
        const MeshConfig &config;
        const math::Point3 &origin;
        Visitor(const MeshConfig &config, const math::Point3 &origin)
            : config(config), origin(origin)
        {}
        geometry::Mesh operator()(const roof::Rectangular &r) const {
            return mesh(r, config, origin);
        }
        geometry::Mesh operator()(const roof::Circular &r) const {
            return mesh(r, config, origin);
        }
    } v(config, origin);
    return boost::apply_visitor(v, roof.instance);
}

geometry::Mesh mesh(const Building &building, const MeshConfig &config
                    , const math::Point3 &origin_)
{
    const auto origin(origin_ + building.origin);

    geometry::Mesh m;
    if (building.mesh.vertices.size())
    {
        if (building.roofs.size())
        {
            LOG(warn4)
                << "Building contains both roof(s) and building mesh. Choosing "
                   "building mesh to produce the semantic mesh.";
        }
        detail::append(m, mesh(building.mesh, config, origin));
    }
    else
    {
        for (const auto& roof : building.roofs)
        {
            detail::append(m, mesh(roof, config, origin + roof.center));
        }
    }

    return m;
}

} // namespace lod2

geometry::Mesh mesh(const World &world, const MeshConfig &config, int lod)
{
    geometry::Mesh m;
    mesh(world, config
         ,[&m](const auto&, const geometry::Mesh &add) {
             detail::append(m, add);
         }
         , lod);
    return m;
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
