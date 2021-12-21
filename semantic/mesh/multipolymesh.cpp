/**
 * Copyright (c) 2021 Melown Technologies SE
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

#include "../mesh.hpp"

namespace semantic
{
namespace lod2
{
inline Material string2Material(const std::string& s)
{
    Material res;
    if(boost::conversion::try_lexical_convert<Material>(s, res)){
        return res;
    }
    return Material::default_;
}

geometry::Mesh mesh(const geometry::MultiPolyMesh<std::string>& mpmesh,
                    const MeshConfig& /* config */,
                    const math::Point3& origin)
{
    std::vector<std::string> faceLabels;
    geometry::Mesh m = mpmesh.triangulateFaces(&faceLabels);

    for (auto& v : m.vertices)
    {
        v += origin;
    }

    for (std::size_t i = 0; i < m.faces.size(); i++)
    {
        m.faces[i].imageId = +string2Material(faceLabels[i]);
    }

    return m;
}

} // namespace lod2
} // namespace semantic
