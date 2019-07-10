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

#include "geometry/meshop.hpp"

#include "featurelayers.hpp"

namespace semantic {

namespace {

class LayerBuilder {
public:
    LayerBuilder(const semantic::World &world, const MeshConfig &config
                 , int lod)
        : world_(world), fid_()
        , materials_(semantic::materials())
    {
        semantic::mesh(world, config
                       , [this](auto&&... args) {
                           this->mesh(std::forward<decltype(args)>(args)...);
                       }
                       , lod);
    }

    geo::FeatureLayers featureLayers() {
        geo::FeatureLayers fl;
        for (auto &item : layers_) {
            fl.layers.emplace_back(std::move(item.second));
        }
        return fl;
    }

private:
    using Layer = geo::FeatureLayers::Layer;
    using LayerMap = std::map<semantic::Class, Layer>;
    using Features = geo::FeatureLayers::Features;

    template <typename Entity>
    void mesh(const Entity &entity, const geometry::Mesh &mesh)
    {
        auto &l(layer(entity.cls));

        for (const auto &sm : geometry::splitById(mesh)) {
            Features::Properties props;
            props["material"] = materials_[sm.faces.front().imageId];

            // add surface
            auto &s(l.features.addSurface(++fid_, entity.id, props));
            s.vertices = sm.vertices;
            for (const auto &face : sm.faces) {
                s.surface.emplace_back(face.a, face.b, face.c);
            }
        }
    }

    Layer& layer(semantic::Class cls) {
        auto flayers(layers_.find(cls));
        if (flayers != layers_.end()) { return flayers->second; }

        const auto name(boost::lexical_cast<std::string>(cls));
        return layers_.emplace
            (std::piecewise_construct
             , std::forward_as_tuple(cls)
             , std::forward_as_tuple(name, world_.srs, true))
            .first->second;
    }

    const semantic::World &world_;
    LayerMap layers_;
    Features::Fid fid_;
    std::vector<std::string> materials_;
};

} // namespace

geo::FeatureLayers featureLayers(const World &world, const MeshConfig &config
                                 , int lod)
{
    return LayerBuilder(world, config, lod).featureLayers();
}

} // namespace semantic
