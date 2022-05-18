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

#include <iostream>
#include <unordered_map>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "dbglog/dbglog.hpp"

#include "ogr.hpp"
#include "io.hpp"
#include "gpkg.hpp"

#include "gpkg/config.hpp"

namespace fs = boost::filesystem;

namespace semantic {

GeoPackage::~GeoPackage() {}

namespace {

using Dataset = std::unique_ptr< ::GDALDataset>;

::GDALDriver* gpkgDriver()
{
    auto driver(::GetGDALDriverManager()->GetDriverByName("GPKG"));
    if (!driver) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot find GPKG GDAL driver.";
    }
    return driver;
}

Dataset createGpkg(const fs::path &path)
{
    auto driver(gpkgDriver());
    return Dataset
        (driver->Create(path.c_str(), 0, 0, 0, GDT_Unknown, nullptr));
}

Dataset openGpkg(const fs::path &path)
{
    // const char *allowed[] = { "GPKG", 0x0 };
    const char **allowed = nullptr;
    auto handle(::GDALOpenEx(path.c_str()
                             , GDAL_OF_VECTOR | GDAL_OF_READONLY
                             , allowed, nullptr, nullptr));
    if (!handle) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to open GPKG dataset at " << path << ".";
    }

    return Dataset(static_cast< ::GDALDataset*>(handle));
}

/** Semantic layers accessor.
 */
class Layers {
public:
    /** Layer descriptor
     */
    struct Layer {
        Class cls;
        ::OGRLayer *layer;
        ::OGRFeatureDefn *definition;
        int id;
        int content;
        int minVerticalExtent;
        int maxVerticalExtent;

        Layer(Class cls, ::OGRLayer *layer)
            : cls(cls), layer(layer), definition(layer->GetLayerDefn())
            , id(-1), content(-1)
            , minVerticalExtent(-1), maxVerticalExtent(-1)
        {}
    };

    /** Create OGR dataset with semantic layers.
     */
    Layers(const fs::path &path, const geo::SrsDefinition &srs
           , GeoPackage::CreationConfig *config);

    /** Open existing OGR dataset with semantic layers.
     */
    Layers(const fs::path &path);

    Layer& operator()(Class cls) {
        auto flayers(layers_.find(cls));
        if (flayers == layers_.end()) {
            LOGTHROW(err3, std::logic_error)
                << "Layer for class <" << cls << "> not found "
                << "in semantic dataset " << path_ << ".";
        }
        return flayers->second;
    }

    const Layer& operator()(Class cls) const {
        return const_cast<Layers*>(this)->operator()(cls);
    }

    Layer* operator()(Class cls, std::nothrow_t) {
        auto flayers(layers_.find(cls));
        if (flayers == layers_.end()) { return nullptr; }
        return &flayers->second;
    }

    const Layer* operator()(Class cls, std::nothrow_t) const {
        return const_cast<Layers*>(this)->operator()(cls, std::nothrow);
    }

    void checkWriteable() const {
        if (readOnly_) {
            LOGTHROW(err1, std::runtime_error)
                << "Dataset " << path_ << " is read-only.";
        }
    }

    void checkCompatible(const geo::SrsDefinition &srs) const {
        if (!areSame(srs, srs_)) {
            LOGTHROW(err1, std::runtime_error)
                << "Semantic dataset " << path_ << ": Incompatible SRS <"
                << srs << "> (expected <" << srs_ << ">).";
        }
    }

    const geo::SrsDefinition& srs() const { return srs_; }

    math::Extents2 extents() const;

private:
    const fs::path &path_;
    bool readOnly_;
    Dataset ds_;
    geo::SrsDefinition srs_;

    using Mapping = std::unordered_map<Class, Layer>;
    Mapping layers_;
};

Layers::Layers(const fs::path &path, const geo::SrsDefinition &srs
               , GeoPackage::CreationConfig *config)
    : path_(path), readOnly_(false), ds_(createGpkg(path)), srs_(srs)
{
    auto srsRef(new ::OGRSpatialReference(srs.reference()));

    // create layers
    for (const auto &cls : enumerationValues(Class())) {
        auto layer(ds_->CreateLayer
                   (boost::lexical_cast<std::string>(cls).c_str()
                    , srsRef));
        auto &l(layers_.emplace(std::piecewise_construct
                                , std::forward_as_tuple(cls)
                                , std::forward_as_tuple(cls, layer))
                .first->second);

        {
            auto id(std::make_unique< ::OGRFieldDefn>
                    ("id", ::OGRFieldType::OFTString));
            l.definition->AddFieldDefn(id.get());
            l.id = l.definition->GetFieldIndex("id");
        }

        if (!config || config->options().includeSemanticJson) {
            auto content(std::make_unique< ::OGRFieldDefn>
                         ("content", ::OGRFieldType::OFTBinary));
            l.definition->AddFieldDefn(content.get());
            l.content = l.definition->GetFieldIndex("content");
        }

        {
            auto ve(std::make_unique< ::OGRFieldDefn>
                    ("minVerticalExtent", ::OGRFieldType::OFTReal));
            l.definition->AddFieldDefn(ve.get());
            l.minVerticalExtent
                = l.definition->GetFieldIndex("minVerticalExtent");
        }

        {
            auto ve(std::make_unique< ::OGRFieldDefn>
                    ("maxVerticalExtent", ::OGRFieldType::OFTReal));
            l.definition->AddFieldDefn(ve.get());
            l.maxVerticalExtent
                = l.definition->GetFieldIndex("maxVerticalExtent");
        }

        if (config) { config->createCustomFields(cls, l.definition); }
    }
}

Layers::Layers(const fs::path &path)
    : path_(path), readOnly_(true), ds_(openGpkg(path))
{
    bool hasSrs(false);

    // get mapping for known layers (if exists)
    for (int i(0), e(ds_->GetLayerCount()); i < e; ++i) {
        auto layer(ds_->GetLayer(i));
        Class cls;
        try {
            cls = boost::lexical_cast<Class>(layer->GetName());
        } catch (const boost::bad_lexical_cast&) {
            LOG(warn2) << "Layer <" << layer->GetName()
                       << "> doesn't match any known class.";
            continue;
        }

        if (!hasSrs) {
            if (const auto *ref = layer->GetSpatialRef()) {
                srs_ = geo::SrsDefinition::fromReference(*ref);
                hasSrs = true;
            }
        }

        auto &l(layers_.emplace(std::piecewise_construct
                                , std::forward_as_tuple(cls)
                                , std::forward_as_tuple(cls, layer))
                .first->second);
        l.id = l.definition->GetFieldIndex("id");
        l.content = l.definition->GetFieldIndex("content");
        l.minVerticalExtent = l.definition->GetFieldIndex("minVerticalExtent");
        l.maxVerticalExtent = l.definition->GetFieldIndex("maxVerticalExtent");
    }
}

math::Extents2 Layers::extents() const {
    math::Extents2 extents(math::InvalidExtents{});
    for (const auto &item : layers_) {
        const auto &layer(item.second);
        if (!layer.layer->GetFeatureCount(TRUE)) { continue; }

        layer.layer->SetSpatialFilter(nullptr);
        ::OGREnvelope envelope;
        if (layer.layer->GetExtent(&envelope, TRUE) == OGRERR_NONE) {
            update(extents, envelope.MinX, envelope.MinY);
            update(extents, envelope.MaxX, envelope.MaxY);
        }
    }
    return extents;
}

/** Spatial filter generator.
 */
class SpatialFilter {
public:
    SpatialFilter(const geo::SrsDefinition &layerSrs
                  , const GeoPackage::Query &query)
    {
        if (!query.extents) { return; }

        // construct polygon from points
        {
            const auto &e(*query.extents);
            const auto samples(query.srs ? query.segmentsPerExtentEdge : 1);

            ::OGRLinearRing ring;

            /** Adds line from a to b in n segments without last point.
             */
            const auto &addLine([&](const math::Point2 &a
                                    , const math::Point2 &b)
            {
                ring.addPoint(a(0), a(1));
                auto step((b - a) / samples);
                for (std::size_t i(1); i < samples; ++i) {
                    const math::Point2 p(a + i * step);
                    ring.addPoint(p(0), p(1));
                }
            });

            addLine(ll(e), ul(e));
            addLine(ul(e), ur(e));
            addLine(ur(e), lr(e));
            addLine(lr(e), ll(e));
            ring.addPoint(e.ll(0), e.ll(1));

            filter = boost::in_place();
            filter->addRing(&ring);
        }

        if (query.srs) {
            // convert from extents srs to layer srs
            auto srcSrs(query.srs->reference());
            filter->assignSpatialReference(&srcSrs);
            auto dstSrs(layerSrs.reference());
            filter->transformTo(&dstSrs);
        }
    }

    void setTo(::OGRLayer *layer) {
        layer->SetSpatialFilter(filter ? &*filter : nullptr);
    }

private:
    boost::optional< ::OGRPolygon> filter;
};

} // namespace

GeoPackage::CreationConfig::CreationConfig(const Options &options)
    : options_(options)
{}

GeoPackage::CreationConfig::~CreationConfig() {}

void GeoPackage::CreationConfig
::createCustomFields(Class, ::OGRFeatureDefn*) {}

void GeoPackage::CreationConfig
::addCustomFields(const Entity&, ::OGRFeature *) {}

/** Geo package interface internals.
 */
struct GeoPackage::Detail {
    Layers layers;
    SaveOptions saveOptions;
    std::shared_ptr<CreationConfig> config;

    Detail(const fs::path &path)
        : layers(path)
    {
        saveOptions.compress = true;
    }

    Detail(const fs::path &path, const geo::SrsDefinition &srs
           , const std::shared_ptr<CreationConfig> &config)
        : layers(path, srs, config.get()), config(config)
    {
        saveOptions.compress = true;
    }

    /** Add all entities from semantic world into this dataset.
     */
    void add(const World &world);

    struct FetchMask {
        using value_type = std::uint16_t;
        enum : value_type {
            id = 0x0001
            , content = 0x0002
            , verticalExtent = 0x0004

            , all = 0xffff
         };
    };

    /** Fetch entities from all layerr matching currently set spatial filter.
     */
    template <typename Op>
    void fetch(const Layers::Layer &layer, const Op &op
               , FetchMask::value_type mask = FetchMask::all) const;

    /** Generate (sub) world from entities matching query.
     */
    World world(const Query &query);

    math::Extent verticalExtent(const Query &query) const;

    math::Extents2 extents() const { return layers.extents(); }
    const geo::SrsDefinition& srs() const { return layers.srs(); }
};

template <typename Op>
void GeoPackage::Detail::fetch(const Layers::Layer &layer, const Op &op
                               , FetchMask::value_type mask) const
{
    auto &l(layer.layer);
    l->ResetReading();
    while (auto *feature = l->GetNextFeature()) {
        // fetch feature info
        const char *id("");
        if (mask & FetchMask::id) {
            id = feature->GetFieldAsString(layer.id);
            if (!id) { id = ""; }
        }

        // data
        const char *data(""); int size(0);
        if ((mask & FetchMask::content)
            && (layer.content >= 0))
        {
            data = reinterpret_cast<char*>
                (feature->GetFieldAsBinary(layer.content, &size));
            if (!data) { data = ""; size = 0; }
        }

        // vertical extent
        math::Extent ve(math::InvalidExtents{});
        if ((mask & FetchMask::verticalExtent)
            && (layer.minVerticalExtent >= 0)
            && (layer.maxVerticalExtent >= 0))
        {
            ve.l = feature->GetFieldAsDouble(layer.minVerticalExtent);
            ve.r = feature->GetFieldAsDouble(layer.maxVerticalExtent);
        }

        // call op
        op(id, data, size, ve);
    }
}

void GeoPackage::Detail::add(const World &world)
{
    layers.checkWriteable();
    layers.checkCompatible(world.srs);

    SerializationOptions options(saveOptions, world.origin);

    OgrConfig ogrConfig;
    if (config) {
        ogrConfig.simplified = config->options().simplified;
    }

    ogr(world, [&](const auto &e, auto &&geometry)
    {
        auto &layer(layers(e.cls));

        // create feature
        auto feature(std::make_unique< ::OGRFeature>(layer.definition));

        // set feature geometry
        if (feature->SetGeometryDirectly(geometry.geometry.release())
            != OGRERR_NONE)
        {
            LOGTHROW(err3, std::runtime_error)
                << "Cannot add geometry of entity <" << e.cls << ">.\""
                << e.id << "\".";
        }

        // set id
        feature->SetField(layer.id, e.id.c_str());

        // set content
        if (layer.content >= 0) {
            const auto content(serialize(e, options));
            feature->SetField
                (layer.content, content.size()
                 , const_cast<GByte*>(reinterpret_cast<const GByte*>
                                      (content.data())));
        }

        // set vertical extent
        feature->SetField(layer.minVerticalExtent, geometry.verticalExtent.l);
        feature->SetField(layer.maxVerticalExtent, geometry.verticalExtent.r);

        // fill in custom fields
        if (config) { config->addCustomFields(e, feature.get()); }

        // store feature in layer
        if (OGRERR_NONE != layer.layer->CreateFeature(feature.get())) {
            LOGTHROW(err3, std::runtime_error)
                << "Cannot create feature for entity <" << e.cls << ">.\""
                << e.id << "\".";
        }
    }, ogrConfig);
}

World GeoPackage::Detail::world(const Query &query)
{
    World world;
    world.srs = layers.srs();

    SpatialFilter filter(world.srs, query);

    for (auto cls : enumerationValues(Class())) {
        if (auto *layer = layers(cls, std::nothrow)) {
            filter.setTo(layer->layer);
            distribute(cls, world, [&](auto &entities)
            {
                this->fetch(*layer
                            , [&](const std::string&, const char *data
                                  , std::size_t size, const auto&...)
                            {
                                entities.emplace_back();
                                deserialize(data, size, entities.back());
                            }
                            , FetchMask::id | FetchMask::content);
            });
        }
    }

    return world;
}

math::Extent GeoPackage::Detail::verticalExtent(const Query &query) const
{
    math::Extent verticalExtent(math::InvalidExtents{});

    SpatialFilter filter(layers.srs(), query);

    for (auto cls : enumerationValues(Class())) {
        if (auto *layer = layers(cls, std::nothrow)) {
            filter.setTo(layer->layer);
            this->fetch(*layer
                        , [&](const auto&, const auto&, const auto&
                              , const math::Extent &ve)
                        {
                            math::update(verticalExtent, ve);
                        }
                        , FetchMask::verticalExtent);
        }
    }

    return verticalExtent;
}

GeoPackage::GeoPackage(const fs::path &path)
    : detail_(std::make_unique<Detail>(path))
{}

GeoPackage::GeoPackage(const fs::path &path, const geo::SrsDefinition &srs
                       , const std::shared_ptr<CreationConfig> &config)
    : detail_(std::make_unique<Detail>(path, srs, config))
{}

void GeoPackage::add(const World &world)
{
    detail_->add(world);
}

World GeoPackage::world(const Query &query) const
{
    return detail_->world(query);
}

math::Extent GeoPackage::verticalExtent(const Query &query) const
{
    return detail_->verticalExtent(query);
}

math::Extents2 GeoPackage::extents() const
{
    return detail_->extents();
}

const geo::SrsDefinition& GeoPackage::srs() const
{
    return detail_->srs();
}

} // namespace semantic
