#include <unordered_map>

#include "dbglog/dbglog.hpp"

#include "ogr.hpp"
#include "io.hpp"
#include "gpkg.hpp"

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
    const char *allowed[] = { "GPKG", 0x0 };
    auto handle(::GDALOpenEx(path.c_str()
                             , GDAL_OF_VECTOR | GDAL_OF_READONLY
                             , allowed, nullptr, nullptr));
    if (!handle) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to open GPKG dataset at " << path << ".";
    }

    return Dataset(static_cast< ::GDALDataset*>(handle));
}

class Layers {
public:
    struct Layer {
        Class cls;
        ::OGRLayer *layer;
        ::OGRFeatureDefn *definition;
        int id;
        int content;

        Layer(Class cls, ::OGRLayer *layer)
            : cls(cls), layer(layer), definition(layer->GetLayerDefn())
            , id(-1), content(-1)
        {}
    };

    Layers(const fs::path &path, const geo::SrsDefinition &srs)
        : gpkg_(createGpkg(path)), srs_(srs)
    {
        auto srsRef(new ::OGRSpatialReference(srs.reference()));

        // create layers
        for (const auto &cls : enumerationValues(Class())) {
            auto layer(gpkg_->CreateLayer
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
                l.id = 0;
            }
            {
                auto content(std::make_unique< ::OGRFieldDefn>
                             ("content", ::OGRFieldType::OFTBinary));
                l.definition->AddFieldDefn(content.get());
                l.content = 1;
            }
        }
    }

    Layers(const fs::path &path)
        : gpkg_(openGpkg(path))
    {
        bool hasSrs(false);

        // get mapping for known layers (if exists)
        for (int i(0), e(gpkg_->GetLayerCount()); i < e; ++i) {
            auto layer(gpkg_->GetLayer(i));
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
        }
    }

    Layer& operator()(Class cls) {
        auto flayers(layers_.find(cls));
        if (flayers == layers_.end()) {
            LOGTHROW(err3, std::logic_error)
                << "Layer for class <" << cls << "> not found.";
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

    void checkCompatible(const geo::SrsDefinition &srs) const {
        if (!areSame(srs, srs_)) {
            LOGTHROW(err1, std::runtime_error)
                << "Incompatible SRS <"
                << srs << "> (expected <" << srs_ << ">).";
        }
    }

    const geo::SrsDefinition& srs() const { return srs_; }

private:
    Dataset gpkg_;
    geo::SrsDefinition srs_;

    using Mapping = std::unordered_map<Class, Layer>;
    Mapping layers_;
};

} // namespace

struct GeoPackage::Detail {
    Layers layers;
    SaveOptions saveOptions;

    Detail(const fs::path &path)
        : layers(path)
    {
        saveOptions.compress = true;
    }

    Detail(const fs::path &path, const geo::SrsDefinition &srs)
        : layers(path, srs)
    {
        saveOptions.compress = true;
    }

    void add(const World &world) {
        layers.checkCompatible(world.srs);

        SerializationOptions options(saveOptions, world.origin);

        ogr(world, [&](const auto &e, auto &&geometry)
        {
            auto &layer(layers(e.cls));

            // create feature
            auto feature(std::make_unique< ::OGRFeature>(layer.definition));

            // set feature geometry
            if (feature->SetGeometryDirectly(geometry.release())
                != OGRERR_NONE)
            {
                LOGTHROW(err3, std::runtime_error)
                    << "Cannot add geometry of entity <" << e.cls << ">.\""
                    << e.id << "\".";
            }

            // set id
            feature->SetField(0, e.id.c_str());

            // set content
            {
                const auto content(serialize(e, options));
                feature->SetField
                    (1, content.size()
                     , const_cast<GByte*>(reinterpret_cast<const GByte*>
                                          (content.data())));
            }

            // store feature in layer
            if (OGRERR_NONE != layer.layer->CreateFeature(feature.get())) {
                LOGTHROW(err3, std::runtime_error)
                    << "Cannot create feature for entity <" << e.cls << ">.\""
                    << e.id << "\".";
            }

        });
    }

    template <typename Op>
    void fetch(const Layers::Layer &layer, const Query &query, const Op &op)
        const
    {
        (void) query;
        // TODO: set query

        auto &l(layer.layer);
        l->ResetReading();
        while (auto *feature = l->GetNextFeature()) {
            // fetch feature info
            const auto *id(feature->GetFieldAsString(layer.id));
            if (!id) { id = ""; }
            int size(0);
            const auto *data
                (reinterpret_cast<char*>
                 (feature->GetFieldAsBinary(layer.content, &size)));
            if (!data) { data = ""; size = 0; }

            // call op
            op(id, data, size);
        }
    }

    World world(const Query &query) const {
        World world;
        world.srs = layers.srs();

        for (auto cls : enumerationValues(Class())) {
            if (const auto *layer  = layers(cls, std::nothrow)) {
                distribute(cls, world, [&](auto &entities)
                {
                    this->fetch(*layer, query
                                , [&](const std::string&, const char *data
                                      , std::size_t size)
                    {
                        // TODO: use boost iostream memory stream
                        std::istringstream is(std::string(data, size));
                        entities.emplace_back();
                        deserialize(is, entities.back());
                    });
                });
            }
        }

        localize(world);
        return world;
    }
};

GeoPackage::GeoPackage(const fs::path &path)
    : detail_(std::make_unique<Detail>(path))
{}

GeoPackage::GeoPackage(const fs::path &path, const geo::SrsDefinition &srs)
    : detail_(std::make_unique<Detail>(path, srs))
{}

void GeoPackage::add(const World &world)
{
    detail_->add(world);
}

World GeoPackage::world(const Query &query) const
{
    return detail_->world(query);
}

} // namespace semantic
