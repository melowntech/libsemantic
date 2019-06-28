#include <cstdlib>
#include <unordered_map>
#include <iostream>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"

#include "service/cmdline.hpp"

#include "geometry/meshop.hpp"

#include "semantic/io.hpp"
#include "semantic/ogr.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Semantic2Gpkg : public service::Cmdline
{
public:
    Semantic2Gpkg()
        : service::Cmdline("semantic2gpkg", BUILD_TARGET_VERSION)
    {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        override;

    virtual void configure(const po::variables_map &vars)
        override;

    virtual bool help(std::ostream &out, const std::string &what) const
        override;

    virtual int run() override;

    std::vector<fs::path> input_;
    fs::path output_;
};

void Semantic2Gpkg::configuration(po::options_description &cmdline
                        , po::options_description &config
                        , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("output", po::value(&output_)->required()
         , "Path to the output GeoPackage file.")
        ("input", po::value(&input_)->required()
         , "One ore more paths to the input files with semantic data.")
        ;

    pd.add("output", 1).add("input", -1);

    (void) config;
}

void Semantic2Gpkg::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Semantic2Gpkg::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(semantic2gpkg
usage
    semantic2gpkg output input* [OPTIONS]

If multiple input files are used their SRS must be the same.

)RAW";
    }
    return false;
}

class Layers {
public:
    struct Layer {
        OGRLayer *layer;
        OGRFeatureDefn *definition;

        Layer(OGRLayer *layer)
            : layer(layer), definition(layer->GetLayerDefn())
        {}
    };

    Layers(GDALDataset &gpkg, const geo::SrsDefinition &srs)
    {
        saveOptions_.compress = true;

        auto srsRef(srs.reference());

        // create layers
        for (const auto &cls : enumerationValues(semantic::Class())) {
            auto layer(gpkg.CreateLayer
                       (boost::lexical_cast<std::string>(cls).c_str()
                        , &srsRef));
            auto &l(layers_.emplace(cls, layer).first->second);

            {
                auto content(std::make_unique< ::OGRFieldDefn>
                             ("id", ::OGRFieldType::OFTString));
                l.definition->AddFieldDefn(content.get());
            }
            {
                auto content(std::make_unique< ::OGRFieldDefn>
                             ("content", ::OGRFieldType::OFTBinary));
                l.definition->AddFieldDefn(content.get());
            }
        }
    }

    Layer& operator()(semantic::Class cls) {
        auto flayers(layers_.find(cls));
        if (flayers == layers_.end()) {
            LOGTHROW(err3, std::logic_error)
                << "Layer for class <" << cls << "> not found.";
        }
        return flayers->second;
    }

    const Layer& operator()(semantic::Class cls) const {
        return const_cast<Layers*>(this)->operator()(cls);
    }

    void add(const semantic::World &world) {
        semantic::SerializationOptions options(saveOptions_, world.origin);

        semantic::ogr(world, [&,&layers=*this](const auto &e, auto &&geometry)
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
                const auto content(semantic::serialize(e, options));
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

    void add(const semantic::World::list &worlds) {
        for (const auto &world : worlds) { add(world); }
    }

private:
    semantic::SaveOptions saveOptions_;
    using Mapping = std::unordered_map<semantic::Class, Layer>;
    Mapping layers_;
};

int Semantic2Gpkg::run()
{
    semantic::World::list worlds;
    boost::optional<geo::SrsDefinition> srs;
    for (const auto &input : input_) {
        worlds.push_back(semantic::load(input));
        const auto &world(worlds.back());
        if (!srs) {
            srs = world.srs;
        } else if (!areSame(*srs, world.srs)) {
            LOG(fatal)
                << "Different SRS in input world file "
                << input << ": '" << world.srs << "'.";
            return EXIT_FAILURE;
        }
    }

    auto driver(::GetGDALDriverManager()->GetDriverByName("GPKG"));
    if (!driver) {
        LOG(fatal) << "Cannot find GPKG GDAL driver.";
        return EXIT_FAILURE;
    }

    // create file
    std::unique_ptr<GDALDataset>
        gpkg(driver->Create(output_.c_str(), 0, 0, 0, GDT_Unknown, nullptr));

    Layers layers(*gpkg, *srs);

    layers.add(worlds);

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    ::OGRRegisterAll();
    return Semantic2Gpkg()(argc, argv);
}
