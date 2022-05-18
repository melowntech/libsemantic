#include <cstdlib>
#include <unordered_map>
#include <iostream>

#include <ogrsf_frmts.h>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"

#include "service/cmdline.hpp"

#include "geometry/meshop.hpp"

#include "semantic/io.hpp"
#include "semantic/gpkg.hpp"
#include "semantic/gpkg/config.hpp"

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

    semantic::GeoPackage::CreationConfig::Options options_;
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

        ("simplified"
         , po::value(&options_.simplified)
         ->default_value(options_.simplified)
         , "Generate simplified geometry.")

        ("includeSemanticJson"
         , po::value(&options_.includeSemanticJson)
         ->default_value(options_.includeSemanticJson)
         , "Include semantic JSON snippet in features attributes.")
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

    auto cc(std::make_shared<semantic::GeoPackage::CreationConfig>(options_));

    {
        semantic::GeoPackage gpkg(output_, *srs, cc);
        for (const auto &world : worlds) { gpkg.add(world); }
    }

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    ::OGRRegisterAll();
    return Semantic2Gpkg()(argc, argv);
}
