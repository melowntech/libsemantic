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

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Gpkg2Semantic : public service::Cmdline
{
public:
    Gpkg2Semantic()
        : service::Cmdline("gpkg2semantic", BUILD_TARGET_VERSION)
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

    fs::path input_;
    fs::path output_;
};

void Gpkg2Semantic::configuration(po::options_description &cmdline
                        , po::options_description &config
                        , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to semantic GeoPackage file.")
        ("output", po::value(&output_)->required()
         , "Path to the output semantic world file.")
        ;

    pd.add("input", 1).add("output", 1);

    (void) config;
}

void Gpkg2Semantic::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Gpkg2Semantic::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(gpkg2semantic
usage
    gpkg2semantic input output [OPTIONS]

)RAW";
    }
    return false;
}

int Gpkg2Semantic::run()
{
    semantic::GeoPackage gpkg(input_);
    auto world(gpkg.world());
    save(world, output_);

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    ::OGRRegisterAll();
    return Gpkg2Semantic()(argc, argv);
}
