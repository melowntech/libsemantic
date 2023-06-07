#include "service/cmdline.hpp"

#include <cstdlib>
#include <iostream>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"
#include "utility/implicit-value.hpp"

#include "service/cmdline.hpp"

#include "geometry/meshop.hpp"

#include "semantic/io.hpp"
#include "semantic/mesh.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

class Semantic2Obj : public service::Cmdline
{
public:
    Semantic2Obj()
        : service::Cmdline("semantic2obj", BUILD_TARGET_VERSION)
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

    semantic::MeshConfig meshConfig_;
    std::vector<fs::path> input_;
    fs::path output_;
};

void Semantic2Obj::configuration(po::options_description &cmdline
                        , po::options_description &config
                        , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to the input file with semantic data.")
        ("output", po::value(&output_)->required()
         , "Path to the output OBJ file.")
        ("closedSurface"
         , utility::implicit_value(&meshConfig_.closedSurface, true)
         ->default_value(false)
         , "Generate closed surface.")
        ("worldCrs"
         , utility::implicit_value(&meshConfig_.worldCrs, true)
         ->default_value(false)
         , "Keep the mesh in world's CRS (i.e. do not shift by world origin).")
        ("repairMesh", utility::implicit_value(&meshConfig_.repairMesh, true)
         ->default_value(false)
         , "Remove zero-area and non-manifold faces from building roof meshes. "
           "Also sets `vertexMergeEps` to some non-zero value if it's zero (to "
           "merge very close vertices).")
        ("vertexMergeEps", po::value(&meshConfig_.vertexMergeEps)
         ->default_value(meshConfig_.vertexMergeEps)
         ,"Min distance between two distinct verticies (vertices are merged "
          "otherwise). Used in building roofs. Set to zero to disable merging.")
        ;

    pd.add("output", 1).add("input", -1);

    (void) config;
}

void Semantic2Obj::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Semantic2Obj::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(semantic2obj
usage
    semantic2obj output input* [OPTIONS]

If multiple input files are used their SRS must be the same.

)RAW";
    }
    return false;
}

int Semantic2Obj::run()
{
    boost::optional<geo::SrsDefinition> srs;

    geometry::Mesh mesh;
    for (const auto &input : input_) {
        const auto world(semantic::load(input));

        if (!srs) {
            srs = world.srs;
        } else if (!areSame(*srs, world.srs)) {
            LOG(fatal)
                << "Different SRS in input world file "
                << input << ": '" << world.srs << "'.";
            return EXIT_FAILURE;
        }

        append(mesh, semantic::mesh(world, meshConfig_));
    }

    const auto mtlPath(utility::addExtension(output_, ".mtl"));

    geometry::ObjMaterial mtl(mtlPath.filename().generic_string());
    mtl.names = semantic::materials();

    // write mtl
    semantic::writeMtl(mtlPath);

    const auto &setStream([&, srsWritten=false](std::ostream &os)
        mutable
    {
        os << std::fixed;
        if (!srsWritten) {
            os << "### SRS: " << srs.value() << "\n\n";
            srsWritten = true;
        }
        return true;
    });

    geometry::saveAsObj(mesh, output_, mtl, setStream);

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Semantic2Obj()(argc, argv);
}
