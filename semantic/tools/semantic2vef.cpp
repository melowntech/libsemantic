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

#include "service/cmdline.hpp"

#include <cstdlib>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"
#include "utility/implicit-value.hpp"
#include "utility/small_map.hpp"

#include "vef/vef.hpp"

#include "service/cmdline.hpp"

#include "geometry/meshop.hpp"

#include "semantic/io.hpp"
#include "semantic/mesh.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace {

// Colors in BGR
const utility::small_map<semantic::Material, cv::Vec3b> materialColors(
    { { semantic::Material::default_, cv::Vec3b(220, 220, 220) },
      { semantic::Material::facade, cv::Vec3b(245, 245, 245) },
      { semantic::Material::roof, cv::Vec3b(91, 114, 226) },
      { semantic::Material::tree_trunk, cv::Vec3b(10, 53, 83) },
      { semantic::Material::tree_crown_deciduous, cv::Vec3b(61, 137, 86) },
      { semantic::Material::tree_crown_coniferous, cv::Vec3b(26, 73, 41) },
      { semantic::Material::terrace, cv::Vec3b(71, 164, 233) },
      { semantic::Material::pole, cv::Vec3b(74, 79, 85) }
    });

class Semantic2Vef : public service::Cmdline
{
public:
    Semantic2Vef()
        : service::Cmdline("semantic2vef", BUILD_TARGET_VERSION)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd)
        override;

    void configure(const po::variables_map &vars)
        override;

    bool help(std::ostream &out, const std::string &what) const
        override;

    int run() override;

    semantic::MeshConfig meshConfig_;
    std::vector<fs::path> input_;
    fs::path output_;
    int txtColorWidth_ = 8;
};

void Semantic2Vef::configuration(po::options_description &cmdline
                        , po::options_description & /* config */
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
        ("txtColorWidth", po::value(&txtColorWidth_)
         ->default_value(txtColorWidth_)
         , "Width of individual color squares in generated texture image.")
        ("repairMesh", po::value(&meshConfig_.repairMesh)
         ->default_value(meshConfig_.repairMesh)
         , "Remove zero-area and non-manifold faces from building roof meshes. "
           "Also sets `vertexMergeEps` to some non-zero value if it's zero (to "
           "merge very close vertices).")
        ("vertexMergeEps", po::value(&meshConfig_.vertexMergeEps)
         ->default_value(meshConfig_.vertexMergeEps)
         ,"Min distance between two distinct verticies (vertices are merged "
          "otherwise). Used in building roofs. Set to zero to disable merging.")
        ;

    pd.add("output", 1).add("input", -1);
}

void Semantic2Vef::configure(const po::variables_map & /* vars */)
{
}

bool Semantic2Vef::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(semantic2vef
usage
    semantic2vef output input* [OPTIONS]

If multiple input files are used their SRS must be the same.

)RAW";
    }
    return false;
}

cv::Mat3b createTextureImg(const int colorWidth)
{
    auto materials = enumerationValues(semantic::Material());
    cv::Mat3b img(colorWidth, materials.size() * colorWidth);

    for (const auto& m : materials)
    {
        // get color
        cv::Vec3b color(0, 0, 0);
        auto it = materialColors.find(m);
        if (it != materialColors.end()) { color = it->second; }

        // get the rect in texture
        int x0 = +m * colorWidth;
        img(cv::Rect2i(x0, 0, colorWidth, colorWidth)) = color;
    }
    return img;
}

void addTextureCoords(geometry::Mesh& mesh, const int colorWidth)
{
    auto materials = enumerationValues(semantic::Material());
    int imWidth = colorWidth * materials.size();

    // create texture coordinates
    mesh.tCoords.resize(materials.size() * 3);
    for (const auto& m : materials)
    {
        // x-coords in image
        double x0 = ((+m + 0.25) * colorWidth) / imWidth;
        double x1 = x0 + ((0.5 * colorWidth) / imWidth);

        int tcIdx0 = +m * 3;
        mesh.tCoords[tcIdx0] = math::Point2(x0, 0.25);
        mesh.tCoords[tcIdx0 + 1] = math::Point2(x1, 0.25);
        mesh.tCoords[tcIdx0 + 2] = math::Point2(x0, 0.75);
    }

    // assign coordinates to faces
    for (auto& f : mesh.faces)
    {
        int tcIdx0 = f.imageId * 3;
        f.ta = tcIdx0;
        f.tb = tcIdx0 + 1;
        f.tc = tcIdx0 + 2;
        f.imageId = 0;
    }
}

int Semantic2Vef::run()
{
    vef::ArchiveWriter ar(fs::path(output_), false);
    boost::optional<geo::SrsDefinition> srs;
    cv::Mat3b txtImg = createTextureImg(txtColorWidth_);

    meshConfig_.worldCrs = true;

    std::size_t i = 0;
    geometry::Mesh mesh;
    for (const auto &input : input_) {
        // load world
        const auto world(semantic::load(input));

        // check srs
        if (!srs) {
            srs = world.srs;
        } else if (!areSame(*srs, world.srs)) {
            LOG(fatal)
                << "Different SRS in input world file "
                << input << ": '" << world.srs << "'.";
            return EXIT_FAILURE;
        }

        std::string windowName = (boost::format("%08d") % i).str();

        math::Matrix4 worldTf(math::identity4());
        worldTf(0, 3) = world.origin(0);
        worldTf(1, 3) = world.origin(1);
        worldTf(2, 3) = world.origin(2);

        vef::Id winId = ar.addWindow(windowName, worldTf);
        vef::Id lodId = ar.addLod(winId);

        // save texture
        vef::Texture txt;
        txt.size = math::Size2(txtImg.size().width, txtImg.size().height);
        txt = ar.addTexture(winId, lodId, txt, vef::Texture::Format::png);
        cv::imwrite(txt.path.string(), txtImg);

        // add mesh
        vef::Mesh& vefMesh = ar.mesh(winId, lodId);

        geometry::Mesh mesh = semantic::mesh(world, meshConfig_);
        addTextureCoords(mesh, txtColorWidth_);

        geometry::saveAsObj(mesh,
                            vefMesh.path,
                            vefMesh.mtlPath().filename().generic_string());
        ++i;
    }

    if (srs) { ar.setSrs(srs.value()); }
    else
    {
        LOGTHROW(err4, std::runtime_error) << "No SRS found in input files";
    }

    ar.flush();

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Semantic2Vef()(argc, argv);
}
