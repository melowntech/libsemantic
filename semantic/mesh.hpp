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

#ifndef semantic_mesh_hpp_included_
#define semantic_mesh_hpp_included_

#include "utility/enum-io.hpp"

#include "geometry/mesh.hpp"

#include "world.hpp"

namespace semantic {

/** Material definition.
 */
enum class Material : int {
    default_ = 0
    , facade = 1
    , roof = 2
    , tree_trunk = 3
    , tree_crown_deciduous = 4
    , tree_crown_coniferous = 5
};

/** Helper for material-enum--to-number converion.
 */
int operator+(Material m);

struct MeshConfig {
    /** Maximum circle segment length. Used for number of segments calculation.
     */
    double maxCircleSegment = 1.0;

    /** Minimum number of circle segments.
     */
    unsigned int minSegmentCount = 16;
};

/** Generate mesh in given LOD.
 */
geometry::Mesh mesh(const World &world, const MeshConfig &config
                    , int lod = 2);

/** Generate mesh in given LOD. Calls
 *      meshCallback(const auto &entity, geometry::Mesh)
 *  for every encountered entity.
 */
template <typename MeshCallback>
void mesh(const World &world, const MeshConfig &config
          , const MeshCallback &meshCallback, int lod = 2);

/** Generates list of materials. Position is equal to imageId in mesh generated
 *  by mesh(...) function.
 */
std::vector<std::string> materials();

/** Inlines
 */

inline int operator+(Material m) { return static_cast<int>(m); }

UTILITY_GENERATE_ENUM_IO(Material,
                         ((default_)("default"))
                         ((facade))
                         ((roof))
                         ((tree_trunk)("tree-trunk"))
                         ((tree_crown_deciduous)("tree_crown_deciduous"))
                         ((tree_crown_coniferous)("tree-crown-coniferous"))
                         )

} // namespace semantic

#define semantic_mesh_hpp_guard
#include "mesh/mesh.incl.hpp"
#undef semantic_mesh_hpp_guard

#endif // semantic_mesh_hpp_included_
