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

#ifndef semantic_world_hpp_included_
#define semantic_world_hpp_included_

#include <vector>
#include <array>

#include <boost/variant.hpp>

#include "utility/enum-io.hpp"
#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"

#include "roof.hpp"

namespace semantic {

/** Entity classes.
 */
UTILITY_GENERATE_ENUM(Class,
                      ((building))
                      ((tree))
                      )

typedef std::vector<Class> Classes;

/** Base entity.
 */
struct Entity {
    std::string id;
    std::string descriptor;
    math::Point3 origin;
};

/** Building. Only roofs so far, facades are implicit.
 */
struct Building : Entity {
    /** Entity class.
     */
    static const constexpr Class cls = Class::building;

    typedef std::vector<Building> list;

    /** List of roofs.
     */
    roof::Roof::list roofs;
};

/** Tree.
 */
struct Tree : Entity {
    /** Entity class.
     */
    static const constexpr Class cls = Class::tree;
    typedef std::vector<Tree> list;

    math::Point3 center;
    double a = 0.0;
    double b = 0.0;
    std::vector<double> harmonics;
};

/** Semantic world.
 *
 * NB: Contains only a list of buildings, so far.
 */
struct World {
    typedef std::vector<World> list;

    /** Spatial reference. If SRS is a projection than vertical adjustment is
     *  applied.
     */
    geo::SrsDefinition srs;

    /** World origin. All entities are relative to this point.
     */
    math::Point3 origin;

    /** All buildings in the world.
     */
    Building::list buildings;

    /** All trees in the world.
     */
    Tree::list trees;
};

/** Localizes world. Sets world origin center of all world bounding box.
 */
void localize(World &world);

/** Get list of entity classes used in given world.
 */
Classes classes(const World &world);

Classes classes(const Classes &l, const Classes &r);

/** Calls provided op with entity list matching given class.
 */
template <typename Op>
void distribute(Class cls, const World &world, const Op &op);

/** Calls provided op with entity list matching given class.
 */
template <typename Op>
void distribute(Class cls, World &world, const Op &op);

// inlines

template <typename Op>
void distribute(Class cls, const World &world, const Op &op)
{
    switch (cls) {
    case Class::building: op(world.buildings); break;
    case Class::tree: op(world.trees); break;
    }
}

template <typename Op>
void distribute(Class cls, World &world, const Op &op)
{
    switch (cls) {
    case Class::building: op(world.buildings); break;
    case Class::tree: op(world.trees); break;
    }
}

} // namespace semantic

#endif // semantic_world_hpp_included_
