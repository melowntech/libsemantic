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

#ifndef semantic_ogr_ogr_incl_hpp_included_
#define semantic_ogr_ogr_incl_hpp_included_

#ifndef semantic_ogr_hpp_guard
#  error "This file must be included from ogr.hpp only."
#endif

#include "roof.hpp"
#include "building.hpp"
#include "tree.hpp"
#include "pole.hpp"
#include "lamp.hpp"
#include "manhole.hpp"

namespace semantic {

template <typename OgrCallback>
void ogr(const Building &building, const math::Point3 &origin
         , const OgrCallback &ogrCallback)
{
    ogrCallback(building, ogr(building, origin));
}

template <typename OgrCallback>
void ogr(const Tree &tree, const math::Point3 &origin
         , const OgrCallback &ogrCallback)
{
    ogrCallback(tree, ogr(tree, origin));
}

template <typename OgrCallback>
void ogr(const Pole &pole, const math::Point3 &origin
         , const OgrCallback &ogrCallback)
{
    ogrCallback(pole, ogr(pole, origin));
}

template <typename OgrCallback>
void ogr(const Lamp &lamp, const math::Point3 &origin
         , const OgrCallback &ogrCallback)
{
    ogrCallback(lamp, ogr(lamp, origin));
}

template <typename OgrCallback>
void ogr(const Manhole &manhole, const math::Point3 &origin
         , const OgrCallback &ogrCallback)
{
    ogrCallback(manhole, ogr(manhole, origin));
}

template <typename OgrCallback>
void ogr(const World &world, const OgrCallback &ogrCallback)
{
#define SEMANTIC_ENTITY_DISTRIBUTE(WHAT)                \
    for (const auto &e : world.WHAT) {                  \
        ogr(e, world.origin, ogrCallback);              \
    }

    // ENTITY: update when adding a new entity
    SEMANTIC_ENTITY_DISTRIBUTE(buildings);
    SEMANTIC_ENTITY_DISTRIBUTE(trees);
    SEMANTIC_ENTITY_DISTRIBUTE(poles);
    SEMANTIC_ENTITY_DISTRIBUTE(lamps);
    SEMANTIC_ENTITY_DISTRIBUTE(manholes);

#undef SEMANTIC_ENTITY_DISTRIBUTE
}

} // namespace semantic

#endif // semantic_ogr_ogr_incl_hpp_included_
