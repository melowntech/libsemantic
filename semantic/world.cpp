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

#include <algorithm>

#include "dbglog/dbglog.hpp"

#include "world.hpp"

namespace semantic {

const Class Building::cls;
const Class Tree::cls;
const Class Railway::cls;
const Class LaneLine::cls;
const Class Pole::cls;
const Class Lamp::cls;
const Class Manhole::cls;
const Class TrafficSign::cls;
const Class TrafficLight::cls;

Classes classes(const World &world)
{
    Classes classes;

    // ENTITY: update when adding a new entity
    if (!world.buildings.empty()) { classes.push_back(Class::building); }

    std::sort(classes.begin(), classes.end());
    return classes;
}

Classes classes(const Classes &l, const Classes &r)
{
    Classes classes;

    std::set_union(l.begin(), l.end(), r.begin(), r.end()
                   , std::back_inserter(classes));

    return classes;
}

void localize(World &world)
{
    (void) world;
    LOG(warn3) << "TODO: implement me";
}

} // namespace semantic
