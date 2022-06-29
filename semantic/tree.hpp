/**
 * Copyright (c) 2022 Melown Technologies SE
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

#ifndef semantic_tree_hpp_included_
#define semantic_tree_hpp_included_

#include <vector>
#include <array>

#include <boost/variant.hpp>

#include "utility/enum-io.hpp"
#include "math/geometry_core.hpp"

namespace semantic { namespace tree {

struct Aerial {
    enum class Type { deciduous, coniferous };

    Type type = Type::deciduous;
    math::Point3 center;
    double a = 0.0;
    double b = 0.0;
    std::vector<double> harmonics;
};

struct GroundLevel {
    struct Circle {
        math::Point3 center;
        double radius = {};
    };

    Circle trunk;
    Circle crown;
    double height = {};
};

typedef boost::variant<Aerial, GroundLevel> Instance;
enum class Kind { aerial = 0, groundLevel };

// inline stuff

/** Tree subclasses.
 */
UTILITY_GENERATE_ENUM_IO(Aerial::Type,
                         ((deciduous))
                         ((coniferous))
                         )

UTILITY_GENERATE_ENUM_IO(Kind,
                         ((aerial))
                         ((groundLevel)("ground-level"))
                         )

} } // namespace tree::semantic

#endif // semantic_tree_hpp_included_
