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

#ifndef semantic_roof_hpp_included_
#define semantic_roof_hpp_included_

#include <vector>
#include <array>

#include <boost/variant.hpp>

#include "utility/enum-io.hpp"
#include "math/geometry_core.hpp"

namespace semantic { namespace roof {

struct Rectangular {
    using Curb = std::array<double, 4>;
    using Hip = std::array<double, 2>;
    using EaveHeight = std::array<double, 4>;

    constexpr static Curb defaultCurb = { 1.0, 1.0, 0.0, 0.0 };
    constexpr static Hip defaultHip = { 1.0, 1.0 };
    constexpr static EaveHeight defaultEaveHeight = { 0.0, 0.0, 0.0, 0.0 };

    math::Size2f size;
    math::Point2d skew;
    double azimuth = 0.0;
    Curb curb = defaultCurb;
    Hip hip = defaultHip;
    double ridgeHeight = 0.0;
    double curbHeight = 0.0;
    EaveHeight eaveHeight = defaultEaveHeight;

    enum class Key { top = 0, bottom = 1, left = 2, right = 3 };
};

/** Casts Key to integer.
 */
int operator+(Rectangular::Key key);

struct Circular {
    double radius = 0.0;
    double curb = 0.0;
    double ridgeHeight = 0.0;
    double curbHeight = 0.0;
    double eaveHeight = 0.0;
};

typedef boost::variant<Circular, Rectangular> Instance;
enum class Type { circular = 0, rectangular };

struct Roof {
    typedef std::vector<Roof> list;

    math::Point3 center;
    Instance instance;

    Type type() const { return static_cast<Type>(instance.which()); }
};

// inline stuff

UTILITY_GENERATE_ENUM_IO(Type,
                         ((rectangular))
                         ((circular))
                         )

inline int operator+(Rectangular::Key key) {
    return static_cast<int>(key);
}

} } // namespace roof::semantic

#endif // semantic_roof_hpp_included_
