/**
 * Copyright (c) 2023 Melown Technologies SE
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

#include "geometry/mesh.hpp"
#include "dbglog/dbglog.hpp"

/**
 * Remove null faces while preserving topology. Written specifically for the
 * case of lod2 roofs.
 */

namespace semantic { namespace lod2 {

namespace {

using Mesh = geometry::Mesh;
using FFTable = bs::SegMesh::FaceFaceTable;
using Face = geometry::Face;
using FIdx = std::size_t;
using VIdx = geometry::Face::index_type;

constexpr double NULL_FACE_THRESH { 1e-2 };
constexpr std::size_t MAX_EDGEFLIP_ITERS { 100 };

/// Checks if face contains edge v1-v2 (in this order)
bool hasEdge(const Face& f, const VIdx v1, const VIdx v2)
{
    return (f.a == v1 && f.b == v2)
           || (f.b == v1 && f.c == v2)
           || (f.c == v1 && f.a == v2);
}

/// Get the other vertex of face. NB: vertices must be given in ccw order!
VIdx theOtherVertex(const Face& f, const VIdx v1, const VIdx v2)
{
    if (f.a == v1 && f.b == v2) { return f.c; }
    if (f.b == v1 && f.c == v2) { return f.a; }
    if (f.c == v1 && f.a == v2) { return f.b; }
    LOGTHROW(err4, std::runtime_error)
        << "Cannot find given vertices in the face";
    throw;
}

/// Flip edge v1-v2 incident with face fI1
void edgeFlip(Mesh& mesh,
              const FFTable& ffTable,
              const FIdx fI1,
              const VIdx v1,
              const VIdx v2)
{
    // find the opposite face
    FIdx fI2;
    bool found = false;
    for (auto& n : ffTable[fI1])
    {
        if (hasEdge(mesh.faces[n], v2, v1))
        {
            fI2 = n;
            found = true;
            break;
        }
    }

    if (!found)
    {
        LOGTHROW(err4, std::runtime_error)
            << "Cannot find edge in any neighbouring faces.";
    }

    // flip edge
    auto a = theOtherVertex(mesh.faces[fI1], v1, v2);
    auto b = theOtherVertex(mesh.faces[fI2], v2, v1);
    mesh.faces[fI1] = Face(a, b, v2);
    mesh.faces[fI2] = Face(b, a, v1);
}

/// Flip the longest edge - works in our case
void removeNullFaceByEdgeFlip(Mesh& mesh, const FFTable& ffTable, const FIdx fI)
{
    auto& f = mesh.faces[fI];

    // flip the longest edge
    auto lab = math::length(mesh.vertices[f.a] - mesh.vertices[f.b]);
    auto lbc = math::length(mesh.vertices[f.b] - mesh.vertices[f.c]);
    auto lca = math::length(mesh.vertices[f.c] - mesh.vertices[f.a]);

    if (lab > lbc && lab > lca) { edgeFlip(mesh, ffTable, fI, f.a, f.b); }
    else
    {
        if (lbc > lca) { edgeFlip(mesh, ffTable, fI, f.b, f.c); }
        else { edgeFlip(mesh, ffTable, fI, f.c, f.a); }
    }
}

/// Check null face = vertices almost on one line
bool isNullFace(const Mesh& mesh, const FaceIdx fI)
{
    auto& f { mesh.faces[fI] };
    auto& a { mesh.vertices[f.a] };
    auto& b { mesh.vertices[f.b] };
    auto& c { mesh.vertices[f.c] };
    return math::pointLineDistance(c, math::Line3(a, b - a)) < NULL_FACE_THRESH;
}

bool attemptToRemoveNullFacesByEdgeFlip(Mesh& mesh)
{
    std::size_t flipped { 0 };
    auto ffTable = bs::ffTableNonManifold(mesh);
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++)
    {
        if (isNullFace(mesh, fI))
        {
            auto& f = mesh.faces[fI];
            removeNullFaceByEdgeFlip(mesh, ffTable, fI);
            ffTable = bs::ffTableNonManifold(mesh);
            ++flipped;
        }
    }
    LOG(info1) << "Flipped " << flipped
               << " edges while attemping to remove null faces.";
    return flipped > 0;
}

/// Remove faces that are adjacent to each other on all three sides
void removeSplicedFaces(Mesh& mesh)
{
    auto ffTable { bs::ffTableNonManifold(mesh) };
    std::vector<bool> removeFace(mesh.faces.size(), false);
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++)
    {
        std::map<FIdx, std::size_t> neighCnt;
        for (auto n : ffTable[fI])
        {
            if (neighCnt.count(n)) { neighCnt[n]++; }
            else { neighCnt[n] = 1; }
        }

        for (auto nc : neighCnt)
        {
            if (nc.second >= 3)
            {
                removeFace[fI] = true;
                removeFace[nc.first] = true;
            }
        }
    }

    Face::list newFaces;
    std::size_t count { 0 };
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++)
    {
        if (!removeFace[fI]) { newFaces.push_back(mesh.faces[fI]); }
        else { ++count; }
    }
    mesh.faces = newFaces;
    LOG(info1) << "Removed " << count << " spliced faces";
}


Mesh removeUnusedVertices(Mesh& mesh)
{
    Mesh res;
    std::vector<int> old2new(mesh.vertices.size(), -1);
    auto getNewIdx = [&](VIdx oldIdx) {
        if (old2new[oldIdx] != -1) { return old2new[oldIdx]; }
        old2new[oldIdx] = res.vertices.size();
        res.vertices.push_back(mesh.vertices[oldIdx]);
        return old2new[oldIdx];
    };

    for (auto& f : mesh.faces)
    {
        res.faces.emplace_back(getNewIdx(f.a), getNewIdx(f.b), getNewIdx(f.c));
    }
    LOG(info1) << "Removed " << mesh.vertices.size() - res.vertices.size()
               << " unused vertices";

    return res;
}

} // namespace

/**
 * Repairs mesh produced by circular/rectangular mesh generators so that the
 * topology is correct
 * 
 * - removes faces that are adjacent to each other from all three sides
 * - removes null faces (collinear vertices) by flipping their longest edge
 * - removes unused vertices
 *
 * @param[in,out] mesh
 */
void repairRoofMesh(geometry::Mesh& mesh)
{
    removeSplicedFaces(mesh);

    std::size_t iter { 0 };
    while (attemptToRemoveNullFacesByEdgeFlip(mesh))
    {
        ++iter;
        if (iter > MAX_EDGEFLIP_ITERS)
        {
            LOG(warn4) << "Unable to remove null faces by edge flipping - "
                          "might cause problems later.";
            break;
        }
    }
    LOG(info1) << "Removed null faces in " << iter << " iterations.";

    mesh = removeUnusedVertices(mesh);
}

} } // namespace semantic::lod2

