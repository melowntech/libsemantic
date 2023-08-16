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

#include <algorithm>
#include <queue>

#include "dbglog/dbglog.hpp"
#include "geometry/mesh.hpp"
#include "geometry/meshop.hpp"

#include "../../mesh.hpp"
#include "../detail.hpp"

#include "math/geometry.hpp"

/**
 * Remove null (zero-area) faces while preserving topology. Written specifically
 * for the case of lod2 roofs.
 */

namespace semantic
{
namespace lod2
{

namespace
{

using Mesh = geometry::Mesh;
using FFTable = geometry::FaceFaceTable;
using Face = geometry::Face;
using FIdx = std::size_t;
using VIdx = geometry::Face::index_type;

/// Min vertex distance from a line formed by two other vertices; should be
/// lower than vertexMergeEps
constexpr double NullFaceThresh { 1e-4 };

/// Max number of passes to remove null faces
constexpr std::size_t MaxEdgeflipIters { 100 };

constexpr double ZeroVolumeThresh { 1e-4 };

/// Checks if face contains edge v1-v2 (in this order)
bool hasEdge(const Face& f, const VIdx v1, const VIdx v2)
{
    return (f.a == v1 && f.b == v2) || (f.b == v1 && f.c == v2)
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

/// Check null face = vertices almost on one line
bool isNullFace(const Mesh& mesh, const Face& f)
{
    auto& a { mesh.vertices[f.a] };
    auto& b { mesh.vertices[f.b] };
    auto& c { mesh.vertices[f.c] };
    return math::pointLineDistance(c, math::Line3(a, b - a)) < NullFaceThresh;
}

/// Flip edge v1-v2 incident with face fI1
bool edgeFlip(Mesh& mesh,
              const FFTable& ffTable,
              const FIdx fI1,
              const VIdx v1,
              const VIdx v2)
{
    // find the opposite face
    FIdx fI2;
    bool found { false };
    for (const auto& n : ffTable[fI1]) {
        if (hasEdge(mesh.faces[n], v2, v1)) {
            fI2 = n;
            found = true;
            break;
        }
    }

    if (!found) {
        LOGTHROW(err4, std::runtime_error)
            << "Cannot find edge in any neighbouring faces.";
    }

    if (std::count(ffTable[fI1].begin(), ffTable[fI1].end(), fI2) > 1) {
        // the triangles share multiple edges (should not happen)
        return false;
    }

    // Verify that there are not any common neighbours between the two faces
    auto neigh1 { ffTable[fI1] };
    std::sort(neigh1.begin(), neigh1.end());
    auto neigh2 { ffTable[fI2] };
    std::sort(neigh2.begin(), neigh2.end());
    std::vector<std::size_t> commonNeighbours;
    std::set_intersection(neigh1.begin(),
                          neigh1.end(),
                          neigh2.begin(),
                          neigh2.end(),
                          std::back_inserter(commonNeighbours));
    if (commonNeighbours.size() > 0) { return false; }

    // Construct the new faces
    auto& f1 { mesh.faces[fI1] };
    auto& f2 { mesh.faces[fI2] };
    auto a { theOtherVertex(f1, v1, v2) };
    auto b { theOtherVertex(f2, v2, v1) };
    auto nf1 { Face(a, b, v2, f2.imageId) };
    auto nf2 { Face(b, a, v1, f2.imageId) };

    // Check they are not null
    if (isNullFace(mesh, nf1) || isNullFace(mesh, nf2)) { return false; }

    mesh.faces[fI1] = nf1;
    mesh.faces[fI2] = nf2;
    return true;
}

/// Flip the longest edge - works in our case
bool removeNullFaceByEdgeFlip(Mesh& mesh, const FFTable& ffTable, const FIdx fI)
{
    auto& f { mesh.faces[fI] };

    // flip the longest edge
    auto lab { math::length(mesh.vertices[f.a] - mesh.vertices[f.b]) };
    auto lbc { math::length(mesh.vertices[f.b] - mesh.vertices[f.c]) };
    auto lca { math::length(mesh.vertices[f.c] - mesh.vertices[f.a]) };

    if (lab > lbc && lab > lca) {
        return edgeFlip(mesh, ffTable, fI, f.a, f.b);
    } else {
        if (lbc > lca) { return edgeFlip(mesh, ffTable, fI, f.b, f.c); }
        else { return edgeFlip(mesh, ffTable, fI, f.c, f.a); }
    }
}


bool attemptToRemoveNullFacesByEdgeFlip(Mesh& mesh)
{
    std::size_t flipped { 0 };
    auto ffTable { geometry::getFaceFaceTableNonManifold(mesh) };
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++) {
        if (isNullFace(mesh, mesh.faces[fI])) {
            if (removeNullFaceByEdgeFlip(mesh, ffTable, fI)) {
                // meshes are small, OK to just recompute it all
                ffTable = geometry::getFaceFaceTableNonManifold(mesh);
                ++flipped;
            }
        }
    }
    LOG(info1) << "Flipped " << flipped
               << " edges while attempting to remove null faces.";
    return flipped > 0;
}

/// Remove faces that are adjacent to each other on all three sides
void removeSplicedFaces(Mesh& mesh)
{
    auto ffTable { geometry::getFaceFaceTableNonManifold(mesh) };
    std::vector<bool> removeFace(mesh.faces.size(), false);
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++) {
        std::map<FIdx, std::size_t> neighCnt;
        for (const auto& n : ffTable[fI]) {
            if (neighCnt.count(n)) { neighCnt[n]++; }
            else { neighCnt[n] = 1; }
        }

        for (const auto& nc : neighCnt) {
            if (nc.second >= 3) {
                removeFace[fI] = true;
                removeFace[nc.first] = true;
            }
        }
    }

    Face::list newFaces;
    std::size_t count { 0 };
    for (FIdx fI = 0; fI < mesh.faces.size(); fI++) {
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

    for (const auto& f : mesh.faces) {
        res.faces.emplace_back(getNewIdx(f.a),
                               getNewIdx(f.b),
                               getNewIdx(f.c),
                               f.imageId);
    }
    LOG(info1) << "Removed " << mesh.vertices.size() - res.vertices.size()
               << " unused vertices";

    return res;
}


void connectedFaces(const geometry::Mesh& mesh,
                    const geometry::EdgeMap& edgeMap,
                    const std::set<Face::index_type>& nonManifoldfaces,
                    std::vector<int>& regions,
                    std::vector<int>& validRegions)
{
    regions.resize(mesh.faces.size());
    std::fill(regions.begin(), regions.end(), -1);

    int comp { 0 };
    for (Face::index_type seed : nonManifoldfaces) {
        bool isValid = true;
        if (regions[seed] != -1) { continue; } // skip assigned
        regions[seed] = comp;                  // mark seed

        std::queue<Face::index_type> q; // faces added to comp in last round
        q.push(seed);

        while (!q.empty()) {
            auto fI { q.front() };
            q.pop();

            auto& face { mesh.faces[fI] };

            std::array<geometry::EdgeKey, 3> triEdges 
                = { geometry::EdgeKey(face.a, face.b),
                    geometry::EdgeKey(face.b, face.c),
                    geometry::EdgeKey(face.c, face.a) };

            for (const auto& e : triEdges) {
                // skip non-manifold edges
                if (edgeMap.at(e).size() > 2) { continue; }

                if (edgeMap.at(e).size() < 2) {
                    LOG(warn1) << "Edge has only one face."
                    << " Marking component " << comp << " as not watertight.";
                    isValid = false;
                }

                for (const auto& nI : edgeMap.at(e)) {
                    if (nI == fI) { continue; }
                    if (regions[nI] != -1) { continue; } // skip enqueued

                    // Add neighbor to component & enqueue it
                    regions[nI] = comp;
                    q.push(nI);
                }
            }
        }
        if (isValid) { validRegions.push_back(comp); }
        // no new faces to add
        ++comp;
    }
}


geometry::Mesh constructMeshPart(const geometry::Mesh& mesh, 
                                 const std::vector<int>& regions, 
                                 const int regionId)
{

    geometry::Mesh meshPart;
    math::Points3::size_type vertexId = 0;
    std::map<math::Point3, math::Points3::size_type> vertexIndexMap;

    for (int id = 0; id < (int)regions.size(); ++id) {
        if (regions[id] != regionId) { continue; }

        geometry::Face face = mesh.faces[id];

        auto getPtIdx = [&](const math::Point3& a){
            if (!vertexIndexMap.count(a)) {
                meshPart.vertices.push_back(a);
                vertexIndexMap[a] = vertexId;
                vertexId += 1;
            }
            return vertexIndexMap[a];
        };

        math::Point3 a = mesh.a(face);
        math::Point3 b = mesh.b(face);
        math::Point3 c = mesh.c(face);
        meshPart.addFace(getPtIdx(a), 
                         getPtIdx(b), 
                         getPtIdx(c), 
                         face.imageId);
    }

    return meshPart;
}


geometry::Mesh getValidComponents(const geometry::Mesh& mesh,
                                  const std::vector<int>& regions,
                                  const std::vector<int>& validRegions)
{
    geometry::Mesh validMesh;
    for (int regionId : validRegions) {
        geometry::Mesh meshPart(constructMeshPart(mesh, regions, regionId));
        if (meshPart.volume() < ZeroVolumeThresh) {
            LOG(info1) << "Mesh part " << regionId << " has zero volume. Removing.";
            continue;
        }
        detail::append(validMesh, meshPart);
    }
    return validMesh;
}


geometry::Mesh removeNon2ManifoldParts(const geometry::Mesh& mesh)
{
    geometry::EdgeMap edgeMap = getNonManifoldEdgeMap(mesh);

    // collect faces incident with non-manifold edge
    std::set<Face::index_type> nonManifoldfaces;
    for (const auto& edge : edgeMap) {
        if (edge.second.size() > 2) {
            for (const auto& fi : edge.second) {
                nonManifoldfaces.insert(fi);
            }
        }
    }

    if (nonManifoldfaces.empty()) { return mesh; }

    std::vector<int> regions;
    std::vector<int> validRegions;
    connectedFaces(mesh, edgeMap, nonManifoldfaces, regions, validRegions);
    geometry::Mesh validMesh = getValidComponents(mesh, regions, validRegions);

    LOG(info1) << "Removed " << mesh.faces.size() - validMesh.faces.size() << " faces "
    << "from non-manifold parts of mesh";

    return validMesh;
}

} // namespace

/**
 * Repairs mesh produced by circular/rectangular roof mesh generators so that
 * the topology is correct. Watertight mesh is required.
 *
 * - removes faces that are adjacent to each other from all three sides
 * - removes null faces (collinear vertices) by flipping their longest edge
 * - removes unused vertices
 * - removes non 2-manifold parts
 *
 * @param[in,out] mesh
 */
void repairRoofMesh(geometry::Mesh& mesh)
{
    removeSplicedFaces(mesh);

    std::size_t iter { 0 };
    while (attemptToRemoveNullFacesByEdgeFlip(mesh)) {
        ++iter;
        if (iter > MaxEdgeflipIters) {
            LOG(warn4) << "Unable to remove null faces by edge flipping - "
                          "might cause problems later.";
            break;
        }
    }
    LOG(info1) << "Removed null faces in " << iter << " iterations.";

    mesh = removeUnusedVertices(mesh);
    mesh = removeNon2ManifoldParts(mesh);
}

} // namespace lod2
} // namespace semantic
