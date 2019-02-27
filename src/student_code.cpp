#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
	  if (evaluatedLevels.size() == controlPoints.size()) {
		  return;
	  }
	  vector<Vector2D> currLevel = evaluatedLevels.back();
	  int size = currLevel.size() - 2;
	  vector<Vector2D> nextLevel;
	  for (int i = 0; i <= size; i++) {
		  Vector2D curvePt = (1 - t) * currLevel[i] + t * currLevel[i + 1]; //interpolate using t
		  nextLevel.push_back(curvePt);
	  }
	  evaluatedLevels.push_back(nextLevel);
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
	  vector <Vector3D> utemp;
	  for (int i = 0; i < controlPoints.size(); i++) {
		  Vector3D temp = evaluate1D(controlPoints[i], u);
		  utemp.push_back(temp);
	  }
	  Vector3D curr = evaluate1D(utemp, v);
	  return curr;
    //return Vector3D();
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
	  Vector3D temp = points[0] * pow(1 - t, 3) + points[1] * 3 * t*pow(1 - t, 2) + points[2] * 3 * pow(t, 2)*(1 - t) + points[3] * pow(t, 3);
	  return temp;
	//return Vector3D();
 }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
	  //return Vector3D();
	  Vector3D n(0, 0, 0); // initialize a vector to store your normal sum
	  HalfedgeCIter h = halfedge(); 
	  h = h->twin(); // Bump over
	  HalfedgeCIter h_orig = h;//halfedge();

	  do {
		  n += cross(-(h->vertex()->position), h->next()->next()->vertex()->position);
		  //cout << n << "\n";
		  h = h->next()->twin();
	  } while (h != h_orig);
	  return n.unit();
    //return Vector3D();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
	  if (e0->halfedge()->isBoundary() || e0->halfedge()->twin()->isBoundary()) { //if either halfedge is boundary edge
		  return e0;
		  //return;
	  }
	  //Halfedges
	  HalfedgeIter half0 = e0->halfedge();
	  HalfedgeIter half1 = half0->next();
	  HalfedgeIter half2 = half1->next();
	  HalfedgeIter half3 = half0->twin();
	  HalfedgeIter half4 = half3->next();
	  HalfedgeIter half5 = half4->next();
	  HalfedgeIter half6 = half1->twin();
	  HalfedgeIter half7 = half2->twin();
	  HalfedgeIter half8 = half4->twin();
	  HalfedgeIter half9 = half5->twin();
	  //Vertices
	  VertexIter v0 = half0->vertex();
	  VertexIter v1 = half3->vertex();
	  VertexIter v2 = half2->vertex();
	  VertexIter v3 = half5->vertex();
	  //Edges
	  EdgeIter e1 = half1->edge();
	  EdgeIter e2 = half2->edge();
	  EdgeIter e3 = half4->edge();
	  EdgeIter e4 = half5->edge();
	  //Faces
	  FaceIter f0 = half0->face();
	  FaceIter f1 = half3->face();

	  //Reset:
	  //Halfedges
	  half0->setNeighbors(half1, half3, v3, e0, f0);
	  half1->setNeighbors(half2, half7, v2, e2, f0);
	  half2->setNeighbors(half0, half8, v0, e3, f0);
	  half3->setNeighbors(half4, half0, v2, e0, f1);
	  half4->setNeighbors(half5, half9, v3, e4, f1);
	  half5->setNeighbors(half3, half6, v1, e1, f1);
	  half6->setNeighbors(half6->next(), half5, v2, e1, half6->face());
	  half7->setNeighbors(half7->next(), half1, v0, e2, half7->face());
	  half8->setNeighbors(half8->next(), half2, v3, e3, half8->face());
	  half9->setNeighbors(half9->next(), half4, v1, e4, half9->face());
	  //Vertices
	  v0->halfedge() = half2;
	  v1->halfedge() = half5;
	  v2->halfedge() = half3;
	  v3->halfedge() = half0;
	  //v2->halfedge() = half1;
	  //v3->halfedge() = half4;
	  //Edges
	  e0->halfedge() = half0;
	  e1->halfedge() = half5;
	  e2->halfedge() = half1;
	  e3->halfedge() = half2;
	  e4->halfedge() = half4;
	  //Faces
	  f0->halfedge() = half0;
	  f1->halfedge() = half3;

	  return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
	  if (e0->halfedge()->isBoundary() || e0->halfedge()->twin()->isBoundary()) { //if either halfedge is boundary edge
		  return e0->halfedge()->vertex();
	  }
	  //Halfedges
	  HalfedgeIter half0 = e0->halfedge();
	  HalfedgeIter half1 = half0->next();
	  HalfedgeIter half2 = half1->next();
	  HalfedgeIter half3 = half0->twin();
	  HalfedgeIter half4 = half3->next();
	  HalfedgeIter half5 = half4->next();
	  HalfedgeIter half6 = half1->twin();
	  HalfedgeIter half7 = half2->twin();
	  HalfedgeIter half8 = half4->twin();
	  HalfedgeIter half9 = half5->twin();
	  //Vertices
	  VertexIter v0 = half0->vertex();
	  VertexIter v1 = half3->vertex();
	  VertexIter v2 = half2->vertex();
	  VertexIter v3 = half5->vertex();
	  //Edges
	  EdgeIter e1 = half1->edge();
	  EdgeIter e2 = half2->edge();
	  EdgeIter e3 = half4->edge();
	  EdgeIter e4 = half5->edge();
	  //Faces
	  FaceIter f0 = half0->face();
	  FaceIter f1 = half3->face();

	  //New vertex
	  VertexIter v4 = newVertex();
	  v4->position = 0.5*(v1->position + v0->position);
	  v4->isNew = true;
	  //New edges
	  EdgeIter e5 = newEdge();
	  //e5->isNew = true;
	  EdgeIter e6 = newEdge();
	  //e6->isNew = true;
	  EdgeIter e7 = newEdge();
	  //New halfedges
	  HalfedgeIter half10 = newHalfedge();
	  HalfedgeIter half11 = newHalfedge();
	  HalfedgeIter half12 = newHalfedge();
	  HalfedgeIter half13 = newHalfedge();
	  HalfedgeIter half14 = newHalfedge();
	  HalfedgeIter half15 = newHalfedge();
	  //New faces
	  FaceIter f2 = newFace();
	  FaceIter f3 = newFace();

	  //Reset:
	  //Halfedges:
	  half0->setNeighbors(half1, half3, v0, e0, f0);
	  half1->setNeighbors(half2, half12, v4, e6, f0);
	  half2->setNeighbors(half0, half7, v2, e2, f0);
	  half3->setNeighbors(half4, half0, v4, e0, f1);
	  half4->setNeighbors(half5, half8, v0, e3, f1);
	  half5->setNeighbors(half3, half14, v3, e5, f1);
	  half6->setNeighbors(half6->next(), half11, v2, e1, half6->face());
	  half7->setNeighbors(half7->next(), half2, v0, e2, half7->face());
	  half8->setNeighbors(half8->next(), half4, v3, e3, half8->face());
	  half9->setNeighbors(half9->next(), half15, v1, e4, half9->face());
	  half10->setNeighbors(half11, half13, v4, e7, f2);
	  half11->setNeighbors(half12, half6, v1, e1, f2);
	  half12->setNeighbors(half10, half1, v2, e6, f2);
	  half13->setNeighbors(half14, half10, v1, e7, f3);
	  half14->setNeighbors(half15, half5, v4, e5, f3);
	  half15->setNeighbors(half13, half9, v3, e4, f3);
	  //Vertices
	  v0->halfedge() = half0;
	  v1->halfedge() = half13;
	  v2->halfedge() = half12;
	  v3->halfedge() = half5;
	  v4->halfedge() = half3;
	  //Edges
	  e0->halfedge() = half0;
	  e1->halfedge() = half11;
	  e2->halfedge() = half2;
	  e3->halfedge() = half4;
	  e4->halfedge() = half15;
	  e5->halfedge() = half14;
	  e6->halfedge() = half12;
	  e7->halfedge() = half10;
	  //Faces
	  f0->halfedge() = half2;
	  f1->halfedge() = half4;
	  f2->halfedge() = half11;
	  f3->halfedge() = half15;

      return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.

	  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
		  v->isNew = false;
		  //Calculate newPosition
		  Size n = v->degree();

		  float u = 3.0f / (8.0f * (float)n);
		  if ((int)n == 3) {
			  u = 3.0f / 16.0f;
		  }
		  HalfedgeCIter half = v->halfedge();
		  HalfedgeCIter half_copy = half;
		  Vector3D neighbor_sum(0.0f, 0.0f, 0.0f);
		  do {
			  neighbor_sum += half->next()->vertex()->position;
			  half = half->next()->next()->twin();
		  } while (half != half_copy);
		  v->newPosition = (1.0f - (float)n * u) * v->position + (u * neighbor_sum);

		  //cout << "Calculating, old vertex position is: " << v->newPosition << " " << v->position << "\n";
	  }

    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
	  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
		  //Splitting edge AB, flanked by C, D: 3/8 * (A + B) + 1/8 * (C + D)
		  Vector3D aPos = e->halfedge()->vertex()->position;
		  Vector3D bPos = e->halfedge()->twin()->vertex()->position;
		  Vector3D cPos = e->halfedge()->next()->next()->vertex()->position;
		  Vector3D dPos = e->halfedge()->twin()->next()->next()->vertex()->position;
		  e->newPosition = (3.0f / 8.0f) * (aPos + bPos) + (1.0f / 8.0f) * (cPos + dPos);
		  //cout << "Calculating, new vertex position is: " << e->newPosition << "\n";
	  }

    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)
	  int numEdges = mesh.nEdges();
	  EdgeIter e = mesh.edgesBegin();
	  for (int i = 0; i < numEdges; i++) {
		  EdgeIter e_next = e;
		  e_next++;
		  Vector3D newPos = e->newPosition;
		  VertexIter newV = mesh.splitEdge(e);
		  newV->halfedge()->edge()->isNew = false;
		  newV->halfedge()->twin()->next()->twin()->next()->edge()->isNew = false;
		  newV->halfedge()->next()->next()->edge()->isNew = true;
		  newV->halfedge()->twin()->next()->edge()->isNew = true;
		  //newV->isNew = true;
		  newV->newPosition = newPos;
		  e = e_next;
	  }

   // // TODO Now flip any new edge that connects an old and new vertex.
	  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
		  if (e->isNew) { //if new edge
			  //if ((e->halfedge()->vertex()->isNew && !(e->halfedge()->twin()->vertex()->isNew)) || (!(e->halfedge()->vertex()->isNew) && e->halfedge()->twin()->vertex()->isNew)) {
			  if (e->halfedge()->vertex()->isNew != e->halfedge()->twin()->vertex()->isNew) {
					mesh.flipEdge(e);
			  }
		  }
	  }

   // // TODO Finally, copy the new vertex positions into final Vertex::position.
	  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
		  v->position = v->newPosition;
	  }

    return;
  }
}
