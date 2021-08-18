import {
	B2_maxFloat,
	B2_angularSlop,
	B2_maxPolygonVertices,
	B2_polygonRadius,
	B2_maxManifoldPoints
} from '../Common/b2Settings'
import { B2Min, B2Vec2, B2Rot, B2Transform } from '../Common/b2Math'
import { B2ContactFeatureType, B2ContactID } from './b2Collision'
import {
	B2Manifold,
	B2ManifoldType,
	B2ManifoldPoint,
	B2ClipVertex,
	B2ClipSegmentToLine
} from './b2Collision'
import { B2CircleShape } from './Shapes/b2CircleShape'
import { B2PolygonShape } from './Shapes/b2PolygonShape'
import { B2EdgeShape } from './Shapes/b2EdgeShape'

const B2CollideEdgeAndCircle_s_Q: B2Vec2 = new B2Vec2()
const B2CollideEdgeAndCircle_s_e: B2Vec2 = new B2Vec2()
const B2CollideEdgeAndCircle_s_d: B2Vec2 = new B2Vec2()
const B2CollideEdgeAndCircle_s_e1: B2Vec2 = new B2Vec2()
const B2CollideEdgeAndCircle_s_e2: B2Vec2 = new B2Vec2()
const B2CollideEdgeAndCircle_s_P: B2Vec2 = new B2Vec2()
const B2CollideEdgeAndCircle_s_n: B2Vec2 = new B2Vec2()
const B2CollideEdgeAndCircle_s_id: B2ContactID = new B2ContactID()

export function B2CollideEdgeAndCircle(
	manifold: B2Manifold,
	edgeA: B2EdgeShape,
	xfA: B2Transform,
	circleB: B2CircleShape,
	xfB: B2Transform
): void {
	manifold.pointCount = 0

	// Compute circle in frame of edge
	const Q: B2Vec2 = B2Transform.MulTXV(
		xfA,
		B2Transform.MulXV(xfB, circleB.m_p, B2Vec2.s_t0),
		B2CollideEdgeAndCircle_s_Q
	)

	const A: B2Vec2 = edgeA.m_vertex1
	const B: B2Vec2 = edgeA.m_vertex2
	const e: B2Vec2 = B2Vec2.SubVV(B, A, B2CollideEdgeAndCircle_s_e)

	// Barycentric coordinates
	const u: number = B2Vec2.DotVV(e, B2Vec2.SubVV(B, Q, B2Vec2.s_t0))
	const v: number = B2Vec2.DotVV(e, B2Vec2.SubVV(Q, A, B2Vec2.s_t0))

	const radius: number = edgeA.m_radius + circleB.m_radius

	// const cf: B2ContactFeature = new B2ContactFeature();
	const id: B2ContactID = B2CollideEdgeAndCircle_s_id
	id.cf.indexB = 0
	id.cf.typeB = B2ContactFeatureType.e_vertex

	// Region A
	if (v <= 0) {
		const P: B2Vec2 = A
		const d: B2Vec2 = B2Vec2.SubVV(Q, P, B2CollideEdgeAndCircle_s_d)
		const dd: number = B2Vec2.DotVV(d, d)
		if (dd > radius * radius) {
			return
		}

		// Is there an edge connected to A?
		if (edgeA.m_hasVertex0) {
			const A1: B2Vec2 = edgeA.m_vertex0
			const B1: B2Vec2 = A
			const e1: B2Vec2 = B2Vec2.SubVV(B1, A1, B2CollideEdgeAndCircle_s_e1)
			const u1: number = B2Vec2.DotVV(e1, B2Vec2.SubVV(B1, Q, B2Vec2.s_t0))

			// Is the circle in Region AB of the previous edge?
			if (u1 > 0) {
				return
			}
		}

		id.cf.indexA = 0
		id.cf.typeA = B2ContactFeatureType.e_vertex
		manifold.pointCount = 1
		manifold.type = B2ManifoldType.e_circles
		manifold.localNormal.SetZero()
		manifold.localPoint.Copy(P)
		manifold.points[0].id.Copy(id)
		// manifold.points[0].id.key = 0;
		// manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint.Copy(circleB.m_p)
		return
	}

	// Region B
	if (u <= 0) {
		const P: B2Vec2 = B
		const d: B2Vec2 = B2Vec2.SubVV(Q, P, B2CollideEdgeAndCircle_s_d)
		const dd: number = B2Vec2.DotVV(d, d)
		if (dd > radius * radius) {
			return
		}

		// Is there an edge connected to B?
		if (edgeA.m_hasVertex3) {
			const B2: B2Vec2 = edgeA.m_vertex3
			const A2: B2Vec2 = B
			const e2: B2Vec2 = B2Vec2.SubVV(B2, A2, B2CollideEdgeAndCircle_s_e2)
			const v2: number = B2Vec2.DotVV(e2, B2Vec2.SubVV(Q, A2, B2Vec2.s_t0))

			// Is the circle in Region AB of the next edge?
			if (v2 > 0) {
				return
			}
		}

		id.cf.indexA = 1
		id.cf.typeA = B2ContactFeatureType.e_vertex
		manifold.pointCount = 1
		manifold.type = B2ManifoldType.e_circles
		manifold.localNormal.SetZero()
		manifold.localPoint.Copy(P)
		manifold.points[0].id.Copy(id)
		// manifold.points[0].id.key = 0;
		// manifold.points[0].id.cf = cf;
		manifold.points[0].localPoint.Copy(circleB.m_p)
		return
	}

	// Region AB
	const den: number = B2Vec2.DotVV(e, e)
	/// b2Assert(den > 0);
	const P: B2Vec2 = B2CollideEdgeAndCircle_s_P
	P.x = (1 / den) * (u * A.x + v * B.x)
	P.y = (1 / den) * (u * A.y + v * B.y)
	const d: B2Vec2 = B2Vec2.SubVV(Q, P, B2CollideEdgeAndCircle_s_d)
	const dd: number = B2Vec2.DotVV(d, d)
	if (dd > radius * radius) {
		return
	}

	const n: B2Vec2 = B2CollideEdgeAndCircle_s_n.Set(-e.y, e.x)
	if (B2Vec2.DotVV(n, B2Vec2.SubVV(Q, A, B2Vec2.s_t0)) < 0) {
		n.Set(-n.x, -n.y)
	}
	n.Normalize()

	id.cf.indexA = 0
	id.cf.typeA = B2ContactFeatureType.e_face
	manifold.pointCount = 1
	manifold.type = B2ManifoldType.e_faceA
	manifold.localNormal.Copy(n)
	manifold.localPoint.Copy(A)
	manifold.points[0].id.Copy(id)
	// manifold.points[0].id.key = 0;
	// manifold.points[0].id.cf = cf;
	manifold.points[0].localPoint.Copy(circleB.m_p)
}

const enum B2EPAxisType {
	e_unknown = 0,
	e_edgeA = 1,
	e_edgeB = 2
}

class B2EPAxis {
	public type: B2EPAxisType = B2EPAxisType.e_unknown
	public index: number = 0
	public separation: number = 0
}

class B2TempPolygon {
	public vertices: B2Vec2[] = B2Vec2.MakeArray(B2_maxPolygonVertices)
	public normals: B2Vec2[] = B2Vec2.MakeArray(B2_maxPolygonVertices)
	public count: number = 0
}

class B2ReferenceFace {
	public i1: number = 0
	public i2: number = 0
	public v1: B2Vec2 = new B2Vec2()
	public v2: B2Vec2 = new B2Vec2()
	public normal: B2Vec2 = new B2Vec2()
	public sideNormal1: B2Vec2 = new B2Vec2()
	public sideOffset1: number = 0
	public sideNormal2: B2Vec2 = new B2Vec2()
	public sideOffset2: number = 0
}

const enum B2EPColliderVertexType {
	e_isolated = 0,
	e_concave = 1,
	e_convex = 2
}

class B2EPCollider {
	public m_polygonB: B2TempPolygon = new B2TempPolygon()
	public m_xf: B2Transform = new B2Transform()
	public m_centroidB: B2Vec2 = new B2Vec2()
	public m_v0: B2Vec2 = new B2Vec2()
	public m_v1: B2Vec2 = new B2Vec2()
	public m_v2: B2Vec2 = new B2Vec2()
	public m_v3: B2Vec2 = new B2Vec2()
	public m_normal0: B2Vec2 = new B2Vec2()
	public m_normal1: B2Vec2 = new B2Vec2()
	public m_normal2: B2Vec2 = new B2Vec2()
	public m_normal: B2Vec2 = new B2Vec2()
	public m_type1 = B2EPColliderVertexType.e_isolated
	public m_type2 = B2EPColliderVertexType.e_isolated
	public m_lowerLimit: B2Vec2 = new B2Vec2()
	public m_upperLimit: B2Vec2 = new B2Vec2()
	public m_radius: number = 0
	public m_front: boolean = false

	private static s_edge1 = new B2Vec2()
	private static s_edge0 = new B2Vec2()
	private static s_edge2 = new B2Vec2()
	private static s_ie = B2ClipVertex.MakeArray(2)
	private static s_rf = new B2ReferenceFace()
	private static s_clipPoints1 = B2ClipVertex.MakeArray(2)
	private static s_clipPoints2 = B2ClipVertex.MakeArray(2)
	private static s_edgeAxis = new B2EPAxis()
	private static s_polygonAxis = new B2EPAxis()

	public Collide(
		manifold: B2Manifold,
		edgeA: B2EdgeShape,
		xfA: B2Transform,
		polygonB: B2PolygonShape,
		xfB: B2Transform
	): void {
		B2Transform.MulTXX(xfA, xfB, this.m_xf)

		B2Transform.MulXV(this.m_xf, polygonB.m_centroid, this.m_centroidB)

		this.m_v0.Copy(edgeA.m_vertex0)
		this.m_v1.Copy(edgeA.m_vertex1)
		this.m_v2.Copy(edgeA.m_vertex2)
		this.m_v3.Copy(edgeA.m_vertex3)

		const hasVertex0: boolean = edgeA.m_hasVertex0
		const hasVertex3: boolean = edgeA.m_hasVertex3

		const edge1: B2Vec2 = B2Vec2.SubVV(
			this.m_v2,
			this.m_v1,
			B2EPCollider.s_edge1
		)
		edge1.Normalize()
		this.m_normal1.Set(edge1.y, -edge1.x)
		const offset1: number = B2Vec2.DotVV(
			this.m_normal1,
			B2Vec2.SubVV(this.m_centroidB, this.m_v1, B2Vec2.s_t0)
		)
		let offset0: number = 0
		let offset2: number = 0
		let convex1: boolean = false
		let convex2: boolean = false

		// Is there a preceding edge?
		if (hasVertex0) {
			const edge0: B2Vec2 = B2Vec2.SubVV(
				this.m_v1,
				this.m_v0,
				B2EPCollider.s_edge0
			)
			edge0.Normalize()
			this.m_normal0.Set(edge0.y, -edge0.x)
			convex1 = B2Vec2.CrossVV(edge0, edge1) >= 0
			offset0 = B2Vec2.DotVV(
				this.m_normal0,
				B2Vec2.SubVV(this.m_centroidB, this.m_v0, B2Vec2.s_t0)
			)
		}

		// Is there a following edge?
		if (hasVertex3) {
			const edge2: B2Vec2 = B2Vec2.SubVV(
				this.m_v3,
				this.m_v2,
				B2EPCollider.s_edge2
			)
			edge2.Normalize()
			this.m_normal2.Set(edge2.y, -edge2.x)
			convex2 = B2Vec2.CrossVV(edge1, edge2) > 0
			offset2 = B2Vec2.DotVV(
				this.m_normal2,
				B2Vec2.SubVV(this.m_centroidB, this.m_v2, B2Vec2.s_t0)
			)
		}

		// Determine front or back collision. Determine collision normal limits.
		if (hasVertex0 && hasVertex3) {
			if (convex1 && convex2) {
				this.m_front = offset0 >= 0 || offset1 >= 0 || offset2 >= 0
				if (this.m_front) {
					this.m_normal.Copy(this.m_normal1)
					this.m_lowerLimit.Copy(this.m_normal0)
					this.m_upperLimit.Copy(this.m_normal2)
				} else {
					this.m_normal.Copy(this.m_normal1).SelfNeg()
					this.m_lowerLimit.Copy(this.m_normal1).SelfNeg()
					this.m_upperLimit.Copy(this.m_normal1).SelfNeg()
				}
			} else if (convex1) {
				this.m_front = offset0 >= 0 || (offset1 >= 0 && offset2 >= 0)
				if (this.m_front) {
					this.m_normal.Copy(this.m_normal1)
					this.m_lowerLimit.Copy(this.m_normal0)
					this.m_upperLimit.Copy(this.m_normal1)
				} else {
					this.m_normal.Copy(this.m_normal1).SelfNeg()
					this.m_lowerLimit.Copy(this.m_normal2).SelfNeg()
					this.m_upperLimit.Copy(this.m_normal1).SelfNeg()
				}
			} else if (convex2) {
				this.m_front = offset2 >= 0 || (offset0 >= 0 && offset1 >= 0)
				if (this.m_front) {
					this.m_normal.Copy(this.m_normal1)
					this.m_lowerLimit.Copy(this.m_normal1)
					this.m_upperLimit.Copy(this.m_normal2)
				} else {
					this.m_normal.Copy(this.m_normal1).SelfNeg()
					this.m_lowerLimit.Copy(this.m_normal1).SelfNeg()
					this.m_upperLimit.Copy(this.m_normal0).SelfNeg()
				}
			} else {
				this.m_front = offset0 >= 0 && offset1 >= 0 && offset2 >= 0
				if (this.m_front) {
					this.m_normal.Copy(this.m_normal1)
					this.m_lowerLimit.Copy(this.m_normal1)
					this.m_upperLimit.Copy(this.m_normal1)
				} else {
					this.m_normal.Copy(this.m_normal1).SelfNeg()
					this.m_lowerLimit.Copy(this.m_normal2).SelfNeg()
					this.m_upperLimit.Copy(this.m_normal0).SelfNeg()
				}
			}
		} else if (hasVertex0) {
			if (convex1) {
				this.m_front = offset0 >= 0 || offset1 >= 0
				if (this.m_front) {
					this.m_normal.Copy(this.m_normal1)
					this.m_lowerLimit.Copy(this.m_normal0)
					this.m_upperLimit.Copy(this.m_normal1).SelfNeg()
				} else {
					this.m_normal.Copy(this.m_normal1).SelfNeg()
					this.m_lowerLimit.Copy(this.m_normal1)
					this.m_upperLimit.Copy(this.m_normal1).SelfNeg()
				}
			} else {
				this.m_front = offset0 >= 0 && offset1 >= 0
				if (this.m_front) {
					this.m_normal.Copy(this.m_normal1)
					this.m_lowerLimit.Copy(this.m_normal1)
					this.m_upperLimit.Copy(this.m_normal1).SelfNeg()
				} else {
					this.m_normal.Copy(this.m_normal1).SelfNeg()
					this.m_lowerLimit.Copy(this.m_normal1)
					this.m_upperLimit.Copy(this.m_normal0).SelfNeg()
				}
			}
		} else if (hasVertex3) {
			if (convex2) {
				this.m_front = offset1 >= 0 || offset2 >= 0
				if (this.m_front) {
					this.m_normal.Copy(this.m_normal1)
					this.m_lowerLimit.Copy(this.m_normal1).SelfNeg()
					this.m_upperLimit.Copy(this.m_normal2)
				} else {
					this.m_normal.Copy(this.m_normal1).SelfNeg()
					this.m_lowerLimit.Copy(this.m_normal1).SelfNeg()
					this.m_upperLimit.Copy(this.m_normal1)
				}
			} else {
				this.m_front = offset1 >= 0 && offset2 >= 0
				if (this.m_front) {
					this.m_normal.Copy(this.m_normal1)
					this.m_lowerLimit.Copy(this.m_normal1).SelfNeg()
					this.m_upperLimit.Copy(this.m_normal1)
				} else {
					this.m_normal.Copy(this.m_normal1).SelfNeg()
					this.m_lowerLimit.Copy(this.m_normal2).SelfNeg()
					this.m_upperLimit.Copy(this.m_normal1)
				}
			}
		} else {
			this.m_front = offset1 >= 0
			if (this.m_front) {
				this.m_normal.Copy(this.m_normal1)
				this.m_lowerLimit.Copy(this.m_normal1).SelfNeg()
				this.m_upperLimit.Copy(this.m_normal1).SelfNeg()
			} else {
				this.m_normal.Copy(this.m_normal1).SelfNeg()
				this.m_lowerLimit.Copy(this.m_normal1)
				this.m_upperLimit.Copy(this.m_normal1)
			}
		}

		// Get polygonB in frameA
		this.m_polygonB.count = polygonB.m_count
		for (let i: number = 0; i < polygonB.m_count; ++i) {
			B2Transform.MulXV(
				this.m_xf,
				polygonB.m_vertices[i],
				this.m_polygonB.vertices[i]
			)
			B2Rot.MulRV(
				this.m_xf.q,
				polygonB.m_normals[i],
				this.m_polygonB.normals[i]
			)
		}

		this.m_radius = 2 * B2_polygonRadius

		manifold.pointCount = 0

		const edgeAxis: B2EPAxis = this.ComputeEdgeSeparation(
			B2EPCollider.s_edgeAxis
		)

		// If no valid normal can be found than this edge should not collide.
		if (edgeAxis.type === B2EPAxisType.e_unknown) {
			return
		}

		if (edgeAxis.separation > this.m_radius) {
			return
		}

		const polygonAxis: B2EPAxis = this.ComputePolygonSeparation(
			B2EPCollider.s_polygonAxis
		)
		if (
			polygonAxis.type !== B2EPAxisType.e_unknown &&
			polygonAxis.separation > this.m_radius
		) {
			return
		}

		// Use hysteresis for jitter reduction.
		const k_relativeTol: number = 0.98
		const k_absoluteTol: number = 0.001

		let primaryAxis: B2EPAxis
		if (polygonAxis.type === B2EPAxisType.e_unknown) {
			primaryAxis = edgeAxis
		} else if (
			polygonAxis.separation >
			k_relativeTol * edgeAxis.separation + k_absoluteTol
		) {
			primaryAxis = polygonAxis
		} else {
			primaryAxis = edgeAxis
		}

		const ie: B2ClipVertex[] = B2EPCollider.s_ie
		const rf: B2ReferenceFace = B2EPCollider.s_rf
		if (primaryAxis.type === B2EPAxisType.e_edgeA) {
			manifold.type = B2ManifoldType.e_faceA

			// Search for the polygon normal that is most anti-parallel to the edge normal.
			let bestIndex: number = 0
			let bestValue: number = B2Vec2.DotVV(
				this.m_normal,
				this.m_polygonB.normals[0]
			)
			for (let i: number = 1; i < this.m_polygonB.count; ++i) {
				const value: number = B2Vec2.DotVV(
					this.m_normal,
					this.m_polygonB.normals[i]
				)
				if (value < bestValue) {
					bestValue = value
					bestIndex = i
				}
			}

			const i1: number = bestIndex
			const i2: number = (i1 + 1) % this.m_polygonB.count

			const ie0: B2ClipVertex = ie[0]
			ie0.v.Copy(this.m_polygonB.vertices[i1])
			ie0.id.cf.indexA = 0
			ie0.id.cf.indexB = i1
			ie0.id.cf.typeA = B2ContactFeatureType.e_face
			ie0.id.cf.typeB = B2ContactFeatureType.e_vertex

			const ie1: B2ClipVertex = ie[1]
			ie1.v.Copy(this.m_polygonB.vertices[i2])
			ie1.id.cf.indexA = 0
			ie1.id.cf.indexB = i2
			ie1.id.cf.typeA = B2ContactFeatureType.e_face
			ie1.id.cf.typeB = B2ContactFeatureType.e_vertex

			if (this.m_front) {
				rf.i1 = 0
				rf.i2 = 1
				rf.v1.Copy(this.m_v1)
				rf.v2.Copy(this.m_v2)
				rf.normal.Copy(this.m_normal1)
			} else {
				rf.i1 = 1
				rf.i2 = 0
				rf.v1.Copy(this.m_v2)
				rf.v2.Copy(this.m_v1)
				rf.normal.Copy(this.m_normal1).SelfNeg()
			}
		} else {
			manifold.type = B2ManifoldType.e_faceB

			const ie0: B2ClipVertex = ie[0]
			ie0.v.Copy(this.m_v1)
			ie0.id.cf.indexA = 0
			ie0.id.cf.indexB = primaryAxis.index
			ie0.id.cf.typeA = B2ContactFeatureType.e_vertex
			ie0.id.cf.typeB = B2ContactFeatureType.e_face

			const ie1: B2ClipVertex = ie[1]
			ie1.v.Copy(this.m_v2)
			ie1.id.cf.indexA = 0
			ie1.id.cf.indexB = primaryAxis.index
			ie1.id.cf.typeA = B2ContactFeatureType.e_vertex
			ie1.id.cf.typeB = B2ContactFeatureType.e_face

			rf.i1 = primaryAxis.index
			rf.i2 = (rf.i1 + 1) % this.m_polygonB.count
			rf.v1.Copy(this.m_polygonB.vertices[rf.i1])
			rf.v2.Copy(this.m_polygonB.vertices[rf.i2])
			rf.normal.Copy(this.m_polygonB.normals[rf.i1])
		}

		rf.sideNormal1.Set(rf.normal.y, -rf.normal.x)
		rf.sideNormal2.Copy(rf.sideNormal1).SelfNeg()
		rf.sideOffset1 = B2Vec2.DotVV(rf.sideNormal1, rf.v1)
		rf.sideOffset2 = B2Vec2.DotVV(rf.sideNormal2, rf.v2)

		// Clip incident edge against extruded edge1 side edges.
		const clipPoints1: B2ClipVertex[] = B2EPCollider.s_clipPoints1
		const clipPoints2: B2ClipVertex[] = B2EPCollider.s_clipPoints2
		let np: number = 0

		// Clip to box side 1
		np = B2ClipSegmentToLine(
			clipPoints1,
			ie,
			rf.sideNormal1,
			rf.sideOffset1,
			rf.i1
		)

		if (np < B2_maxManifoldPoints) {
			return
		}

		// Clip to negative box side 1
		np = B2ClipSegmentToLine(
			clipPoints2,
			clipPoints1,
			rf.sideNormal2,
			rf.sideOffset2,
			rf.i2
		)

		if (np < B2_maxManifoldPoints) {
			return
		}

		// Now clipPoints2 contains the clipped points.
		if (primaryAxis.type === B2EPAxisType.e_edgeA) {
			manifold.localNormal.Copy(rf.normal)
			manifold.localPoint.Copy(rf.v1)
		} else {
			manifold.localNormal.Copy(polygonB.m_normals[rf.i1])
			manifold.localPoint.Copy(polygonB.m_vertices[rf.i1])
		}

		let pointCount: number = 0
		for (let i: number = 0; i < B2_maxManifoldPoints; ++i) {
			let separation: number

			separation = B2Vec2.DotVV(
				rf.normal,
				B2Vec2.SubVV(clipPoints2[i].v, rf.v1, B2Vec2.s_t0)
			)

			if (separation <= this.m_radius) {
				const cp: B2ManifoldPoint = manifold.points[pointCount]

				if (primaryAxis.type === B2EPAxisType.e_edgeA) {
					B2Transform.MulTXV(this.m_xf, clipPoints2[i].v, cp.localPoint)
					cp.id = clipPoints2[i].id
				} else {
					cp.localPoint.Copy(clipPoints2[i].v)
					cp.id.cf.typeA = clipPoints2[i].id.cf.typeB
					cp.id.cf.typeB = clipPoints2[i].id.cf.typeA
					cp.id.cf.indexA = clipPoints2[i].id.cf.indexB
					cp.id.cf.indexB = clipPoints2[i].id.cf.indexA
				}

				++pointCount
			}
		}

		manifold.pointCount = pointCount
	}

	public ComputeEdgeSeparation(out: B2EPAxis): B2EPAxis {
		const axis: B2EPAxis = out
		axis.type = B2EPAxisType.e_edgeA
		axis.index = this.m_front ? 0 : 1
		axis.separation = B2_maxFloat

		for (let i: number = 0; i < this.m_polygonB.count; ++i) {
			const s: number = B2Vec2.DotVV(
				this.m_normal,
				B2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v1, B2Vec2.s_t0)
			)
			if (s < axis.separation) {
				axis.separation = s
			}
		}

		return axis
	}

	private static s_n = new B2Vec2()
	private static s_perp = new B2Vec2()

	public ComputePolygonSeparation(out: B2EPAxis): B2EPAxis {
		const axis: B2EPAxis = out
		axis.type = B2EPAxisType.e_unknown
		axis.index = -1
		axis.separation = -B2_maxFloat

		const perp: B2Vec2 = B2EPCollider.s_perp.Set(
			-this.m_normal.y,
			this.m_normal.x
		)

		for (let i: number = 0; i < this.m_polygonB.count; ++i) {
			const n: B2Vec2 = B2Vec2.NegV(
				this.m_polygonB.normals[i],
				B2EPCollider.s_n
			)

			const s1: number = B2Vec2.DotVV(
				n,
				B2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v1, B2Vec2.s_t0)
			)
			const s2: number = B2Vec2.DotVV(
				n,
				B2Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v2, B2Vec2.s_t0)
			)
			const s: number = B2Min(s1, s2)

			if (s > this.m_radius) {
				// No collision
				axis.type = B2EPAxisType.e_edgeB
				axis.index = i
				axis.separation = s
				return axis
			}

			// Adjacency
			if (B2Vec2.DotVV(n, perp) >= 0) {
				if (
					B2Vec2.DotVV(
						B2Vec2.SubVV(n, this.m_upperLimit, B2Vec2.s_t0),
						this.m_normal
					) < -B2_angularSlop
				) {
					continue
				}
			} else {
				if (
					B2Vec2.DotVV(
						B2Vec2.SubVV(n, this.m_lowerLimit, B2Vec2.s_t0),
						this.m_normal
					) < -B2_angularSlop
				) {
					continue
				}
			}

			if (s > axis.separation) {
				axis.type = B2EPAxisType.e_edgeB
				axis.index = i
				axis.separation = s
			}
		}

		return axis
	}
}

const B2CollideEdgeAndPolygon_s_collider: B2EPCollider = new B2EPCollider()

export function B2CollideEdgeAndPolygon(
	manifold: B2Manifold,
	edgeA: B2EdgeShape,
	xfA: B2Transform,
	polygonB: B2PolygonShape,
	xfB: B2Transform
): void {
	const collider: B2EPCollider = B2CollideEdgeAndPolygon_s_collider
	collider.Collide(manifold, edgeA, xfA, polygonB, xfB)
}
