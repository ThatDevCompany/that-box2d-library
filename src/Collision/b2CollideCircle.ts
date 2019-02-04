import { B2_maxFloat, B2_epsilon } from '../Common/b2Settings'
import { B2Vec2, B2Transform } from '../Common/b2Math'
import { B2Manifold, B2ManifoldType } from './b2Collision'
import { B2CircleShape } from './Shapes/b2CircleShape'
import { B2PolygonShape } from './Shapes/b2PolygonShape'

const B2CollideCircles_s_pA: B2Vec2 = new B2Vec2()
const B2CollideCircles_s_pB: B2Vec2 = new B2Vec2()

export function B2CollideCircles(
	manifold: B2Manifold,
	circleA: B2CircleShape,
	xfA: B2Transform,
	circleB: B2CircleShape,
	xfB: B2Transform
): void {
	manifold.pointCount = 0

	const pA: B2Vec2 = B2Transform.MulXV(xfA, circleA.m_p, B2CollideCircles_s_pA)
	const pB: B2Vec2 = B2Transform.MulXV(xfB, circleB.m_p, B2CollideCircles_s_pB)

	const distSqr: number = B2Vec2.DistanceSquaredVV(pA, pB)
	const radius: number = circleA.m_radius + circleB.m_radius
	if (distSqr > radius * radius) {
		return
	}

	manifold.type = B2ManifoldType.e_circles
	manifold.localPoint.Copy(circleA.m_p)
	manifold.localNormal.SetZero()
	manifold.pointCount = 1

	manifold.points[0].localPoint.Copy(circleB.m_p)
	manifold.points[0].id.key = 0
}

const B2CollidePolygonAndCircle_s_c: B2Vec2 = new B2Vec2()
const B2CollidePolygonAndCircle_s_cLocal: B2Vec2 = new B2Vec2()
const B2CollidePolygonAndCircle_s_faceCenter: B2Vec2 = new B2Vec2()

export function B2CollidePolygonAndCircle(
	manifold: B2Manifold,
	polygonA: B2PolygonShape,
	xfA: B2Transform,
	circleB: B2CircleShape,
	xfB: B2Transform
): void {
	manifold.pointCount = 0

	// Compute circle position in the frame of the polygon.
	const c: B2Vec2 = B2Transform.MulXV(
		xfB,
		circleB.m_p,
		B2CollidePolygonAndCircle_s_c
	)
	const cLocal: B2Vec2 = B2Transform.MulTXV(
		xfA,
		c,
		B2CollidePolygonAndCircle_s_cLocal
	)

	// Find the min separating edge.
	let normalIndex: number = 0
	let separation: number = -B2_maxFloat
	const radius: number = polygonA.m_radius + circleB.m_radius
	const vertexCount: number = polygonA.m_count
	const vertices: B2Vec2[] = polygonA.m_vertices
	const normals: B2Vec2[] = polygonA.m_normals

	for (let i: number = 0; i < vertexCount; ++i) {
		const s: number = B2Vec2.DotVV(
			normals[i],
			B2Vec2.SubVV(cLocal, vertices[i], B2Vec2.s_t0)
		)

		if (s > radius) {
			// Early out.
			return
		}

		if (s > separation) {
			separation = s
			normalIndex = i
		}
	}

	// Vertices that subtend the incident face.
	const vertIndex1: number = normalIndex
	const vertIndex2: number = (vertIndex1 + 1) % vertexCount
	const v1: B2Vec2 = vertices[vertIndex1]
	const v2: B2Vec2 = vertices[vertIndex2]

	// If the center is inside the polygon ...
	if (separation < B2_epsilon) {
		manifold.pointCount = 1
		manifold.type = B2ManifoldType.e_faceA
		manifold.localNormal.Copy(normals[normalIndex])
		B2Vec2.MidVV(v1, v2, manifold.localPoint)
		manifold.points[0].localPoint.Copy(circleB.m_p)
		manifold.points[0].id.key = 0
		return
	}

	// Compute barycentric coordinates
	const u1: number = B2Vec2.DotVV(
		B2Vec2.SubVV(cLocal, v1, B2Vec2.s_t0),
		B2Vec2.SubVV(v2, v1, B2Vec2.s_t1)
	)
	const u2: number = B2Vec2.DotVV(
		B2Vec2.SubVV(cLocal, v2, B2Vec2.s_t0),
		B2Vec2.SubVV(v1, v2, B2Vec2.s_t1)
	)
	if (u1 <= 0) {
		if (B2Vec2.DistanceSquaredVV(cLocal, v1) > radius * radius) {
			return
		}

		manifold.pointCount = 1
		manifold.type = B2ManifoldType.e_faceA
		B2Vec2.SubVV(cLocal, v1, manifold.localNormal).SelfNormalize()
		manifold.localPoint.Copy(v1)
		manifold.points[0].localPoint.Copy(circleB.m_p)
		manifold.points[0].id.key = 0
	} else if (u2 <= 0) {
		if (B2Vec2.DistanceSquaredVV(cLocal, v2) > radius * radius) {
			return
		}

		manifold.pointCount = 1
		manifold.type = B2ManifoldType.e_faceA
		B2Vec2.SubVV(cLocal, v2, manifold.localNormal).SelfNormalize()
		manifold.localPoint.Copy(v2)
		manifold.points[0].localPoint.Copy(circleB.m_p)
		manifold.points[0].id.key = 0
	} else {
		const faceCenter: B2Vec2 = B2Vec2.MidVV(
			v1,
			v2,
			B2CollidePolygonAndCircle_s_faceCenter
		)
		separation = B2Vec2.DotVV(
			B2Vec2.SubVV(cLocal, faceCenter, B2Vec2.s_t1),
			normals[vertIndex1]
		)
		if (separation > radius) {
			return
		}

		manifold.pointCount = 1
		manifold.type = B2ManifoldType.e_faceA
		manifold.localNormal.Copy(normals[vertIndex1]).SelfNormalize()
		manifold.localPoint.Copy(faceCenter)
		manifold.points[0].localPoint.Copy(circleB.m_p)
		manifold.points[0].id.key = 0
	}
}
