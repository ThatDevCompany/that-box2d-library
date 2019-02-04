/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import {
	B2_epsilon,
	B2_maxFloat,
	B2_linearSlop,
	B2_polygonRadius,
	B2_maxPolygonVertices,
	B2MakeNumberArray
} from '../../Common/b2Settings'
import { B2Min, B2Vec2, B2Rot, B2Transform } from '../../Common/b2Math'
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../b2Collision'
import { B2DistanceProxy } from '../b2Distance'
import { B2MassData } from './b2Shape'
import { B2Shape, B2ShapeType } from './b2Shape'

///  A convex polygon. It is assumed that the interior of the polygon is to
///  the left of each edge.
///  Polygons have a maximum number of vertices equal to B2_maxPolygonVertices.
///  In most cases you should not need many vertices for a convex polygon.
export class B2PolygonShape extends B2Shape {
	public m_centroid: B2Vec2 = new B2Vec2(0, 0)
	public m_vertices: B2Vec2[] = B2Vec2.MakeArray(B2_maxPolygonVertices)
	public m_normals: B2Vec2[] = B2Vec2.MakeArray(B2_maxPolygonVertices)
	public m_count: number = 0

	constructor() {
		super(B2ShapeType.e_polygonShape, B2_polygonRadius)
	}

	///  Implement B2Shape.
	public Clone(): B2PolygonShape {
		return new B2PolygonShape().Copy(this)
	}

	public Copy(other: B2PolygonShape): B2PolygonShape {
		super.Copy(other)

		/// b2Assert(other instanceof B2PolygonShape);

		this.m_centroid.Copy(other.m_centroid)
		this.m_count = other.m_count
		for (let i: number = 0; i < this.m_count; ++i) {
			this.m_vertices[i].Copy(other.m_vertices[i])
			this.m_normals[i].Copy(other.m_normals[i])
		}
		return this
	}

	///  @see B2Shape::GetChildCount
	public GetChildCount(): number {
		return 1
	}

	///  Create a convex hull from the given array of points.
	///  The count must be in the range [3, B2_maxPolygonVertices].
	///  @warning the points may be re-ordered, even if they form a convex polygon
	///  @warning collinear points are handled but not removed. Collinear points
	///  may lead to poor stacking behavior.
	private static Set_s_ps = B2Vec2.MakeArray(B2_maxPolygonVertices)
	private static Set_s_hull = B2MakeNumberArray(B2_maxPolygonVertices)
	private static Set_s_r = new B2Vec2()
	private static Set_s_v = new B2Vec2()

	public Set(
		vertices: B2Vec2[],
		count: number = vertices.length,
		start: number = 0
	): B2PolygonShape {
		/// b2Assert(3 <= count && count <= B2_maxPolygonVertices);
		if (count < 3) {
			return this.SetAsBox(1, 1)
		}

		let n: number = B2Min(count, B2_maxPolygonVertices)

		// Perform welding and copy vertices into local buffer.
		const ps: B2Vec2[] = B2PolygonShape.Set_s_ps
		let tempCount = 0
		for (let i = 0; i < n; ++i) {
			const /*b2Vec2*/ v = vertices[start + i]

			let /*bool*/ unique = true
			for (let /*int32*/ j = 0; j < tempCount; ++j) {
				if (
					B2Vec2.DistanceSquaredVV(v, ps[j]) <
					0.5 * B2_linearSlop * (0.5 * B2_linearSlop)
				) {
					unique = false
					break
				}
			}

			if (unique) {
				ps[tempCount++].Copy(v) // ps[tempCount++] = v;
			}
		}

		n = tempCount
		if (n < 3) {
			// Polygon is degenerate.
			/// b2Assert(false);
			return this.SetAsBox(1.0, 1.0)
		}

		// Create the convex hull using the Gift wrapping algorithm
		// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

		// Find the right most point on the hull
		let i0: number = 0
		let x0: number = ps[0].x
		for (let i: number = 1; i < n; ++i) {
			const x: number = ps[i].x
			if (x > x0 || (x === x0 && ps[i].y < ps[i0].y)) {
				i0 = i
				x0 = x
			}
		}

		const hull: number[] = B2PolygonShape.Set_s_hull
		let m: number = 0
		let ih: number = i0

		for (;;) {
			hull[m] = ih

			let ie: number = 0
			for (let j: number = 1; j < n; ++j) {
				if (ie === ih) {
					ie = j
					continue
				}

				const r: B2Vec2 = B2Vec2.SubVV(
					ps[ie],
					ps[hull[m]],
					B2PolygonShape.Set_s_r
				)
				const v: B2Vec2 = B2Vec2.SubVV(
					ps[j],
					ps[hull[m]],
					B2PolygonShape.Set_s_v
				)
				const c: number = B2Vec2.CrossVV(r, v)
				if (c < 0) {
					ie = j
				}

				// Collinearity check
				if (c === 0 && v.LengthSquared() > r.LengthSquared()) {
					ie = j
				}
			}

			++m
			ih = ie

			if (ie === i0) {
				break
			}
		}

		this.m_count = m

		// Copy vertices.
		for (let i: number = 0; i < m; ++i) {
			this.m_vertices[i].Copy(ps[hull[i]])
		}

		// Compute normals. Ensure the edges have non-zero length.
		for (let i: number = 0; i < m; ++i) {
			const vertexi1: B2Vec2 = this.m_vertices[i]
			const vertexi2: B2Vec2 = this.m_vertices[(i + 1) % m]
			const edge: B2Vec2 = B2Vec2.SubVV(vertexi2, vertexi1, B2Vec2.s_t0) // edge uses s_t0
			/// b2Assert(edge.LengthSquared() > B2_epsilon_sq);
			B2Vec2.CrossVOne(edge, this.m_normals[i]).SelfNormalize()
		}

		// Compute the polygon centroid.
		B2PolygonShape.ComputeCentroid(this.m_vertices, m, this.m_centroid)

		return this
	}

	public SetAsArray(
		vertices: B2Vec2[],
		count: number = vertices.length
	): B2PolygonShape {
		return this.Set(vertices, count)
	}

	///  Build vertices to represent an axis-aligned box or an oriented box.
	///  @param hx the half-width.
	///  @param hy the half-height.
	///  @param center the center of the box in local coordinates.
	///  @param angle the rotation of the box in local coordinates.
	public SetAsBox(
		hx: number,
		hy: number,
		center?: B2Vec2,
		angle: number = 0
	): B2PolygonShape {
		this.m_count = 4
		this.m_vertices[0].Set(-hx, -hy)
		this.m_vertices[1].Set(hx, -hy)
		this.m_vertices[2].Set(hx, hy)
		this.m_vertices[3].Set(-hx, hy)
		this.m_normals[0].Set(0, -1)
		this.m_normals[1].Set(1, 0)
		this.m_normals[2].Set(0, 1)
		this.m_normals[3].Set(-1, 0)
		this.m_centroid.SetZero()

		if (center instanceof B2Vec2) {
			this.m_centroid.Copy(center)

			const xf: B2Transform = new B2Transform()
			xf.SetPosition(center)
			xf.SetRotationAngle(angle)

			// Transform vertices and normals.
			for (let i: number = 0; i < this.m_count; ++i) {
				B2Transform.MulXV(xf, this.m_vertices[i], this.m_vertices[i])
				B2Rot.MulRV(xf.q, this.m_normals[i], this.m_normals[i])
			}
		}

		return this
	}

	///  @see B2Shape::TestPoint
	private static TestPoint_s_pLocal = new B2Vec2()

	public TestPoint(xf: B2Transform, p: B2Vec2): boolean {
		const pLocal: B2Vec2 = B2Transform.MulTXV(
			xf,
			p,
			B2PolygonShape.TestPoint_s_pLocal
		)

		for (let i: number = 0; i < this.m_count; ++i) {
			const dot: number = B2Vec2.DotVV(
				this.m_normals[i],
				B2Vec2.SubVV(pLocal, this.m_vertices[i], B2Vec2.s_t0)
			)
			if (dot > 0) {
				return false
			}
		}

		return true
	}

	/// #if B2_ENABLE_PARTICLE
	///  @see B2Shape::ComputeDistance
	private static ComputeDistance_s_pLocal = new B2Vec2()
	private static ComputeDistance_s_normalForMaxDistance = new B2Vec2()
	private static ComputeDistance_s_minDistance = new B2Vec2()
	private static ComputeDistance_s_distance = new B2Vec2()

	public ComputeDistance(
		xf: B2Transform,
		p: B2Vec2,
		normal: B2Vec2,
		childIndex: number
	): number {
		const pLocal = B2Transform.MulTXV(
			xf,
			p,
			B2PolygonShape.ComputeDistance_s_pLocal
		)
		let maxDistance = -B2_maxFloat
		const normalForMaxDistance = B2PolygonShape.ComputeDistance_s_normalForMaxDistance.Copy(
			pLocal
		)

		for (let i = 0; i < this.m_count; ++i) {
			const dot = B2Vec2.DotVV(
				this.m_normals[i],
				B2Vec2.SubVV(pLocal, this.m_vertices[i], B2Vec2.s_t0)
			)
			if (dot > maxDistance) {
				maxDistance = dot
				normalForMaxDistance.Copy(this.m_normals[i])
			}
		}

		if (maxDistance > 0) {
			const minDistance = B2PolygonShape.ComputeDistance_s_minDistance.Copy(
				normalForMaxDistance
			)
			let minDistance2 = maxDistance * maxDistance
			for (let i = 0; i < this.m_count; ++i) {
				const distance = B2Vec2.SubVV(
					pLocal,
					this.m_vertices[i],
					B2PolygonShape.ComputeDistance_s_distance
				)
				const distance2 = distance.LengthSquared()
				if (minDistance2 > distance2) {
					minDistance.Copy(distance)
					minDistance2 = distance2
				}
			}

			B2Rot.MulRV(xf.q, minDistance, normal)
			normal.Normalize()
			return Math.sqrt(minDistance2)
		} else {
			B2Rot.MulRV(xf.q, normalForMaxDistance, normal)
			return maxDistance
		}
	}

	/// #endif

	///  Implement B2Shape.
	private static RayCast_s_p1 = new B2Vec2()
	private static RayCast_s_p2 = new B2Vec2()
	private static RayCast_s_d = new B2Vec2()

	public RayCast(
		output: B2RayCastOutput,
		input: B2RayCastInput,
		xf: B2Transform,
		childIndex: number
	): boolean {
		// Put the ray into the polygon's frame of reference.
		const p1: B2Vec2 = B2Transform.MulTXV(
			xf,
			input.p1,
			B2PolygonShape.RayCast_s_p1
		)
		const p2: B2Vec2 = B2Transform.MulTXV(
			xf,
			input.p2,
			B2PolygonShape.RayCast_s_p2
		)
		const d: B2Vec2 = B2Vec2.SubVV(p2, p1, B2PolygonShape.RayCast_s_d)

		let lower: number = 0,
			upper = input.maxFraction

		let index: number = -1

		for (let i: number = 0; i < this.m_count; ++i) {
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
			const numerator: number = B2Vec2.DotVV(
				this.m_normals[i],
				B2Vec2.SubVV(this.m_vertices[i], p1, B2Vec2.s_t0)
			)
			const denominator: number = B2Vec2.DotVV(this.m_normals[i], d)

			if (denominator === 0) {
				if (numerator < 0) {
					return false
				}
			} else {
				// Note: we want this predicate without division:
				// lower < numerator / denominator, where denominator < 0
				// Since denominator < 0, we have to flip the inequality:
				// lower < numerator / denominator <==> denominator * lower > numerator.
				if (denominator < 0 && numerator < lower * denominator) {
					// Increase lower.
					// The segment enters this half-space.
					lower = numerator / denominator
					index = i
				} else if (denominator > 0 && numerator < upper * denominator) {
					// Decrease upper.
					// The segment exits this half-space.
					upper = numerator / denominator
				}
			}

			// The use of epsilon here causes the assert on lower to trip
			// in some cases. Apparently the use of epsilon was to make edge
			// shapes work, but now those are handled separately.
			// if (upper < lower - B2_epsilon)
			if (upper < lower) {
				return false
			}
		}

		/// b2Assert(0 <= lower && lower <= input.maxFraction);

		if (index >= 0) {
			output.fraction = lower
			B2Rot.MulRV(xf.q, this.m_normals[index], output.normal)
			return true
		}

		return false
	}

	///  @see B2Shape::ComputeAABB
	private static ComputeAABB_s_v = new B2Vec2()

	public ComputeAABB(aabb: B2AABB, xf: B2Transform, childIndex: number): void {
		const lower: B2Vec2 = B2Transform.MulXV(
			xf,
			this.m_vertices[0],
			aabb.lowerBound
		)
		const upper: B2Vec2 = aabb.upperBound.Copy(lower)

		for (let i: number = 0; i < this.m_count; ++i) {
			const v: B2Vec2 = B2Transform.MulXV(
				xf,
				this.m_vertices[i],
				B2PolygonShape.ComputeAABB_s_v
			)
			B2Vec2.MinV(v, lower, lower)
			B2Vec2.MaxV(v, upper, upper)
		}

		const r: number = this.m_radius
		lower.SelfSubXY(r, r)
		upper.SelfAddXY(r, r)
	}

	///  @see B2Shape::ComputeMass
	private static ComputeMass_s_center = new B2Vec2()
	private static ComputeMass_s_s = new B2Vec2()
	private static ComputeMass_s_e1 = new B2Vec2()
	private static ComputeMass_s_e2 = new B2Vec2()

	public ComputeMass(massData: B2MassData, density: number): void {
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals
		// for each triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.

		/// b2Assert(this.m_count >= 3);

		const center: B2Vec2 = B2PolygonShape.ComputeMass_s_center.SetZero()
		let area: number = 0
		let I: number = 0

		// s is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		const s: B2Vec2 = B2PolygonShape.ComputeMass_s_s.SetZero()

		// This code would put the reference point inside the polygon.
		for (let i: number = 0; i < this.m_count; ++i) {
			s.SelfAdd(this.m_vertices[i])
		}
		s.SelfMul(1 / this.m_count)

		const k_inv3: number = 1 / 3

		for (let i: number = 0; i < this.m_count; ++i) {
			// Triangle vertices.
			const e1: B2Vec2 = B2Vec2.SubVV(
				this.m_vertices[i],
				s,
				B2PolygonShape.ComputeMass_s_e1
			)
			const e2: B2Vec2 = B2Vec2.SubVV(
				this.m_vertices[(i + 1) % this.m_count],
				s,
				B2PolygonShape.ComputeMass_s_e2
			)

			const D: number = B2Vec2.CrossVV(e1, e2)

			const triangleArea: number = 0.5 * D
			area += triangleArea

			// Area weighted centroid
			center.SelfAdd(
				B2Vec2.MulSV(
					triangleArea * k_inv3,
					B2Vec2.AddVV(e1, e2, B2Vec2.s_t0),
					B2Vec2.s_t1
				)
			)

			const ex1: number = e1.x
			const ey1: number = e1.y
			const ex2: number = e2.x
			const ey2: number = e2.y

			const intx2: number = ex1 * ex1 + ex2 * ex1 + ex2 * ex2
			const inty2: number = ey1 * ey1 + ey2 * ey1 + ey2 * ey2

			I += 0.25 * k_inv3 * D * (intx2 + inty2)
		}

		// Total mass
		massData.mass = density * area

		// Center of mass
		/// b2Assert(area > B2_epsilon);
		center.SelfMul(1 / area)
		B2Vec2.AddVV(center, s, massData.center)

		// Inertia tensor relative to the local origin (point s).
		massData.I = density * I

		// Shift to center of mass then to original body origin.
		massData.I +=
			massData.mass *
			(B2Vec2.DotVV(massData.center, massData.center) -
				B2Vec2.DotVV(center, center))
	}

	private static Validate_s_e = new B2Vec2()
	private static Validate_s_v = new B2Vec2()

	public Validate(): boolean {
		for (let i: number = 0; i < this.m_count; ++i) {
			const i1 = i
			const i2 = (i + 1) % this.m_count
			const p: B2Vec2 = this.m_vertices[i1]
			const e: B2Vec2 = B2Vec2.SubVV(
				this.m_vertices[i2],
				p,
				B2PolygonShape.Validate_s_e
			)

			for (let j: number = 0; j < this.m_count; ++j) {
				if (j === i1 || j === i2) {
					continue
				}

				const v: B2Vec2 = B2Vec2.SubVV(
					this.m_vertices[j],
					p,
					B2PolygonShape.Validate_s_v
				)
				const c: number = B2Vec2.CrossVV(e, v)
				if (c < 0) {
					return false
				}
			}
		}

		return true
	}

	public SetupDistanceProxy(proxy: B2DistanceProxy, index: number): void {
		proxy.m_vertices = this.m_vertices
		proxy.m_count = this.m_count
		proxy.m_radius = this.m_radius
	}

	private static ComputeSubmergedArea_s_normalL = new B2Vec2()
	private static ComputeSubmergedArea_s_depths = B2MakeNumberArray(
		B2_maxPolygonVertices
	)
	private static ComputeSubmergedArea_s_md = new B2MassData()
	private static ComputeSubmergedArea_s_intoVec = new B2Vec2()
	private static ComputeSubmergedArea_s_outoVec = new B2Vec2()
	private static ComputeSubmergedArea_s_center = new B2Vec2()

	public ComputeSubmergedArea(
		normal: B2Vec2,
		offset: number,
		xf: B2Transform,
		c: B2Vec2
	): number {
		// Transform plane into shape co-ordinates
		const normalL: B2Vec2 = B2Rot.MulTRV(
			xf.q,
			normal,
			B2PolygonShape.ComputeSubmergedArea_s_normalL
		)
		const offsetL: number = offset - B2Vec2.DotVV(normal, xf.p)

		const depths: number[] = B2PolygonShape.ComputeSubmergedArea_s_depths
		let diveCount: number = 0
		let intoIndex: number = -1
		let outoIndex: number = -1

		let lastSubmerged: boolean = false
		for (let i: number = 0; i < this.m_count; ++i) {
			depths[i] = B2Vec2.DotVV(normalL, this.m_vertices[i]) - offsetL
			const isSubmerged: boolean = depths[i] < -B2_epsilon
			if (i > 0) {
				if (isSubmerged) {
					if (!lastSubmerged) {
						intoIndex = i - 1
						diveCount++
					}
				} else {
					if (lastSubmerged) {
						outoIndex = i - 1
						diveCount++
					}
				}
			}
			lastSubmerged = isSubmerged
		}
		switch (diveCount) {
			case 0:
				if (lastSubmerged) {
					// Completely submerged
					const md: B2MassData = B2PolygonShape.ComputeSubmergedArea_s_md
					this.ComputeMass(md, 1)
					B2Transform.MulXV(xf, md.center, c)
					return md.mass
				} else {
					// Completely dry
					return 0
				}
			case 1:
				if (intoIndex === -1) {
					intoIndex = this.m_count - 1
				} else {
					outoIndex = this.m_count - 1
				}
				break
		}
		const intoIndex2: number = (intoIndex + 1) % this.m_count
		const outoIndex2: number = (outoIndex + 1) % this.m_count
		const intoLamdda: number =
			(0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex])
		const outoLamdda: number =
			(0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex])

		const intoVec: B2Vec2 = B2PolygonShape.ComputeSubmergedArea_s_intoVec.Set(
			this.m_vertices[intoIndex].x * (1 - intoLamdda) +
				this.m_vertices[intoIndex2].x * intoLamdda,
			this.m_vertices[intoIndex].y * (1 - intoLamdda) +
				this.m_vertices[intoIndex2].y * intoLamdda
		)
		const outoVec: B2Vec2 = B2PolygonShape.ComputeSubmergedArea_s_outoVec.Set(
			this.m_vertices[outoIndex].x * (1 - outoLamdda) +
				this.m_vertices[outoIndex2].x * outoLamdda,
			this.m_vertices[outoIndex].y * (1 - outoLamdda) +
				this.m_vertices[outoIndex2].y * outoLamdda
		)

		// Initialize accumulator
		let area: number = 0
		const center: B2Vec2 = B2PolygonShape.ComputeSubmergedArea_s_center.SetZero()
		let p2: B2Vec2 = this.m_vertices[intoIndex2]
		let p3: B2Vec2

		// An awkward loop from intoIndex2+1 to outIndex2
		let i: number = intoIndex2
		while (i !== outoIndex2) {
			i = (i + 1) % this.m_count
			if (i === outoIndex2) {
				p3 = outoVec
			} else {
				p3 = this.m_vertices[i]
			}
			const triangleArea: number =
				0.5 *
				((p2.x - intoVec.x) * (p3.y - intoVec.y) -
					(p2.y - intoVec.y) * (p3.x - intoVec.x))
			area += triangleArea
			// Area weighted centroid
			center.x += (triangleArea * (intoVec.x + p2.x + p3.x)) / 3
			center.y += (triangleArea * (intoVec.y + p2.y + p3.y)) / 3

			p2 = p3
		}

		// Normalize and transform centroid
		center.SelfMul(1 / area)
		B2Transform.MulXV(xf, center, c)

		return area
	}

	public Dump(log: (format: string, ...args: any[]) => void): void {
		log('    const shape: B2PolygonShape = new B2PolygonShape();\n')
		log(
			'    const vs: B2Vec2[] = B2Vec2.MakeArray(%d);\n',
			B2_maxPolygonVertices
		)
		for (let i: number = 0; i < this.m_count; ++i) {
			log(
				'    vs[%d].Set(%.15f, %.15f);\n',
				i,
				this.m_vertices[i].x,
				this.m_vertices[i].y
			)
		}
		log('    shape.Set(vs, %d);\n', this.m_count)
	}

	private static ComputeCentroid_s_pRef = new B2Vec2()
	private static ComputeCentroid_s_e1 = new B2Vec2()
	private static ComputeCentroid_s_e2 = new B2Vec2()

	public static ComputeCentroid(
		vs: B2Vec2[],
		count: number,
		out: B2Vec2
	): B2Vec2 {
		/// b2Assert(count >= 3);

		const c: B2Vec2 = out
		c.SetZero()
		let area: number = 0

		// s is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		const pRef: B2Vec2 = B2PolygonShape.ComputeCentroid_s_pRef.SetZero()
		/*
	#if 0
		// This code would put the reference point inside the polygon.
		for (let i: number = 0; i < count; ++i) {
		  pRef.SelfAdd(vs[i]);
		}
		pRef.SelfMul(1 / count);
	#endif
		*/

		const inv3: number = 1 / 3

		for (let i: number = 0; i < count; ++i) {
			// Triangle vertices.
			const p1: B2Vec2 = pRef
			const p2: B2Vec2 = vs[i]
			const p3: B2Vec2 = vs[(i + 1) % count]

			const e1: B2Vec2 = B2Vec2.SubVV(
				p2,
				p1,
				B2PolygonShape.ComputeCentroid_s_e1
			)
			const e2: B2Vec2 = B2Vec2.SubVV(
				p3,
				p1,
				B2PolygonShape.ComputeCentroid_s_e2
			)

			const D: number = B2Vec2.CrossVV(e1, e2)

			const triangleArea: number = 0.5 * D
			area += triangleArea

			// Area weighted centroid
			c.x += triangleArea * inv3 * (p1.x + p2.x + p3.x)
			c.y += triangleArea * inv3 * (p1.y + p2.y + p3.y)
		}

		// Centroid
		/// b2Assert(area > B2_epsilon);
		c.SelfMul(1 / area)
		return c
	}

	/*
	public static ComputeOBB(obb, vs, count) {
	  const i: number = 0;
	  const p: Array = [count + 1];
	  for (i = 0; i < count; ++i) {
		p[i] = vs[i];
	  }
	  p[count] = p[0];
	  const minArea = B2_maxFloat;
	  for (i = 1; i <= count; ++i) {
		const root = p[i - 1];
		const uxX = p[i].x - root.x;
		const uxY = p[i].y - root.y;
		const length = B2Sqrt(uxX * uxX + uxY * uxY);
		uxX /= length;
		uxY /= length;
		const uyX = (-uxY);
		const uyY = uxX;
		const lowerX = B2_maxFloat;
		const lowerY = B2_maxFloat;
		const upperX = (-B2_maxFloat);
		const upperY = (-B2_maxFloat);
		for (let j: number = 0; j < count; ++j) {
		  const dX = p[j].x - root.x;
		  const dY = p[j].y - root.y;
		  const rX = (uxX * dX + uxY * dY);
		  const rY = (uyX * dX + uyY * dY);
		  if (rX < lowerX) lowerX = rX;
		  if (rY < lowerY) lowerY = rY;
		  if (rX > upperX) upperX = rX;
		  if (rY > upperY) upperY = rY;
		}
		const area = (upperX - lowerX) * (upperY - lowerY);
		if (area < 0.95 * minArea) {
		  minArea = area;
		  obb.R.ex.x = uxX;
		  obb.R.ex.y = uxY;
		  obb.R.ey.x = uyX;
		  obb.R.ey.y = uyY;
		  const center_x: number = 0.5 * (lowerX + upperX);
		  const center_y: number = 0.5 * (lowerY + upperY);
		  const tMat = obb.R;
		  obb.center.x = root.x + (tMat.ex.x * center_x + tMat.ey.x * center_y);
		  obb.center.y = root.y + (tMat.ex.y * center_x + tMat.ey.y * center_y);
		  obb.extents.x = 0.5 * (upperX - lowerX);
		  obb.extents.y = 0.5 * (upperY - lowerY);
		}
	  }
	}
	*/
}
