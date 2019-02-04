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

import { B2_linearSlop, B2_maxPolygonVertices } from '../Common/b2Settings'
import {
	B2Abs,
	B2Max,
	B2Vec2,
	B2Rot,
	B2Transform,
	B2Sweep
} from '../Common/b2Math'
import { B2Timer } from '../Common/b2Timer'
import {
	B2Distance,
	B2DistanceInput,
	B2DistanceOutput,
	B2DistanceProxy,
	B2SimplexCache
} from './b2Distance'

export let B2_toiTime: number = 0
export let B2_toiMaxTime: number = 0
export let B2_toiCalls: number = 0
export let B2_toiIters: number = 0
export let B2_toiMaxIters: number = 0
export let B2_toiRootIters: number = 0
export let B2_toiMaxRootIters: number = 0

const B2TimeOfImpact_s_xfA: B2Transform = new B2Transform()
const B2TimeOfImpact_s_xfB: B2Transform = new B2Transform()
const B2TimeOfImpact_s_pointA: B2Vec2 = new B2Vec2()
const B2TimeOfImpact_s_pointB: B2Vec2 = new B2Vec2()
const B2TimeOfImpact_s_normal: B2Vec2 = new B2Vec2()
const B2TimeOfImpact_s_axisA: B2Vec2 = new B2Vec2()
const B2TimeOfImpact_s_axisB: B2Vec2 = new B2Vec2()

///  Input parameters for B2TimeOfImpact
export class B2TOIInput {
	public proxyA: B2DistanceProxy = new B2DistanceProxy()
	public proxyB: B2DistanceProxy = new B2DistanceProxy()
	public sweepA: B2Sweep = new B2Sweep()
	public sweepB: B2Sweep = new B2Sweep()
	public tMax: number = 0 // defines sweep interval [0, tMax]
}

export const enum B2TOIOutputState {
	e_unknown = 0,
	e_failed = 1,
	e_overlapped = 2,
	e_touching = 3,
	e_separated = 4
}

export class B2TOIOutput {
	public state = B2TOIOutputState.e_unknown
	public t: number = 0
}

export const enum B2SeparationFunctionType {
	e_unknown = -1,
	e_points = 0,
	e_faceA = 1,
	e_faceB = 2
}

export class B2SeparationFunction {
	public m_proxyA: B2DistanceProxy
	public m_proxyB: B2DistanceProxy
	public m_sweepA: B2Sweep = new B2Sweep()
	public m_sweepB: B2Sweep = new B2Sweep()
	public m_type: B2SeparationFunctionType = B2SeparationFunctionType.e_unknown
	public m_localPoint: B2Vec2 = new B2Vec2()
	public m_axis: B2Vec2 = new B2Vec2()

	public Initialize(
		cache: B2SimplexCache,
		proxyA: B2DistanceProxy,
		sweepA: B2Sweep,
		proxyB: B2DistanceProxy,
		sweepB: B2Sweep,
		t1: number
	): number {
		this.m_proxyA = proxyA
		this.m_proxyB = proxyB
		const count: number = cache.count
		/// b2Assert(0 < count && count < 3);

		this.m_sweepA.Copy(sweepA)
		this.m_sweepB.Copy(sweepB)

		const xfA: B2Transform = B2TimeOfImpact_s_xfA
		const xfB: B2Transform = B2TimeOfImpact_s_xfB
		this.m_sweepA.GetTransform(xfA, t1)
		this.m_sweepB.GetTransform(xfB, t1)

		if (count === 1) {
			this.m_type = B2SeparationFunctionType.e_points
			const localPointA: B2Vec2 = this.m_proxyA.GetVertex(cache.indexA[0])
			const localPointB: B2Vec2 = this.m_proxyB.GetVertex(cache.indexB[0])
			const pointA: B2Vec2 = B2Transform.MulXV(
				xfA,
				localPointA,
				B2TimeOfImpact_s_pointA
			)
			const pointB: B2Vec2 = B2Transform.MulXV(
				xfB,
				localPointB,
				B2TimeOfImpact_s_pointB
			)
			B2Vec2.SubVV(pointB, pointA, this.m_axis)
			const s: number = this.m_axis.Normalize()
			/// #if B2_ENABLE_PARTICLE
			this.m_localPoint.SetZero()
			/// #endif
			return s
		} else if (cache.indexA[0] === cache.indexA[1]) {
			// Two points on B and one on A.
			this.m_type = B2SeparationFunctionType.e_faceB
			const localPointB1: B2Vec2 = this.m_proxyB.GetVertex(cache.indexB[0])
			const localPointB2: B2Vec2 = this.m_proxyB.GetVertex(cache.indexB[1])

			B2Vec2.CrossVOne(
				B2Vec2.SubVV(localPointB2, localPointB1, B2Vec2.s_t0),
				this.m_axis
			).SelfNormalize()
			const normal: B2Vec2 = B2Rot.MulRV(
				xfB.q,
				this.m_axis,
				B2TimeOfImpact_s_normal
			)

			B2Vec2.MidVV(localPointB1, localPointB2, this.m_localPoint)
			const pointB: B2Vec2 = B2Transform.MulXV(
				xfB,
				this.m_localPoint,
				B2TimeOfImpact_s_pointB
			)

			const localPointA: B2Vec2 = this.m_proxyA.GetVertex(cache.indexA[0])
			const pointA: B2Vec2 = B2Transform.MulXV(
				xfA,
				localPointA,
				B2TimeOfImpact_s_pointA
			)

			let s: number = B2Vec2.DotVV(
				B2Vec2.SubVV(pointA, pointB, B2Vec2.s_t0),
				normal
			)
			if (s < 0) {
				this.m_axis.SelfNeg()
				s = -s
			}
			return s
		} else {
			// Two points on A and one or two points on B.
			this.m_type = B2SeparationFunctionType.e_faceA
			const localPointA1: B2Vec2 = this.m_proxyA.GetVertex(cache.indexA[0])
			const localPointA2: B2Vec2 = this.m_proxyA.GetVertex(cache.indexA[1])

			B2Vec2.CrossVOne(
				B2Vec2.SubVV(localPointA2, localPointA1, B2Vec2.s_t0),
				this.m_axis
			).SelfNormalize()
			const normal: B2Vec2 = B2Rot.MulRV(
				xfA.q,
				this.m_axis,
				B2TimeOfImpact_s_normal
			)

			B2Vec2.MidVV(localPointA1, localPointA2, this.m_localPoint)
			const pointA: B2Vec2 = B2Transform.MulXV(
				xfA,
				this.m_localPoint,
				B2TimeOfImpact_s_pointA
			)

			const localPointB: B2Vec2 = this.m_proxyB.GetVertex(cache.indexB[0])
			const pointB: B2Vec2 = B2Transform.MulXV(
				xfB,
				localPointB,
				B2TimeOfImpact_s_pointB
			)

			let s: number = B2Vec2.DotVV(
				B2Vec2.SubVV(pointB, pointA, B2Vec2.s_t0),
				normal
			)
			if (s < 0) {
				this.m_axis.SelfNeg()
				s = -s
			}
			return s
		}
	}

	public FindMinSeparation(
		indexA: number[],
		indexB: number[],
		t: number
	): number {
		const xfA: B2Transform = B2TimeOfImpact_s_xfA
		const xfB: B2Transform = B2TimeOfImpact_s_xfB
		this.m_sweepA.GetTransform(xfA, t)
		this.m_sweepB.GetTransform(xfB, t)

		switch (this.m_type) {
			case B2SeparationFunctionType.e_points: {
				const axisA: B2Vec2 = B2Rot.MulTRV(
					xfA.q,
					this.m_axis,
					B2TimeOfImpact_s_axisA
				)
				const axisB: B2Vec2 = B2Rot.MulTRV(
					xfB.q,
					B2Vec2.NegV(this.m_axis, B2Vec2.s_t0),
					B2TimeOfImpact_s_axisB
				)

				indexA[0] = this.m_proxyA.GetSupport(axisA)
				indexB[0] = this.m_proxyB.GetSupport(axisB)

				const localPointA: B2Vec2 = this.m_proxyA.GetVertex(indexA[0])
				const localPointB: B2Vec2 = this.m_proxyB.GetVertex(indexB[0])

				const pointA: B2Vec2 = B2Transform.MulXV(
					xfA,
					localPointA,
					B2TimeOfImpact_s_pointA
				)
				const pointB: B2Vec2 = B2Transform.MulXV(
					xfB,
					localPointB,
					B2TimeOfImpact_s_pointB
				)

				const separation: number = B2Vec2.DotVV(
					B2Vec2.SubVV(pointB, pointA, B2Vec2.s_t0),
					this.m_axis
				)
				return separation
			}

			case B2SeparationFunctionType.e_faceA: {
				const normal: B2Vec2 = B2Rot.MulRV(
					xfA.q,
					this.m_axis,
					B2TimeOfImpact_s_normal
				)
				const pointA: B2Vec2 = B2Transform.MulXV(
					xfA,
					this.m_localPoint,
					B2TimeOfImpact_s_pointA
				)

				const axisB: B2Vec2 = B2Rot.MulTRV(
					xfB.q,
					B2Vec2.NegV(normal, B2Vec2.s_t0),
					B2TimeOfImpact_s_axisB
				)

				indexA[0] = -1
				indexB[0] = this.m_proxyB.GetSupport(axisB)

				const localPointB: B2Vec2 = this.m_proxyB.GetVertex(indexB[0])
				const pointB: B2Vec2 = B2Transform.MulXV(
					xfB,
					localPointB,
					B2TimeOfImpact_s_pointB
				)

				const separation: number = B2Vec2.DotVV(
					B2Vec2.SubVV(pointB, pointA, B2Vec2.s_t0),
					normal
				)
				return separation
			}

			case B2SeparationFunctionType.e_faceB: {
				const normal: B2Vec2 = B2Rot.MulRV(
					xfB.q,
					this.m_axis,
					B2TimeOfImpact_s_normal
				)
				const pointB: B2Vec2 = B2Transform.MulXV(
					xfB,
					this.m_localPoint,
					B2TimeOfImpact_s_pointB
				)

				const axisA: B2Vec2 = B2Rot.MulTRV(
					xfA.q,
					B2Vec2.NegV(normal, B2Vec2.s_t0),
					B2TimeOfImpact_s_axisA
				)

				indexB[0] = -1
				indexA[0] = this.m_proxyA.GetSupport(axisA)

				const localPointA: B2Vec2 = this.m_proxyA.GetVertex(indexA[0])
				const pointA: B2Vec2 = B2Transform.MulXV(
					xfA,
					localPointA,
					B2TimeOfImpact_s_pointA
				)

				const separation: number = B2Vec2.DotVV(
					B2Vec2.SubVV(pointA, pointB, B2Vec2.s_t0),
					normal
				)
				return separation
			}

			default:
				/// b2Assert(false);
				indexA[0] = -1
				indexB[0] = -1
				return 0
		}
	}

	public Evaluate(indexA: number, indexB: number, t: number): number {
		const xfA: B2Transform = B2TimeOfImpact_s_xfA
		const xfB: B2Transform = B2TimeOfImpact_s_xfB
		this.m_sweepA.GetTransform(xfA, t)
		this.m_sweepB.GetTransform(xfB, t)

		switch (this.m_type) {
			case B2SeparationFunctionType.e_points: {
				const localPointA: B2Vec2 = this.m_proxyA.GetVertex(indexA)
				const localPointB: B2Vec2 = this.m_proxyB.GetVertex(indexB)

				const pointA: B2Vec2 = B2Transform.MulXV(
					xfA,
					localPointA,
					B2TimeOfImpact_s_pointA
				)
				const pointB: B2Vec2 = B2Transform.MulXV(
					xfB,
					localPointB,
					B2TimeOfImpact_s_pointB
				)
				const separation: number = B2Vec2.DotVV(
					B2Vec2.SubVV(pointB, pointA, B2Vec2.s_t0),
					this.m_axis
				)

				return separation
			}

			case B2SeparationFunctionType.e_faceA: {
				const normal: B2Vec2 = B2Rot.MulRV(
					xfA.q,
					this.m_axis,
					B2TimeOfImpact_s_normal
				)
				const pointA: B2Vec2 = B2Transform.MulXV(
					xfA,
					this.m_localPoint,
					B2TimeOfImpact_s_pointA
				)

				const localPointB: B2Vec2 = this.m_proxyB.GetVertex(indexB)
				const pointB: B2Vec2 = B2Transform.MulXV(
					xfB,
					localPointB,
					B2TimeOfImpact_s_pointB
				)

				const separation: number = B2Vec2.DotVV(
					B2Vec2.SubVV(pointB, pointA, B2Vec2.s_t0),
					normal
				)
				return separation
			}

			case B2SeparationFunctionType.e_faceB: {
				const normal: B2Vec2 = B2Rot.MulRV(
					xfB.q,
					this.m_axis,
					B2TimeOfImpact_s_normal
				)
				const pointB: B2Vec2 = B2Transform.MulXV(
					xfB,
					this.m_localPoint,
					B2TimeOfImpact_s_pointB
				)

				const localPointA: B2Vec2 = this.m_proxyA.GetVertex(indexA)
				const pointA: B2Vec2 = B2Transform.MulXV(
					xfA,
					localPointA,
					B2TimeOfImpact_s_pointA
				)

				const separation: number = B2Vec2.DotVV(
					B2Vec2.SubVV(pointA, pointB, B2Vec2.s_t0),
					normal
				)
				return separation
			}

			default:
				/// b2Assert(false);
				return 0
		}
	}
}

const B2TimeOfImpact_s_timer: B2Timer = new B2Timer()
const B2TimeOfImpact_s_cache: B2SimplexCache = new B2SimplexCache()
const B2TimeOfImpact_s_distanceInput: B2DistanceInput = new B2DistanceInput()
const B2TimeOfImpact_s_distanceOutput: B2DistanceOutput = new B2DistanceOutput()
const B2TimeOfImpact_s_fcn: B2SeparationFunction = new B2SeparationFunction()
const B2TimeOfImpact_s_indexA = [0]
const B2TimeOfImpact_s_indexB = [0]
const B2TimeOfImpact_s_sweepA: B2Sweep = new B2Sweep()
const B2TimeOfImpact_s_sweepB: B2Sweep = new B2Sweep()

export function B2TimeOfImpact(output: B2TOIOutput, input: B2TOIInput): void {
	const timer: B2Timer = B2TimeOfImpact_s_timer.Reset()

	++B2_toiCalls

	output.state = B2TOIOutputState.e_unknown
	output.t = input.tMax

	const proxyA: B2DistanceProxy = input.proxyA
	const proxyB: B2DistanceProxy = input.proxyB

	const sweepA: B2Sweep = B2TimeOfImpact_s_sweepA.Copy(input.sweepA)
	const sweepB: B2Sweep = B2TimeOfImpact_s_sweepB.Copy(input.sweepB)

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	sweepA.Normalize()
	sweepB.Normalize()

	const tMax: number = input.tMax

	const totalRadius: number = proxyA.m_radius + proxyB.m_radius
	const target: number = B2Max(B2_linearSlop, totalRadius - 3 * B2_linearSlop)
	const tolerance: number = 0.25 * B2_linearSlop
	/// b2Assert(target > tolerance);

	let t1: number = 0
	const k_maxIterations: number = 20 // TODO_ERIN B2Settings
	let iter: number = 0

	// Prepare input for distance query.
	const cache: B2SimplexCache = B2TimeOfImpact_s_cache
	cache.count = 0
	const distanceInput: B2DistanceInput = B2TimeOfImpact_s_distanceInput
	distanceInput.proxyA = input.proxyA
	distanceInput.proxyB = input.proxyB
	distanceInput.useRadii = false

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for (;;) {
		const xfA: B2Transform = B2TimeOfImpact_s_xfA
		const xfB: B2Transform = B2TimeOfImpact_s_xfB
		sweepA.GetTransform(xfA, t1)
		sweepB.GetTransform(xfB, t1)

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.transformA.Copy(xfA)
		distanceInput.transformB.Copy(xfB)
		const distanceOutput: B2DistanceOutput = B2TimeOfImpact_s_distanceOutput
		B2Distance(distanceOutput, cache, distanceInput)

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= 0) {
			// Failure!
			output.state = B2TOIOutputState.e_overlapped
			output.t = 0
			break
		}

		if (distanceOutput.distance < target + tolerance) {
			// Victory!
			output.state = B2TOIOutputState.e_touching
			output.t = t1
			break
		}

		// Initialize the separating axis.
		const fcn: B2SeparationFunction = B2TimeOfImpact_s_fcn
		fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1)
		/*
		#if 0
			// Dump the curve seen by the root finder {
			  const int32 N = 100;
			  float32 dx = 1.0f / N;
			  float32 xs[N+1];
			  float32 fs[N+1];

			  float32 x = 0.0f;

			  for (int32 i = 0; i <= N; ++i) {
				sweepA.GetTransform(&xfA, x);
				sweepB.GetTransform(&xfB, x);
				float32 f = fcn.Evaluate(xfA, xfB) - target;

				printf('%g %g\n', x, f);

				xs[i] = x;
				fs[i] = f;

				x += dx;
			  }
			}
		#endif
		*/

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		let done: boolean = false
		let t2: number = tMax
		let pushBackIter: number = 0
		for (;;) {
			// Find the deepest point at t2. Store the witness point indices.
			const indexA: number[] = B2TimeOfImpact_s_indexA
			const indexB: number[] = B2TimeOfImpact_s_indexB
			let s2: number = fcn.FindMinSeparation(indexA, indexB, t2)

			// Is the final configuration separated?
			if (s2 > target + tolerance) {
				// Victory!
				output.state = B2TOIOutputState.e_separated
				output.t = tMax
				done = true
				break
			}

			// Has the separation reached tolerance?
			if (s2 > target - tolerance) {
				// Advance the sweeps
				t1 = t2
				break
			}

			// Compute the initial separation of the witness points.
			let s1: number = fcn.Evaluate(indexA[0], indexB[0], t1)

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < target - tolerance) {
				output.state = B2TOIOutputState.e_failed
				output.t = t1
				done = true
				break
			}

			// Check for touching
			if (s1 <= target + tolerance) {
				// Victory! t1 should hold the TOI (could be 0.0).
				output.state = B2TOIOutputState.e_touching
				output.t = t1
				done = true
				break
			}

			// Compute 1D root of: f(x) - target = 0
			let rootIterCount: number = 0
			let a1: number = t1
			let a2: number = t2
			for (;;) {
				// Use a mix of the secant rule and bisection.
				let t: number = 0
				if (rootIterCount & 1) {
					// Secant rule to improve convergence.
					t = a1 + ((target - s1) * (a2 - a1)) / (s2 - s1)
				} else {
					// Bisection to guarantee progress.
					t = 0.5 * (a1 + a2)
				}

				++rootIterCount
				++B2_toiRootIters

				const s: number = fcn.Evaluate(indexA[0], indexB[0], t)

				if (B2Abs(s - target) < tolerance) {
					// t2 holds a tentative value for t1
					t2 = t
					break
				}

				// Ensure we continue to bracket the root.
				if (s > target) {
					a1 = t
					s1 = s
				} else {
					a2 = t
					s2 = s
				}

				if (rootIterCount === 50) {
					break
				}
			}

			B2_toiMaxRootIters = B2Max(B2_toiMaxRootIters, rootIterCount)

			++pushBackIter

			if (pushBackIter === B2_maxPolygonVertices) {
				break
			}
		}

		++iter
		++B2_toiIters

		if (done) {
			break
		}

		if (iter === k_maxIterations) {
			// Root finder got stuck. Semi-victory.
			output.state = B2TOIOutputState.e_failed
			output.t = t1
			break
		}
	}

	B2_toiMaxIters = B2Max(B2_toiMaxIters, iter)

	const time: number = timer.GetMilliseconds()
	B2_toiMaxTime = B2Max(B2_toiMaxTime, time)
	B2_toiTime += time
}
