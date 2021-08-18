import {
	B2_epsilon,
	B2_linearSlop,
	B2_maxLinearCorrection,
	B2MakeNullArray,
	B2MakeNumberArray
} from '../../Common/b2Settings'
import { B2Sq, B2Sqrt, B2Vec2 } from '../../Common/b2Math'
import { B2Joint, B2JointDef, B2JointType } from './b2Joint'
import { B2DistanceJoint, B2DistanceJointDef } from './b2DistanceJoint'
import { B2SolverData } from '../b2TimeStep'
import { B2Body } from '../b2Body'
import { B2World } from '../b2World'

export class B2AreaJointDef extends B2JointDef {
	public world: B2World = null

	public bodies: B2Body[] = []

	public frequencyHz: number = 0

	public dampingRatio: number = 0

	constructor() {
		super(B2JointType.e_areaJoint)
	}

	public AddBody(body: B2Body): void {
		this.bodies.push(body)

		if (this.bodies.length === 1) {
			this.bodyA = body
		} else if (this.bodies.length === 2) {
			this.bodyB = body
		}
	}
}

export class B2AreaJoint extends B2Joint {
	public m_bodies: B2Body[] = null
	public m_frequencyHz: number = 0
	public m_dampingRatio: number = 0

	// Solver shared
	public m_impulse: number = 0

	// Solver temp
	public m_targetLengths: number[] = null
	public m_targetArea: number = 0
	public m_normals: B2Vec2[] = null
	public m_joints: B2DistanceJoint[] = null
	public m_deltas: B2Vec2[] = null
	public m_delta: B2Vec2 = null

	constructor(def: B2AreaJointDef) {
		super(def)

		/// b2Assert(def.bodies.length >= 3, 'You cannot create an area joint with less than three bodies.');

		this.m_bodies = def.bodies
		this.m_frequencyHz = def.frequencyHz
		this.m_dampingRatio = def.dampingRatio

		this.m_targetLengths = B2MakeNumberArray(def.bodies.length)
		this.m_normals = B2Vec2.MakeArray(def.bodies.length)
		this.m_joints = B2MakeNullArray(def.bodies.length)
		this.m_deltas = B2Vec2.MakeArray(def.bodies.length)
		this.m_delta = new B2Vec2()

		const djd: B2DistanceJointDef = new B2DistanceJointDef()
		djd.frequencyHz = def.frequencyHz
		djd.dampingRatio = def.dampingRatio

		this.m_targetArea = 0

		for (let i: number = 0; i < this.m_bodies.length; ++i) {
			const body: B2Body = this.m_bodies[i]
			const next: B2Body = this.m_bodies[(i + 1) % this.m_bodies.length]

			const body_c: B2Vec2 = body.GetWorldCenter()
			const next_c: B2Vec2 = next.GetWorldCenter()

			this.m_targetLengths[i] = B2Vec2.DistanceVV(body_c, next_c)

			this.m_targetArea += B2Vec2.CrossVV(body_c, next_c)

			djd.Initialize(body, next, body_c, next_c)
			this.m_joints[i] = <B2DistanceJoint>def.world.CreateJoint(djd)
		}

		this.m_targetArea *= 0.5
	}

	public GetAnchorA(out: B2Vec2): B2Vec2 {
		return out.SetZero()
	}

	public GetAnchorB(out: B2Vec2): B2Vec2 {
		return out.SetZero()
	}

	public GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2 {
		return out.SetZero()
	}

	public GetReactionTorque(inv_dt: number): number {
		return 0
	}

	public SetFrequency(hz: number): void {
		this.m_frequencyHz = hz

		for (let i: number = 0; i < this.m_joints.length; ++i) {
			this.m_joints[i].SetFrequency(hz)
		}
	}

	public GetFrequency() {
		return this.m_frequencyHz
	}

	public SetDampingRatio(ratio: number): void {
		this.m_dampingRatio = ratio

		for (let i: number = 0; i < this.m_joints.length; ++i) {
			this.m_joints[i].SetDampingRatio(ratio)
		}
	}

	public GetDampingRatio() {
		return this.m_dampingRatio
	}

	public Dump(log: (format: string, ...args: any[]) => void) {
		log('Area joint dumping is not supported.\n')
	}

	public InitVelocityConstraints(data: B2SolverData): void {
		for (let i: number = 0; i < this.m_bodies.length; ++i) {
			const prev: B2Body = this.m_bodies[
				(i + this.m_bodies.length - 1) % this.m_bodies.length
			]
			const next: B2Body = this.m_bodies[(i + 1) % this.m_bodies.length]
			const prev_c: B2Vec2 = data.positions[prev.m_islandIndex].c
			const next_c: B2Vec2 = data.positions[next.m_islandIndex].c
			const delta: B2Vec2 = this.m_deltas[i]

			B2Vec2.SubVV(next_c, prev_c, delta)
		}

		if (data.step.warmStarting) {
			this.m_impulse *= data.step.dtRatio

			for (let i: number = 0; i < this.m_bodies.length; ++i) {
				const body: B2Body = this.m_bodies[i]
				const body_v: B2Vec2 = data.velocities[body.m_islandIndex].v
				const delta: B2Vec2 = this.m_deltas[i]

				body_v.x += body.m_invMass * delta.y * 0.5 * this.m_impulse
				body_v.y += body.m_invMass * -delta.x * 0.5 * this.m_impulse
			}
		} else {
			this.m_impulse = 0
		}
	}

	public SolveVelocityConstraints(data: B2SolverData): void {
		let dotMassSum: number = 0
		let crossMassSum: number = 0

		for (let i: number = 0; i < this.m_bodies.length; ++i) {
			const body: B2Body = this.m_bodies[i]
			const body_v: B2Vec2 = data.velocities[body.m_islandIndex].v
			const delta: B2Vec2 = this.m_deltas[i]

			dotMassSum += delta.LengthSquared() / body.GetMass()
			crossMassSum += B2Vec2.CrossVV(body_v, delta)
		}

		const lambda: number = (-2 * crossMassSum) / dotMassSum
		// lambda = B2Clamp(lambda, -B2_maxLinearCorrection, B2_maxLinearCorrection);

		this.m_impulse += lambda

		for (let i: number = 0; i < this.m_bodies.length; ++i) {
			const body: B2Body = this.m_bodies[i]
			const body_v: B2Vec2 = data.velocities[body.m_islandIndex].v
			const delta: B2Vec2 = this.m_deltas[i]

			body_v.x += body.m_invMass * delta.y * 0.5 * lambda
			body_v.y += body.m_invMass * -delta.x * 0.5 * lambda
		}
	}

	public SolvePositionConstraints(data: B2SolverData): boolean {
		let perimeter: number = 0
		let area: number = 0

		for (let i: number = 0; i < this.m_bodies.length; ++i) {
			const body: B2Body = this.m_bodies[i]
			const next: B2Body = this.m_bodies[(i + 1) % this.m_bodies.length]
			const body_c: B2Vec2 = data.positions[body.m_islandIndex].c
			const next_c: B2Vec2 = data.positions[next.m_islandIndex].c

			const delta: B2Vec2 = B2Vec2.SubVV(next_c, body_c, this.m_delta)

			let dist: number = delta.Length()
			if (dist < B2_epsilon) {
				dist = 1
			}

			this.m_normals[i].x = delta.y / dist
			this.m_normals[i].y = -delta.x / dist

			perimeter += dist

			area += B2Vec2.CrossVV(body_c, next_c)
		}

		area *= 0.5

		const deltaArea: number = this.m_targetArea - area
		const toExtrude: number = (0.5 * deltaArea) / perimeter
		let done: boolean = true

		for (let i: number = 0; i < this.m_bodies.length; ++i) {
			const body: B2Body = this.m_bodies[i]
			const body_c: B2Vec2 = data.positions[body.m_islandIndex].c
			const next_i: number = (i + 1) % this.m_bodies.length

			const delta: B2Vec2 = B2Vec2.AddVV(
				this.m_normals[i],
				this.m_normals[next_i],
				this.m_delta
			)
			delta.SelfMul(toExtrude)

			const norm_sq: number = delta.LengthSquared()
			if (norm_sq > B2Sq(B2_maxLinearCorrection)) {
				delta.SelfMul(B2_maxLinearCorrection / B2Sqrt(norm_sq))
			}
			if (norm_sq > B2Sq(B2_linearSlop)) {
				done = false
			}

			body_c.x += delta.x
			body_c.y += delta.y
		}

		return done
	}
}
