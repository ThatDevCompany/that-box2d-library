import { B2Vec2 } from '../Common'

export interface B2AppliedForce {
	force: B2Vec2
	point: B2Vec2
}

export abstract class AbstractB2AppliedForce implements B2AppliedForce {
	force: B2Vec2
	point: B2Vec2
}
