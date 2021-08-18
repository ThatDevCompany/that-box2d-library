import { B2Transform } from '../Common/b2Math';
import { B2Manifold } from './b2Collision';
import { B2PolygonShape } from './Shapes/b2PolygonShape';
export declare function B2CollidePolygons(manifold: B2Manifold, polyA: B2PolygonShape, xfA: B2Transform, polyB: B2PolygonShape, xfB: B2Transform): void;
