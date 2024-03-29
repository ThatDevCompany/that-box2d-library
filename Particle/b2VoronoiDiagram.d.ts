import { B2Vec2 } from '../Common/b2Math';
/**
 * A field representing the nearest generator from each point.
 */
export declare class B2VoronoiDiagram {
    m_generatorBuffer: B2VoronoiDiagram.Generator[];
    m_generatorCapacity: number;
    m_generatorCount: number;
    m_countX: number;
    m_countY: number;
    m_diagram: B2VoronoiDiagram.Generator[];
    constructor(generatorCapacity: number);
    /**
     * Add a generator.
     *
     * @param center the position of the generator.
     * @param tag a tag used to identify the generator in callback functions.
     * @param necessary whether to callback for nodes associated with the generator.
     */
    AddGenerator(center: B2Vec2, tag: number, necessary: boolean): void;
    /**
     * Generate the Voronoi diagram. It is rasterized with a given
     * interval in the same range as the necessary generators exist.
     *
     * @param radius the interval of the diagram.
     * @param margin margin for which the range of the diagram is extended.
     */
    Generate(radius: number, margin: number): void;
    /**
     * Enumerate all nodes that contain at least one necessary
     * generator.
     */
    GetNodes(callback: B2VoronoiDiagram.NodeCallback): void;
}
export declare namespace B2VoronoiDiagram {
    /**
     * Callback used by GetNodes().
     *
     * Receive tags for generators associated with a node.
     */
    type NodeCallback = (a: number, b: number, c: number) => void;
    class Generator {
        center: B2Vec2;
        tag: number;
        necessary: boolean;
    }
    class Task {
        m_x: number;
        m_y: number;
        m_i: number;
        m_generator: B2VoronoiDiagram.Generator;
        constructor(x: number, y: number, i: number, g: B2VoronoiDiagram.Generator);
    }
}
